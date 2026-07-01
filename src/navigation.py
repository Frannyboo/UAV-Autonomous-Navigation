#navigation.py
from dronekit import connect, VehicleMode, LocationGlobalRelative
from path_planning import *
import shared
import time
import math
import logging


logger = logging.getLogger(__name__)  # Get a logger for this module


def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters

    # Convert degrees to radians
    φ1 = math.radians(lat1)
    φ2 = math.radians(lat2)
    Δφ = math.radians(lat2 - lat1)
    Δλ = math.radians(lon2 - lon1)

    # Apply the haversine formula
    a = math.sin(Δφ / 2) ** 2 + math.cos(φ1) * math.cos(φ2) * math.sin(Δλ / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c
    return distance


# Distance calculator
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2[0] - aLocation1[0]
    dlon = aLocation2[1] - aLocation1[1]
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)


def connect_vehicle(connection_string):
    # logging.info("Connecting to drone...")
    # vehicle = connect(connection_string, wait_ready=True)
    logging.info("Connecting to drone...")
    while True:
        try:
            vehicle = connect(connection_string, wait_ready=True)
            logging.info("Vehicle connected")
            return vehicle
        except Exception as e:
            logging.warning(f"Connection failed: {e}")
            time.sleep(2)
        return vehicle


def wait_for_valid_position(vehicle):
    logging.info("Waiting for valid position estimate...")
    while True:
        loc = vehicle.location.global_relative_frame
        if loc.lat != 0 and loc.lon != 0:
            break
        time.sleep(1)
    logging.info("Valid position received.")


def get_home_location(vehicle):
    logging.info('Getting home location...')
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            logging.info(" Waiting for home location ...")

    # We have a home location.
    logging.info("Home location found")
    shared.home_lat = vehicle.home_location.lat
    shared.home_lon = vehicle.home_location.lon
    return vehicle.home_location


def heading_updater(vehicle):
    while shared.drone_active:
        with shared.obstacle_lock:
            shared.vehicle_heading = vehicle.heading
        time.sleep(0.5)


def arm_and_takeoff(vehicle, target_altitude):
    """Arms the drone and flies to target altitude."""
    logging.info("[INFO] Checking if vehicle is armable...")
    if not shared.wait_for_mode(vehicle, "GUIDED", timeout=15):
        return []
    logging.info("[INFO] Setting mode to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    if not shared.wait_for_mode(vehicle, "GUIDED", timeout=15):
        return []
    logging.info("[INFO] Arming the vehicle...")
    vehicle.armed = True
    if not shared.wait_for_mode(vehicle, "GUIDED", timeout=15):
        return []
    logging.info(f"[INFO] Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        logging.info(f"  Altitude: {current_alt:.2f}m")
        if current_alt >= target_altitude * 0.95:
            logging.info("[INFO] Target altitude reached.")
            break
        time.sleep(1)


def build_patrol_waypoints(vehicle, relative_waypoints):
    """
    Converts relative patrol coordinates into fixed GPS waypoints.
    """
    gps_waypoints = []
    ref_heading = shared.reference_heading
    start = vehicle.location.global_relative_frame
    start_lat = start.lat
    start_lon = start.lon
    heading_rad = math.radians(ref_heading)
    for forward_m, right_m, altitude in relative_waypoints:
        north = (
            forward_m * math.cos(heading_rad)
            - right_m * math.sin(heading_rad)
        )
        east = (
            forward_m * math.sin(heading_rad)
            + right_m * math.cos(heading_rad)
        )
        dlat = north / 111320.0
        dlon = east / (
            111320.0 * math.cos(math.radians(start_lat))
        )
        lat = start_lat + dlat
        lon = start_lon + dlon
        gps_waypoints.append(
            LocationGlobalRelative(lat, lon, altitude)
        )
    return gps_waypoints


def go_to_waypoints(vehicle, all_wp, pending_wp, completed_wp):
    i = 0
    k = 0

    while i < len(pending_wp):
        # --- Recover from BRAKE if replanning finished ---
        if vehicle.mode.name == "BRAKE" and not shared.replan_required:
            logging.info("[NAV] Returning to GUIDED mode")
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)

        destination = pending_wp[i]

        current = vehicle.location.global_relative_frame

        distance_to_main_wp = get_distance_metres((current.lat, current.lon), (destination.lat, destination.lon))

        #logging.info("[NAV] Using planner")
        temp_list = plan_path(current=current, destination=destination,
            grid=shared.grid, grid_origin=shared.grid_origin, vehicle_heading_deg=shared.filtered_heading)

        # Already at waypoint
        if not temp_list:
            logging.info(f"[WP] Already at waypoint {i + 1}, marking complete.")
            completed_wp.append(destination)
            pending_wp.pop(i)
            continue

        j = 0
        # Execute each sub-waypoint from planner
        # for j, (lat, lon, alt) in enumerate(temp_list):
        while j < len(temp_list):
            lat, lon, alt = temp_list[j]
            target = LocationGlobalRelative(lat, lon, alt)

            start_time = time.time()

            prev_distance = float("inf")

            if vehicle.mode.name != "GUIDED":
                logging.warning("[SAFETY] Refusing simple_goto outside GUIDED mode")
                return

            vehicle.simple_goto(target)
            while shared.drone_active:
                # ----- SAFETY EXIT -----
                if vehicle.mode.name != "GUIDED":
                    logging.warning(f"[SAFETY] Mode changed to {vehicle.mode.name}, Aborting navigation.")
                    return

                current = vehicle.location.global_relative_frame
                distance = get_distance_metres((current.lat, current.lon), (target.lat, target.lon))

                logging.info(f"[WP] Distance to sub-waypoint {j + 1}/{len(temp_list)}: {distance:.2f}m")

                # ---- Emergency stop ----
                with shared.obstacle_lock:
                    if shared.emergency_stop:
                        logging.warning("[OBSTACLE] EMERGENCY STOP triggered! Switching to BRAKE...")
                        vehicle.mode = VehicleMode(shared.EMERGENCY_MODE)
                        shared.wait_for_mode(vehicle, shared.EMERGENCY_MODE)

                        # Replan while hovering
                        temp_list = plan_path(current=current, destination=destination, grid=shared.grid,
                                              grid_origin=shared.grid_origin, vehicle_heading_deg=shared.filtered_heading)

                        if not temp_list:
                            logging.warning("[NAV] Emergency stop replan failed, holding position")
                            vehicle.mode = VehicleMode("BRAKE")
                            return

                        # Back to GUIDED
                        logging.info("[OBSTACLE] Switching back to GUIDED...")
                        vehicle.mode = VehicleMode(shared.RESUME_MODE)
                        shared.wait_for_mode(vehicle, shared.RESUME_MODE)

                        shared.emergency_stop = False
                        j = 0  # restart from beginning of new local plan
                        break

                    # ---- Normal replan ----
                    if shared.replan_required:
                        now = time.time()
                        if now - shared.last_replan_time >= shared.REPLAN_COOLDOWN_S:
                            logging.info("[OBSTACLE] Normal replan triggered... moving to brake mode")
                            shared.last_replan_time = now

                            # Hover temporarily
                            vehicle.mode = VehicleMode("BRAKE")
                            shared.wait_for_mode(vehicle, "BRAKE")

                            #Replanned path
                            new_temp_list = plan_path(current=current, destination=destination, grid=shared.grid,
                                                  grid_origin=shared.grid_origin, vehicle_heading_deg=shared.filtered_heading)
                            time.sleep(2)

                            # reset flags AFTER replan
                            shared.replan_required = False
                            shared.global_obstacle_pos = None
                            shared.global_obstacle_radius = None

                            #No path available
                            if not new_temp_list:
                                logging.warning("[NAV] Replan failed, staying in brake mode")
                                # CRITICAL: stop executing old path
                                shared.replan_required = False
                                temp_List = []
                                return

                            logging.info("[NAV] Replan successful, resuming GUIDED")

                            vehicle.mode = VehicleMode("GUIDED")
                            shared.wait_for_mode(vehicle, "GUIDED")

                            #Replace old path
                            temp_list = new_temp_list

                            #Restart sub-waypoint execution from beginning
                            j = 0
                            break

                # Target reached
                if distance <= 1.2:
                    logging.info(f"[WP] Reached sub-waypoint {j + 1}")
                    j += 1
                    break

                # passed waypoint
                if prev_distance is not None and distance > prev_distance + 0.3:
                    logging.info(f"[WP] Passed sub-waypoint {j + 1}")
                    j += 1
                    break

                # Timeout
                if time.time() - start_time > 12:
                    logging.warning(f"[WP] Timeout on sub-waypoint {j + 1}, moving on.")
                    j += 1
                    break

                prev_distance = distance
                time.sleep(1)

        logging.info(f"[WP] Reached main waypoint {k + 1}")
        completed_wp.append(destination)
        pending_wp.pop(i)
        k += 1


def patrol_loop(vehicle, all_wp, pending_wp):
    completed_wp = []
    shared.grid, shared.grid_origin = initialize_planner(shared.home_lat, shared.home_lon)
    shared.planner_initialized = True

    if shared.reference_heading is None:
        shared.reference_heading = vehicle.heading
        logging.info(f"[PATROL] Reference heading locked at {shared.reference_heading}")

    while shared.drone_active:
        if vehicle.mode.name in ["LAND", "RTL"]:
            logging.info("[PATROL] Exiting patrol loop")
            return

        # Wait if paused
        if shared.drone_paused:
            logging.info("[PATROL] Paused — waiting for RESUME...")
            while shared.drone_paused:
                time.sleep(1)
            logging.info("[PATROL] Resuming patrol...")

        if not shared.all_waypoints:
            logging.info("[PATROL] Waiting for waypoint initialization...")
            time.sleep(1)
            continue

        if not pending_wp:
            if len(all_wp) <= 1:
                logging.info("[PATROL] Single waypoint reached. Holding position.")
                time.sleep(5)
                continue

            logging.info("[PATROL] All waypoints done. Restarting patrol...")
            time.sleep(2)
            pending_wp[:] = all_wp.copy()
            completed_wp.clear()

        go_to_waypoints(vehicle, all_wp, pending_wp, completed_wp)

