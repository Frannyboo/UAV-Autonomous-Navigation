# path_planning.py
from dstar_lite import *
import shared
import logging
import time
import heapq
import math

logger = logging.getLogger(__name__)  # Get a logger for this module

FREE = 0
OBSTACLE = 1

resolution = 0.5  # meters/cell
grid_rows = 500
grid_cols = 500

EARTH_RADIUS = 6378137.0  # meters

GEOFENCE_RADIUS_M = 250.0  # adjust to your actual patrol region
EDGE_MARGIN_CELLS = 15


def initialize_planner(home_lat, home_lon):
    grid = [[FREE for _ in range(grid_cols)] for _ in range(grid_rows)]

    grid_origin = {
        "home_lat": home_lat,
        "home_lon": home_lon,

        # home located at center of grid
        "row0": grid_rows // 2,
        "col0": grid_cols // 2,
    }

    return grid, grid_origin


def gps_to_enu(lat, lon, origin_lat, origin_lon):
    lat_r = math.radians(lat)
    lon_r = math.radians(lon)
    lat0_r = math.radians(origin_lat)
    lon0_r = math.radians(origin_lon)

    dlat = lat_r - lat0_r
    dlon = lon_r - lon0_r

    east = dlon * math.cos(lat0_r) * EARTH_RADIUS
    north = dlat * EARTH_RADIUS

    if abs(east) < 0.75:
        east = 0

    if abs(north) < 0.75:
        north = 0

    return east, north


def enu_to_gps(east, north, origin_lat, origin_lon):
    lat0_r = math.radians(origin_lat)

    dlat = north / EARTH_RADIUS
    dlon = east / (EARTH_RADIUS * math.cos(lat0_r))

    lat = origin_lat + math.degrees(dlat)
    lon = origin_lon + math.degrees(dlon)
    return lat, lon


def enu_to_grid(east, north, grid_origin):
    # north positive -> row goes UP (smaller index)
    row = grid_origin["row0"] - int(north / resolution)
    col = grid_origin["col0"] + int(east / resolution)
    return (row, col)


def grid_to_enu(row, col, grid_origin):
    north = (grid_origin["row0"] - row) * resolution
    east = (col - grid_origin["col0"]) * resolution
    return east, north


def gps_to_grid(lat, lon, grid_origin):
    east, north = gps_to_enu(lat, lon, grid_origin["home_lat"], grid_origin["home_lon"])
    logging.info(f"[GRID] ENU to destination -> North={north:.2f}m East={east:.2f}m")
    logging.info(
        f"[GRID DEBUG] "
        f"dLat={lat - grid_origin['home_lat']:.8f}, "
        f"dLon={lon - grid_origin['home_lon']:.8f}, "
        f"North={north:.3f}, "
        f"East={east:.3f}"
    )
    return enu_to_grid(east, north, grid_origin)


def grid_to_gps(cell, grid_origin):
    row, col = cell
    east, north = grid_to_enu(row, col, grid_origin)
    return enu_to_gps(east, north, grid_origin["home_lat"], grid_origin["home_lon"])


def in_bounds(cell):
    r, c = cell
    return 0 <= r < grid_rows and 0 <= c < grid_cols


def calculate_obstacle_cell(drone_cell, heading_deg, direction, distance_m):
    """
    drone_cell: (row, col)
    heading_deg: 0 North, 90 East
    direction: 'F', 'L', 'R'
    distance_m: meters
    """
    if direction == 'F':
        offset_deg = 0
    elif direction == 'L':
        offset_deg = -90
    elif direction == 'R':
        offset_deg = 90
    else:
        raise ValueError("direction must be 'F', 'L', or 'R'")

    total = math.radians(heading_deg + offset_deg)

    # ENU deltas for heading convention 0=N
    east = distance_m * math.sin(total)
    north = distance_m * math.cos(total)

    drow = -int(round(north / resolution))
    dcol = int(round(east / resolution))

    return (drone_cell[0] + drow, drone_cell[1] + dcol)


def clamp_cell(cell):
    r, c = cell
    r = max(0, min(grid_rows - 1, r))
    c = max(0, min(grid_cols - 1, c))
    return (r, c)


def downsample_path(path):
    """
    Adaptive path downsampling.
    Keeps short paths intact.
    Reduces only long straight paths.
    """
    if not path:
        return path
    n = len(path)

    # Short paths -> keep everything
    if n <= 8:
        return path
    # Medium paths
    elif n <= 20:
        step = 2
    # Long paths
    elif n <= 50:
        step = 4
    # Very long paths
    else:
        step = 6

    sampled = path[::step]
    if sampled[-1] != path[-1]:
        sampled.append(path[-1])
    return sampled


def downsample_gps_waypoints(waypoints, min_spacing_m=8.0, max_items=80):
    """
    waypoints: [(lat, lon, alt), ...]
    Ensures waypoints are not too dense.
    """
    if not waypoints:
        return []

    out = [waypoints[0]]
    last = waypoints[0]

    for wp in waypoints[1:]:
        d = haversine_m(last[0], last[1], wp[0], wp[1])
        if d >= min_spacing_m:
            out.append(wp)
            last = wp

    if out[-1] != waypoints[-1]:
        out.append(waypoints[-1])

    if len(out) > max_items:
        step = math.ceil(len(out) / max_items)
        out = out[::step]
        if out[-1] != waypoints[-1]:
            out.append(waypoints[-1])

    return out


def directional_obstacle_cells(obs_cell, start_cell, goal_cell):
    """
    Create directional obstacle expansion based on motion direction.

    If movement is mostly horizontal:
        expand vertically

    If movement is mostly vertical:
        expand horizontally
    """

    r, c = obs_cell

    dr = goal_cell[0] - start_cell[0]
    dc = goal_cell[1] - start_cell[1]

    cells = [obs_cell]

    # Mostly horizontal motion
    if abs(dc) >= abs(dr):

        cells.extend([
            (r - 1, c),
            (r + 1, c),
        ])

    # Mostly vertical motion
    else:
        cells.extend([
            (r, c - 1),
            (r, c + 1),
        ])

    return cells


def reduce_collinear(path):
    """
    Removes unnecessary intermediate cells that lie on a straight line.
    Works for 4-connected paths.
    """
    if not path or len(path) < 3:
        return path

    reduced = [path[0]]
    for i in range(1, len(path) - 1):
        r0, c0 = reduced[-1]
        r1, c1 = path[i]
        r2, c2 = path[i + 1]

        dr1, dc1 = r1 - r0, c1 - c0
        dr2, dc2 = r2 - r1, c2 - c1

        # if direction changes, keep the point
        if (dr1, dc1) != (dr2, dc2):
            reduced.append((r1, c1))

    reduced.append(path[-1])
    return reduced


def make_dynamic_origin(center_lat, center_lon):
    """
    Creates a grid origin such that the given GPS position becomes the grid center.
    """
    return {
        "home_lat": center_lat,
        "home_lon": center_lon,
        "row0": grid_rows // 2,
        "col0": grid_cols // 2,
    }


def near_edge(cell, margin_cells=15):
    """
    True if cell is close to the grid boundary.
    Used to trigger recentering before out-of-bounds occurs.
    """
    r, c = cell
    return (
            r < margin_cells or c < margin_cells or
            r >= grid_rows - margin_cells or
            c >= grid_cols - margin_cells
    )


def recenter_planner(current):
    """
    Recenters grid around current drone position.
    Clears dstar state (because indices change).
    """
    # rebuild grid + origin
    shared.grid = [[FREE for _ in range(grid_cols)] for _ in range(grid_rows)]
    shared.grid_origin = make_dynamic_origin(current.lat, current.lon)

    # reset dstar state (critical)
    shared.dstar = None
    shared.current_goal_cell = None
    shared.current_start_cell = None

    # clear obstacle memory (safest)
    if hasattr(shared, "obstacle_cells"):
        shared.obstacle_cells.clear()
    else:
        shared.obstacle_cells = set()

    logging.warning("[PLANNER] Re-centered planning grid around drone position.")


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def plan_path(current, destination, grid, grid_origin, vehicle_heading_deg=None):
    # 0) Geofence safety check
    if shared.home_lat is not None and shared.home_lon is not None:
        dist_home = haversine_m(shared.home_lat, shared.home_lon, destination.lat, destination.lon)
        if dist_home > GEOFENCE_RADIUS_M:
            logging.error(
                f"[PLANNER] Destination outside geofence ({dist_home:.1f}m > {GEOFENCE_RADIUS_M}m). Rejecting.")
            return []

    # 1) Compute start / goal cells
    start = gps_to_grid(current.lat, current.lon, grid_origin)
    goal = gps_to_grid(destination.lat, destination.lon, grid_origin)

    logging.info(f"START CELL: {start}")
    logging.info(f"GOAL CELL: {goal}")

    # 2) Dynamic recentering policy
    if (not in_bounds(start)) or (not in_bounds(goal)) or near_edge(start, EDGE_MARGIN_CELLS) or near_edge(goal,
                                                                                                           EDGE_MARGIN_CELLS):
        logging.warning("[PLANNER] Start/goal out-of-bounds or near edge -> recentering planner.")
        recenter_planner(current)

        # refresh from shared
        grid = shared.grid
        grid_origin = shared.grid_origin

        start = gps_to_grid(current.lat, current.lon, grid_origin)
        goal = gps_to_grid(destination.lat, destination.lon, grid_origin)

        if not in_bounds(goal):
            logging.error(f"[PLANNER] Goal still outside after recenter: {goal}. Rejecting.")
            # return []

    # 3) Reuse D* Lite planner if possible
    if shared.dstar is None or shared.current_goal_cell != goal:
        shared.dstar = DStarLite(grid, start, goal)
        shared.current_goal_cell = goal
        shared.current_start_cell = start
        logging.info("[PLANNER] Initialized new D* Lite planner.")
    else:
        if shared.current_start_cell != start:
            shared.dstar.move_and_update(start)
            shared.current_start_cell = start

    dstar = shared.dstar

    # 4) Add obstacle if present (with memory + inflation)
    if shared.global_obstacle_pos and shared.global_obstacle_radius:
        drone_cell = start
        heading = vehicle_heading_deg if vehicle_heading_deg is not None else shared.filtered_heading

        obs_cell = calculate_obstacle_cell(
            drone_cell=drone_cell,
            heading_deg=heading,
            direction=shared.global_obstacle_pos,
            distance_m=shared.global_obstacle_radius
        )

        if obs_cell == start:
            logging.warning("[PLANNER] Obstacle overlaps drone cell — projecting obstacle.")

            x, y = start

            heading = heading % 360
            if 45 <= heading < 135:
                forward = (0, 1)  # east
            elif 135 <= heading < 225:
                forward = (1, 0)  # south
            elif 225 <= heading < 315:
                forward = (0, -1)  # west
            else:
                forward = (-1, 0)  # north

            fx, fy = forward
            left = (-fy, fx)
            right = (fy, -fx)

            if shared.global_obstacle_pos == "F":
                obs_cell = (x + 2 * fx, y + 2 * fy)

            elif shared.global_obstacle_pos == "L":
                obs_cell = (x + left[0], y + left[1])

            elif shared.global_obstacle_pos == "R":
                obs_cell = (x + right[0], y + right[1])

        if not in_bounds(obs_cell):
            logging.warning("[PLANNER] Projected obstacle outside grid.")
            return []

        # Clear previous dynamic obstacles
        shared.obstacle_cells.clear()
        logging.info(f"[PLANNER] Obstacle center cell: {obs_cell}")

        # Generate directional obstacle expansion
        expanded_cells = directional_obstacle_cells(
            obs_cell,
            start,
            goal
        )
        for cell in expanded_cells:
            if not in_bounds(cell):
                continue
            shared.obstacle_cells.add(cell)
            grid[cell[0]][cell[1]] = OBSTACLE
            dstar.add_obstacle(cell)

    # CRITICAL: recompute planner after graph changes
    dstar.compute_shortest_path()

    # 5) Extract path
    local_path = dstar.reconstruct_path()
    if not local_path:
        logging.error("[PLANNER] No valid path found!")
        return []

    # Remove starting cell (drone is already there)
    if local_path and local_path[0] == start:
        local_path = local_path[1:]

    if len(local_path) >= 2:
        first = local_path[0]
        second = local_path[1]

        dr = second[0] - first[0]
        dc = second[1] - first[1]

        if abs(dr) <= 1 and abs(dc) <= 1:
            local_path.pop(0)

    # If nothing left, we are already at destination
    if not local_path:
        logging.info("[PLANNER] Already at destination.")
        return []

    logging.info(f"[PLANNER] Raw path cells = {len(local_path)}")

    # 6) Path conditioning (grid)
    if len(local_path) > 25:
        local_path = reduce_collinear(local_path)

    local_path = downsample_path(local_path)

    # 7) Convert to GPS waypoints
    alt = destination.alt if hasattr(destination, "alt") else current.alt

    waypoints = []
    for cell in local_path:
        lat, lon = grid_to_gps(cell, grid_origin)
        waypoints.append((lat, lon, alt))

    # 8) Waypoint conditioning (GPS)
    waypoints = downsample_gps_waypoints(
        waypoints,
        min_spacing_m=5.0,
        max_items=80
    )

    logging.info(f"[PLANNER] Planned cells={len(local_path)} -> GPS waypoints={len(waypoints)}")
    return waypoints
