#dstar_lite.py
#Dstar lite path plannikng algorithm code for 4 and 8 connected grids

#4 connected grids
'''
import heapq
import math

FREE = 0
OBSTACLE = 1


# --------- Heuristic ---------
def heuristic(a, b):
    """Manhattan distance between two grid cells"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# --------- Node Class ---------
class Node:
    def __init__(self, pos):
        self.pos = pos
        self.g = float('inf')
        self.rhs = float('inf')
        self.h = 0

    def __lt__(self, other):
        return self.pos < other.pos


# --------- D* Lite Class ---------
class DStarLite:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.km = 0
        self.nodes = {}
        self.open_list = []
        self.last = start

        for r in range(len(grid)):
            for c in range(len(grid[0])):
                self.nodes[(r, c)] = Node((r, c))

        self.nodes[goal].rhs = 0
        heapq.heappush(self.open_list, self.calculate_key(goal))
        self.compute_shortest_path()

    def calculate_key(self, pos):
        node = self.nodes[pos]
        min_g_rhs = min(node.g, node.rhs)
        return min_g_rhs + heuristic(self.start, pos) + self.km, min_g_rhs, pos

    def update_vertex(self, pos):
        node = self.nodes[pos]
        if pos != self.goal:
            min_rhs = float('inf')
            for s in self.get_neighbors(pos):
                cost = 1
                min_rhs = min(min_rhs, self.nodes[s].g + cost)
            node.rhs = min_rhs

        in_open = any(pos == k[2] for k in self.open_list)
        if in_open:
            self.open_list = [k for k in self.open_list if k[2] != pos]
            heapq.heapify(self.open_list)
        if node.g != node.rhs:
            heapq.heappush(self.open_list, self.calculate_key(pos))

    def compute_shortest_path(self, max_iters=200000):
        """
        D* Lite termination:
        while (topKey < key(start)) OR (rhs(start) != g(start))
        """
        iters = 0

        while self.open_list:
            iters += 1
            if iters > max_iters:
                raise RuntimeError("compute_shortest_path exceeded max_iters (likely infinite loop)")

            k_top = self.open_list[0]
            start_key = self.calculate_key(self.start)

            if not (k_top < start_key or self.nodes[self.start].rhs != self.nodes[self.start].g):
                break

            k_old = heapq.heappop(self.open_list)
            u_pos = k_old[2]
            u = self.nodes[u_pos]
            k_new = self.calculate_key(u_pos)

            if k_old < k_new:
                heapq.heappush(self.open_list, k_new)

            elif u.g > u.rhs:
                u.g = u.rhs
                for s in self.get_neighbors(u_pos):
                    self.update_vertex(s)

            else:
                u.g = float('inf')
                self.update_vertex(u_pos)
                for s in self.get_neighbors(u_pos):
                    self.update_vertex(s)


    def get_neighbors(self, pos):
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        result = []
        for dr, dc in dirs:
            nr, nc = pos[0] + dr, pos[1] + dc
            if 0 <= nr < len(self.grid) and 0 <= nc < len(self.grid[0]):
                if self.grid[nr][nc] != OBSTACLE:
                    result.append((nr, nc))
        return result

    def next_step(self):
        min_val = float('inf')
        best = None
        for s in self.get_neighbors(self.start):
            val = self.nodes[s].g + 1
            if val < min_val:
                min_val = val
                best = s
        return best

    def move_and_update(self, new_start):
        self.start = new_start
        self.km += heuristic(self.last, self.start)
        self.last = self.start
        self.compute_shortest_path()

    def add_obstacle(self, pos):
        self.grid[pos[0]][pos[1]] = OBSTACLE
        for s in [pos] + self.get_neighbors(pos):
            self.update_vertex(s)

    def reconstruct_path(self, max_steps=None):
        start = self.start
        goal = self.goal

        path = [start]
        current = start

        limit = max_steps if max_steps is not None else len(self.grid) * len(self.grid[0]) + 10

        for _ in range(limit):
            if current == goal:
                return path

            # choose best neighbor of current
            best = None
            best_val = float('inf')

            for s in self.get_neighbors(current):
                val = self.nodes[s].g + 1
                if val < best_val:
                    best_val = val
                    best = s

            if best is None:
                return None

            # If best neighbor is still inf => no known route
            if self.nodes[best].g == float("inf"):
                return None

            path.append(best)
            current = best

        return None
'''
    

#8 connected grid
import heapq
import math

FREE = 0
OBSTACLE = 1

# --------- Heuristic ---------
def heuristic(a, b):
    """Octile distance is more accurate for 8-connected grids"""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

# --------- Node Class ---------
class Node:
    def __init__(self, pos):
        self.pos = pos
        self.g = float('inf')
        self.rhs = float('inf')
        self.h = 0

    def __lt__(self, other):
        return self.pos < other.pos

# --------- D* Lite Class ---------
class DStarLite:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.km = 0
        self.nodes = {}
        self.open_list = []
        self.last = start

        for r in range(len(grid)):
            for c in range(len(grid[0])):
                self.nodes[(r, c)] = Node((r, c))

        self.nodes[goal].rhs = 0
        heapq.heappush(self.open_list, self.calculate_key(goal))
        self.compute_shortest_path()

    def calculate_key(self, pos):
        node = self.nodes[pos]
        min_g_rhs = min(node.g, node.rhs)
        return (min_g_rhs + heuristic(self.start, pos) + self.km, min_g_rhs, pos)

    def update_vertex(self, pos):
        node = self.nodes[pos]
        if pos != self.goal:
            min_rhs = float('inf')
            for s, cost in self.get_neighbors(pos):
                min_rhs = min(min_rhs, self.nodes[s].g + cost)
            node.rhs = min_rhs

        in_open = any(pos == k[2] for k in self.open_list)
        if in_open:
            self.open_list = [k for k in self.open_list if k[2] != pos]
            heapq.heapify(self.open_list)
        if node.g != node.rhs:
            heapq.heappush(self.open_list, self.calculate_key(pos))

    def compute_shortest_path(self, max_iters=200000):
        """
        D* Lite termination:
        while (topKey < key(start)) OR (rhs(start) != g(start))
        """
        iters = 0

        while self.open_list:
            iters += 1
            if iters > max_iters:
                raise RuntimeError("compute_shortest_path exceeded max_iters (likely infinite loop)")

            k_top = self.open_list[0]
            start_key = self.calculate_key(self.start)

            if not (k_top < start_key or self.nodes[self.start].rhs != self.nodes[self.start].g):
                break

            k_old = heapq.heappop(self.open_list)
            u_pos = k_old[2]
            u = self.nodes[u_pos]
            k_new = self.calculate_key(u_pos)

            if k_old < k_new:
                heapq.heappush(self.open_list, k_new)

            elif u.g > u.rhs:
                u.g = u.rhs
                for s, cost in self.get_neighbors(u_pos):
                    self.update_vertex(s)

            else:
                u.g = float('inf')
                self.update_vertex(u_pos)
                for s, cost in self.get_neighbors(u_pos):
                    self.update_vertex(s)

    def get_neighbors(self, pos):
        directions = [

            (-1, 0), (1, 0), (0, -1), (0, 1),  # N S W E

            (-1, -1), (-1, 1),
            (1, -1), (1, 1)  # diagonals
        ]

        result = []
        for dr, dc in directions:
            nr = pos[0] + dr
            nc = pos[1] + dc
            if not (0 <= nr < len(self.grid) and
                    0 <= nc < len(self.grid[0])):
                continue
            # blocked target cell
            if self.grid[nr][nc] == OBSTACLE:
                continue
            # Prevent diagonal corner cutting
            if dr != 0 and dc != 0:
                adj1_r = pos[0] + dr
                adj1_c = pos[1]

                adj2_r = pos[0]
                adj2_c = pos[1] + dc
                if (
                        self.grid[adj1_r][adj1_c] == OBSTACLE or
                        self.grid[adj2_r][adj2_c] == OBSTACLE
                ):
                    continue
                cost = math.sqrt(2)
            else:
                cost = 1.0
            result.append(((nr, nc), cost))
        return result

    def next_step(self):
        min_val = float('inf')
        best = None
        for s, cost in self.get_neighbors(self.start):
            val = self.nodes[s].g + cost
            if val < min_val:
                min_val = val
                best = s
        return best

    def move_and_update(self, new_start):
        self.start = new_start
        self.km += heuristic(self.last, self.start)
        self.last = self.start
        self.compute_shortest_path()

    def add_obstacle(self, pos):
        self.grid[pos[0]][pos[1]] = OBSTACLE
        self.update_vertex(pos)
        for s, cost in self.get_neighbors(pos):
            self.update_vertex(s)

    def reconstruct_path(self, max_steps=None):
        start = self.start
        goal = self.goal

        path = [start]
        current = start

        limit = max_steps if max_steps is not None else len(self.grid) * len(self.grid[0]) + 10

        for _ in range(limit):
            visited = set()
            if current in visited:
                return None

            visited.add(current)

            if current == goal:
                return path
            # choose best neighbor of current
            best = None
            best_val = float('inf')
            for s, cost in self.get_neighbors(current):
                val = self.nodes[s].g + cost
                if val < best_val:
                    best_val = val
                    best = s
            if best is None:
                return None
            # If best neighbor is still inf => no known route
            if self.nodes[best].g == float("inf"):
                return None
            path.append(best)
            current = best

        return None
