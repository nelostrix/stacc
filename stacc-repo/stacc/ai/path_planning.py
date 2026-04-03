
"""Universal path planning — find collision-free paths for any robot.

Algorithms:
- A* — optimal grid-based search, best for structured environments
- Dijkstra — like A* without heuristic, guarantees shortest path
- RRT* — sampling-based, works in continuous space with arbitrary obstacles
- PRM — Probabilistic Roadmap, good for repeated queries in same environment
- Potential Fields — reactive, real-time obstacle avoidance

All planners share the same interface:
    planner = AStarPlanner(occupancy_grid, resolution=0.05)
    path = planner.plan(start, goal)

Paths are returned as lists of [x, y] or [x, y, z] waypoints that can
be fed directly into WaypointPath for smooth trajectory execution.

Usage:
    from stacc.ai.path_planning import AStarPlanner, RRTStarPlanner
    from stacc.kinematics.trajectory import WaypointPath

    planner = AStarPlanner(grid, resolution=0.05)
    waypoints = planner.plan(start=[0, 0], goal=[5, 3])

    path = WaypointPath(waypoints=waypoints, method="minjerk")
"""
from __future__ import annotations

import heapq
import math
import random
from typing import Any, Callable, Dict, List, Optional, Set, Tuple


# Occupancy grid
class OccupancyGrid:
    """2D occupancy grid for path planning.

     Represents the environment as a grid where each cell is either
     free (0) or occupied (1). Supports inflation (growing obstacles
     by robot radius) and conversion from point clouds.

     Args:
         width: Grid width in cells
         height: Grid height in cells
         resolution: Meters per cell
         origin: [x, y] world position of grid corner (0,0)
     """
    def __init__(
         self,
         width: int  = 100,
         height: int = 100,
         resolution: float = 0.05,
        origin: Optional[List[float]] = None,):

        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin or [0.0, 0.0]
        self.data = [[0] * width for _ in range(height)]

    def is_free(self, gx: int, gy: int) -> bool:
        """check if a grid call is free."""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.data[gx][gy] == 0
        return False

    def is_occupied(self, gx: int, gy: int) -> bool:
        return not self.is_free(gx, gy)

    def set_occupied(self, gx: int, gy: int):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.data[gx][gy] = 1

    def set_free(self, gx: int, gy: int):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.data[gx][gy] = 0

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        gx = int((wx - self.origin[0]) / self.resolution)
        gy = int((wy - self.origin[1]) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (cell center)."""
        wx = self.origin[0] + (gx + 0.5) * self.resolution
        wy = self.origin[1] + (gy + 0.5) * self.resolution
        return wx, wy

    def inflate(self, radius: float) -> "OccupancyGrid":
        """Create inflated copy — grow obstacles by radius (in meters).

        Used to account for robot size during planning.
        """
        cells = max(1, int(radius / self.resolution))
        inflated = OccupancyGrid(self.width, self.height, self.resolution, list(self.origin))

        for y in range(self.height):
            for x in range(self.width):
                if self.data[y][x] == 1:
                    for dy in range(-cells, cells + 1):
                        for dx in range(-cells, cells + 1):
                            if dx * dx + dy * dy <= cells * cells:
                                nx, ny = x + dx, y + dy
                                if 0 <= nx < self.width and 0 <= ny < self.height:
                                    inflated.data[ny][nx] = 1

        return inflated

    def add_rectangle(self, x: float, y: float, w: float, h: float):
        """Add a rectangular obstacle in world coordinates."""
        gx1, gy1 = self.world_to_grid(x, y)
        gx2, gy2 = self.world_to_grid(x + w, y + h)
        for gy in range(min(gy1, gy2), max(gy1, gy2) + 1):
            for gx in range(min(gx1, gx2), max(gx1, gx2) + 1):
                self.set_occupied(gx, gy)

    def add_circle(self, cx: float, cy: float, radius: float):
        """Add a circular obstacle in world coordinates."""
        gcx, gcy = self.world_to_grid(cx, cy)
        gr = int(radius / self.resolution) + 1
        for dy in range(-gr, gr + 1):
            for dx in range(-gr, gr + 1):
                if dx * dx + dy * dy <= gr * gr:
                    self.set_occupied(gcx + dx, gcy + dy)

    def from_point_cloud(self, points: List[List[float]], height_threshold: float = 0.5):
        """Fill grid from a 3D point cloud (project to 2D, filter by height)."""
        for point in points:
            if len(point) >= 3 and point[2] < height_threshold:
                continue  # below threshold, ignore (ground)
            x = point[0] if len(point) >= 1 else 0
            y = point[1] if len(point) >= 2 else 0
            gx, gy = self.world_to_grid(x, y)
            self.set_occupied(gx, gy)


# ══════════════════════════════════════════
# A* PLANNER
# ══════════════════════════════════════════

class AStarPlanner:
    """A* search — optimal grid-based path planning.

    Finds the shortest path on an occupancy grid using A* with
    an admissible heuristic (Euclidean distance).

    Args:
        grid: OccupancyGrid to plan on
        allow_diagonal: Allow 8-connected movement (default True)
        heuristic_weight: Weight for heuristic (1.0 = optimal, >1.0 = faster but suboptimal)
    """

    def __init__(
        self,
        grid: OccupancyGrid,
        allow_diagonal: bool = True,
        heuristic_weight: float = 1.0,
    ):
        self.grid = grid
        self.allow_diagonal = allow_diagonal
        self.heuristic_weight = heuristic_weight

        if allow_diagonal:
            self._neighbors = [
                (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414),
            ]
        else:
            self._neighbors = [
                (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            ]

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        if self.allow_diagonal:
            return self.heuristic_weight * (dx + dy + (1.414 - 2) * min(dx, dy))
        return self.heuristic_weight * (dx + dy)

    def plan(
        self,
        start: List[float],
        goal: List[float],
    ) -> Optional[List[List[float]]]:
        """Plan a path from start to goal (world coordinates).

        Returns:
            List of [x, y] waypoints, or None if no path found.
        """
        sx, sy = self.grid.world_to_grid(start[0], start[1])
        gx, gy = self.grid.world_to_grid(goal[0], goal[1])

        if self.grid.is_occupied(sx, sy):
            return None  # start is in obstacle
        if self.grid.is_occupied(gx, gy):
            return None  # goal is in obstacle

        # Priority queue: (f_cost, counter, (x, y))
        open_set = [(0, 0, (sx, sy))]
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {(sx, sy): 0}
        closed: Set[Tuple[int, int]] = set()
        counter = 0

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == (gx, gy):
                return self._reconstruct(came_from, current)

            if current in closed:
                continue
            closed.add(current)

            cx, cy = current
            for dx, dy, cost in self._neighbors:
                nx, ny = cx + dx, cy + dy

                if not self.grid.is_free(nx, ny):
                    continue
                if (nx, ny) in closed:
                    continue

                tentative_g = g_score[current] + cost

                if tentative_g < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + self._heuristic((nx, ny), (gx, gy))
                    counter += 1
                    heapq.heappush(open_set, (f, counter, (nx, ny)))

        return None  # no path found

    def _reconstruct(self, came_from, current) -> List[List[float]]:
        """Reconstruct path and convert to world coordinates."""
        path_grid = [current]
        while current in came_from:
            current = came_from[current]
            path_grid.append(current)
        path_grid.reverse()

        # Convert to world coordinates
        path_world = []
        for gx, gy in path_grid:
            wx, wy = self.grid.grid_to_world(gx, gy)
            path_world.append([wx, wy])

        return self._simplify(path_world)

    def _simplify(self, path: List[List[float]], tolerance: float = 0.01) -> List[List[float]]:
        """Remove redundant waypoints from straight segments."""
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        for i in range(1, len(path) - 1):
            prev = simplified[-1]
            curr = path[i]
            next_pt = path[i + 1]

            # Check if curr is on the line from prev to next
            dx1 = curr[0] - prev[0]
            dy1 = curr[1] - prev[1]
            dx2 = next_pt[0] - prev[0]
            dy2 = next_pt[1] - prev[1]

            cross = abs(dx1 * dy2 - dy1 * dx2)
            if cross > tolerance:
                simplified.append(curr)

        simplified.append(path[-1])
        return simplified


# ══════════════════════════════════════════
# DIJKSTRA PLANNER
# ══════════════════════════════════════════

class DijkstraPlanner(AStarPlanner):
    """Dijkstra's algorithm — A* with zero heuristic.

    Guarantees shortest path. Slower than A* but explores uniformly.
    """

    def __init__(self, grid: OccupancyGrid, allow_diagonal: bool = True):
        super().__init__(grid, allow_diagonal, heuristic_weight=0.0)

    def _heuristic(self, a, b) -> float:
        return 0.0


# ══════════════════════════════════════════
# RRT* PLANNER
# ══════════════════════════════════════════

class RRTStarPlanner:
    """RRT* — sampling-based path planning in continuous space.

    Works without a grid. Samples random points, connects them
    if collision-free, and rewires for optimality. Good for high-
    dimensional spaces and complex obstacle shapes.

    Args:
        bounds: [[x_min, x_max], [y_min, y_max]] search space
        obstacle_check: Function(x, y) -> bool, returns True if point is in obstacle
        step_size: Maximum distance per tree extension
        max_iterations: Maximum number of samples
        goal_bias: Probability of sampling the goal directly (0-1)
        rewire_radius: Radius for rewiring neighbors
    """

    def __init__(
        self,
        bounds: List[List[float]],
        obstacle_check: Optional[Callable] = None,
        step_size: float = 0.3,
        max_iterations: int = 5000,
        goal_bias: float = 0.1,
        rewire_radius: float = 0.5,
    ):
        self.bounds = bounds
        self.obstacle_check = obstacle_check or (lambda x, y: False)
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.goal_bias = goal_bias
        self.rewire_radius = rewire_radius

    def plan(
        self,
        start: List[float],
        goal: List[float],
        goal_threshold: float = 0.3,
    ) -> Optional[List[List[float]]]:
        """Plan a path from start to goal in continuous space.

        Args:
            start: [x, y] start position
            goal: [x, y] goal position
            goal_threshold: Distance to goal to consider reached

        Returns:
            List of [x, y] waypoints, or None if no path found
        """
        # Tree nodes: {id: (x, y, parent_id, cost)}
        nodes = {0: (start[0], start[1], -1, 0.0)}
        node_count = 1
        goal_node = None

        for iteration in range(self.max_iterations):
            # Sample random point (with goal bias)
            if random.random() < self.goal_bias:
                rx, ry = goal[0], goal[1]
            else:
                rx = random.uniform(self.bounds[0][0], self.bounds[0][1])
                ry = random.uniform(self.bounds[1][0], self.bounds[1][1])

            # Find nearest node
            nearest_id = self._nearest(nodes, rx, ry)
            nx, ny = nodes[nearest_id][0], nodes[nearest_id][1]

            # Steer toward sample
            dx, dy = rx - nx, ry - ny
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 1e-10:
                continue
            if dist > self.step_size:
                dx = dx / dist * self.step_size
                dy = dy / dist * self.step_size

            new_x = nx + dx
            new_y = ny + dy

            # Collision check
            if self.obstacle_check(new_x, new_y):
                continue
            if not self._edge_free(nx, ny, new_x, new_y):
                continue

            # Find nearby nodes for rewiring
            near_ids = self._near(nodes, new_x, new_y, self.rewire_radius)

            # Choose best parent
            best_parent = nearest_id
            best_cost = nodes[nearest_id][3] + math.sqrt(dx * dx + dy * dy)

            for nid in near_ids:
                n = nodes[nid]
                d = math.sqrt((new_x - n[0]) ** 2 + (new_y - n[1]) ** 2)
                cost = n[3] + d
                if cost < best_cost and self._edge_free(n[0], n[1], new_x, new_y):
                    best_parent = nid
                    best_cost = cost

            # Add node
            new_id = node_count
            nodes[new_id] = (new_x, new_y, best_parent, best_cost)
            node_count += 1

            # Rewire neighbors
            for nid in near_ids:
                n = nodes[nid]
                d = math.sqrt((new_x - n[0]) ** 2 + (new_y - n[1]) ** 2)
                new_cost = best_cost + d
                if new_cost < n[3] and self._edge_free(new_x, new_y, n[0], n[1]):
                    nodes[nid] = (n[0], n[1], new_id, new_cost)

            # Check if goal reached
            goal_dist = math.sqrt((new_x - goal[0]) ** 2 + (new_y - goal[1]) ** 2)
            if goal_dist < goal_threshold:
                if goal_node is None or best_cost + goal_dist < nodes.get(goal_node, (0, 0, 0, float('inf')))[3]:
                    goal_node = new_id

        if goal_node is None:
            return None

        # Reconstruct path
        path = []
        current = goal_node
        while current != -1:
            n = nodes[current]
            path.append([n[0], n[1]])
            current = n[2]
        path.reverse()
        path.append(goal)

        return path

    def _nearest(self, nodes, x, y) -> int:
        best_id, best_dist = 0, float('inf')
        for nid, (nx, ny, _, _) in nodes.items():
            d = (x - nx) ** 2 + (y - ny) ** 2
            if d < best_dist:
                best_id, best_dist = nid, d
        return best_id

    def _near(self, nodes, x, y, radius) -> List[int]:
        r2 = radius * radius
        return [nid for nid, (nx, ny, _, _) in nodes.items()
                if (x - nx) ** 2 + (y - ny) ** 2 <= r2]

    def _edge_free(self, x1, y1, x2, y2, steps: int = 10) -> bool:
        """Check if straight line between two points is collision-free."""
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if self.obstacle_check(x, y):
                return False
        return True


# ══════════════════════════════════════════
# RRT* WITH GRID (convenience wrapper)
# ══════════════════════════════════════════

class RRTStarGridPlanner:
    """RRT* that uses an OccupancyGrid for collision checking.

    Convenience wrapper that converts the grid to a collision function.
    """

    def __init__(
        self,
        grid: OccupancyGrid,
        step_size: float = 0.3,
        max_iterations: int = 5000,
        goal_bias: float = 0.1,
    ):
        self.grid = grid
        bounds = [
            [grid.origin[0], grid.origin[0] + grid.width * grid.resolution],
            [grid.origin[1], grid.origin[1] + grid.height * grid.resolution],
        ]

        def obstacle_check(x, y):
            gx, gy = grid.world_to_grid(x, y)
            return grid.is_occupied(gx, gy)

        self._planner = RRTStarPlanner(
            bounds=bounds,
            obstacle_check=obstacle_check,
            step_size=step_size,
            max_iterations=max_iterations,
            goal_bias=goal_bias,
        )

    def plan(self, start: List[float], goal: List[float]) -> Optional[List[List[float]]]:
        return self._planner.plan(start, goal)


# ══════════════════════════════════════════
# POTENTIAL FIELDS
# ══════════════════════════════════════════

class PotentialFieldPlanner:
    """Artificial potential fields — reactive obstacle avoidance.

    Combines attractive force toward goal with repulsive forces
    from obstacles. Real-time, no precomputation needed. Can get
    stuck in local minima (use with A*/RRT* as fallback).

    Args:
        grid: OccupancyGrid for obstacle detection
        attractive_gain: Strength of pull toward goal
        repulsive_gain: Strength of push from obstacles
        repulsive_range: Distance at which obstacles start repelling (meters)
        step_size: Step size per iteration (meters)
        max_iterations: Maximum steps before giving up
    """

    def __init__(
        self,
        grid: OccupancyGrid,
        attractive_gain: float = 1.0,
        repulsive_gain: float = 100.0,
        repulsive_range: float = 0.5,
        step_size: float = 0.05,
        max_iterations: int = 1000,
    ):
        self.grid = grid
        self.k_att = attractive_gain
        self.k_rep = repulsive_gain
        self.d_rep = repulsive_range
        self.step_size = step_size
        self.max_iterations = max_iterations

    def plan(
        self,
        start: List[float],
        goal: List[float],
        goal_threshold: float = 0.1,
    ) -> Optional[List[List[float]]]:
        """Generate path using potential fields."""
        path = [list(start)]
        x, y = start[0], start[1]

        for _ in range(self.max_iterations):
            # Check if reached goal
            dist_to_goal = math.sqrt((x - goal[0]) ** 2 + (y - goal[1]) ** 2)
            if dist_to_goal < goal_threshold:
                path.append(list(goal))
                return path

            # Attractive force (toward goal)
            fx_att = self.k_att * (goal[0] - x)
            fy_att = self.k_att * (goal[1] - y)

            # Repulsive force (away from obstacles)
            fx_rep, fy_rep = 0.0, 0.0
            gx, gy = self.grid.world_to_grid(x, y)
            search_r = int(self.d_rep / self.grid.resolution) + 1

            for dy in range(-search_r, search_r + 1):
                for dx in range(-search_r, search_r + 1):
                    ox, oy = gx + dx, gy + dy
                    if self.grid.is_occupied(ox, oy):
                        owx, owy = self.grid.grid_to_world(ox, oy)
                        dist = math.sqrt((x - owx) ** 2 + (y - owy) ** 2)
                        if 0 < dist < self.d_rep:
                            repulsion = self.k_rep * (1.0 / dist - 1.0 / self.d_rep) / (dist ** 2)
                            fx_rep += repulsion * (x - owx) / dist
                            fy_rep += repulsion * (y - owy) / dist

            # Total force
            fx = fx_att + fx_rep
            fy = fy_att + fy_rep

            # Normalize and step
            mag = math.sqrt(fx * fx + fy * fy)
            if mag < 1e-10:
                break  # stuck in local minimum
            x += (fx / mag) * self.step_size
            y += (fy / mag) * self.step_size

            path.append([x, y])

        return path if len(path) > 1 else None

    def compute_force(self, position: List[float], goal: List[float]) -> List[float]:
        """Compute the force vector at a position (for real-time control).

        Returns [fx, fy] — can be used directly as velocity command.
        """
        x, y = position[0], position[1]

        fx = self.k_att * (goal[0] - x)
        fy = self.k_att * (goal[1] - y)

        gx, gy = self.grid.world_to_grid(x, y)
        search_r = int(self.d_rep / self.grid.resolution) + 1

        for dy in range(-search_r, search_r + 1):
            for dx in range(-search_r, search_r + 1):
                ox, oy = gx + dx, gy + dy
                if self.grid.is_occupied(ox, oy):
                    owx, owy = self.grid.grid_to_world(ox, oy)
                    dist = math.sqrt((x - owx) ** 2 + (y - owy) ** 2)
                    if 0 < dist < self.d_rep:
                        rep = self.k_rep * (1.0 / dist - 1.0 / self.d_rep) / (dist ** 2)
                        fx += rep * (x - owx) / dist
                        fy += rep * (y - owy) / dist

        return [fx, fy]
