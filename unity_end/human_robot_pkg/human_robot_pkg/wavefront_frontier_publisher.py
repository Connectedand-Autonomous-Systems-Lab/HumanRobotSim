#!/usr/bin/env python3
import math
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Deque
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray


# ----------------------------
# Frontier data structure
# ----------------------------
@dataclass
class Frontier:
    points: List[Point] = field(default_factory=list)  # all frontier cells in world coords
    size: int = 0
    initial: Point = field(default_factory=Point)      # first detected contact point
    centroid: Point = field(default_factory=Point)     # arithmetic mean of points
    middle: Point = field(default_factory=Point)       # closest frontier point to robot
    min_distance: float = math.inf
    cost: float = math.inf


# ----------------------------
# Core Frontier Search (WFD)
# ----------------------------
class FrontierSearch:
    """
    OccupancyGrid-based reimplementation of explore_lite’s frontier search
    """

    # We emulate Nav2 cost semantics for the explore_lite logic
    FREE_SPACE = 0
    LETHAL_OBSTACLE = 254
    NO_INFORMATION = 255

    def __init__(self,
                 potential_scale: float,
                 gain_scale: float,
                 min_frontier_size_m: float,
                 min_wall_distance_m: float):
        self.potential_scale = potential_scale
        self.gain_scale = gain_scale
        self.min_frontier_size_m = min_frontier_size_m
        self.min_wall_distance_m = min_wall_distance_m

        # Map cache
        self._grid: Optional[OccupancyGrid] = None
        self._map_costs: Optional[List[int]] = None  # cost-style array [0..255]
        self._size_x: int = 0
        self._size_y: int = 0
        self._res: float = 0.0
        self._origin_x: float = 0.0
        self._origin_y: float = 0.0

    # ---------- Map helpers ----------
    def set_map(self, grid: OccupancyGrid) -> None:
        self._grid = grid
        self._size_x = grid.info.width
        self._size_y = grid.info.height
        self._res = float(grid.info.resolution)
        self._origin_x = float(grid.info.origin.position.x)
        self._origin_y = float(grid.info.origin.position.y)

        # Convert OccupancyGrid values (-1, 0..100) into cost-style values
        # to mimic explore_lite style comparisons.
        # - Unknown (-1) -> 255 (NO_INFORMATION)
        # - Occupied (>=65 by convention) -> 254 (LETHAL_OBSTACLE)
        # - Otherwise -> 0 (FREE_SPACE)
        # You can tune the occupied threshold if your map uses different conventions.
        occ_thresh = 65
        costs = []
        for v in grid.data:
            if v < 0:
                costs.append(self.NO_INFORMATION)
            elif v >= occ_thresh:
                costs.append(self.LETHAL_OBSTACLE)
            else:
                costs.append(self.FREE_SPACE)
        self._map_costs = costs

    def world_to_map(self, wx: float, wy: float) -> Optional[Tuple[int, int]]:
        if self._grid is None:
            return None
        mx = int(math.floor((wx - self._origin_x) / self._res))
        my = int(math.floor((wy - self._origin_y) / self._res))
        if 0 <= mx < self._size_x and 0 <= my < self._size_y:
            return mx, my
        return None

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        # Center of cell (mx+0.5, my+0.5)
        wx = self._origin_x + (mx + 0.5) * self._res
        wy = self._origin_y + (my + 0.5) * self._res
        return wx, wy

    def index(self, mx: int, my: int) -> int:
        return my * self._size_x + mx

    def index_to_cells(self, idx: int) -> Tuple[int, int]:
        mx = idx % self._size_x
        my = idx // self._size_x
        return mx, my

    def nhood4(self, idx: int) -> List[int]:
        mx, my = self.index_to_cells(idx)
        nbrs = []
        if mx > 0:
            nbrs.append(self.index(mx - 1, my))
        if mx + 1 < self._size_x:
            nbrs.append(self.index(mx + 1, my))
        if my > 0:
            nbrs.append(self.index(mx, my - 1))
        if my + 1 < self._size_y:
            nbrs.append(self.index(mx, my + 1))
        return nbrs

    def nhood8(self, idx: int) -> List[int]:
        mx, my = self.index_to_cells(idx)
        nbrs = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = mx + dx
                ny = my + dy
                if 0 <= nx < self._size_x and 0 <= ny < self._size_y:
                    nbrs.append(self.index(nx, ny))
        return nbrs

    # ---------- Explore-lite frontier logic ----------
    def is_new_frontier_cell(self, idx: int, frontier_flag: List[bool]) -> bool:
        # check that cell is unknown and not already marked as frontier
        if self._map_costs[idx] != self.NO_INFORMATION or frontier_flag[idx]:
            return False

        # frontier cells should have at least one 4-connected neighbour that is free
        for nbr in self.nhood4(idx):
            if self._map_costs[nbr] == self.FREE_SPACE:
                return True
        return False

    def nearest_cell(self, start_idx: int, desired_cost: int, max_radius_cells: int = 50) -> Optional[int]:
        """
        Find nearest cell with given desired_cost using BFS expanding in rings.
        Similar spirit to explore_lite's nearestCell(...) helper.
        max_radius_cells limits runtime if robot is far from free space.
        """
        if self._map_costs is None:
            return None

        visited = [False] * (self._size_x * self._size_y)
        q: Deque[Tuple[int, int]] = deque()
        q.append((start_idx, 0))
        visited[start_idx] = True

        while q:
            idx, dist = q.popleft()
            if self._map_costs[idx] == desired_cost:
                return idx
            if dist >= max_radius_cells:
                continue
            for nbr in self.nhood8(idx):
                if not visited[nbr]:
                    visited[nbr] = True
                    q.append((nbr, dist + 1))
        return None

    def build_new_frontier(self, initial_cell: int, reference_idx: int, frontier_flag: List[bool]) -> Frontier:
        out = Frontier()
        out.size = 1
        out.min_distance = math.inf
        out.centroid.x = 0.0
        out.centroid.y = 0.0

        # record initial contact point
        ix, iy = self.index_to_cells(initial_cell)
        wx, wy = self.map_to_world(ix, iy)
        out.initial.x = wx
        out.initial.y = wy

        # reference (robot start) in world coords
        rx, ry = self.index_to_cells(reference_idx)
        ref_x, ref_y = self.map_to_world(rx, ry)

        bfs: Deque[int] = deque()
        bfs.append(initial_cell)

        while bfs:
            idx = bfs.popleft()

            # add cells in 8-connected neighborhood to frontier cluster
            for nbr in self.nhood8(idx):
                if self.is_new_frontier_cell(nbr, frontier_flag):
                    frontier_flag[nbr] = True

                    mx, my = self.index_to_cells(nbr)
                    wx, wy = self.map_to_world(mx, my)

                    p = Point()
                    p.x = wx
                    p.y = wy
                    p.z = 0.0
                    out.points.append(p)

                    out.size += 1
                    out.centroid.x += wx
                    out.centroid.y += wy

                    # closest point to robot (min_distance) becomes "middle"
                    d = math.sqrt((ref_x - wx) ** 2 + (ref_y - wy) ** 2)
                    if d < out.min_distance:
                        out.min_distance = d
                        out.middle.x = wx
                        out.middle.y = wy
                        out.middle.z = 0.0

                    bfs.append(nbr)

        # average centroid (note: explore_lite sums include all points it adds;
        # we also should include initial cell in centroid sum for exact parity.
        # Here, initial cell is not in out.points yet; we can include it now.)
        # Include initial cell into centroid calculation for better match:
        out.centroid.x += out.initial.x
        out.centroid.y += out.initial.y
        out.centroid.x /= float(out.size)
        out.centroid.y /= float(out.size)
        out.centroid.z = 0.0

        # If out.points didn’t include initial cell, add it for completeness
        init_point = Point()
        init_point.x = out.initial.x
        init_point.y = out.initial.y
        init_point.z = 0.0
        out.points.insert(0, init_point)

        # If min_distance never updated (shouldn’t happen), set it from initial
        if not math.isfinite(out.min_distance):
            out.min_distance = math.sqrt((ref_x - out.initial.x) ** 2 + (ref_y - out.initial.y) ** 2)
            out.middle.x = out.initial.x
            out.middle.y = out.initial.y
            out.middle.z = 0.0

        return out

    def frontier_cost(self, f: Frontier) -> float:
        # Same structure as explore_lite:
        # cost = potential_scale * min_distance*res - gain_scale * size*res
        return (self.potential_scale * f.min_distance) - (self.gain_scale * f.size * self._res)

    def search_from(self, robot_world: Point) -> List[Frontier]:
        if self._grid is None or self._map_costs is None:
            return []

        # Sanity check robot inside map
        mxy = self.world_to_map(robot_world.x, robot_world.y)
        if mxy is None:
            return []
        mx, my = mxy
        pos_idx = self.index(mx, my)

        frontier_list: List[Frontier] = []

        # visited & frontier flags
        frontier_flag = [False] * (self._size_x * self._size_y)
        visited_flag = [False] * (self._size_x * self._size_y)

        # Initialize BFS: find closest clear cell (FREE_SPACE)
        bfs: Deque[int] = deque()
        clear = self.nearest_cell(pos_idx, self.FREE_SPACE)
        if clear is not None:
            bfs.append(clear)
        else:
            bfs.append(pos_idx)

        visited_flag[bfs[0]] = True

        # Global BFS over reachable-ish space (4-connected), detect frontiers on edges
        while bfs:
            idx = bfs.popleft()

            for nbr in self.nhood4(idx):
                # Similar to explore_lite's "descending" condition:
                # allow stepping to equal/lower cost (toward free space),
                # and avoid revisiting.
                if (self._map_costs[nbr] <= self._map_costs[idx]) and (not visited_flag[nbr]):
                    visited_flag[nbr] = True
                    bfs.append(nbr)
                elif self.is_new_frontier_cell(nbr, frontier_flag):
                    frontier_flag[nbr] = True
                    nf = self.build_new_frontier(nbr, pos_idx, frontier_flag)

                    # Filter by physical size in meters (size * resolution)
                    if nf.size * self._res >= self.min_frontier_size_m and self._has_wall_clearance(nf):
                        frontier_list.append(nf)

        # Score & sort
        for f in frontier_list:
            f.cost = self.frontier_cost(f)

        frontier_list.sort(key=lambda x: x.cost)
        return frontier_list

    def _has_wall_clearance(self, frontier: Frontier) -> bool:
        """Return True if the frontier's middle point is at least min_wall_distance_m from obstacles."""
        if self._map_costs is None or self._grid is None:
            return True

        mxy = self.world_to_map(frontier.middle.x, frontier.middle.y)
        if mxy is None:
            return False

        mx, my = mxy
        radius_cells = max(1, int(math.ceil(self.min_wall_distance_m / self._res)))

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                nx = mx + dx
                ny = my + dy
                if 0 <= nx < self._size_x and 0 <= ny < self._size_y:
                    dist = math.hypot(dx * self._res, dy * self._res)
                    if dist <= self.min_wall_distance_m:
                        idx = self.index(nx, ny)
                        if self._map_costs[idx] == self.LETHAL_OBSTACLE:
                            return False
        return True


# ----------------------------
# ROS2 Node
# ----------------------------
class FrontierDetectorNode(Node):
    def __init__(self):
        super().__init__('frontier_detector')

        # Params
        self.declare_parameter('map_topic', '/merged_map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frontiers_topic', '/frontiers')
        self.declare_parameter('markers_topic', '/frontier_markers')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('potential_scale', 50.0)
        self.declare_parameter('gain_scale', 0.01)
        self.declare_parameter('min_frontier_size_m', 0.1)
        self.declare_parameter('min_wall_distance_m', 0.3)
        self.declare_parameter('frame_id', 'map')

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.frontiers_topic = self.get_parameter('frontiers_topic').get_parameter_value().string_value
        self.markers_topic = self.get_parameter('markers_topic').get_parameter_value().string_value

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        potential_scale = float(self.get_parameter('potential_scale').value)
        gain_scale = float(self.get_parameter('gain_scale').value)
        min_frontier_size_m = float(self.get_parameter('min_frontier_size_m').value)
        min_wall_distance_m = float(self.get_parameter('min_wall_distance_m').value)

        self.searcher = FrontierSearch(
            potential_scale=potential_scale,
            gain_scale=gain_scale,
            min_frontier_size_m=min_frontier_size_m,
            min_wall_distance_m=min_wall_distance_m
        )

        # Shared state
        self._lock = threading.Lock()
        self._latest_map: Optional[OccupancyGrid] = None
        self._robot_point: Optional[Point] = None
        self._map_received = False
        self._odom_received = False

        # Subs
        # Keep explicit references to subscriptions so they are not garbage-collected
        self._map_sub = self.create_subscription(OccupancyGrid, map_topic, self._on_map, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 50)

        # Pubs
        self.frontiers_pub = self.create_publisher(PoseArray, self.frontiers_topic, 10)
        self.markers_pub = self.create_publisher(MarkerArray, self.markers_topic, 10)

        # Timer
        period = 1.0 / max(0.1, publish_rate)
        # Drive timer off a steady (wall) clock, so it still fires if use_sim_time is true but /clock is absent
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        # Keep a reference to the timer so Python's GC does not drop it
        self._timer = self.create_timer(period, self._tick, clock=self._steady_clock)

        self.get_logger().info(f"FrontierDetectorNode listening to {map_topic} and {odom_topic}")
        self.get_logger().info(f"Publishing PoseArray on {self.frontiers_topic} and MarkerArray on {self.markers_topic}")

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._latest_map = msg
            self._map_received = True
            # Update searcher cache immediately
            self.searcher.set_map(msg)

    def _on_odom(self, msg: Odometry) -> None:
        p = Point()
        p.x = float(msg.pose.pose.position.x)
        p.y = float(msg.pose.pose.position.y)
        p.z = 0.0
        with self._lock:
            self._robot_point = p
            self._odom_received = True

    def _tick(self) -> None:
        with self._lock:
            if not self._map_received:
                self.get_logger().warn("Tick: no map received yet")
                return
            if not self._odom_received:
                self.get_logger().warn("Tick: no odom received yet")
                return
            if self._latest_map is None or self._robot_point is None:
                self.get_logger().warn("Tick: map or robot point is None")
                return
            robot = self._robot_point

        frontiers = self.searcher.search_from(robot)

        # Publish PoseArray of "middle" points (closest-to-robot per cluster)
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = self.frame_id

        for f in frontiers:
            pose = Pose()
            pose.position.x = f.middle.x
            pose.position.y = f.middle.y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pa.poses.append(pose)

        self.frontiers_pub.publish(pa)

        # Publish markers (centroid spheres + middle spheres)
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # centroid markers
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.frame_id
            m.ns = "frontier_centroids"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = f.centroid
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            # leave color default; RViz will show it if you set it or via topic tools
            m.color.a = 1.0
            ma.markers.append(m)

        # middle markers
        base = 100000
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.frame_id
            m.ns = "frontier_middles"
            m.id = base + i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = f.middle
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.a = 1.0
            ma.markers.append(m)

        self.markers_pub.publish(ma)
        self.get_logger().info(f"Published {len(frontiers)} frontiers")


def main():
    rclpy.init()
    node = FrontierDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
