import sys
import time
from collections import deque

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import Costmap
from nav_msgs.msg  import OccupancyGrid
from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from enum import Enum

import math

###################################################################
# This renders the entire map every time it receives a map update to update the frontier points.
# This is not efficient and should be replaced with a more efficient method.
#####################################################################

MIN_FRONTIER_SIZE = 5
# Parameters that mirror explore_lite cost computation
POTENTIAL_SCALE = 4000.0
GAIN_SCALE = 2.0
DEFAULT_MAX_FRONTIER_DISTANCE = 0  # meters; 0 disables filtering
DEFAULT_MIN_WALL_DISTANCE = 0.5  # meters; 0 disables filtering

namespace = ""

class OccupancyGrid2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx

def _index_to_cells(idx, width):
    return idx % width, idx // width


def _nhood4(idx, width, height):
    """Return 4-connected neighbor indices for a cell index."""
    x, y = _index_to_cells(idx, width)
    neighbors = []
    if x > 0:
        neighbors.append(idx - 1)
    if x + 1 < width:
        neighbors.append(idx + 1)
    if y > 0:
        neighbors.append(idx - width)
    if y + 1 < height:
        neighbors.append(idx + width)
    return neighbors


def _nhood8(idx, width, height):
    """Return 8-connected neighbor indices for a cell index."""
    x, y = _index_to_cells(idx, width)
    neighbors = []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx = x + dx
            ny = y + dy
            if 0 <= nx < width and 0 <= ny < height:
                neighbors.append(ny * width + nx)
    return neighbors


def _nearest_free_cell(start_idx, map_data, width, height):
    """Find the closest free cell to seed the BFS, mirroring Explore Lite."""
    queue = deque([start_idx])
    visited = {start_idx}
    while queue:
        idx = queue.popleft()
        if map_data[idx] == OccupancyGrid2d.CostValues.FreeSpace.value:
            return idx
        for nbr in _nhood4(idx, width, height):
            if nbr not in visited:
                visited.add(nbr)
                queue.append(nbr)
    return start_idx


def _is_new_frontier_cell(idx, map_data, frontier_flag, width, height):
    if map_data[idx] != OccupancyGrid2d.CostValues.NoInformation.value or frontier_flag[idx]:
        return False
    for nbr in _nhood4(idx, width, height):
        if map_data[nbr] == OccupancyGrid2d.CostValues.FreeSpace.value:
            return True
    return False


def _is_near_wall(idx, map_data, width, height, min_clearance_cells):
    if not min_clearance_cells or min_clearance_cells <= 0.0:
        return False
    radius = int(math.ceil(min_clearance_cells))
    x, y = _index_to_cells(idx, width)
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            nx = x + dx
            ny = y + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            if math.hypot(dx, dy) > min_clearance_cells:
                continue
            neighbor_idx = ny * width + nx
            if map_data[neighbor_idx] >= OccupancyGrid2d.CostValues.InscribedInflated.value:
                return True
    return False


def _world_coords(costmap, idx):
    mx, my = _index_to_cells(idx, costmap.getSizeX())
    return costmap.mapToWorld(mx, my)


def _build_new_frontier(initial_idx, costmap, frontier_flag, map_data, width, height, robot_x, robot_y):
    """Flood-fill an entire frontier cluster, compute centroid and robot distance."""
    queue = deque([initial_idx])
    centroid_x = 0.0
    centroid_y = 0.0
    size = 0
    frontier_flag[initial_idx] = True
    min_distance = float('inf')

    while queue:
        idx = queue.popleft()
        wx, wy = _world_coords(costmap, idx)
        centroid_x += wx
        centroid_y += wy
        size += 1

        distance = math.hypot(robot_x - wx, robot_y - wy)
        if distance < min_distance:
            min_distance = distance

        for nbr in _nhood8(idx, width, height):
            if _is_new_frontier_cell(nbr, map_data, frontier_flag, width, height):
                frontier_flag[nbr] = True
                queue.append(nbr)

    return (centroid_x / size, centroid_y / size), size, min_distance


def _frontier_cost(min_distance, size, resolution):
    """Calculate cost using same formulation as frontier_search.cpp."""
    covered_distance = max(min_distance, 0.0)
    covered_size = max(size, 1)
    return (POTENTIAL_SCALE * covered_distance * resolution) - (GAIN_SCALE * covered_size * resolution)


def getFrontier(pose, costmap, logger, max_frontier_distance=None, min_wall_distance=None):
    width = costmap.getSizeX()
    height = costmap.getSizeY()
    total_cells = width * height
    # Copy occupancy data once so repeated indexing stays in C lists
    map_data = list(costmap.map.data)

    try:
        mx, my = costmap.worldToMap(pose.position.x, pose.position.y)
    except Exception as exc:
        logger.warn(f"Robot pose outside map bounds: {exc}")
        return []

    robot_x, robot_y = costmap.mapToWorld(mx, my)
    start_idx = my * width + mx
    clear_idx = _nearest_free_cell(start_idx, map_data, width, height)

    visited_flag = [False] * total_cells
    frontier_flag = [False] * total_cells

    bfs = deque([clear_idx])
    visited_flag[clear_idx] = True

    frontiers = []
    resolution = costmap.map.info.resolution

    min_wall_clearance_cells = (min_wall_distance / resolution) if (min_wall_distance and min_wall_distance > 0.0) else 0.0

    while bfs:
        idx = bfs.popleft()

        for nbr in _nhood4(idx, width, height):
            if not visited_flag[nbr] and map_data[nbr] <= map_data[idx]:
                visited_flag[nbr] = True
                bfs.append(nbr)
            elif _is_new_frontier_cell(nbr, map_data, frontier_flag, width, height):
                wx, wy = _world_coords(costmap, nbr)
                if max_frontier_distance is not None and math.hypot(robot_x - wx, robot_y - wy) > max_frontier_distance:
                    continue
                if min_wall_clearance_cells and _is_near_wall(nbr, map_data, width, height, min_wall_clearance_cells):
                    continue
                frontier_flag[nbr] = True
                centroid, size, min_distance = _build_new_frontier(
                    nbr,
                    costmap,
                    frontier_flag,
                    map_data,
                    width,
                    height,
                    robot_x,
                    robot_y
                )
                if size >= MIN_FRONTIER_SIZE:
                    cost = _frontier_cost(min_distance, size, resolution)
                    frontiers.append((cost, centroid))

    frontiers.sort(key=lambda entry: entry[0])
    return [centroid for _, centroid in frontiers]
     
class FrontierPublisher(Node):
    def __init__(self):
        super().__init__(node_name='frontier_publisher')
        self.costmapSub = self.create_subscription(OccupancyGrid, namespace + '/map', self.occupancyGridCallback, 1)
        self.odom_sub = self.create_subscription(Odometry, namespace + '/odom', self.poseCallback, 1)
        self.frontierPub = self.create_publisher(PoseArray, namespace + '/frontiers', 10)
        self.get_logger().info('Running Waypoint Test')
        self.initial_pose_received = False
        self.initial_map_received = False
        self.declare_parameter('max_frontier_distance', DEFAULT_MAX_FRONTIER_DISTANCE)
        self.max_frontier_distance = float(self.get_parameter('max_frontier_distance').value)
        self.declare_parameter('min_wall_distance', DEFAULT_MIN_WALL_DISTANCE)
        self.min_wall_distance = float(self.get_parameter('min_wall_distance').value)

    def occupancyGridCallback(self, msg):
        # self.get_logger().info("Got map")
        self.costmap = OccupancyGrid2d(msg)
        self.initial_map_received = True
        if self.initial_pose_received and self.initial_map_received:
            distance_limit = self.max_frontier_distance if self.max_frontier_distance > 0.0 else None
            wall_limit = self.min_wall_distance if self.min_wall_distance > 0.0 else None
            frontiers = getFrontier(self.currentPose, self.costmap, self.get_logger(), distance_limit, wall_limit)
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id =   'map'
            for frontier in frontiers:
                pose = Pose()
                pose.position.x = float(frontier[0])  # x coordinate
                pose.position.y = float(frontier[1])  # y coordinate
                pose.position.z = 0.0             # Set z to 0 for 2D points
                pose_array.poses.append(pose)
            self.frontierPub.publish(pose_array)

    def poseCallback(self, msg: Odometry):
        # odom already includes pose with covariance, so use pose.pose
        self.currentPose = msg.pose.pose
        self.initial_pose_received = True
    
    def info_msg(self, msg: str):
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierPublisher()
    
    rclpy.spin(node)    
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
