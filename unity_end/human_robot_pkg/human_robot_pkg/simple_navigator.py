import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from math import sqrt
import tf2_ros
from nav_msgs.msg  import OccupancyGrid

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription_frontiers = self.create_subscription(
            PoseArray,
            '/frontiers',
            self.frontiers_callback,
            qos_profile)
        
        self.create_subscription(
            PoseStamped,
            '/odom',  # or your custom odom topic
            self.odom_callback,
            10
        )

        self.costmapSub = self.create_subscription(
            OccupancyGrid, 
            '/merged_map', 
            self.occupancyGridCallback, 
            1
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.navigator = BasicNavigator()
        self.frontiers = []
        self.robot_pose = None
        self.threshold_dist = 2  # meters
        self.occupancy_grid = None
    
    def occupancyGridCallback(self, msg):
        self.occupancy_grid = msg.data
        self.map_info = msg.info

    def odom_callback(self, msg):
        # Extract position
        self.robot_pose = msg.pose.position

    def frontiers_callback(self, msg):
        self.frontiers = msg.poses

    def is_too_close_to_obstacle(self, frontier):
        # Convert frontier position to map indices
        map_x = int((frontier.position.x - self.map_info.origin.position.x) / self.map_info.resolution)
        map_y = int((frontier.position.y - self.map_info.origin.position.y) / self.map_info.resolution)

        search_radius = int(self.threshold_dist / self.map_info.resolution)

        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                x = map_x + dx
                y = map_y + dy
                index = y * self.map_info.width + x

                # Check bounds
                if 0 <= x < self.map_info.width and 0 <= y < self.map_info.height:
                    # Check for obstacle (assuming occupancy > 50 is considered occupied)
                    if self.occupancy_grid[index] > 50:
                        return True
        return False
    
    def get_closest_frontier(self):
        if self.frontiers is None:
            self.get_logger().info("Waiting: frontiers is None")
            return None

        if self.robot_pose is None:
            self.get_logger().info("Waiting: robot_pose is None")
            return None

        if self.occupancy_grid is None:
            self.get_logger().info("Waiting: occupancy_grid is None")
            return None

        def distance(f):
            return sqrt((f.position.x - self.robot_pose.x) ** 2 + (f.position.y - self.robot_pose.y) ** 2)

        # Filter frontiers too close to obstacles
        valid_frontiers = [
            f for f in self.frontiers
            if not self.is_too_close_to_obstacle(f)
        ]

        if not valid_frontiers:
            return None

        closest_frontier = min(valid_frontiers, key=distance, default=None)
        # closest_frontier = min(self.frontiers, key=distance, default=None)
        return closest_frontier
    
    def navigate_to_frontier(self):
        closest_frontier = self.get_closest_frontier()
        if closest_frontier is None:
            # self.get_logger().info("No frontiers available to explore.")
            return
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = closest_frontier
        
        self.get_logger().info(f"Navigating to frontier: {closest_frontier.position.x}, {closest_frontier.position.y}")
        self.navigator.goToPose(goal_pose)
        
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self)
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        self.get_logger().info("Reached frontier.")
        self.frontiers = [f for f in self.frontiers if f != closest_frontier]

        
def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()
    
    try:
        while rclpy.ok():
            explorer.navigate_to_frontier()
            rclpy.spin_once(explorer)
    except KeyboardInterrupt:
        explorer.get_logger().info("KeyboardInterrupt received.")
    finally:
        if rclpy.ok():  # only destroy if context still active
            explorer.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
