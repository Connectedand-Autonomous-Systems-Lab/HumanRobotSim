import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class PointGoalNavigator(Node):
    def __init__(self):
        super().__init__('point_goal_navigator')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.point_sub = self.create_subscription(
            PointStamped,
            '/target_point',
            self.point_callback,
            qos_profile
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/odom',
            self.odom_callback,
            10
        )

        self.navigator = BasicNavigator()
        self.active_goal = False
        self.goal_in_progress = False  # Track if goal is being processed
        self.get_logger().info("Navigator is ready. Listening to /target_point...")
        self.point = None
        self.current_orientation = None
        self.reset_on_new_goal = True

    def point_callback(self, msg: PointStamped):
        self.get_logger().info("Received new target point.")
        self.point = msg.point
        
        if self.reset_on_new_goal:
            # If a goal is already being processed, cancel it
            if self.goal_in_progress:
                self.get_logger().info("Cancelling previous goal...")
                self.navigator.cancelTask()
                self.goal_in_progress = False

            # Immediately navigate to new point
            self.navigate_to_point()

    def odom_callback(self, msg: PoseStamped):
        self.current_orientation = msg.pose.orientation

    def navigate_to_point(self):
        if self.goal_in_progress:
            return  # Do not process if a goal is already in progress
        
        if self.point is None:
            self.get_logger().info("No point set. Waiting for a target point...")
            return

        if self.current_orientation is None:
            self.get_logger().warn("Current orientation unknown. Waiting for odometry...")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = self.point
        goal_pose.pose.orientation = self.current_orientation

        self.navigator.goToPose(goal_pose)
        self.goal_in_progress = True  # Mark that a goal is now active

    def spin_once(self):
        """Handle navigator task progress."""
        if self.goal_in_progress and self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
            elif result == TaskResult.FAILED:
                self.get_logger().info('Goal failed!')
                self.navigate_to_point()
            self.goal_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = PointGoalNavigator()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.navigate_to_point()
            
            node.spin_once()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PointGoalNavigator.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
