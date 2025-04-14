import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler

class FrontierNavigator(Node):
    def __init__(self):
        super().__init__('frontier_navigator')
        # self.subscription = self.create_subscription(
        #     PoseArray,
        #     '/frontiers',
        #     self.frontier_callback,
        #     10)
        self.publisher = self.create_publisher(PoseStamped, '/target_frontier', 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/tb3_0/navigate_to_pose')
        self.hard_goal_pub()

    def frontier_callback(self, msg):
        if not msg.poses:
            self.get_logger().warn("No frontiers received.")
            return
        
        target_pose = random.choice(msg.poses)
        
        self.get_logger().info(f"Selected target frontier at x: {target_pose.position.x}, y: {target_pose.position.y}")
        target_pose_stamped = self.create_target_pose(target_pose)
        
        self.publisher.publish(target_pose_stamped)
        self.send_navigation_goal(target_pose_stamped)

    def create_target_pose(self, target_pose):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'tb3_0/map'
        pose.pose = target_pose
        return pose

    def send_navigation_goal(self, target_pose):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available!")
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        
        self.get_logger().info("Sending navigation goal...")
        self.nav_client.send_goal_async(goal_msg)

    def hard_goal_pub(self):
        pose = Pose()
        pose.position.x = 10.0
        pose.position.y = 10.0
        pose.position.z = 0.0

        # Convert yaw angle to quaternion
        q = quaternion_from_euler(0.0, 0.0, 1.0)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self.get_logger().info(f"Selected target frontier at x: {pose.position.x}, y: {pose.position.y}")
        target_pose_stamped = self.create_target_pose(pose)
        self.publisher.publish(target_pose_stamped)
        self.send_navigation_goal(target_pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    navigator = FrontierNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
