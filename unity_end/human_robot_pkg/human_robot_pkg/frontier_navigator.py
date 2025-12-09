#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose


class FrontierNavigator(Node):

    def __init__(self):
        super().__init__('frontier_navigator')

        self.declare_parameter('goal_refresh_timeout', 0.0)
        # self.goal_refresh_timeout = float(self.get_parameter('goal_refresh_timeout').value)
        self.goal_refresh_timeout = 3.0
        self.last_goal_time = None

        # Subscribe to frontier PoseArray
        self.frontier_sub = self.create_subscription(
            PoseArray,
            '/frontiers',          # change if your topic name is different
            self.frontier_callback,
            10
        )

        # Publish the selected frontier as PoseStamped
        self.selected_frontier_pub = self.create_publisher(
            PoseStamped,
            '/selected_frontier',
            10
        )

        # Nav2 NavigateToPose action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Track whether Nav2 currently has an active goal
        self.navigating = False

        self.get_logger().info('FrontierNavigator node started.')

    def frontier_callback(self, msg: PoseArray):
        """Called whenever a new set of frontiers (PoseArray) is received."""
        if not msg.poses:
            self.get_logger().warn('Received empty frontier list.')
            return

        if self.navigating and self.goal_refresh_timeout > 0.0 and self.last_goal_time is not None:
            elapsed = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
            if elapsed < self.goal_refresh_timeout:
                self.get_logger().debug('Ignoring new frontiers until refresh timeout elapses.')
                return
            self.get_logger().info('Refresh timeout hit, preempting current goal for new frontier.')
        elif self.navigating:
            self.get_logger().info('New frontier set received; preempting current goal.')

        # frontier_publisher orders poses by ascending cost, so index 0 is cheapest
        selected_pose = msg.poses[0]
        self.get_logger().info('Selected cheapest frontier as goal.')

        # Create PoseStamped from selected pose
        selected_ps = PoseStamped()
        selected_ps.header = msg.header      # use same frame as PoseArray (e.g. "map")
        selected_ps.pose = selected_pose

        # Publish selected frontier
        self.selected_frontier_pub.publish(selected_ps)
        self.get_logger().info('Published selected frontier to /selected_frontier.')

        # Send navigation goal
        self.send_navigation_goal(selected_ps)

    def send_navigation_goal(self, goal_pose: PoseStamped):
        """Send a NavigateToPose goal to Nav2."""
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self.nav_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        goal_msg.behavior_tree = ''  # use default BT

        self.get_logger().info(
            f'Sending navigation goal to x={goal_pose.pose.position.x:.2f}, '
            f'y={goal_pose.pose.position.y:.2f}'
        )

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when the action server accepts or rejects the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal was rejected by server.')
            self.navigating = False
            return

        self.navigating = True
        self.last_goal_time = self.get_clock().now()
        self.get_logger().info('Navigation goal accepted.')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Logs navigation feedback (no 'status' field in NavigateToPose feedback)."""
        feedback = feedback_msg.feedback

        dist = feedback.distance_remaining
        nav_time = feedback.navigation_time
        recovs = feedback.number_of_recoveries

        self.get_logger().debug(
            f'[Nav2 feedback] distance_remaining={dist:.2f} m, '
            f'navigation_time={nav_time.sec}.{int(nav_time.nanosec/1e8)} s, '
            f'recoveries={recovs}'
        )

    def result_callback(self, future):
        """Logs the result when navigation finishes."""
        result_wrapper = future.result()

        # result_wrapper.result is std_msgs/Empty in your Humble Nav2
        status = result_wrapper.status  # action_msgs/GoalStatus enum

        self.get_logger().info(f'Navigation finished with status={status}')
        self.navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
