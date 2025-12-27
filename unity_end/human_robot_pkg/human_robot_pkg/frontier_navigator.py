#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose
import time


class FrontierNavigator(Node):

    def __init__(self):
        super().__init__('frontier_navigator')

        self.declare_parameter('goal_refresh_timeout', 0.0)
        self.declare_parameter('blacklist_radius', 0.5)
        # self.goal_refresh_timeout = float(self.get_parameter('goal_refresh_timeout').value)
        self.goal_refresh_timeout = 3.0
        self.last_goal_time = None
        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self.latest_frontiers = PoseArray()

        self._blacklisted_frontiers = []
        self._current_goal = None

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

        self.tick_timer = self.create_timer(1.0, self.navigate_to_frontier)

        # Nav2 NavigateToPose action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Track whether Nav2 currently has an active goal
        self.navigating = False

        self.get_logger().info('FrontierNavigator node started.')

    def navigate_to_frontier(self):
        if not self.latest_frontiers.poses:
            self.get_logger().info('No frontiers available to navigate to.')
            return
        elif self.navigating:
            self.get_logger().debug('Already navigating to a frontier; ignoring new frontiers.')
            return

        if self.navigating and self.goal_refresh_timeout > 0.0 and self.last_goal_time is not None:
            elapsed = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
            if elapsed < self.goal_refresh_timeout:
                self.get_logger().debug('Ignoring new frontiers until refresh timeout elapses.')
                return
            self.get_logger().warn('Refresh timeout hit, preempting current goal for new frontier.')
        elif self.navigating:
            self.get_logger().info('New frontier set received; preempting current goal.')

        # frontier_publisher orders poses by ascending cost, so choose the first allowed frontier
        selected_pose = None
        for pose in self.latest_frontiers.poses:
            if not self._is_blacklisted(pose):
                selected_pose = pose
                break

        if selected_pose is None:
            self.get_logger().warn('All received frontiers are currently blacklisted.')
            return

        self.get_logger().debug('Selected cheapest non-blacklisted frontier as goal.')

        # Create PoseStamped from selected pose
        selected_ps = PoseStamped()
        selected_ps.header = self.latest_frontiers.header      # use same frame as PoseArray (e.g. "map")
        selected_ps.pose = selected_pose

        # Publish selected frontier
        self.selected_frontier_pub.publish(selected_ps)
        self.get_logger().info('Published selected frontier to /selected_frontier.')

        # Send navigation goal
        self.send_navigation_goal(selected_ps)

    def frontier_callback(self, msg: PoseArray):
        """Called whenever a new set of frontiers (PoseArray) is received."""
        if not msg.poses:
            self.get_logger().warn('Received empty frontier list.')
            return
        self.latest_frontiers = msg
        

    def send_navigation_goal(self, goal_pose: PoseStamped):
        """Send a NavigateToPose goal to Nav2."""
        # if self.navigating:
        #     return
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
        self._current_goal = goal_pose

    def goal_response_callback(self, future):
        """Called when the action server accepts or rejects the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal was rejected by server.')
            self.navigating = False
            return

        self.navigating = True
        self.last_goal_time = self.get_clock().now()
        # self.get_logger().info('Navigation goal accepted.')
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
        status_str_list = [ 'UNKNOWN', 'ACCEPTED', 'EXECUTING', 'CANCELING', 'SUCCEEDED', 'CANCELLED', 'ABORTED' ]
        self.get_logger().info(f'Navigation finished with status={status_str_list[status]}')
        self.get_logger().info(f'_____________________________________________________________')
        if status == GoalStatus.STATUS_ABORTED or status == GoalStatus.STATUS_SUCCEEDED:
            self._blacklist_current_goal()
        self.navigating = False
        self._current_goal = None
        self.navigate_to_frontier()

    def _is_blacklisted(self, pose):
        """Return True if pose aligns with any blacklisted frontier."""
        px = pose.position.x
        py = pose.position.y
        for bx, by in self._blacklisted_frontiers:
            if math.hypot(px - bx, py - by) <= self.blacklist_radius:
                return True
        return False

    def _blacklist_current_goal(self):
        if self._current_goal is None:
            return
        px = self._current_goal.pose.position.x
        py = self._current_goal.pose.position.y
        self._blacklisted_frontiers.append((px, py))
        self.get_logger().warn(
            f'Blacklisting frontier at ({px:.2f}, {py:.2f}) because its goal was aborted/succeeded.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
