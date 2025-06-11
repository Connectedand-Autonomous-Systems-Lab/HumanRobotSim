import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped, PoseArray
from visualization_msgs.msg import Marker
from rclpy.logging import LoggingSeverity
import numpy as np
import cv2
from std_msgs.msg import Header, String
import std_msgs.msg
# import time
import threading
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
import launch_ros.actions
# from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.launch_service import LaunchService
import asyncio
import multiprocessing
from colorama import Fore, Style
import time
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose
from rclpy.duration import Duration
SEVERITY = LoggingSeverity.ERROR


class SensorSubscriber(Node):
    def __init__(self):
        super().__init__("sensor_subscriber")
        self.get_logger().set_level(SEVERITY)
        self.subscriber_ = self.create_subscription(
            LaserScan, "tb3_0/scan", self.scan_listener_callback, 1
        )
        # self.subscriber_ = self.create_subscription(
        #     Odometry, "tb3_0/odom", self.odom_listener_callback, 1
        # )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/tb3_0/pose',  # Replace with your topic name
            self.pose_callback,
            10)
        self.subscriber_ = self.create_subscription(
            PoseArray, "frontiers", self.frontier_callback, 1
        )
        self.latest_position = None
        self.latest_heading = None
        self.latest_scan = None
        self.frontiers = []

    def scan_listener_callback(self, msg):
        self.latest_scan = msg.ranges[:]

    def frontier_callback(self, msg):
        
        for pose in msg.poses:
            self.frontiers.append([pose.position.x, pose.position.y])
        # print(self.frontiers)

    # def odom_listener_callback(self, msg):
    #     self.latest_position = msg.pose.pose.position
    #     self.latest_heading = msg.pose.pose.orientation
    #     self.odom_frame = msg.header.frame_id
    #     self.robot_position = msg

    def pose_callback(self, msg):
        # print("pose callback")
        self.latest_position = msg.pose.pose.position
        self.latest_heading = msg.pose.pose.orientation

    def get_latest_sensor(self):
        # print(self.latest_scan, self.latest_position, self.latest_heading)
        # print(self.latest_position)
        return self.latest_scan, self.latest_position, self.latest_heading


class OdomSubscriber(Node):
    def __init__(self):
        super().__init__("odom_subscriber")
        self.get_logger().set_level(SEVERITY)
        self.subscriber_ = self.create_subscription(
            Odometry, "odom", self.listener_callback, 1
        )
        self.latest_position = None
        self.latest_heading = None

    def listener_callback(self, msg):
        self.latest_position = msg.pose.pose.position
        self.latest_heading = msg.pose.pose.orientation

    def get_latest_odom(self):
        return self.latest_position, self.latest_heading

class ResetWorldClient(Node):
    def __init__(self):
        super().__init__("reset_world_client")
        self.get_logger().set_level(SEVERITY)
        self.publisher_ = self.create_publisher(String, "reset_world", 10)
        # self.timer = self.create_timer(0.1, self.reset_world)

    def reset_world(self):
        print("reset published")
        reset_signal = String()
        reset_signal.data = 'reset'
        self.publisher_.publish(reset_signal)

class PhysicsClient(Node):
    def __init__(self):
        super().__init__("physics_client")
        self.get_logger().set_level(SEVERITY)
        self.publisher_ = self.create_publisher(String, "pause_world", 1)

    def pause_physics(self):
        pause_signal = String()
        pause_signal.data = "pause"
        self.publisher_.publish(pause_signal)

    def unpause_physics(self):
        unpause_signal = String()
        unpause_signal.data = "unpause"
        self.publisher_.publish(unpause_signal)

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__("cmd_vel_publisher")
        self.get_logger().set_level(SEVERITY)
        self.publisher_ = self.create_publisher(Twist, "tb3_0/cmd_vel", 1)
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

    def publish_cmd_vel(self, linear_velocity=0.0, angular_velocity=0.0):
        twist_msg = Twist()
        # Set linear and angular velocities
        twist_msg.linear.x = float(linear_velocity)  # Example linear velocity (m/s)
        twist_msg.angular.z = float(
            angular_velocity
        )  # Example angular velocity (rad/s)
        self.publisher_.publish(twist_msg)


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("marker_publisher")
        self.get_logger().set_level(SEVERITY)
        self.publisher = self.create_publisher(Marker, "visualization_marker", 1)

    def publish(self, x, y):
        marker = Marker()
        marker.header.frame_id = "tb3_0/odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.publisher.publish(marker)
        self.get_logger().info("Publishing Marker")

class SlamHandler:  
    def __init__(self):
        self._process = None
        self._stop_event = None

    def generate_launch_description(self):
        """Create the launch description for Cartographer SLAM."""
        use_sim_time = LaunchConfiguration('use_sim_time', default='true')


        self.slam_toolbox_tb3_0 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('human_robot_pkg'), 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'namespace':'tb3_0'
            }.items()
        )

        self.ros_tcp_connector = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('human_robot_pkg'), 'launch', 'unity_slam_example.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true'
            }.items()
        )

        return LaunchDescription([
            DeclareLaunchArgument('use_sim_time',default_value='true',description='Use simulation (Gazebo) clock if true'),
            self.slam_toolbox_tb3_0
            # self.ros_tcp_connector
        ])

    def _run_process(self, stop_event, launch_description):
        """Runs the launch service asynchronously in a separate process."""
        loop = asyncio.new_event_loop()  # Use a new event loop
        asyncio.set_event_loop(loop)

        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        launch_task = loop.create_task(launch_service.run_async())

        # Wait until stop_event is set
        loop.run_until_complete(loop.run_in_executor(None, stop_event.wait))

        # Shutdown ROS node cleanly
        if not launch_task.done():
            loop.run_until_complete(launch_service.shutdown())
            loop.run_until_complete(launch_task)

    def start(self):
        """Start the ROS 2 launch service in a separate process."""
        if self._process and self._process.is_alive():
            print("SLAM node is already running.")
            return

        print(Fore.BLUE+ "Starting SLAM node..."+Style.RESET_ALL)
        self._stop_event = multiprocessing.Event()
        self._process = multiprocessing.Process(
            target=self._run_process,
            args=(self._stop_event, self.generate_launch_description()),
            daemon=True
        )
        self._process.start()
        time.sleep(1)
        print(Fore.BLUE+ "SLAM node started"+Style.RESET_ALL)

    def stop(self):
        """Stop the ROS 2 launch service."""
        if not self._process or not self._process.is_alive():
            print("SLAM node is not running.")
            return

        print(Fore.BLUE+"Stopping SLAM node..."+Style.RESET_ALL)
        self._stop_event.set()  # Signal the process to stop
        self._process.join()  # Wait for the process to exit
        self._process = None
        self._stop_event = None
        time.sleep(1)
        print(Fore.BLUE+"SLAM node stopped."+Style.RESET_ALL)


def run_scan(args=None):
    rclpy.init()
    reading_laser = ScanSubscriber()
    reading_laser.get_logger().info("Hello friend!")
    rclpy.spin(reading_laser)

    reading_laser.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_scan()
