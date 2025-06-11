import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped
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
            OccupancyGrid, "tb3_0/map", self.map_listener_callback, 1
        )

        # self.publisher = self.create_publisher(std_msgs.msg.Empty , '/reset_time', 10)
        self.latest_position = None
        self.latest_heading = None
        self.latest_scan = None
        self.latest_map = None
        self.canvas_size_x = 512
        self.canvas_size_y = 512
        self.map_value = 0
        self.map_resolution = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.timer = self.create_timer(0.05, self.lookup_transform)
        self.transform = None

    def scan_listener_callback(self, msg):
        self.latest_scan = msg.ranges[:]

    def odom_listener_callback(self, msg):
        self.latest_position = msg.pose.pose.position
        self.latest_heading = msg.pose.pose.orientation
        self.odom_frame = msg.header.frame_id
        self.robot_position = msg
    
    def pose_callback(self, msg):
        self.latest_position = msg.pose.pose.position
        # self.get_logger().info(f'Received Pose: x={x:.3f}, y={y:.3f}')

    def map_listener_callback(self, msg):
        self.latest_map = msg

        # only getting this for the first time
        if self.map_resolution == None:
            self.map_resolution = msg.info.resolution
            self.map_frame = msg.header.frame_id

    def get_latest_sensor(self, is_transform_available):
        # print(self.latest_scan, self.latest_position, self.latest_heading)
        if self.latest_map == None:
            latest_map = np.zeros((self.canvas_size_x, self.canvas_size_y), dtype=np.uint8)
            return self.latest_scan, self.latest_position, self.latest_heading, self.get_map_free_pixels()
        # while self.latest_map == None:
        #     continue

        return self.latest_scan, self.latest_position, self.latest_heading, self.get_map_free_pixels()

    def get_map_free_pixels(self):
        self.previous_map_value = self.map_value
        self.map_value=0
        if self.latest_map == None:
            return 0
        for data in self.latest_map.data:
            if data == -1: # unknown areas shouldnt be calculated. Otherwise the map value will go to negative
                continue
            else:
                self.map_value += data
        return self.map_value

    def lookup_transform(self):
        try:
            self.transform = self.tf_buffer.lookup_transform(
                'map', 'odom',
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            )
            return True
        except TransformException as e:
            print(Fore.RED + f'Could not get transform: {e}' + Style.RESET_ALL)
            return False

    def reset_time(self):
        self.publisher.publish(Empty())

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__("scan_subscriber")
        self.get_logger().set_level(SEVERITY)
        self.subscriber_ = self.create_subscription(
            LaserScan, "scan", self.listener_callback, 1
        )
        self.latest_scan = None

    def listener_callback(self, msg):
        self.latest_scan = msg.ranges[:]

    def get_latest_scan(self):
        return self.latest_scan

class MapSubscriber(Node):
    def __init__(self):
        super().__init__("map_subscriber")
        self.subscriber_ = self.create_subscription(
            OccupancyGrid, "map", self.listener_callback, 10
        )
        self.latest_map = None

    def listener_callback(self, msg):
        # self.latest_map = msg.ranges[:]
        print("map")
        self.latest_map = msg

    def get_map_free_pixels(self):
        free_num=0
        if self.latest_map == None:
            return 0
        for data in self.latest_map.data:
            if data == 0:
                free_num += 1
        return free_num

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
        marker.header.frame_id = "odom"
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

class EmptyMapPublisher(Node):
    def __init__(self):
        super().__init__('empty_map_publisher')
        
        # Create a publisher for the OccupancyGrid
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)

        # Log info
        # self.get_logger().info("Empty Map Publisher Node Started. Publishing empty map...")

        # Publish an empty map immediately upon starting
        # self.publish_empty_map()

    def publish_empty_map(self):
        """Publishes an empty OccupancyGrid to reset the map."""
        empty_map = OccupancyGrid()
        
        # Fill the header
        empty_map.header = Header()
        empty_map.header.stamp = self.get_clock().now().to_msg()
        empty_map.header.frame_id = "map"

        # Set map metadata
        empty_map.info.resolution = 0.05  # 5 cm per cell
        empty_map.info.width = 100  # 100x100 grid
        empty_map.info.height = 100
        empty_map.info.origin.position.x = -2.5  # Set origin
        empty_map.info.origin.position.y = -2.5
        empty_map.info.origin.position.z = 0.0
        empty_map.info.origin.orientation.w = 1.0

        # Create an empty map (all values = 0, meaning free space)
        empty_map.data = [-1] * (empty_map.info.width * empty_map.info.height)  # -1 means unknown

        # Publish the empty map
        self.map_publisher.publish(empty_map)   
        self.get_logger().info("Published an empty map!")

    def restart_ros2_node(self, node_name, package_name, executable_name):
        """
        Restarts a ROS 2 node by killing it and relaunching it.
        
        :param node_name: Name of the ROS 2 node to restart.
        :param package_name: Name of the ROS 2 package.
        :param executable_name: Name of the node's executable.
        """
        print(f"Restarting node: {node_name}")

        # Kill the existing node
        try:
            subprocess.run(["ros2", "node", "kill", node_name], check=True)
            print(f"Node {node_name} killed successfully.")
        except subprocess.CalledProcessError:
            print(f"Failed to kill node {node_name}. It may not be running.")

        # Wait a bit before restarting
        time.sleep(2)

        # Relaunch the node
        try:
            subprocess.Popen(["ros2", "run", package_name, executable_name])
            print(f"Node {node_name} restarted successfully.")
        except Exception as e:
            print(f"Failed to restart node {node_name}: {e}")

def run_scan(args=None):
    rclpy.init()
    reading_laser = ScanSubscriber()
    reading_laser.get_logger().info("Hello friend!")
    rclpy.spin(reading_laser)

    reading_laser.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    run_scan()
