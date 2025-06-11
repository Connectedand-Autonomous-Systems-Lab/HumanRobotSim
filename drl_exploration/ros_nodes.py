import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist, PoseStamped
from visualization_msgs.msg import Marker
from rclpy.logging import LoggingSeverity
import numpy as np
import cv2
from std_msgs.msg import Header
import std_msgs.msg
# import time
import threading
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
# from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            LaserScan, "scan", self.scan_listener_callback, 1
        )
        self.subscriber_ = self.create_subscription(
            Odometry, "odom", self.odom_listener_callback, 1
        )
        self.subscriber_ = self.create_subscription(
            OccupancyGrid, "map", self.map_listener_callback, 1
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
        self.timer = self.create_timer(0.05, self.lookup_transform)
        self.transform = None

    def scan_listener_callback(self, msg):
        self.latest_scan = msg.ranges[:]

    def odom_listener_callback(self, msg):
        self.latest_position = msg.pose.pose.position
        self.latest_heading = msg.pose.pose.orientation
        self.odom_frame = msg.header.frame_id
        self.robot_position = msg
    
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
            return latest_map, self.latest_scan, self.latest_position, self.latest_heading, self.get_map_free_pixels()
        # while self.latest_map == None:
        #     continue
        return self.map_show(is_transform_available, True), self.latest_scan, self.latest_position, self.latest_heading, self.get_map_free_pixels()

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
    
    def occupancy_grid_to_opencv(self):
        """
        Convert an OccupancyGrid message to an OpenCV image.
        occupancy grid numbering starts from bottom left corner
        opencv is top left
        """
        width = self.latest_map.info.width
        height = self.latest_map.info.height

        origin_x = self.latest_map.info.origin.position.x
        origin_y = self.latest_map.info.origin.position.y

        # print("origin : ", origin_x, origin_y)
        # Convert the occupancy grid data to a numpy array
        map_data = np.array(self.latest_map.data, dtype=np.int8).reshape((height, width))
        
        # print(f"unique image raw data {np.unique(map_data)}")
        # Map the OccupancyGrid values to grayscale (0-255)
        image_data = np.zeros_like(map_data, dtype=np.uint8)
        image_data[map_data == -1] = 0  # Unknown cells (gray)
        image_data[map_data >= 0] = (100 - map_data[map_data >= 0]) * 2.55  # Scale free/occupied cells

        # print(f"unique processed image data {np.unique(image_data)}")
        return image_data, [origin_x, origin_y]

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

    def map_show(self, is_transform_available, show_map=False):
        """
        Callback to process the OccupancyGrid message.
        """
        cv_image, origin = self.occupancy_grid_to_opencv()
        # fixed_size_image = self.fix_sizes(cv_image)
        
        if self.transform == None:
            pass
        else:
            robot_position = self.get_robot_position_on_map(origin)
            try:
                cv2.circle(cv_image,robot_position,2,(255),-1)
            except: 
                print(Fore.RED + f'Invalid Robot position!!!!!' + Style.RESET_ALL)
        cv_image = np.fliplr(cv_image)  # Flip vertically
        try:
            fixed_size_image = cv2.resize(cv_image, (512,512))
        except:
            fixed_size_image = np.zeros((512, 512), dtype=np.uint8)

        # Display the map using OpenCV
        if show_map==True:
            cv2.imshow("Occupancy Grid Map", fixed_size_image)
            cv2.waitKey(1)  # Allow OpenCV to process GUI events
        # cv2.circle(fixed_size_image, robot_position,5,(255),-1)
        return fixed_size_image

    def get_robot_position_on_map(self, origin):
        
        # self.map_frame = self.latest_map.header.frame_id
        # print(self.odom_frame , self.map_frame)
        # Convert Odometry to PoseStamped
        # pose_stamped = PoseStamped()
        # pose_stamped.header = self.robot_position.header
        # pose_stamped.pose = self.robot_position.pose.pose  # Extract only the pose

        # pose_out = self.tf_buffer.transform(pose_stamped, 'map', rclpy.duration.Duration(seconds=1))
        # print("robot position on odom frame : ", self.robot_position.pose.pose.position.x, self.robot_position.pose.pose.position.y)
        # Lookup the transform from 'odom' to 'map'

        
        # try:
        #     transform = self.tf_buffer.lookup_transform(
        #         self.map_frame, self.odom_frame,# Transform from 'odom' to 'map'
        #         rclpy.time.Time(),
        #         timeout=rclpy.duration.Duration(seconds=5.0))

        # except TransformException as e:
        #     print(Fore.YELLOW+ str(e) + Style.RESET_ALL)

        # Transform pose using do_transform_pose()
        pose_out = do_transform_pose(self.robot_position.pose.pose, self.transform)
        x = pose_out.position.x - origin[0]
        y = pose_out.position.y - origin[1]
        # print("robot positin on map frame : ", x,y)
        robot_position_on_map_in_cells = [int(x/self.map_resolution) , int(y/self.map_resolution)]
        # print(robot_position_on_map_in_cells)
        return robot_position_on_map_in_cells

    def fix_sizes(self,input_image):
        
        # canvas_size = 300
        canvas = np.zeros((self.canvas_size_x, self.canvas_size_y), dtype=np.uint8)
        
        origin_x = self.latest_map.info.origin.position.x
        origin_y = self.latest_map.info.origin.position.y

        # Take the real world corrdinate of the (0,0) pixel of the map and convert that to pixels.
        # 1m eqauls to 50 pixels
        position_of_map_origin_x = int(origin_x*50)
        position_of_map_origin_y = int(origin_y*50)

        if self.latest_map.info.origin.orientation.x or self.latest_map.info.origin.orientation.y or self.latest_map.info.origin.orientation.z or self.latest_map.info.origin.orientation.w != 1 :
            print("There is an orientation in map")
            print(self.latest_map.info.origin.orientation)

        input_size_x = input_image.shape[0]
        input_size_y = input_image.shape[1]

        # Keep it at bottom left
        # canvas[canvas_size - input_image.shape[0]: canvas_size, 0:input_image.shape[1]] = input_image

        # Keep the origin at the center of the canvas
        # canvas[canvas_size_x/2 - origin_x : canvas_size_x/2 + (input_size_x - origin_x)    ,    canvas_size_y/2 - origin_y : canvas_size_y/2 + (input_size_y - origin_y)] = input_image
        
        # Keep the map unmoved
        canvas[int(self.canvas_size_x/2) + position_of_map_origin_x: int(self.canvas_size_x/2) + position_of_map_origin_x+input_size_x, 
               int(self.canvas_size_y/2)+ position_of_map_origin_y: int(self.canvas_size_y/2)+ position_of_map_origin_y+input_size_y] = input_image
        
        map_origin_canvas_x = int(self.canvas_size_x / 2) + position_of_map_origin_x
        map_origin_canvas_y = int(self.canvas_size_y / 2) + position_of_map_origin_y

        # Mark the map origin on the canvas with a red dot
        # print(position_of_map_origin_x, position_of_map_origin_y)
        # print(map_origin_canvas_x, map_origin_canvas_y)
        cv2.circle(canvas, (map_origin_canvas_y, map_origin_canvas_x), 5, (255), -1)  # White dot (255) for grayscale
        cv2.circle(canvas, (0, 0), 5, (255), -1)  # White dot (255) for grayscale

        return canvas

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
        self.reset_client = self.create_client(Empty, "/reset_world")   # Reset models to the original poses. DO NOt reset time

        self.wait_for_service(self.reset_client, "reset_world")

    def wait_for_service(self, client, service_name, timeout=10.0):
        self.get_logger().info(f"Waiting for {service_name} service...")
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f"Service {service_name} not available after waiting."
            )
            raise RuntimeError(f"Service {service_name} not available.")

    def reset_world(self):
        self.get_logger().info("Calling /gazebo/reset_world service...")
        request = Empty.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("World reset successfully.")
        else:
            self.get_logger().error(f"Failed to reset world: {future.exception()}")

class PhysicsClient(Node):
    def __init__(self):
        super().__init__("physics_client")
        self.get_logger().set_level(SEVERITY)
        self.unpause_client = self.create_client(Empty, "/unpause_physics")
        self.pause_client = self.create_client(Empty, "/pause_physics")

        self.wait_for_service(self.unpause_client, "unpause_physics")
        self.wait_for_service(self.pause_client, "pause_physics")

    def wait_for_service(self, client, service_name, timeout=10.0):
        self.get_logger().info(f"Waiting for {service_name} service...")
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f"Service {service_name} not available after waiting."
            )
            raise RuntimeError(f"Service {service_name} not available.")

    def pause_physics(self):
        self.get_logger().info("Calling /gazebo/pause_physics service...")
        request = Empty.Request()
        future = self.pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Physics paused successfully.")
        else:
            self.get_logger().error(f"Failed to pause physics: {future.exception()}")

    def unpause_physics(self):
        self.get_logger().info("Calling /gazebo/unpause_physics service...")
        request = Empty.Request()
        future = self.unpause_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Physics unpaused successfully.")
        else:
            self.get_logger().error(f"Failed to unpause physics: {future.exception()}")

class SetModelStateClient(Node):
    def __init__(self):
        super().__init__("set_entity_state_client")
        self.get_logger().set_level(SEVERITY)
        self.client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
            print("set entity state is not found")
        self.request = SetEntityState.Request()

    def set_state(self, name, new_pose):
        self.request.state.name = name
        self.request.state.pose = new_pose
        self.future = self.client.call_async(self.request)

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__("cmd_vel_publisher")
        self.get_logger().set_level(SEVERITY)
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 1)
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
        turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
        cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
            turtlebot3_cartographer_prefix, 'config'))
        configuration_basename = LaunchConfiguration('configuration_basename', default='turtlebot3_lds_2d.lua')
        resolution = LaunchConfiguration('resolution', default='0.05')
        publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
        slam_params_file = LaunchConfiguration('slam_params_file')

        # Create Cartographer Node
        self.cartographer_node = launch_ros.actions.Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time, 'ros__parameters': {'log_level': 'warn'}}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '--ros-args', '--log-level', 'fatal']
        )

        self.occupancy_grid_node = launch_ros.actions.Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time, 'ros__parameters': {'log_level': 'warn'}}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec ,'--ros-args', '--log-level', 'fatal']
            )

        self.start_async_slam_toolbox_node = launch_ros.actions.Node(
            parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
            ],
            arguments=['--ros-args', '--log-level', 'fatal'],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='log'
        )
        
        self.static_transform = launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"],
            output="log"
        )

        return LaunchDescription([
            # For cartographer
            DeclareLaunchArgument(
                'cartographer_config_dir',
                default_value=cartographer_config_dir,
                description='Full path to config file to load'),
            DeclareLaunchArgument(
                'configuration_basename',
                default_value=configuration_basename,
                description='Name of lua file for cartographer'),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation (Gazebo) clock if true'),

            # For Occupancy grid
            DeclareLaunchArgument(
                'resolution',
                default_value=resolution,
                description='Resolution of a grid cell in the published occupancy grid'),
            DeclareLaunchArgument(
                'publish_period_sec',
                default_value=publish_period_sec,
                description='OccupancyGrid publishing period'),

            DeclareLaunchArgument(
                'slam_params_file',
                default_value=os.path.join('/home/mayooran/Documents/DRL-Robot-Navigation-ROS2/src/drl_exploration',
                                        'config', 'slam.yaml'),
                description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),

            self.cartographer_node,  # Add Cartographer node to launch description
            self.occupancy_grid_node,
            # self.start_async_slam_toolbox_node
            # self.static_transform
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
