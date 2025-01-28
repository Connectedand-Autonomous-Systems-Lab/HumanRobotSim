import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist
from visualization_msgs.msg import Marker
from rclpy.logging import LoggingSeverity
import numpy as np
import cv2

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

        self.latest_position = None
        self.latest_heading = None
        self.latest_scan = None
        self.latest_map = None
        self.canvas_size_x = 512
        self.canvas_size_y = 512

    def scan_listener_callback(self, msg):
        self.latest_scan = msg.ranges[:]

    def odom_listener_callback(self, msg):
        self.latest_position = msg.pose.pose.position
        self.latest_heading = msg.pose.pose.orientation
    
    def map_listener_callback(self, msg):
        self.latest_map = msg

    def get_latest_sensor(self):
        # print(self.latest_scan, self.latest_position, self.latest_heading)
        if self.latest_map == None:
            latest_map = np.zeros((self.canvas_size_x, self.canvas_size_y), dtype=np.uint8)
            return latest_map, self.latest_scan, self.latest_position, self.latest_heading, self.get_map_free_pixels()
        return self.map_show(), self.latest_scan, self.latest_position, self.latest_heading, self.get_map_free_pixels()

    def get_map_free_pixels(self):
        
        free_num=0
        if self.latest_map == None:
            return 0
        _ = self.map_show(True)
        for data in self.latest_map.data:
            if data == 0:
                free_num += 1
        return free_num
    
    def occupancy_grid_to_opencv(self):
        """
        Convert an OccupancyGrid message to an OpenCV image.
        """
        width = self.latest_map.info.width
        height = self.latest_map.info.height

        # Convert the occupancy grid data to a numpy array
        map_data = np.array(self.latest_map.data, dtype=np.int8).reshape((height, width))

        # Map the OccupancyGrid values to grayscale (0-255)
        image_data = np.zeros_like(map_data, dtype=np.uint8)
        image_data[map_data == -1] = 127  # Unknown cells (gray)
        image_data[map_data >= 0] = (100 - map_data[map_data >= 0]) * 2.55  # Scale free/occupied cells

        return image_data

    def map_show(self,show_map=False):
        """
        Callback to process the OccupancyGrid message.
        """
        cv_image = self.occupancy_grid_to_opencv()
        fixed_size_image = self.fix_sizes(cv_image)
        # Display the map using OpenCV
        if show_map==True:
            cv2.imshow("Occupancy Grid Map", fixed_size_image)
            cv2.waitKey(1)  # Allow OpenCV to process GUI events
        return fixed_size_image

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
        canvas[int(self.canvas_size_x/2) + position_of_map_origin_x: int(self.canvas_size_x/2) + position_of_map_origin_x+input_size_x , int(self.canvas_size_y/2)+ position_of_map_origin_y: int(self.canvas_size_y/2)+ position_of_map_origin_y+input_size_y] = input_image
        
        return canvas

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
        self.reset_client = self.create_client(Empty, "/reset_world")

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


def run_scan(args=None):
    rclpy.init()
    reading_laser = ScanSubscriber()
    reading_laser.get_logger().info("Hello friend!")
    rclpy.spin(reading_laser)

    reading_laser.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_scan()
