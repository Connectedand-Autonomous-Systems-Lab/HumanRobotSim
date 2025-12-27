import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String 
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import math
import csv
import os
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from rclpy.time import Time

class MapLoggerNode(Node):
    def __init__(self):
        super().__init__('map_logger_node')

        # Subscriptions
        self.human_map_sub = self.create_subscription(
            OccupancyGrid,
            'human/map',
            self.human_map_callback,
            10
        )


        self.human_pose_sub = self.create_subscription(
            Odometry,
            'human/odom',
            self.human_pose_callback,
            10
        )

        # Subscriptions
        self.tb_map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.tb_map_callback,
            10
        )


        self.tb_pose_sub = self.create_subscription(
            Odometry,
            'odom',
            self.tb_pose_callback,
            10
        )

        self.merged_map_sub = self.create_subscription(
            OccupancyGrid,
            'merged_map',
            self.merged_map_callback,
            10
        )

        self.human_det_sub = self.create_subscription(
            String,
            'human/detection',
            self.human_detection_callback,
            10
        )

        self.tb3_det_sub = self.create_subscription(
            String,
            'detection',
            self.tb3_detection_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.start_time = self.get_clock().now()

        self.human_map_data = None
        self.human_previous_pose = None
        self.human_trajectory_length = 0.0

        self.tb_map_data = None
        self.tb_previous_pose = None
        self.tb_trajectory_length = 0.0

        self.merged_map_data = None
        self.save_map = False

        self.only_tb = False
        self.only_human = False
        self.both = True
        # self.start_time = None

        self.human_detection_list  = []
        self.tb3_detection_list  = []
        self.total_detections_list = []

        # Setup persistent CSV file object
        package_src_dir = os.path.dirname(os.path.realpath(__file__))
        package_dir = os.path.abspath(os.path.join(package_src_dir, '..'))
        relative_path = 'logs/user_study/yang/hard'
        self.output_file_path = os.path.join(package_dir, relative_path, 'log.csv')
        os.makedirs(os.path.join(package_dir, relative_path), exist_ok=True)

        self.csv_file = open(self.output_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time Elapsed (s)', 'tb exploration', 'tb trajectory', 'human exploration', 'human trajectory', 'merged exploration', 'human detections', 'tb3 detections', 'total detections'])

        self.get_logger().info(f"Logging started: {self.output_file_path}")

    def human_map_callback(self, msg):
        self.human_map_data = msg

    def human_pose_callback(self, msg):
        pose = msg.pose.pose
        x, y = pose.position.x, pose.position.y
        # self.get_logger().info("inside pose callback")
        # self.get_logger().info(f"odom x: {x} y:{y}")
        if self.human_previous_pose:
            dx = x - self.human_previous_pose[0]
            dy = y - self.human_previous_pose[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            self.human_trajectory_length += distance

        self.human_previous_pose = (x, y)

    def tb_map_callback(self, msg):
        self.tb_map_data = msg

    def tb_pose_callback(self, msg):
        pose = msg.pose.pose
        x, y = pose.position.x, pose.position.y
        # self.get_logger().info("inside pose callback")
        # self.get_logger().info(f"odom x: {x} y:{y}")
        if self.tb_previous_pose:
            dx = x - self.tb_previous_pose[0]
            dy = y - self.tb_previous_pose[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            self.tb_trajectory_length += distance

        self.tb_previous_pose = (x, y)

    def merged_map_callback(self, msg):
        self.merged_map_data = msg
        if self.save_map:
            width = msg.info.width
            height = msg.info.height
            map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

            # Convert occupancy values to grayscale image
            # -1 (unknown) -> 127 (gray)
            # 0 (free)     -> 255 (white)
            # 100 (occupied) -> 0 (black)
            image = np.zeros((height, width), dtype=np.uint8)
            image[map_data == -1] = 127
            image[map_data == 0] = 255
            image[map_data == 100] = 0

            # Flip vertically to match visual orientation
            image = np.flipud(image)

            # Create directory if needed
            package_src_dir = os.path.dirname(os.path.realpath(__file__))
            package_dir = os.path.abspath(os.path.join(package_src_dir, '..'))
            self.costmaps_path = os.path.join(package_dir, 'rosbag', 'costmaps')
            os.makedirs(self.costmaps_path, exist_ok=True)

            # Inside your callback
            if self.start_time is None:
                self.start_time = Time.from_msg(msg.header.stamp)

            current_time = Time.from_msg(msg.header.stamp)
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
            elapsed_str = f"{elapsed_time:.2f}".replace('.', '_')  # Replace dot to make valid filename

            # Save the image
            file_path = os.path.join(self.costmaps_path, f'{elapsed_str}.png')
            cv2.imwrite(file_path, image)

            # self.get_logger().info(f"Map saved to {file_path}")

    def timer_callback(self):
        if self.tb_map_data is None or self.human_map_data is None or self.merged_map_data is None:
            if self.tb_map_data is None:
                if self.only_tb or self.both:
                    self.get_logger().info("Waiting for TB map...")
                    return
            if self.human_map_data is None:
                if self.only_human or self.both:
                    self.get_logger().info("Waiting for Human map...")
                    return
            if self.merged_map_data is None:
                if self.both:
                    self.get_logger().info("Waiting for Merged map...")
                    return
                
        if self.only_tb:
            tb_explored_cells = sum(1 for cell in self.tb_map_data.data if cell != -1)
            merged_explored_cells = 0
            human_explored_cells = 0
            self.human_trajectory_length = 0
        elif self.only_human:
            tb_explored_cells = 0
            merged_explored_cells = 0
            human_explored_cells = sum(1 for cell in self.human_map_data.data if cell != -1)
            self.tb_trajectory_length = 0
        else:
            human_explored_cells = sum(1 for cell in self.human_map_data.data if cell != -1)
            tb_explored_cells = sum(1 for cell in self.tb_map_data.data if cell != -1)
            merged_explored_cells = sum(1 for cell in self.merged_map_data.data if cell != -1)
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        self.total_detections_list = list(set(self.human_detection_list+self.tb3_detection_list))
        # self.get_logger().info(f"Human: {self.human_detection_list}  tb3: {self.tb3_detection_list}  total: {self.total_detections_list}")

        self.csv_writer.writerow([elapsed_time, tb_explored_cells, self.tb_trajectory_length, human_explored_cells, self.human_trajectory_length, merged_explored_cells, len(self.human_detection_list), len(self.tb3_detection_list), len(self.total_detections_list)])
        self.csv_file.flush()

        # self.get_logger().info(
        #     f"Time: {elapsed_time:.1f}s | Explored: {merged_explored_cells}"
        # )

    def human_detection_callback(self, msg):
        if msg.data not in self.human_detection_list:
            self.human_detection_list.append(msg.data)

    def tb3_detection_callback(self, msg):
        if msg.data not in self.tb3_detection_list:
            self.tb3_detection_list.append(msg.data)

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info("Shutting down. Closing log file.")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received.")
    finally:
        node.destroy_node()
        if rclpy.ok():  # only destroy if context still active
            rclpy.shutdown()

if __name__ == '__main__':
    main()