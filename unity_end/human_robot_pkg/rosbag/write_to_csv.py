import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import csv
import os
from datetime import datetime

class GazeLogger(Node):
    def __init__(self):
        super().__init__('gaze_logger_node')

        # Create output CSV file with timestamp in name
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"gaze_log.csv"
        self.filepath = os.path.join(os.getcwd(), self.filename)

        # Open file and write header
        self.csv_file = open(self.filepath, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'z'])

        # Subscriber to /gaze/point
        self.subscription = self.create_subscription(
            PointStamped,
            '/gaze/point',
            self.listener_callback,
            10
        )

        self.get_logger().info(f"Logging gaze points to {self.filepath}")

    def listener_callback(self, msg: PointStamped):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x, y, z = msg.point.x, msg.point.y, msg.point.z

        # Write to CSV
        self.csv_writer.writerow([timestamp, x, y, z])
        self.get_logger().info(f"Gaze point: t={timestamp:.3f}, x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
