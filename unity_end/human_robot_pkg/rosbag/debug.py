import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarCounter(Node):
    def __init__(self):
        super().__init__('lidar_counter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/human/scan_cropped',  # Replace with your LiDAR topic if different
            self.lidar_callback,
            10
        )
        self.get_logger().info("LidarCounter node has started. Waiting for /scan...")

    def lidar_callback(self, msg: LaserScan):
        num_ranges = len(msg.ranges)
        self.get_logger().info(f"Received LiDAR scan with {num_ranges} values")


def main(args=None):
    rclpy.init(args=args)
    node = LidarCounter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
