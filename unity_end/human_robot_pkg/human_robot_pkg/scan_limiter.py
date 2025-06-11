import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/human/scan',  # Input topic (assumes -180° to 180°)
            self.lidar_callback,
            10
        )

        self.publisher = self.create_publisher(
            LaserScan,
            '/human/scan_cropped',  # Output topic (filtered -90° to 90°)
            10
        )

        self.get_logger().info("LidarFilterNode is active. Filtering -90° to 90°...")

    def lidar_callback(self, msg: LaserScan):
        angle_min = msg.angle_min  # in radians
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment

        total_ranges = len(msg.ranges)

        # Compute angle at each index
        angles = [angle_min + i * angle_increment for i in range(total_ranges)]

        
        foveal_angle = 60
        min_angle = -math.pi*foveal_angle /360 
        max_angle = math.pi*foveal_angle /360 

        filtered_ranges = []
        filtered_angles = []

        for i, angle in enumerate(angles):
            if min_angle <= angle <= max_angle:
                r = msg.ranges[i]
                filtered_ranges.append(min(r,msg.range_max))
                filtered_angles.append(angle)

        if not filtered_ranges:
            self.get_logger().warn("No valid data found in the -90° to 90° range.")
            return

        # self.get_logger().info(msg.ranges)
        # self.get_logger().info("--------------------------------------------------------------")
        # self.get_logger().info(filtered_ranges)
        # Build new LaserScan message
        new_scan = LaserScan()
        new_scan.header = msg.header
        new_scan.angle_min = filtered_angles[0]
        new_scan.angle_max = filtered_angles[-1]
        new_scan.angle_increment = angle_increment
        new_scan.time_increment = msg.time_increment
        new_scan.scan_time = msg.scan_time
        new_scan.range_min = msg.range_min
        new_scan.range_max = msg.range_max
        new_scan.ranges = [float(r) for r in filtered_ranges]
        new_scan.intensities = []  # You can similarly clean intensities if needed

        self.publisher.publish(new_scan)
        # self.get_logger().info(f"Published filtered scan with {len(filtered_ranges)} points")

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
