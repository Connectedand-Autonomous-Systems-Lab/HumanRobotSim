import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import hashlib

class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')
        self.get_logger().info('Map_merger started!')

        # Subscriptions to the two map topics
        self.sub_map1 = self.create_subscription(
            OccupancyGrid,
            '/human/map',
            self.map1_callback,
            10
        )
        self.sub_map2 = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map2_callback,
            10
        )
        self.pub_merged_map = self.create_publisher(OccupancyGrid, '/merged_map', 10)

        # Store maps
        self.map1 = None
        self.map2 = None

    def map1_callback(self, msg):
        # self.get_logger().info('Received human map')
        self.map1 = msg
        self.try_merge_maps()

    def map2_callback(self, msg):
        # self.get_logger().info('Received tb3_0 map')
        self.map2 = msg
        self.try_merge_maps()

    def try_merge_maps(self):
        if self.map1 and self.map2:
            # Ensure both maps have the same resolution
            if self.map1.info.resolution == self.map2.info.resolution:
                # self.get_logger().info("Trying to merge maps...")
                resolution = self.map1.info.resolution

                # Determine merged map boundaries
                min_x = min(self.map1.info.origin.position.x, self.map2.info.origin.position.x)
                min_y = min(self.map1.info.origin.position.y, self.map2.info.origin.position.y)
                max_x = max(
                    self.map1.info.origin.position.x + self.map1.info.width * resolution,
                    self.map2.info.origin.position.x + self.map2.info.width * resolution
                )
                max_y = max(
                    self.map1.info.origin.position.y + self.map1.info.height * resolution,
                    self.map2.info.origin.position.y + self.map2.info.height * resolution
                )

                # Calculate merged map dimensions
                merged_width = int((max_x - min_x) / resolution)
                merged_height = int((max_y - min_y) / resolution)

                # Initialize merged map with unknown values (-1)
                merged_data = np.full((merged_height, merged_width), -1, dtype=np.int8)
                # self.get_logger().info(f"Merged map size: {merged_width} x {merged_height}")

                # Function to place a map onto the merged map
                def place_map(map_msg):
                    offset_x = int((map_msg.info.origin.position.x - min_x) / resolution)
                    offset_y = int((map_msg.info.origin.position.y - min_y) / resolution)
                    h = map_msg.info.height
                    w = map_msg.info.width

                    map_data = np.array(map_msg.data, dtype=np.int8).reshape((h, w))

                    # Clip placement if it exceeds merged bounds
                    max_y_merge = merged_data.shape[0]
                    max_x_merge = merged_data.shape[1]

                    clip_h = min(h, max_y_merge - offset_y)
                    clip_w = min(w, max_x_merge - offset_x)

                    if clip_h <= 0 or clip_w <= 0:
                        self.get_logger().warn("Map placement is completely outside merged area — skipping")
                        return

                    # Safely extract valid region
                    clipped_map = map_data[:clip_h, :clip_w]
                    valid_mask = (clipped_map != -1)

                    target = merged_data[offset_y:offset_y + clip_h, offset_x:offset_x + clip_w]
                    np.putmask(target, valid_mask, np.maximum(target, clipped_map))


                place_map(self.map1)
                place_map(self.map2)

                merged_msg = OccupancyGrid()
                merged_msg.header.stamp = self.get_clock().now().to_msg()
                merged_msg.header.frame_id = "map"
                merged_msg.info.resolution = resolution
                merged_msg.info.width = merged_width
                merged_msg.info.height = merged_height
                merged_msg.info.origin.position.x = min_x
                merged_msg.info.origin.position.y = min_y
                merged_msg.info.origin.position.z = 0.0
                merged_msg.info.origin.orientation.w = 1.0
                merged_msg.data = merged_data.flatten().tolist()

                self.pub_merged_map.publish(merged_msg)
                # self.get_logger().info("Published optimized merged map")

def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
