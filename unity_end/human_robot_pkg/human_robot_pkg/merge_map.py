import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

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
            '/tb3_0/map',
            self.map2_callback,
            10
        )

        # Publisher for the merged map
        self.pub_merged_map = self.create_publisher(OccupancyGrid, '/merged_map', 10)

        # Store maps
        self.map1 = None
        self.map2 = None

    def map1_callback(self, msg):
        self.get_logger().info('Received human map')
        self.map1 = msg
        self.try_merge_maps()

    def map2_callback(self, msg):
        self.get_logger().info('Received tb3_0 map')
        self.map2 = msg
        self.try_merge_maps()

    def try_merge_maps(self):
        if self.map1 and self.map2:
            # Ensure both maps have the same resolution
            if self.map1.info.resolution == self.map2.info.resolution:
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

                # Function to place a map onto the merged map
                def place_map(map_msg):
                    offset_x = int((map_msg.info.origin.position.x - min_x) / resolution)
                    offset_y = int((map_msg.info.origin.position.y - min_y) / resolution)

                    map_data = np.array(map_msg.data, dtype=np.int8).reshape(
                        (map_msg.info.height, map_msg.info.width)
                    )

                    for y in range(map_msg.info.height):
                        for x in range(map_msg.info.width):
                            if map_data[y, x] != -1:  # Ignore unknown cells
                                merged_cell = merged_data[offset_y + y, offset_x + x]
                                merged_data[offset_y + y, offset_x + x] = max(merged_cell, map_data[y, x])

                # Place both maps
                place_map(self.map1)
                place_map(self.map2)

                # Create merged OccupancyGrid message
                merged_map = OccupancyGrid()
                merged_map.header.stamp = self.get_clock().now().to_msg()
                merged_map.header.frame_id = self.map1.header.frame_id
                merged_map.info.resolution = resolution
                merged_map.info.width = merged_width
                merged_map.info.height = merged_height
                merged_map.info.origin.position.x = min_x
                merged_map.info.origin.position.y = min_y
                merged_map.info.origin.position.z = 0.0

                merged_map.info.origin.orientation.w = 1.0

                merged_map.data = merged_data.flatten().tolist()
                self.pub_merged_map.publish(merged_map)
                self.get_logger().info('Published merged map.')
            else:
                self.get_logger().warn('Maps have different resolutions.')


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
