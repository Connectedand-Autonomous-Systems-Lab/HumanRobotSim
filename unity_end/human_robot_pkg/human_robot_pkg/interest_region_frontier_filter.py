from pathlib import Path
from typing import Dict, List, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, Point
import rclpy
from rclpy.node import Node


BoundingBox = Tuple[Tuple[float, float], Tuple[float, float]]


def get_bounding_boxes(tiles_center_path: str) -> List[BoundingBox]:
    tiles_center = []
    with open(tiles_center_path, "r", encoding="utf-8") as f:
        for line in f:
            x_str, y_str = line.strip().split()
            tiles_center.append((float(x_str), float(y_str)))

    boxes = []
    box_size = 12
    for center in tiles_center:
        top_left = [center[0] - box_size // 2, center[1] - box_size // 2]
        bottom_right = [center[0] + box_size // 2, center[1] + box_size // 2]
        boxes.append([top_left, bottom_right])

    unity_to_ros_coordinate = lambda x, y: (y, -x)

    converted_boxes = []
    for box in boxes:
        (x1, y1), (x2, y2) = box
        x1_ros, y1_ros = unity_to_ros_coordinate(x1, y1)
        x2_ros, y2_ros = unity_to_ros_coordinate(x2, y2)
        converted_boxes.append(((x1_ros, y1_ros), (x2_ros, y2_ros)))

    return converted_boxes


def is_inside_interest_region(
    frontier_point: Point,
    bounding_boxes: Sequence[BoundingBox],
) -> bool:
    x = float(frontier_point.x)
    y = float(frontier_point.y)

    for (x1, y1), (x2, y2) in bounding_boxes:
        min_x, max_x = sorted((x1, x2))
        min_y, max_y = sorted((y1, y2))
        if min_x <= x <= max_x and min_y <= y <= max_y:
            return True
    return False


class InterestRegionFrontierFilter(Node):
    def __init__(self) -> None:
        super().__init__("interest_region_frontier_filter")

        default_tile_dir = "/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/tile_centers"

        self.declare_parameter("input_frontiers_topic", "/frontiers")
        self.declare_parameter("output_frontiers_topic", "/interest_region_frontiers")
        self.declare_parameter("tile_centers_dir", default_tile_dir)
        self.declare_parameter(
            "tile_center_files",
            [
                # "Lturn_tile_centers.txt",
                "Tturn_tile_centers.txt",
                # "Uturn_tile_centers.txt",
                # "straight_tile_centers.txt",
            ],
        )

        self.input_frontiers_topic = self.get_parameter(
            "input_frontiers_topic"
        ).get_parameter_value().string_value
        self.output_frontiers_topic = self.get_parameter(
            "output_frontiers_topic"
        ).get_parameter_value().string_value
        self.tile_centers_dir = Path(
            self.get_parameter("tile_centers_dir").get_parameter_value().string_value
        )
        self.tile_center_files = list(self.get_parameter("tile_center_files").value)

        self.bounding_boxes_by_type = self._load_bounding_boxes()
        self.bounding_boxes = [
            box
            for boxes in self.bounding_boxes_by_type.values()
            for box in boxes
        ]

        self.frontiers_sub = self.create_subscription(
            PoseArray,
            self.input_frontiers_topic,
            self.frontiers_callback,
            10,
        )
        self.filtered_frontiers_pub = self.create_publisher(
            PoseArray,
            self.output_frontiers_topic,
            10,
        )

        total_boxes = len(self.bounding_boxes)
        self.get_logger().info(
            f"Filtering {self.input_frontiers_topic} using {total_boxes} bounding boxes from "
            f"{self.tile_centers_dir} and publishing matches on {self.output_frontiers_topic}"
        )

    def _load_bounding_boxes(self) -> Dict[str, List[BoundingBox]]:
        bounding_boxes_by_type: Dict[str, List[BoundingBox]] = {}

        for file_name in self.tile_center_files:
            file_path = self.tile_centers_dir / file_name
            if not file_path.is_file():
                self.get_logger().warn(f"Skipping missing tile center file: {file_path}")
                continue

            region_name = file_path.stem.replace("_tile_centers", "")
            bounding_boxes_by_type[region_name] = get_bounding_boxes(str(file_path))

        if not bounding_boxes_by_type:
            self.get_logger().warn(
                f"No tile center files were loaded from {self.tile_centers_dir}. "
                "Filtered frontier output will remain empty."
            )

        return bounding_boxes_by_type

    def frontiers_callback(self, msg: PoseArray) -> None:
        filtered_frontiers = PoseArray()
        filtered_frontiers.header = msg.header

        for pose in msg.poses:
            if is_inside_interest_region(pose.position, self.bounding_boxes):
                filtered_frontiers.poses.append(pose)

        self.filtered_frontiers_pub.publish(filtered_frontiers)
        self.get_logger().debug(
            f"Received {len(msg.poses)} frontiers, published {len(filtered_frontiers.poses)} filtered frontiers"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InterestRegionFrontierFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down interest_region_frontier_filter...")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
