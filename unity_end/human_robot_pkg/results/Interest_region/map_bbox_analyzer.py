import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
from PIL import Image, ImageColor, ImageDraw


BoundingBox = Tuple[Tuple[float, float], Tuple[float, float]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Project interest-region bounding boxes onto a saved map image and count explored cells."
    )

    parent_dir = "src/DRL-exploration/unity_end/human_robot_pkg/results/Interest_region/W3_controlled_experiment/1000"
    parser.add_argument("--map-image", default=parent_dir + "/final_map.png", help="Path to the saved map image.")
    parser.add_argument(
        "--map-metadata",
        default=parent_dir + "/final_map_metadata.json",
        help="Path to the JSON metadata saved alongside the map image.",
    )
    parser.add_argument(
        "--tile-centers-dir",
        default="src/DRL-exploration/unity_end/human_robot_pkg/tile_centers",
        help="Directory containing tile center text files.",
    )
    parser.add_argument(
        "--tile-center-files",
        nargs="+",
        default=[
            # "Lturn_tile_centers.txt",
            "Tturn_tile_centers.txt",
            # "Uturn_tile_centers.txt",
            # "straight_tile_centers.txt",
        ],
        help="Tile center filenames to load.",
    )
    parser.add_argument(
        "--box-size",
        type=float,
        default=12.0,
        help="Bounding box edge length in world units.",
    )
    parser.add_argument(
        "--output-dir",
        default=parent_dir,
        help="Directory for counts and annotated output.",
    )
    parser.add_argument(
        "--output-prefix",
        default="interest_regions",
        help="Prefix for generated files.",
    )
    return parser.parse_args()


def load_map_metadata(metadata_path: Path) -> Dict[str, object]:
    with metadata_path.open("r", encoding="utf-8") as metadata_file:
        return json.load(metadata_file)


def get_bounding_boxes(
    tiles_center_path: Path,
    box_size: float,
) -> List[BoundingBox]:
    tiles_center = []
    with tiles_center_path.open("r", encoding="utf-8") as tile_file:
        for line in tile_file:
            stripped = line.strip()
            if not stripped:
                continue
            x_str, y_str = stripped.split()
            tiles_center.append((float(x_str), float(y_str)))

    half_box = box_size / 2.0
    boxes = []
    for center_x, center_y in tiles_center:
        top_left = (center_x - half_box, center_y - half_box)
        bottom_right = (center_x + half_box, center_y + half_box)
        boxes.append((top_left, bottom_right))

    converted_boxes = []
    for (x1, y1), (x2, y2) in boxes:
        x1_ros, y1_ros = unity_to_ros_coordinate(x1, y1)
        x2_ros, y2_ros = unity_to_ros_coordinate(x2, y2)
        converted_boxes.append(((x1_ros, y1_ros), (x2_ros, y2_ros)))

    return converted_boxes


def unity_to_ros_coordinate(x: float, y: float) -> Tuple[float, float]:
    return y, -x


def ros_to_image_pixel(
    x: float,
    y: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
    image_height: int,
) -> Tuple[int, int]:
    grid_x = int(np.floor((x - origin_x) / resolution))
    grid_y = int(np.floor((y - origin_y) / resolution))
    pixel_x = grid_x
    pixel_y = image_height - 1 - grid_y
    return pixel_x, pixel_y


def clamp_box_to_image(
    box: BoundingBox,
    metadata: Dict[str, object],
) -> Tuple[int, int, int, int] | None:
    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    origin_x = float(origin["x"])
    origin_y = float(origin["y"])
    width = int(metadata["width"])
    height = int(metadata["height"])

    (x1, y1), (x2, y2) = box
    min_x, max_x = sorted((x1, x2))
    min_y, max_y = sorted((y1, y2))

    left, bottom = ros_to_image_pixel(min_x, min_y, resolution, origin_x, origin_y, height)
    right, top = ros_to_image_pixel(max_x, max_y, resolution, origin_x, origin_y, height)

    raw_x_min = min(left, right)
    raw_x_max = max(left, right)
    raw_y_min = min(top, bottom)
    raw_y_max = max(top, bottom)

    if raw_x_max < 0 or raw_y_max < 0 or raw_x_min >= width or raw_y_min >= height:
        return None

    x_min = max(0, raw_x_min)
    x_max = min(width - 1, raw_x_max)
    y_min = max(0, raw_y_min)
    y_max = min(height - 1, raw_y_max)
    return x_min, y_min, x_max, y_max


def load_boxes_by_region(
    tile_centers_dir: Path,
    tile_center_files: Sequence[str],
    box_size: float,
) -> Dict[str, List[BoundingBox]]:
    boxes_by_region: Dict[str, List[BoundingBox]] = {}
    for file_name in tile_center_files:
        file_path = tile_centers_dir / file_name
        if not file_path.is_file():
            raise FileNotFoundError(f"Missing tile center file: {file_path}")
        region_name = file_path.stem.replace("_tile_centers", "")
        boxes_by_region[region_name] = get_bounding_boxes(file_path, box_size)
    return boxes_by_region


def count_cells_in_boxes(
    image: np.ndarray,
    boxes_by_region: Dict[str, List[BoundingBox]],
    metadata: Dict[str, object],
    unknown_value: int,
) -> Tuple[List[Dict[str, int]], int, int, np.ndarray]:
    explored_mask = image != unknown_value
    union_mask = np.zeros(image.shape[:2], dtype=bool)
    counts: List[Dict[str, int]] = []

    for region_name, boxes in boxes_by_region.items():
        for index, box in enumerate(boxes, start=1):
            clamped_box = clamp_box_to_image(box, metadata)
            if clamped_box is None:
                counts.append(
                    {
                        "region": region_name,
                        "box_index": index,
                        "x_min": -1,
                        "y_min": -1,
                        "x_max": -1,
                        "y_max": -1,
                        "explored_cells": 0,
                        "total_cells": 0,
                    }
                )
                continue

            x_min, y_min, x_max, y_max = clamped_box
            box_mask = np.zeros(image.shape[:2], dtype=bool)
            box_mask[y_min:y_max + 1, x_min:x_max + 1] = True

            explored_cells = int(np.count_nonzero(explored_mask & box_mask))
            total_cells = int(np.count_nonzero(box_mask))
            union_mask |= box_mask

            counts.append(
                {
                    "region": region_name,
                    "box_index": index,
                    "x_min": x_min,
                    "y_min": y_min,
                    "x_max": x_max,
                    "y_max": y_max,
                    "explored_cells": explored_cells,
                    "total_cells": total_cells,
                }
            )

    explored_in_union = int(np.count_nonzero(explored_mask & union_mask))
    total_union_cells = int(np.count_nonzero(union_mask))
    return counts, explored_in_union, total_union_cells, union_mask


def save_counts_csv(output_path: Path, counts: Sequence[Dict[str, int]]) -> None:
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        fieldnames = [
            "region",
            "box_index",
            "x_min",
            "y_min",
            "x_max",
            "y_max",
            "explored_cells",
            "total_cells",
        ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(counts)


def save_summary_json(
    output_path: Path,
    map_image_path: Path,
    metadata_path: Path,
    counts: Sequence[Dict[str, int]],
    explored_in_union: int,
    total_union_cells: int,
) -> None:
    summary = {
        "map_image": str(map_image_path.resolve()),
        "map_metadata": str(metadata_path.resolve()),
        "box_count": len(counts),
        "explored_cells_in_box_union": explored_in_union,
        "total_cells_in_box_union": total_union_cells,
        "boxes": list(counts),
    }
    with output_path.open("w", encoding="utf-8") as summary_file:
        json.dump(summary, summary_file, indent=2)


def render_annotated_image(
    image: np.ndarray,
    counts: Sequence[Dict[str, int]],
    union_mask: np.ndarray,
    output_path: Path,
) -> None:
    annotated = np.stack([image, image, image], axis=-1)
    annotated[union_mask] = (
        0.35 * annotated[union_mask] + 0.65 * np.array([200, 235, 255], dtype=np.float32)
    ).astype(np.uint8)

    annotated_image = Image.fromarray(annotated, mode="RGB")
    draw = ImageDraw.Draw(annotated_image)
    box_color = ImageColor.getrgb("red")
    label_color = ImageColor.getrgb("blue")

    for box in counts:
        if box["total_cells"] == 0:
            continue
        top_left = (box["x_min"], box["y_min"])
        bottom_right = (box["x_max"], box["y_max"])
        draw.rectangle([top_left, bottom_right], outline=box_color, width=2)

        label = f'{box["region"]}_{box["box_index"]}: {box["explored_cells"]}'
        label_origin = (box["x_min"], max(15, box["y_min"] - 6))
        draw.text(label_origin, label, fill=label_color)

    annotated_image.save(output_path)


def main() -> None:
    args = parse_args()

    map_image_path = Path(args.map_image)
    metadata_path = Path(args.map_metadata)
    tile_centers_dir = Path(args.tile_centers_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    image = np.array(Image.open(map_image_path).convert("L"))

    metadata = load_map_metadata(metadata_path)
    boxes_by_region = load_boxes_by_region(
        tile_centers_dir,
        args.tile_center_files,
        args.box_size,
    )

    unknown_value = int(metadata.get("unknown_value", 127))
    counts, explored_in_union, total_union_cells, union_mask = count_cells_in_boxes(
        image,
        boxes_by_region,
        metadata,
        unknown_value,
    )

    csv_path = output_dir / f"{args.output_prefix}_counts.csv"
    json_path = output_dir / f"{args.output_prefix}_summary.json"
    annotated_path = output_dir / f"{args.output_prefix}_annotated.png"

    save_counts_csv(csv_path, counts)
    save_summary_json(
        json_path,
        map_image_path,
        metadata_path,
        counts,
        explored_in_union,
        total_union_cells,
    )
    render_annotated_image(image, counts, union_mask, annotated_path)

    print(f"Saved counts CSV: {csv_path}")
    print(f"Saved summary JSON: {json_path}")
    print(f"Saved annotated image: {annotated_path}")
    print(f"Explored cells in bounding-box union: {explored_in_union}")


if __name__ == "__main__":
    main()
