#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def compute_interest_region_quality(csv_path: Path) -> float:
    """
    For each box_index:
      - keep rows where explored_cells != 0
      - compute explored_cells / total_cells
      - average within that box_index
    Then average across all box_index values.
    """
    with csv_path.open("r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = set(reader.fieldnames or [])
        rows = list(reader)

    required_cols = {"box_index", "explored_cells", "total_cells"}
    missing = required_cols - fieldnames
    if missing:
        raise ValueError(f"{csv_path} is missing columns: {missing}")

    ratios_by_box: dict[int, list[float]] = {}
    for row in rows:
        try:
            box_index = int(row["box_index"])
            explored_cells = float(row["explored_cells"])
            total_cells = float(row["total_cells"])
        except (TypeError, ValueError):
            continue

        if explored_cells == 0 or total_cells <= 0:
            continue

        ratios_by_box.setdefault(box_index, []).append(explored_cells / total_cells)

    if not ratios_by_box:
        return np.nan

    per_box_means = [float(np.mean(ratios)) for ratios in ratios_by_box.values()]
    return float(np.mean(per_box_means) * 100.0)


def read_final_merged_exploration(log_csv_path: Path) -> float:
    with log_csv_path.open("r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = set(reader.fieldnames or [])
        rows = list(reader)

    if "merged exploration" not in fieldnames:
        raise ValueError(f"{log_csv_path} does not contain 'merged exploration' column")

    values = []
    for row in rows:
        raw_value = row.get("merged exploration")
        if raw_value is None or raw_value == "":
            continue
        try:
            values.append(float(raw_value))
        except ValueError:
            continue

    if not values:
        return np.nan

    return float(values[-1])


def read_time_inside_any_box_pct(csv_path: Path) -> float:
    with csv_path.open("r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = set(reader.fieldnames or [])
        rows = list(reader)

    required_cols = {"region", "robot_time_inside_pct"}
    missing = required_cols - fieldnames
    if missing:
        raise ValueError(f"{csv_path} is missing columns: {missing}")

    for row in rows:
        if row.get("region") != "ALL_BOXES":
            continue
        raw_value = row.get("robot_time_inside_pct")
        if raw_value is None or raw_value == "":
            return np.nan
        try:
            return float(raw_value)
        except ValueError as exc:
            raise ValueError(f"{csv_path} has invalid ALL_BOXES robot_time_inside_pct") from exc

    raise ValueError(f"{csv_path} does not contain an ALL_BOXES summary row")


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_parent = script_dir / "IRS_experiment"

    parser = argparse.ArgumentParser(
        description=(
            "Plot the average final merged exploration and interest-region quality "
            "over all users for each IRS value."
        )
    )
    parser.add_argument(
        "--parent-dir",
        type=Path,
        default=default_parent,
        help="Parent directory containing per-user IRS experiment folders.",
    )
    parser.add_argument(
        "--irs-values",
        nargs="+",
        type=int,
        default=None,
        help="Optional list of IRS values to plot. Defaults to auto-discovery.",
    )
    parser.add_argument(
        "--save-path",
        type=Path,
        default=None,
        help="Optional output image path. If omitted, the plot is shown interactively.",
    )
    return parser.parse_args()


def discover_irs_values(parent_dir: Path) -> list[int]:
    irs_values: set[int] = set()
    for user_dir in parent_dir.iterdir():
        if not user_dir.is_dir():
            continue
        for run_dir in user_dir.iterdir():
            if run_dir.is_dir():
                try:
                    irs_values.add(int(run_dir.name))
                except ValueError:
                    continue
    return sorted(irs_values)


def collect_metrics(parent_dir: Path, irs_values: list[int]) -> list[dict[str, float | int]]:
    user_dirs = sorted(path for path in parent_dir.iterdir() if path.is_dir())
    results: list[dict[str, float | int]] = []

    for irs_value in irs_values:
        ir_qualities = []
        merged_explorations = []
        time_inside_any_box_pcts = []
        included_users = []

        for user_dir in user_dirs:
            run_dir = user_dir / str(irs_value)
            ir_csv = run_dir / "interest_regions_counts.csv"
            log_csv = run_dir / "log.csv"

            if not ir_csv.exists() or not log_csv.exists():
                print(f"Skipping {user_dir.name}/{irs_value}: missing required CSV files")
                continue

            try:
                ir_quality = compute_interest_region_quality(ir_csv)
                merged_exp = read_final_merged_exploration(log_csv)
                time_inside_any_box_pct = read_time_inside_any_box_pct(ir_csv)
            except Exception as exc:
                print(f"Skipping {user_dir.name}/{irs_value}: {exc}")
                continue

            if np.isnan(ir_quality) or np.isnan(merged_exp) or np.isnan(time_inside_any_box_pct):
                print(f"Skipping {user_dir.name}/{irs_value}: metric evaluated to NaN")
                continue

            ir_qualities.append(ir_quality)
            merged_explorations.append(merged_exp)
            time_inside_any_box_pcts.append(time_inside_any_box_pct)
            included_users.append(user_dir.name)

        if not included_users:
            print(f"Skipping IRS {irs_value}: no valid user runs found")
            continue

        results.append(
            {
                "irs_value": irs_value,
                "avg_interest_region_quality": float(np.mean(ir_qualities)),
                "avg_final_merged_exploration": float(np.mean(merged_explorations)),
                "avg_time_inside_any_box_pct": float(np.mean(time_inside_any_box_pcts)),
                "num_users": len(included_users),
            }
        )

        print(
            f"IRS {irs_value}: users={len(included_users)} ({', '.join(included_users)}) | "
            f"avg interest-region quality={np.mean(ir_qualities):.2f}% | "
            f"avg time inside any box={np.mean(time_inside_any_box_pcts):.2f}% | "
            f"avg final merged exploration={np.mean(merged_explorations):.2f}"
        )

    return results


def plot_results(results: list[dict[str, float | int]], save_path: Path | None) -> None:
    irs_values = [int(result["irs_value"]) for result in results]
    avg_ir_quality = [float(result["avg_interest_region_quality"]) for result in results]
    avg_merged_exploration = [float(result["avg_final_merged_exploration"]) for result in results]
    avg_time_inside_any_box_pct = [float(result["avg_time_inside_any_box_pct"]) for result in results]

    x = np.arange(len(irs_values))
    width = 0.24

    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax2 = ax1.twinx()

    color1 = "#1f77b4"
    color2 = "#ff7f0e"
    color3 = "#2ca02c"

    bars1 = ax1.bar(
        x - width,
        avg_ir_quality,
        width,
        label="Average interest-region quality (%)",
        color=color1,
    )
    bars2 = ax1.bar(
        x,
        avg_time_inside_any_box_pct,
        width,
        label="Average time inside any box (%)",
        color=color3,
    )
    bars3 = ax2.bar(
        x + width,
        avg_merged_exploration,
        width,
        label="Average final merged exploration",
        color=color2,
    )

    ax1.set_xlabel("IRS value")
    ax1.set_ylabel("Average interest-region quality (%)", color=color1)
    ax2.set_ylabel("Average final merged exploration", color=color2)

    ax1.set_xticks(x)
    ax1.set_xticklabels([str(value) for value in irs_values])
    ax1.set_title("Average Metrics Over All Users for Each IRS Value")
    ax1.tick_params(axis="y", labelcolor=color1)
    ax2.tick_params(axis="y", labelcolor=color2)

    handles1, labels1 = ax1.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    fig.legend(
        handles1 + handles2,
        labels1 + labels2,
        loc="upper center",
        ncol=3,
        bbox_to_anchor=(0.5, 0.98),
        frameon=True,
    )

    for bar in bars1:
        height = bar.get_height()
        ax1.text(
            bar.get_x() + bar.get_width() / 2,
            height,
            f"{height:.1f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )

    for bar in bars2:
        height = bar.get_height()
        ax1.text(
            bar.get_x() + bar.get_width() / 2,
            height,
            f"{height:.1f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )

    for bar in bars3:
        height = bar.get_height()
        ax2.text(
            bar.get_x() + bar.get_width() / 2,
            height,
            f"{height:.0f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )

    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.92))

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"Saved plot to {save_path}")
    else:
        plt.show()


def main() -> None:
    args = parse_args()
    parent_dir = args.parent_dir.expanduser().resolve()

    if not parent_dir.exists():
        raise FileNotFoundError(f"Parent directory does not exist: {parent_dir}")

    irs_values = sorted(args.irs_values) if args.irs_values is not None else discover_irs_values(parent_dir)
    if not irs_values:
        raise ValueError(f"No IRS values found under {parent_dir}")

    results = collect_metrics(parent_dir, irs_values)
    if not results:
        raise ValueError("No valid data found to plot.")

    save_path = args.save_path.expanduser().resolve() if args.save_path is not None else None
    plot_results(results, save_path)


if __name__ == "__main__":
    main()
