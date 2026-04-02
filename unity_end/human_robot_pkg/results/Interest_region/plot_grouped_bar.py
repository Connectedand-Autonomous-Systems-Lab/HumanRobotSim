#!/usr/bin/env python3

import os
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def compute_interest_region_quality(csv_path: Path) -> float:
    """
    For each box_index:
      - keep rows where explored_cells != 0
      - compute explored_cells / total_cells
      - average within that box_index
    Then average across all box_index values.
    """
    df = pd.read_csv(csv_path)

    required_cols = {"box_index", "explored_cells", "total_cells"}
    missing = required_cols - set(df.columns)
    if missing:
        raise ValueError(f"{csv_path} is missing columns: {missing}")

    # Keep only rows where explored_cells is non-zero
    df = df[df["explored_cells"] != 0].copy()

    if df.empty:
        return np.nan

    # Avoid divide-by-zero
    df = df[df["total_cells"] > 0].copy()
    if df.empty:
        return np.nan

    df["exploration_ratio"] = df["explored_cells"] / df["total_cells"]

    # Average ratio within each box_index
    per_box = df.groupby("box_index")["exploration_ratio"].mean()

    # Average across box_index values
    return per_box.mean() * 100.0  # percentage


def read_final_merged_exploration(log_csv_path: Path) -> float:
    df = pd.read_csv(log_csv_path)

    if "merged exploration" not in df.columns:
        raise ValueError(f"{log_csv_path} does not contain 'merged exploration' column")

    series = df["merged exploration"].dropna()
    if series.empty:
        return np.nan

    value = float(series.iloc[-1])

    return value


def main():
    # Change this to your parent directory
    parent_dir = Path("/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/results/Interest_region/W3_controlled_experiment")


    scales = [0, 5, 10, 100, 1000]

    valid_scales = []
    avg_interest_region_quality = []
    final_merged_exploration = []

    for scale in scales:
        folder = parent_dir / str(scale)
        ir_csv = folder / "interest_regions_counts.csv"
        log_csv = folder / "log.csv"

        if not ir_csv.exists():
            print(f"Skipping {scale}: missing {ir_csv}")
            continue
        if not log_csv.exists():
            print(f"Skipping {scale}: missing {log_csv}")
            continue

        try:
            ir_quality = compute_interest_region_quality(ir_csv)
            merged_exp = read_final_merged_exploration(log_csv)
        except Exception as e:
            print(f"Skipping {scale}: {e}")
            continue

        valid_scales.append(scale)
        avg_interest_region_quality.append(ir_quality)
        final_merged_exploration.append(merged_exp)

        print(
            f"Scale {scale}: "
            f"interest-region avg = {ir_quality:.2f}%, "
            f"final merged exploration = {merged_exp:.2f}"
        )

    if not valid_scales:
        print("No valid data found.")
        return

    x = np.arange(len(valid_scales))
    width = 0.36

    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax2 = ax1.twinx()

    # Use clearly different colors
    color1 = "#1f77b4"   # blue
    color2 = "#ff7f0e"   # orange

    bars1 = ax1.bar(
        x - width / 2,
        avg_interest_region_quality,
        width,
        label="Interest-region quality (%)",
        color=color1
    )

    bars2 = ax2.bar(
        x + width / 2,
        final_merged_exploration,
        width,
        label="Final merged exploration",
        color=color2
    )

    ax1.set_xlabel("Interest region scale")
    ax1.set_ylabel("Interest region quality (%)", color=color1)
    ax2.set_ylabel("Final merged exploration", color=color2)

    ax1.set_xticks(x)
    ax1.set_xticklabels([str(s) for s in valid_scales])
    ax1.set_title("Interest Region Quality vs Final Merged Exploration")

    # Match axis colors to bars (cleaner look)
    ax1.tick_params(axis='y', labelcolor=color1)
    ax2.tick_params(axis='y', labelcolor=color2)

    # --- Legend placement (pick ONE of these) ---

    # Option 1: legend on top center (recommended)
    handles1, labels1 = ax1.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    fig.legend(
        handles1 + handles2,
        labels1 + labels2,
        loc="upper center",
        ncol=2,
        bbox_to_anchor=(0.5, 1.05)
    )

    # Option 2: legend on right side (uncomment if you prefer)
    # fig.legend(
    #     handles1 + handles2,
    #     labels1 + labels2,
    #     loc="center left",
    #     bbox_to_anchor=(1.02, 0.5)
    # )

    # Annotate bars
    for b in bars1:
        h = b.get_height()
        if not np.isnan(h):
            ax1.text(
                b.get_x() + b.get_width() / 2,
                h,
                f"{h:.1f}",
                ha="center",
                va="bottom",
                fontsize=9
            )

    for b in bars2:
        h = b.get_height()
        if not np.isnan(h):
            ax2.text(
                b.get_x() + b.get_width() / 2,
                h,
                f"{h:.0f}",
                ha="center",
                va="bottom",
                fontsize=9
            )

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()