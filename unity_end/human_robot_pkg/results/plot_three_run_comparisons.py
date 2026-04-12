import os

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.ticker import ScalarFormatter


def format_y_axis_scientific(ax):
    formatter = ScalarFormatter(useMathText=True)
    formatter.set_scientific(True)
    formatter.set_powerlimits((0, 0))
    ax.yaxis.set_major_formatter(formatter)
    ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))


def load_data(csv_dir):
    path = os.path.expanduser(os.path.join(csv_dir, 'log.csv'))
    df = pd.read_csv(path)
    df.columns = df.columns.str.strip()
    return df


def get_uniform_y_limits(dataframes, columns, padding_ratio=0.05):
    values = pd.concat([df[columns] for df in dataframes], ignore_index=True)
    y_min = values.min().min()
    y_max = values.max().max()

    if y_min == y_max:
        padding = max(abs(y_max) * padding_ratio, 1.0)
    else:
        padding = (y_max - y_min) * padding_ratio

    return y_min - padding, y_max + padding


run_configs = [
    {
        'name': 'Uncoordinated',
        'path': 'uncoordinated',
        'merged_label': 'Uncoordinated',
    },
    {
        'name': 'Artifact 0',
        'path': 'Artifact/0',
        'merged_label': 'Artifact 0',
    },
    {
        'name': 'Artifact 1000',
        'path': 'Artifact/1000',
        'merged_label': 'Artifact 1000',
    },
]

time_col = 'Time Elapsed (s)'
runs = []

for config in run_configs:
    runs.append(
        {
            'name': config['name'],
            'merged_label': config['merged_label'],
            'df': load_data(config['path']),
        }
    )

all_dataframes = [run['df'] for run in runs]

exploration_ylim = get_uniform_y_limits(
    all_dataframes,
    ['tb exploration', 'human exploration', 'merged exploration']
)
trajectory_ylim = get_uniform_y_limits(
    all_dataframes,
    ['tb trajectory', 'human trajectory']
)
detections_ylim = get_uniform_y_limits(
    all_dataframes,
    ['human detections', 'tb3 detections', 'total detections']
)

fig, axes = plt.subplots(3, 3, figsize=(24, 14), sharex=False)

for row_idx, run in enumerate(runs):
    df = run['df']

    exploration_ax = axes[row_idx, 0]
    exploration_ax.plot(df[time_col], df['tb exploration'], label='Robot only', linewidth=2.0)
    exploration_ax.plot(df[time_col], df['human exploration'], label='Human only', linestyle=':', linewidth=2.0)
    exploration_ax.plot(
        df[time_col],
        df['merged exploration'],
        label=run['merged_label'],
        linestyle='--',
        linewidth=2.0,
    )
    exploration_ax.set_xlabel('Time Elapsed (s)')
    exploration_ax.set_ylabel('Explored Cells')
    exploration_ax.set_title(f"{run['name']} Exploration Over Time")
    exploration_ax.set_ylim(exploration_ylim)
    exploration_ax.grid(True)
    exploration_ax.tick_params(axis='both', labelsize=12)
    format_y_axis_scientific(exploration_ax)
    exploration_ax.legend(fontsize=9)

    trajectory_ax = axes[row_idx, 1]
    trajectory_ax.plot(df[time_col], df['tb trajectory'], label='TurtleBot Trajectory', linewidth=2.0)
    trajectory_ax.plot(df[time_col], df['human trajectory'], label='Human Trajectory', linewidth=2.0)
    trajectory_ax.set_xlabel('Time Elapsed (s)')
    trajectory_ax.set_ylabel('Trajectory Length (m)')
    trajectory_ax.set_title(f"{run['name']} Trajectory Length Over Time")
    trajectory_ax.set_ylim(trajectory_ylim)
    trajectory_ax.grid(True)
    trajectory_ax.tick_params(axis='both', labelsize=12)
    format_y_axis_scientific(trajectory_ax)
    trajectory_ax.legend(fontsize=9)

    detections_ax = axes[row_idx, 2]
    detections_ax.plot(df[time_col], df['human detections'], label='Detection by human', linewidth=2.0)
    detections_ax.plot(df[time_col], df['tb3 detections'], label='Detection by tb3', linewidth=2.0)
    detections_ax.plot(df[time_col], df['total detections'], label='Total unique detections', linewidth=2.0)
    detections_ax.set_xlabel('Time Elapsed (s)')
    detections_ax.set_ylabel('Number of detections')
    detections_ax.set_title(f"{run['name']} Detections Over Time")
    detections_ax.set_ylim(detections_ylim)
    detections_ax.grid(True)
    detections_ax.tick_params(axis='both', labelsize=12)
    format_y_axis_scientific(detections_ax)
    detections_ax.legend(fontsize=9)

plt.suptitle('Uncoordinated vs Artifact Run Metrics Over Time')
plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
plt.show()
