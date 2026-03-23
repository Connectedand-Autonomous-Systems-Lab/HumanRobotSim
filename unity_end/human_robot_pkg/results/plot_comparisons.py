import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib.ticker import ScalarFormatter

# -----------------------helpers-----------------------
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


# -----------------------load data-----------------------
uncoordinated_df = load_data('uncoordinated')
concord_df = load_data('Concord')

time_col = 'Time Elapsed (s)'

fig, axes = plt.subplots(2, 3, figsize=(20, 10), sharex=True)
axes = axes.flatten()

# 1) Uncoordinated exploration
axes[0].plot(uncoordinated_df[time_col], uncoordinated_df['tb exploration'], label='Robot only', linewidth=2.0)
axes[0].plot(uncoordinated_df[time_col], uncoordinated_df['human exploration'], label='Human only', linestyle=':', linewidth=2.0)
axes[0].plot(uncoordinated_df[time_col], uncoordinated_df['merged exploration'], label='Uncoordinated', linestyle='--', linewidth=2.0)
axes[0].set_xlabel('Time Elapsed (s)')
axes[0].set_ylabel('Explored Cells')
axes[0].set_title('Uncoordinated Exploration Over Time')
axes[0].grid(True)
axes[0].tick_params(axis='both', labelsize=14)
format_y_axis_scientific(axes[0])
axes[0].legend(fontsize=10)

# 2) Uncoordinated trajectory
axes[1].plot(uncoordinated_df[time_col], uncoordinated_df['tb trajectory'], label='TurtleBot Trajectory')
axes[1].plot(uncoordinated_df[time_col], uncoordinated_df['human trajectory'], label='Human Trajectory')
axes[1].set_xlabel('Time Elapsed (s)')
axes[1].set_ylabel('Trajectory Length (m)')
axes[1].set_title('Uncoordinated Trajectory Length Over Time')
axes[1].grid(True)
axes[1].tick_params(axis='both', labelsize=14)
format_y_axis_scientific(axes[1])
axes[1].legend(fontsize=10)

# 3) Uncoordinated detections
axes[2].plot(uncoordinated_df[time_col], uncoordinated_df['human detections'], label='Detection by human')
axes[2].plot(uncoordinated_df[time_col], uncoordinated_df['tb3 detections'], label='Detection by tb3')
axes[2].plot(uncoordinated_df[time_col], uncoordinated_df['total detections'], label='Total unique detections')
axes[2].set_xlabel('Time Elapsed (s)')
axes[2].set_ylabel('Number of detections')
axes[2].set_title('Uncoordinated Detections Over Time')
axes[2].grid(True)
axes[2].tick_params(axis='both', labelsize=14)
format_y_axis_scientific(axes[2])
axes[2].legend(fontsize=10)

# 4) Concord exploration
axes[3].plot(concord_df[time_col], concord_df['tb exploration'], label='Robot only', linewidth=2.0)
axes[3].plot(concord_df[time_col], concord_df['human exploration'], label='Human only', linestyle=':', linewidth=2.0)
axes[3].plot(concord_df[time_col], concord_df['merged exploration'], label='Concord', linestyle='--', linewidth=2.0)
axes[3].set_xlabel('Time Elapsed (s)')
axes[3].set_ylabel('Explored Cells')
axes[3].set_title('Concord Exploration Over Time')
axes[3].grid(True)
axes[3].tick_params(axis='both', labelsize=14)
format_y_axis_scientific(axes[3])
axes[3].legend(fontsize=10)

# 5) Concord trajectory
axes[4].plot(concord_df[time_col], concord_df['tb trajectory'], label='TurtleBot Trajectory')
axes[4].plot(concord_df[time_col], concord_df['human trajectory'], label='Human Trajectory')
axes[4].set_xlabel('Time Elapsed (s)')
axes[4].set_ylabel('Trajectory Length (m)')
axes[4].set_title('Concord Trajectory Length Over Time')
axes[4].grid(True)
axes[4].tick_params(axis='both', labelsize=14)
format_y_axis_scientific(axes[4])
axes[4].legend(fontsize=10)

# 6) Concord detections
axes[5].plot(concord_df[time_col], concord_df['human detections'], label='Detection by human')
axes[5].plot(concord_df[time_col], concord_df['tb3 detections'], label='Detection by tb3')
axes[5].plot(concord_df[time_col], concord_df['total detections'], label='Total unique detections')
axes[5].set_xlabel('Time Elapsed (s)')
axes[5].set_ylabel('Number of detections')
axes[5].set_title('Concord Detections Over Time')
axes[5].grid(True)
axes[5].tick_params(axis='both', labelsize=14)
format_y_axis_scientific(axes[5])
axes[5].legend(fontsize=10)

plt.suptitle('Uncoordinated vs Concord Metrics Over Time')
plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
plt.show()
