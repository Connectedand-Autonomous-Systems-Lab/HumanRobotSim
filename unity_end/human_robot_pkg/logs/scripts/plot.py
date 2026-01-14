import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib.ticker import ScalarFormatter

csv_dir = '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/akhita/easy'
csv_path = os.path.expanduser(csv_dir + '/log.csv')

df = pd.read_csv(csv_path)
df.columns = df.columns.str.strip()  # Clean headers just in case

print("CSV Columns:", df.columns.tolist())  # Debug print


def format_y_axis_scientific(ax):
    formatter = ScalarFormatter(useMathText=True)
    formatter.set_scientific(True)
    formatter.set_powerlimits((0, 0))
    ax.yaxis.set_major_formatter(formatter)
    ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))


# Plot Exploration Metrics
plt.figure()
plt.plot(df['Time Elapsed (s)'], df['tb exploration'], label='Robot only', linewidth= 2.0)
plt.plot(df['Time Elapsed (s)'], df['human exploration'], label='Human only', linestyle= ':', linewidth= 2.0)
# plt.plot(df['Time Elapsed (s)'], df['tb uncoordinated traj'], label='Human only', linestyle= ':', linewidth= 2.0)
plt.plot(df['Time Elapsed (s)'], df['merged exploration'], label='Uncoordinated', linestyle= '--', linewidth= 2.0)
# plt.plot(df['Time Elapsed (s)'], df['merged_exploration_merged_map'], label='Concord - M', linestyle= '-.', linewidth= 2.0)
# plt.plot(df['Time Elapsed (s)'], df['with_human_knowledge'], label='Concord', linestyle=(0, (3, 5, 1, 5)), linewidth= 2.0)
plt.xlabel('Time Elapsed (s)', fontsize=18)
plt.ylabel('Explored Cells', fontsize= 18)
# plt.title('Exploration Over Time')
plt.grid(True)
plt.tick_params(axis='both', labelsize=18)
format_y_axis_scientific(plt.gca())
plt.legend(fontsize=14)
plt.tight_layout()
# plt.show()
plt.savefig(csv_dir + "/exploration_over_time.png", dpi=300, bbox_inches="tight")

# Plot Trajectory Metrics
plt.figure()
plt.plot(df['Time Elapsed (s)'], df['tb trajectory'], label='TurtleBot Trajectory')
# plt.plot(df['Time Elapsed (s)'], df['tb uncoordinated traj'], label='TurtleBot Trajectory')
plt.plot(df['Time Elapsed (s)'], df['human trajectory'], label='Human Trajectory')
plt.xlabel('Time Elapsed (s)')
plt.ylabel('Trajectory Length (m)')
plt.title('Trajectory Length Over Time')
plt.grid(True)
plt.tick_params(axis='both', labelsize=14)
format_y_axis_scientific(plt.gca())
plt.legend(fontsize=14)
plt.tight_layout()
plt.show()

# Plot Detection over time
plt.figure()
plt.plot(df['Time Elapsed (s)'], df['human detections'], label='Detection by human')
plt.plot(df['Time Elapsed (s)'], df['tb3 detections'], label='Detection by tb3')
plt.plot(df['Time Elapsed (s)'], df['total detections'], label='Total unique detections')
plt.xlabel('Time Elapsed (s)')
plt.ylabel('Number of detections')
plt.title('Number of detections Over Time')
plt.grid(True)
plt.tick_params(axis='both', labelsize=14)
format_y_axis_scientific(plt.gca())
plt.legend(fontsize=14)
plt.tight_layout()
plt.show()
