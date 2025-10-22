import pandas as pd
import matplotlib.pyplot as plt
import os

csv_path = os.path.expanduser('/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/exploration_log_merged.csv')

df = pd.read_csv(csv_path)
df.columns = df.columns.str.strip()  # Clean headers just in case

print("CSV Columns:", df.columns.tolist())  # Debug print


# Plot Exploration Metrics
# plt.figure()
# plt.plot(df['Time Elapsed (s)'], df['tb exploration'], label='TurtleBot Exploration')
# plt.plot(df['Time Elapsed (s)'], df['human exploration'], label='Human Exploration')
# plt.plot(df['Time Elapsed (s)'], df['merged exploration'], label='Merged Exploration')
# plt.xlabel('Time Elapsed (s)')
# plt.ylabel('Explored Cells')
# plt.title('Exploration Over Time')
# plt.grid(True)
# plt.legend()
# plt.tight_layout()
# plt.show()

# Plot Trajectory Metrics
# plt.figure()
# plt.plot(df['Time Elapsed (s)'], df['tb trajectory'], label='TurtleBot Trajectory')
# plt.plot(df['Time Elapsed (s)'], df['human trajectory'], label='Human Trajectory')
# plt.xlabel('Time Elapsed (s)')
# plt.ylabel('Trajectory Length (m)')
# plt.title('Trajectory Length Over Time')
# plt.grid(True)
# plt.legend()
# plt.tight_layout()
# plt.show()

# Plot Detection over time
plt.figure()
plt.plot(df['Time Elapsed (s)'], df['human detections'], label='Detection by human')
plt.plot(df['Time Elapsed (s)'], df['tb3 detections'], label='Detection by tb3')
plt.plot(df['Time Elapsed (s)'], df['total detections'], label='Total unique detections')
plt.xlabel('Time Elapsed (s)')
plt.ylabel('Number of detections')
plt.title('Number of detections Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()