import pandas as pd
import matplotlib.pyplot as plt
import os
from pathlib import Path
import numpy as np

base_dir = '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M'
# csv_path = os.path.expanduser(csv_dir + '/log.csv')

# dirs = [p for p in Path(base_dir).iterdir() if p.is_dir()]

dirs = ['/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/akhita',
        # '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/alireza',
        '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/bavan',
        # '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/mehri',
        '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/mohammed',
        '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/mugunthan',
        '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/raj',
        '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/rikesh',
        '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/subal',
        '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/yang',
        # '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/bimal',
        # '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/sugirthan',
        # '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/user_study/Concord_M/kasthuri'
        ]

user_series = []

for dir_path in dirs:
    csv_dir = os.path.join(str(dir_path), 'easy')
    csv_path = os.path.expanduser(os.path.join(csv_dir, 'log.csv'))
    try:
        df = pd.read_csv(csv_path)
    except FileNotFoundError:
        print(f"CSV file not found in {csv_dir}, skipping.")
        continue
    time_elapsed = df['Time Elapsed (s)'].to_numpy()
    user_series.append(
        (
            time_elapsed,
            df['tb exploration'].to_numpy(),
            df['human exploration'].to_numpy(),
            df['merged exploration'].to_numpy(),
        )
    )

min_time = max(series[0][0] for series in user_series)
max_time = min(series[0][-1] for series in user_series)
base_time = user_series[0][0]
common_time = base_time[(base_time >= min_time) & (base_time <= max_time)]

tb_interp = []
human_interp = []
merged_interp = []
for time_elapsed, tb_exp, human_exp, merged_exp in user_series:
    tb_interp.append(np.interp(common_time, time_elapsed, tb_exp))
    human_interp.append(np.interp(common_time, time_elapsed, human_exp))
    merged_interp.append(np.interp(common_time, time_elapsed, merged_exp))

avg_tb_exploration = np.mean(tb_interp, axis=0)
avg_human_exploration = np.mean(human_interp, axis=0)
avg_merged_exploration = np.mean(merged_interp, axis=0)

plt.figure()
plt.plot(common_time, avg_tb_exploration, label='Robot only', linewidth=2.0)
plt.plot(common_time, avg_human_exploration, label='Human only', linestyle=':', linewidth=2.0)
plt.plot(common_time, avg_merged_exploration, label='Concord', linestyle='--', linewidth=2.0)
plt.xlabel('Time Elapsed (s)')
plt.ylabel('Explored Cells')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(base_dir, "exploration_over_time_avg_easy_SST<=6.png"), dpi=300, bbox_inches="tight")
