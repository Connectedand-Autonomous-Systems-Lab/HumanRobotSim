import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

BASE_DIR = '/home/mayooran/Documents/iros/src/DRL-exploration/unity_end/human_robot_pkg/logs/mobisys'
RUN_CONFIGS = [
    # ('setting1.csv', 'Potential=50 gain=0.05'),
    # ('setting2.csv', 'Potential=300 gain=0.05'),
    # ('setting3.csv', 'Potential=500 gain=0.05'),
    # ('setting4.csv', 'Potential=50 gain=0.5 '),
    # ('setting1copy.csv', 'Potential=50 gain=0.05'),
    ('setting5.csv', "Potential=50 gain=0.5"),
    ('setting6.csv', "Potential=50 gain=2.0"),
    ('setting7.csv', 'Potential=50 gain=0.05'),
]

TIME_COLUMN = 'Time Elapsed (s)'

EXPLORATION_METRICS = [
    ('tb exploration', 'TurtleBot Exploration', 'Explored Cells'),
    ('human exploration', 'Human Exploration', 'Explored Cells'),
    ('merged exploration', 'Merged Exploration', 'Explored Cells'),
]

TRAJECTORY_METRICS = [
    ('tb trajectory', 'TurtleBot Trajectory', 'Trajectory Length (m)'),
    ('human trajectory', 'Human Trajectory', 'Trajectory Length (m)'),
]


def load_runs():
    runs = []
    for filename, label in RUN_CONFIGS:
        csv_path = os.path.join(BASE_DIR, filename)
        df = pd.read_csv(csv_path)
        df.columns = df.columns.str.strip()
        runs.append({'label': label, 'data': df, 'path': csv_path})
    return runs

def format_y_axis_scientific(ax):
    formatter = ScalarFormatter(useMathText=True)
    formatter.set_scientific(True)
    formatter.set_powerlimits((0, 0))
    ax.yaxis.set_major_formatter(formatter)
    ax.ticklabel_format(axis='y', style='sci', scilimits=(0, 0))


def plot_metric(runs, metric_key, title, ylabel):
    plt.figure()
    line_styles=['-.',':','--']
    for i,run in enumerate(runs):
        df = run['data']
        plt.plot(df[TIME_COLUMN], df[metric_key], label=run['label'], linestyle=line_styles[i])
    plt.xlabel(TIME_COLUMN, fontsize=18)
    plt.ylabel('Explored Cells', fontsize= 18)
    plt.title(f'{title} Comparison')
    plt.grid(True)
    plt.tick_params(axis='both', labelsize=14)
    format_y_axis_scientific(plt.gca())
    plt.legend(fontsize=14)
    plt.tight_layout()


def main():
    runs = load_runs()

    for metric_key, title, ylabel in EXPLORATION_METRICS:
        plot_metric(runs, metric_key, title, ylabel)

    for metric_key, title, ylabel in TRAJECTORY_METRICS:
        plot_metric(runs, metric_key, title, ylabel)

    plt.show()


if __name__ == '__main__':
    main()
