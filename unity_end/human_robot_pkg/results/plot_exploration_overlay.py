import os

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.ticker import ScalarFormatter

TITLE_FONT_SIZE = 38
LABEL_FONT_SIZE = 30
TICK_FONT_SIZE = 24
LEGEND_FONT_SIZE = 24


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


time_col = 'Time Elapsed (s)'

uncoordinated_df = load_data('uncoordinated')
artifact_0_df = load_data('Artifact/0')
artifact_1000_df = load_data('Artifact/1000')

fig, ax = plt.subplots(figsize=(16, 10))

ax.plot(
    uncoordinated_df[time_col],
    uncoordinated_df['human exploration'],
    label='Human only',
    linestyle=':',
    linewidth=2.5,
)
ax.plot(
    artifact_0_df[time_col],
    artifact_0_df['merged exploration'],
    label='w3=0 (Concord-Complementary)',
    linewidth=2.5,
)
ax.plot(
    artifact_1000_df[time_col],
    artifact_1000_df['merged exploration'],
    label='w3=1000 (Concord-Supplementary)',
    linewidth=2.5,
)

ax.set_xlabel('Time Elapsed (s)')
ax.set_ylabel('Explored Cells')
ax.set_title('Exploration Over Time', fontsize=TITLE_FONT_SIZE)
ax.xaxis.label.set_size(LABEL_FONT_SIZE)
ax.yaxis.label.set_size(LABEL_FONT_SIZE)
ax.grid(True)
ax.tick_params(axis='both', labelsize=TICK_FONT_SIZE)
format_y_axis_scientific(ax)
ax.legend(fontsize=LEGEND_FONT_SIZE)

plt.tight_layout()
plt.show()
