"""Plot an actual Lab 3 node CSV without relabelling simulated data as measured."""

import argparse
import csv
from pathlib import Path

import numpy as np


def main(args=None):
    """Save a trajectory, command and front-clearance figure from a run CSV."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_file')
    parser.add_argument('--output', default=None)
    opts = parser.parse_args(args)
    path = Path(opts.csv_file)
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        parser.error('CSV contains no samples')
    required = ('elapsed_s', 'x_m', 'y_m', 'v_m_s', 'w_rad_s', 'F', 'goal_x_m', 'goal_y_m')
    if any(key not in rows[0] for key in required):
        parser.error('Expected a CSV produced by lab3_controller')
    data = {key: np.asarray([float(r[key]) for r in rows]) for key in required}
    if not np.any(np.isfinite(data['x_m'])):
        parser.error('No valid odometry in this run; inspect sensor/TF status')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    axes[0].plot(data['x_m'], data['y_m'], label='Odometry')
    if rows[0].get('mode') == 'goal':
        axes[0].scatter([data['goal_x_m'][-1]], [data['goal_y_m'][-1]], marker='*', s=110, label='Goal')
    axes[0].set(xlabel='x (m)', ylabel='y (m)', title='Fixed starting frame')
    axes[0].axis('equal')
    axes[0].legend()
    axes[1].plot(data['elapsed_s'], data['v_m_s'], label='v (m/s)')
    axes[1].plot(data['elapsed_s'], data['w_rad_s'], label='w (rad/s)')
    axes[1].set(xlabel='Time (s)', title='Published commands')
    axes[1].legend()
    axes[2].plot(data['elapsed_s'], data['F'])
    axes[2].set(xlabel='Time (s)', ylabel='Range (m)', title='Front sector minimum')
    for ax in axes:
        ax.grid(alpha=.2)
    fig.suptitle(f'{path.name} | final state: {rows[-1]["state"]}')
    output = Path(opts.output) if opts.output else path.with_suffix('.png')
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)
    print(output)


if __name__ == '__main__':
    main()
