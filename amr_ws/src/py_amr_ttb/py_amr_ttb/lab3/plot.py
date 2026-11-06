"""Plot Lab 3 measured and commanded motion from hardware or simulation CSV."""

import argparse
from pathlib import Path
import numpy as np


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path)
    options = parser.parse_args(args)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data = np.atleast_1d(np.genfromtxt(options.csv, delimiter=',', names=True, dtype=None, encoding='utf-8'))
    figure, axes = plt.subplots(1, 3, figsize=(15, 4))
    axes[0].plot(data['x'], data['y'])
    axes[0].set(xlabel='x (m)', ylabel='y (m)', title='Trajectory', aspect='equal')
    t = data['time']-data['time'][0]
    for axis, name, unit in ((axes[1], 'v', 'm/s'), (axes[2], 'w', 'rad/s')):
        axis.plot(t, data['measured_'+name], label='measured')
        axis.plot(t, data['command_'+name], label='command', alpha=.7)
        axis.set(xlabel='time (s)', ylabel=unit)
        axis.legend()
    figure.tight_layout()
    destination = options.output or options.csv.with_suffix('.png')
    figure.savefig(destination, dpi=150)
    plt.close(figure)
    print(destination)
