"""Reproducible ideal 2-D ray-cast tests, explicitly not robot measurements."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from .core import Config, process_scan, wrap
from .navigation import make_navigator


SCENARIOS = {
    'clear': [],
    'center_box': [(.65, -.20, 1.0, .20)],
    'left_box': [(.65, -.03, 1.0, .40)],
    'right_box': [(.65, -.40, 1.0, .03)],
    'wide_box': [(.65, -.40, 1.0, .40)],
}


def raycast(pose, boxes, rays=360):
    """Intersect rays against rectangles and enclosing room walls."""
    x, y, yaw = pose
    angles = np.linspace(-math.pi, math.pi, rays, endpoint=False)
    dx, dy = np.cos(angles + yaw), np.sin(angles + yaw)
    dx = np.where(np.abs(dx) < 1e-12, 1e-12, dx)
    dy = np.where(np.abs(dy) < 1e-12, 1e-12, dy)
    result = np.full(rays, math.inf)
    for xmin, ymin, xmax, ymax in [(-4, -4, 4, 4), *boxes]:
        tx1, tx2 = (xmin - x) / dx, (xmax - x) / dx
        ty1, ty2 = (ymin - y) / dy, (ymax - y) / dy
        near = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
        far = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
        hit = np.where(near > 0, near, far)
        hit = np.where((far >= near) & (far > 0), hit, math.inf)
        result = np.minimum(result, hit)
    return result, angles[0], 2 * math.pi / rays


def clearance(pose, boxes):
    """Exact robot-centre clearance to the rectangles, independent of lidar."""
    x, y, _ = pose
    values = [4 - abs(x), 4 - abs(y)]
    for xmin, ymin, xmax, ymax in boxes:
        values.append(math.hypot(max(xmin - x, 0, x - xmax),
                                 max(ymin - y, 0, y - ymax)))
    return min(values)


def run_scenario(name, boxes, config=None, seconds=120.0, scan_noise=0.0,
                 scan_period=.10, seed=42):
    """Integrate the actual planner using ideal unicycle kinematics."""
    config = config or Config()
    control = make_navigator(config)
    pose = np.zeros(3)
    history = []
    min_clearance = math.inf
    dt = config.control_period
    rng = np.random.default_rng(seed)
    next_scan = 0.0
    scan = None
    for tick in range(int(seconds / dt)):
        if scan is None or tick*dt >= next_scan-1e-9:
            ranges, angle_min, increment = raycast(pose, boxes)
            if scan_noise:
                ranges += rng.normal(0, scan_noise, len(ranges))
            scan = process_scan(ranges, angle_min, increment, .05, 12.)
            next_scan += scan_period
        v, w = control.step(pose, scan, tick * dt, dt)
        history.append((tick * dt, *pose, v, w, control.state))
        mid = pose[2] + w * dt / 2
        pose[0] += v * math.cos(mid) * dt
        pose[1] += v * math.sin(mid) * dt
        pose[2] = wrap(pose[2] + w * dt)
        min_clearance = min(min_clearance, clearance(pose, boxes))
        if control.done or control.failure:
            break
    return {
        'scenario': name, 'kind': 'ideal_2d_simulation_not_robot_data',
        'algorithm': config.planner,
        'fallback_count': getattr(control, 'fallback_count', 0),
        'scan_period_s': scan_period, 'scan_noise_std_m': scan_noise, 'seed': seed,
        'reached': control.done, 'state': control.state,
        'elapsed_s': history[-1][0],
        'goal_error_m': math.hypot(pose[0] - config.goal_x, pose[1] - config.goal_y),
        'minimum_center_clearance_m': min_clearance,
        'minimum_body_clearance_m': min_clearance - config.robot_radius,
        'path_length_m': sum(row[4] * dt for row in history),
        'behavior_sequence': [row[6] for i, row in enumerate(history)
                              if i == 0 or row[6] != history[i-1][6]],
    }, history


def main(args=None):
    """Write simulated trajectories, metrics and a labelled figure."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='lab3_simulation')
    parser.add_argument('--planner', choices=('potential', 'lecture'), default='potential')
    opts = parser.parse_args(args)
    output = Path(opts.output)
    output.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    fig, axes = plt.subplots(1, len(SCENARIOS), figsize=(18, 4), constrained_layout=True)
    metrics = []
    for ax, (name, boxes) in zip(axes, SCENARIOS.items()):
        metric, rows = run_scenario(name, boxes, Config(planner=opts.planner))
        metrics.append(metric)
        with (output / f'{name}.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('time_s', 'x_m', 'y_m', 'yaw_rad', 'v_m_s', 'w_rad_s', 'state'))
            writer.writerows(rows)
        data = np.asarray([r[:6] for r in rows])
        ax.plot(data[:, 1], data[:, 2], color='#167d9a', lw=2)
        ax.add_patch(Circle((0, 0), .19, fill=False, color='#167d9a'))
        for x1, y1, x2, y2 in boxes:
            ax.add_patch(Rectangle((x1, y1), x2-x1, y2-y1, color='#ac794d'))
        ax.scatter([1.5], [0], marker='*', color='#e37631', s=120)
        ax.set(title=f'{name}\n{metric["state"]} | {metric["elapsed_s"]:.1f}s',
               xlabel='x (m)', ylabel='y (m)', xlim=(-.3, 1.9), ylim=(-1.1, 1.1))
        ax.set_aspect('equal')
        ax.grid(alpha=.2)
    fig.suptitle(f'Lab 3 | {opts.planner} | Ideal 2-D simulation, NOT robot data', fontsize=14)
    fig.savefig(output / 'simulation.png', dpi=170)
    plt.close(fig)
    (output / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
    print(json.dumps(metrics, indent=2))
    if not all(m['reached'] and m['minimum_body_clearance_m'] > 0 for m in metrics):
        raise SystemExit('A navigation scenario failed')


if __name__ == '__main__':
    main()
