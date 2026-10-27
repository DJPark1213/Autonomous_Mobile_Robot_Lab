"""Record odometry or plot a circle run, independently of the shared plot script."""

import argparse
import csv
import json
import math
from pathlib import Path
import time

from py_amr_ttb.lab2.circle_config import CircleConfig as C
from py_amr_ttb.lab2.pure_pursuit import CIRCLE_FIELDS, CirclePath, CircleProgress
from py_amr_ttb.lab2.circle_odom import read_planar_pose, relative_pose


def record(args):
    """Preserve --record CLI; write paired target/robot samples from circle."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String

    if not args.topic.endswith('/odom'):
        raise ValueError('--topic must be the robot odometry topic, e.g. /TTB10/odom')
    sample_topic = args.topic[:-len('/odom')] + '/lab2_circle/sample'
    args.record.parent.mkdir(parents=True, exist_ok=True)
    with args.record.open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=CIRCLE_FIELDS)
        writer.writeheader()
        stream.flush()
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Node('circle_trajectory_recorder')
        ready = False
        count = 0
        started = time.monotonic()

        def on_odom(message):
            nonlocal ready
            if ready:
                return
            try:
                read_planar_pose(message)
            except ValueError:
                return
            ready = True
            node.get_logger().info(
                'First valid odometry received. Ready to start controller.')

        def on_sample(message):
            nonlocal count
            try:
                sample = json.loads(message.data)
                if not isinstance(sample, dict) or any(key not in sample for key in CIRCLE_FIELDS):
                    raise ValueError('Sample schema mismatch; replace all five circle files')
                row = {key: float('nan') if sample[key] is None else sample[key]
                       for key in CIRCLE_FIELDS}
            except (ValueError, TypeError) as error:
                node.get_logger().warning(str(error))
                return
            writer.writerow(row)
            stream.flush()
            count += 1

        odom_subscription = node.create_subscription(
            Odometry, args.topic, on_odom, qos_profile_sensor_data)
        sample_subscription = node.create_subscription(
            String, sample_topic, on_sample, 10)
        node.get_logger().info(
            f'Recorder ready. Waiting for {sample_topic}; keep robot still, '
            'then start lab2_circle in another terminal.')
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.1)
                if args.duration is not None and time.monotonic() - started >= args.duration:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_subscription(odom_subscription)
            node.destroy_subscription(sample_subscription)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        print(f'Saved {count} paired target/robot samples to {args.record}')
        if not count:
            raise ValueError('No paired circle samples received; CSV has no trial data')


def plot_moving_target(args, rows):
    """Use recorded targets, including live speed changes; preserve old CLI."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    stop = next((i + 1 for i, row in enumerate(rows) if float(row['done'])), len(rows))
    rows = rows[:stop]
    if len(rows) < 2:
        raise ValueError('At least two trial samples are required')
    def values(key):
        return [float(row[key]) for row in rows]
    times = values('elapsed_s')
    if any(not math.isfinite(t) for t in times) or any(
            b < a for a, b in zip(times, times[1:])):
        raise ValueError('Recorded times must be finite and nondecreasing')
    if any(not math.isfinite(float(row[field])) for row in rows for field in
           ('target_x_m', 'target_y_m', 'target_phase_rad', 'radius_m')):
        raise ValueError('Non-finite target trajectory')
    valid = [row for row in rows if float(row['odom_fresh'])
             and math.isfinite(float(row['x_m'])) and math.isfinite(float(row['y_m']))]
    if not valid:
        raise ValueError('No fresh robot poses in the trial')
    if any(not math.isfinite(float(row[key])) for row in valid for key in
           ('position_error_m', 'radial_error_m', 'robot_phase_rad', 'phase_gap_rad')):
        raise ValueError('Invalid fresh-pose errors')
    if args.output.exists() or args.output.with_suffix('.json').exists():
        raise FileExistsError('Choose a new output name to preserve previous results')
    path = CirclePath(float(rows[0]['radius_m']), int(float(rows[0]['direction'])))
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    xy, errors, progress, speeds = axes.flat
    route = [path.point(i * math.tau / 360) for i in range(361)]
    xy.plot(*zip(*route), '--', color='0.6', label='Reference circle')
    xy.plot(values('target_x_m'), values('target_y_m'), label='Moving target', alpha=.8)
    xy.plot(values('x_m'), values('y_m'), label='Robot odometry')
    for i in sorted({0, len(valid) // 3, 2 * len(valid) // 3, len(valid) - 1}):
        row = valid[i]
        xy.plot([float(row['x_m']), float(row['target_x_m'])],
                [float(row['y_m']), float(row['target_y_m'])],
                color='0.5', lw=.8, alpha=.5)
    xy.set(xlabel='Starting-frame x (m)', ylabel='Starting-frame y (m)',
           title='Trajectories; gray lines connect equal-time samples')
    xy.set_aspect('equal', adjustable='box')
    errors.plot(times, values('position_error_m'), label='Distance to moving target')
    errors.plot(times, [abs(x) for x in values('radial_error_m')], label='Absolute radial error')
    errors.set(xlabel='Time (s)', ylabel='Error (m)', title='Tracking error')
    progress.plot(times, [x / math.tau for x in values('target_phase_rad')],
                  label='Target phase / 2pi')
    progress.plot(times, [x / math.tau for x in values('robot_phase_rad')],
                  label='Robot phase / 2pi')
    progress.set(xlabel='Time (s)', ylabel='Unwrapped revolutions',
                 title='Progress around circle center, including initial lead')
    for field, label, style in (
            ('target_speed_mps', 'Target speed', '--'),
            ('robot_speed_mps', 'Robot speed request', ':'),
            ('linear_command_mps', 'Robot command', '-'),
            ('measured_linear_mps', 'Robot odom speed', '-')):
        speeds.plot(times, values(field), style, label=label, alpha=.8)
    speeds.set(xlabel='Time (s)', ylabel='Speed (m/s)', title='Speed changes and response')
    for ax in axes.flat:
        ax.grid(True, alpha=.3)
        ax.legend(fontsize=8)
    simulated = rows[0].get('data_source') == 'ideal_simulation'
    fig.suptitle('IDEAL SIMULATION — NOT ROBOT DATA' if simulated
                 else 'Circle: time-driven target and recorded robot odometry')
    fig.tight_layout(rect=(0, 0, 1, .96))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=170)
    plt.close(fig)
    points = [(float(row['x_m']), float(row['y_m'])) for row in valid]
    radial = [float(row['radial_error_m']) for row in valid]
    distance = [float(row['position_error_m']) for row in valid]
    stats = {
        'source_csv': str(args.csv.resolve()), 'samples': len(rows),
        'fresh_samples': len(valid), 'duration_s': times[-1] - times[0],
        'radius_m': path.radius, 'direction': path.direction,
        'closure_error_m': math.hypot(*points[-1]),
        'radial_rmse_m': math.sqrt(sum(e * e for e in radial) / len(radial)),
        'radial_max_abs_error_m': max(map(abs, radial)),
        'directed_laps_from_odometry': float(valid[-1]['robot_phase_rad']) / math.tau,
        'target_laps': float(rows[-1]['target_laps']),
        'position_error_rmse_m': math.sqrt(sum(e * e for e in distance) / len(distance)),
        'position_error_max_m': max(distance),
        'final_phase_gap_rad': float(valid[-1]['phase_gap_rad']),
        'completed': bool(float(rows[-1]['done'])),
        'target_speed_range_mps': [min(values('target_speed_mps')), max(values('target_speed_mps'))],
        'data_source': rows[0].get('data_source', 'robot_odometry'),
        'note': 'Odom-relative data, not external ground truth. Target is sampled at control '
                'time; robot pose is the latest received measurement. See odom_age_s. '
                'NaN gaps represent stale poses. Geometric winding is not traveled '
                'distance along the circle. Initial lead gives nonzero initial error.',
    }
    args.output.with_suffix('.json').write_text(
        json.dumps(stats, indent=2) + '\n', encoding='utf-8')
    print(f'Saved {args.output} and {args.output.with_suffix(".json")}')


def plot(args):
    """Overlay an actual CSV on the selected starting-frame circle."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    path = CirclePath(args.radius, args.direction)
    with args.csv.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError('At least two recorded odometry poses are required')
    if 'target_x_m' in rows[0]:
        return plot_moving_target(args, rows)
    points = [(float(row['x_m']), float(row['y_m'])) for row in rows]
    times = [float(row['elapsed_s']) for row in rows]
    if not all(math.isfinite(value) for point in points for value in point):
        raise ValueError('Trajectory contains non-finite coordinates')
    if not all(math.isfinite(value) for value in times) or any(
            later < earlier for earlier, later in zip(times, times[1:])):
        raise ValueError('Recorded times must be finite and nondecreasing')
    if args.output.exists() or args.output.with_suffix('.json').exists():
        raise FileExistsError('Choose a new output name to preserve previous results')
    route = [path.point(i * math.tau / 360) for i in range(361)]
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(*zip(*route), '--', label='Intended circle')
    ax.plot(*zip(*points), label='Measured odometry')
    ax.scatter(*points[0], marker='s', label='Measured start')
    ax.scatter(*points[-1], marker='x', label='Measured end')
    ax.set(xlabel='Starting-frame x (m, forward)', ylabel='Starting-frame y (m, left)',
           title='Lab 2 circle: measured trajectory')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    plt.close(fig)
    errors = [path.radial_error(*point) for point in points]
    progress = CircleProgress(path)
    for point in points:
        progress.update(*point)
    stats = {
        'source_csv': str(args.csv.resolve()), 'samples': len(points),
        'duration_s': times[-1] - times[0], 'radius_m': path.radius,
        'direction': path.direction,
        'measured_path_length_m': sum(math.dist(a, b) for a, b in zip(points, points[1:])),
        'closure_error_m': math.hypot(*points[-1]),
        'radial_rmse_m': math.sqrt(sum(error ** 2 for error in errors) / len(errors)),
        'radial_max_abs_error_m': max(map(abs, errors)),
        'directed_laps_from_odometry': progress.radians / math.tau,
        'note': 'Odom-relative errors, not independent ground truth. '
                'Unwrapping assumes consecutive poses advance less than pi radians.',
    }
    args.output.with_suffix('.json').write_text(json.dumps(stats, indent=2) + '\n')
    print(f'Saved {args.output} and {args.output.with_suffix(".json")}')


def main():
    """Run either recording or offline plotting."""
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--record', type=Path)
    source.add_argument('--csv', type=Path)
    parser.add_argument('--topic', default=C.ODOM_TOPIC)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--radius', type=float, default=C.CIRCLE_RADIUS)
    parser.add_argument('--direction', type=int, choices=(-1, 1), default=C.CIRCLE_DIRECTION)
    parser.add_argument('--output', type=Path, default=Path('lab_runs/circle/run01.png'))
    args = parser.parse_args()
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be finite and positive')
    try:
        record(args) if args.record else plot(args)
    except (ValueError, OSError, KeyError) as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    main()
