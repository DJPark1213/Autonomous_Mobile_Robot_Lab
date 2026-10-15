"""Record odometry or plot a circle run, independently of the shared plot script."""

import argparse
import csv
import json
import math
from pathlib import Path
import time

from py_amr_ttb.lab2.circle_config import CircleConfig as C
from py_amr_ttb.lab2.pure_pursuit import CirclePath, CircleProgress
from py_amr_ttb.lab2.circle_odom import read_planar_pose, relative_pose


def record(args):
    """Record measured odometry while keeping a fixed stationary origin."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    args.record.parent.mkdir(parents=True, exist_ok=True)
    with args.record.open('x', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('elapsed_s', 'odom_x_m', 'odom_y_m', 'odom_yaw_rad',
                         'x_m', 'y_m', 'yaw_rad'))
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Node('circle_trajectory_recorder')
        origin = None
        count = 0
        started = time.monotonic()

        def on_odom(message):
            nonlocal origin, count
            try:
                pose = read_planar_pose(message)
            except ValueError as error:
                node.get_logger().warning(str(error))
                return
            if origin is None:
                origin = pose
                node.get_logger().info('First valid odometry received. Ready to start controller.')
            writer.writerow((time.monotonic() - started, *pose, *relative_pose(*pose, *origin)))
            stream.flush()
            count += 1

        subscription = node.create_subscription(
            Odometry, args.topic, on_odom, qos_profile_sensor_data)
        node.get_logger().info('Recording; keep robot still until controller starts.')
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.1)
                if args.duration is not None and time.monotonic() - started >= args.duration:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        print(f'Saved {count} measured poses to {args.record}')
        if not count:
            raise ValueError('No valid odometry received; CSV has no trajectory')


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
