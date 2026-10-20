"""Record real odometry as CSV, or plot a recorded CSV against a Lab 2 route."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time

# Also works directly from the checkout before colcon install.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/py_amr_ttb'))
from py_amr_ttb.lab_config import LabConfig as C  # noqa: E402
from py_amr_ttb.lab2.navigation import heart_points, star_waypoints, square_waypoints  # noqa: E402
from py_amr_ttb.odom_utils import read_planar_pose, relative_pose  # noqa: E402


def record(args):
    """Capture valid measured poses; terminate with Ctrl+C or --duration."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    args.record.parent.mkdir(parents=True, exist_ok=True)
    # Refuse to overwrite an earlier measured run.
    with args.record.open('x', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('elapsed_s', 'odom_x_m', 'odom_y_m', 'odom_yaw_rad',
                         'x_m', 'y_m', 'yaw_rad'))
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Node('lab2_trajectory_recorder')
        origin = None
        started = time.monotonic()
        count = 0

        def on_odom(message):
            nonlocal origin, count
            try:
                pose = read_planar_pose(message)
            except ValueError as error:
                node.get_logger().warning(str(error))
                return
            if origin is None:
                origin = pose
            local = relative_pose(*pose, *origin)
            writer.writerow((time.monotonic() - started, *pose, *local))
            stream.flush()
            count += 1

        subscription = node.create_subscription(
            Odometry, args.topic, on_odom, qos_profile_sensor_data,
        )
        node.get_logger().info('Recording odometry; keep robot still until controller starts.')
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
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
    """Plot actual measured points without creating synthetic robot results."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    with args.csv.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError('At least two recorded odometry poses are required')
    points = [(float(r['x_m']), float(r['y_m'])) for r in rows]
    times = [float(r['elapsed_s']) for r in rows]
    if not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('Trajectory contains non-finite coordinates')
    if not all(math.isfinite(v) for v in times) or any(b < a for a, b in zip(times, times[1:])):
        raise ValueError('Recorded times must be finite and nondecreasing')
    route = {
        'goal': [(C.GOAL_X_OFFSET, C.GOAL_Y_OFFSET)],
        'square': square_waypoints(C.SQUARE_SIDE_LENGTH),
        'star': star_waypoints(C.STAR_RADIUS),
        'heart': heart_points(args.heart_width),
    }[args.route]
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(*zip(*([(0, 0)] + route)), '--',
            marker=None if args.route == 'heart' else 'o', label='Intended route')
    ax.plot(*zip(*points), label='Measured odometry')
    ax.scatter(*points[0], marker='s', label='Measured start')
    ax.scatter(*points[-1], marker='x', label='Measured end')
    ax.set(xlabel='Starting-frame x (m, forward)', ylabel='Starting-frame y (m, left)',
           title=f'Lab 2 {args.route}: measured trajectory')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists() or args.output.with_suffix('.json').exists():
        raise FileExistsError('Choose a new output name to preserve previous results')
    fig.savefig(args.output, dpi=180)
    plt.close(fig)
    stats = {
        'source_csv': str(args.csv.resolve()), 'samples': len(points),
        'route': args.route,
        'duration_s': times[-1] - times[0],
        'measured_path_length_m': sum(math.dist(a, b) for a, b in zip(points, points[1:])),
        'final_distance_to_last_waypoint_m': math.dist(points[-1], route[-1]),
        'note': 'Final distance alone does not prove that every waypoint was visited.',
    }
    if args.route == 'heart':
        stats['heart_width_m'] = args.heart_width
    args.output.with_suffix('.json').write_text(json.dumps(stats, indent=2) + '\n')
    print(f'Saved {args.output} and {args.output.with_suffix(".json")}')


def main():
    """Parse a mutually exclusive recording or offline plotting operation."""
    # Preserve paths containing non-ASCII characters in Windows terminals.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, 'reconfigure'):
            stream.reconfigure(encoding='utf-8', errors='replace')
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--record', type=Path, help='New CSV path; requires ROS 2')
    source.add_argument('--csv', type=Path, help='Existing measured CSV to plot')
    parser.add_argument('--topic', default=C.ODOM_TOPIC)
    parser.add_argument('--duration', type=float, help='Recording seconds; default until Ctrl+C')
    parser.add_argument('--route', choices=('goal', 'square', 'star', 'heart'), default='goal')
    parser.add_argument('--heart-width', type=float, default=C.HEART_WIDTH,
                        help='Width used by the heart controller, meters')
    parser.add_argument('--output', type=Path, default=Path('output/lab2/trajectory.png'))
    args = parser.parse_args()
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be finite and positive')
    if not math.isfinite(args.heart_width) or args.heart_width <= 0:
        parser.error('--heart-width must be finite and positive')
    try:
        record(args) if args.record else plot(args)
    except (ValueError, OSError, KeyError) as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    main()
