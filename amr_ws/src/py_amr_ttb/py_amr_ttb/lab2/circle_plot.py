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
