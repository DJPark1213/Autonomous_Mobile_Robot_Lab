"""ROS 2 adapter for Lab 3: goal, wander and optional cluster-follow modes."""

import csv
from dataclasses import fields
from datetime import datetime, timezone
import math
from pathlib import Path
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .odometry import read_planar_pose, relative_pose
from .core import Config, process_scan
from .navigation import make_navigator
from .follow import Follower


class Lab3Controller(Node):
    """Circle-style absolute robot topics and local receipt watchdogs."""

    def __init__(self):
        super().__init__('lab3_controller')
        defaults = Config()
        values = {f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
                  for f in fields(Config)}
        self.cfg = Config(**values)
        self.mode = self.declare_parameter('mode', 'goal').value
        if self.mode not in ('goal', 'wander', 'follow'):
            raise ValueError('mode must be goal, wander or follow')
        self.robot_namespace = self.declare_parameter(
            'robot_namespace', Config.ROBOT_NAMESPACE).value
        prefix = self.robot_namespace
        if not prefix.startswith('/') or prefix == '/' or prefix.endswith('/'):
            raise ValueError('robot_namespace must be absolute without a trailing slash, e.g. /TTB10')
        self.odom_topic = prefix + '/odom'
        self.scan_topic = prefix + '/scan'
        self.cmd_topic = prefix + '/cmd_vel'
        self.diagnostic_prefix = prefix + '/lab3'
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.log_dir = self.declare_parameter('log_dir', '~/amr_lab3_runs').value
        self.pose = None
        self.origin = None
        self.odom_frame = None
        self.scan = None
        self.odom_received = None
        self.scan_received = None
        self.last_tick = None
        self.status = ''
        self.last_problem = ''
        self.sensor_problems = {'odom': 'waiting for first valid message',
                                'scan': 'waiting for first valid scan and scan-to-base TF'}
        self.last_status_detail = ''
        self.last_status_log = -math.inf
        self.fatal = ''
        self.nav = make_navigator(self.cfg)
        self.follower = Follower(self.cfg)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_publisher = self.create_publisher(String, self.diagnostic_prefix+'/status', 10)
        # Use the same sensor QoS as the working Lab 2 circle node.
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.reset_service = self.create_service(
            Trigger, self.diagnostic_prefix+'/reset', self.reset_callback)
        self.stop_service = self.create_service(
            Trigger, self.diagnostic_prefix+'/stop', self.stop_callback)
        self.log_file = None
        self.writer = None
        if self.log_dir:
            folder = Path(self.log_dir).expanduser()
            folder.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            name = f'{self.mode}_{stamp}_{uuid.uuid4().hex[:6]}.csv'
            self.log_file = (folder / name).open('x', newline='', buffering=1)
            self.writer = csv.writer(self.log_file)
            self.writer.writerow(('elapsed_s', 'mode', 'state', 'x_m', 'y_m', 'yaw_rad',
                                  'goal_x_m', 'goal_y_m', 'goal_error_m', 'v_m_s', 'w_rad_s',
                                  'F', 'FL', 'L', 'BL', 'B', 'BR', 'R', 'FR',
                                  'behavior', 'obstacle_distance_m', 'wall_entry_goal_distance_m',
                                  'goal_normal_dot', 'goal_corridor_clear', 'planner',
                                  'field_x', 'field_y', 'repulsion_magnitude', 'tangent_weight',
                                  'fallback_count', 'fallback_reason'))
            self.get_logger().info(f'CSV: {folder / name}')
        self.started_at = time.monotonic()
        self.timer = self.create_timer(self.cfg.control_period, self.tick)
        self.get_logger().info(
            f'Lab3 mode={self.mode}, planner={self.cfg.planner}; '
            f'odom={self.odom_topic}, scan={self.scan_topic}, cmd_vel={self.cmd_topic}. '
            'Waiting for odometry and laser data; freshness uses local receipt time.')

    def sensor_problem(self, kind, reason):
        """Retain independent sensor errors so one stream cannot hide the other."""
        self.sensor_problems[kind] = reason
        self.last_problem = f'{kind}: {reason}'

    def odom_callback(self, message):
        """Capture the initial pose once and retain its coordinate frame."""
        try:
            raw = read_planar_pose(message)
        except ValueError as error:
            self.odom_received = None
            self.sensor_problem('odom', str(error))
            return
        now = time.monotonic()
        frame = message.header.frame_id
        if not frame:
            self.odom_received = None
            self.sensor_problem('odom', 'empty odometry frame')
            return
        if self.origin is None:
            self.origin, self.odom_frame = raw, frame
            self.get_logger().info('Starting pose captured; fixed goal origin is ready.')
        if frame != self.odom_frame:
            self.fatal = 'ODOM_FRAME_CHANGED_RESET_REQUIRED'
            self.odom_received = None
            return
        new_pose = relative_pose(*raw, *self.origin)
        if self.pose is not None and self.odom_received is not None:
            elapsed = now - self.odom_received
            if (elapsed < 1.0 and (math.dist(new_pose[:2], self.pose[:2]) > .15 + .6*elapsed
                                  or abs(math.atan2(math.sin(new_pose[2]-self.pose[2]),
                                                    math.cos(new_pose[2]-self.pose[2]))) > .4 + 2*elapsed)):
                self.fatal = 'ODOM_JUMP_RESET_REQUIRED'
                self.odom_received = None
                return
        self.pose = new_pose
        self.odom_received = now
        self.sensor_problems['odom'] = ''

    def scan_callback(self, message):
        """Transform laser hits to base_link; missing TF never assumes alignment."""
        self.scan_received = None
        try:
            if not message.header.frame_id:
                raise ValueError('empty scan frame')
            if message.header.frame_id == self.base_frame:
                transform = (0., 0., 0.)
                angle_sign = 1.
            else:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, message.header.frame_id, Time.from_msg(message.header.stamp))
                t, q = tf.transform.translation, tf.transform.rotation
                norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
                if not math.isfinite(norm) or norm < 1e-9:
                    raise ValueError('invalid laser TF quaternion')
                x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
                zz = 1 - 2*(x*x + y*y)
                if abs(zz) < .995:
                    raise ValueError('laser scan plane is not horizontal')
                yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
                transform = (t.x, t.y, yaw)
                angle_sign = 1. if zz > 0 else -1.
            scan = process_scan(message.ranges, message.angle_min, message.angle_increment,
                                message.range_min, message.range_max, transform,
                                self.cfg.allow_positive_inf, angle_sign=angle_sign)
        except (TransformException, ValueError) as error:
            self.sensor_problem('scan', f'TF/geometry: {error}')
            return
        first_valid = self.scan is None or not self.scan.healthy
        self.scan = scan
        if not scan.healthy:
            self.sensor_problem('scan', scan.reason)
            return
        self.scan_received = time.monotonic()
        self.sensor_problems['scan'] = ''
        if first_valid:
            self.get_logger().info(
                f'First valid laser scan received: {message.header.frame_id} -> {self.base_frame}.')
        if self.mode == 'follow' and self.fresh(self.odom_received, self.scan_received):
            self.follower.observe(scan, self.pose, self.scan_received)

    def fresh(self, received, now):
        """Check a monotonic receipt timestamp."""
        return received is not None and 0 <= now - received <= self.cfg.sensor_timeout

    def set_status(self, status):
        """Publish the reason; log changed faults even while still stopped."""
        detail = self.last_problem if status == 'SENSOR_STOP' else ''
        rendered = f'{status} ({detail})' if detail else status
        self.status_publisher.publish(String(data=rendered))
        now = time.monotonic()
        if (status != self.status or
                (detail != self.last_status_detail and now-self.last_status_log >= 1.)):
            self.get_logger().info(rendered)
            self.last_status_detail = detail
            self.last_status_log = now
        self.status = status

    def publish(self, v, w):
        """Send a command and record exactly the sent values."""
        command = Twist()
        command.linear.x, command.angular.z = float(v), float(w)
        self.publisher.publish(command)
        if self.writer:
            pose = self.pose or (math.nan, math.nan, math.nan)
            distance = math.hypot(self.cfg.goal_x-pose[0], self.cfg.goal_y-pose[1])
            sectors = self.scan.sectors if self.scan else {}
            self.writer.writerow((time.monotonic()-self.started_at, self.mode, self.status, *pose,
                                  self.cfg.goal_x, self.cfg.goal_y,
                                  distance if self.mode == 'goal' else math.nan, v, w,
                                  *(sectors.get(k, math.nan) for k in
                                    ('F', 'FL', 'L', 'BL', 'B', 'BR', 'R', 'FR')),
                                  self.nav.behavior if self.mode != 'follow' else self.follower.state,
                                  *(self.nav.diagnostics.get(k, math.nan) for k in
                                    ('obstacle_distance_m', 'wall_entry_goal_distance_m',
                                     'goal_normal_dot', 'goal_corridor_clear')),
                                  self.cfg.planner,
                                  *(self.nav.diagnostics.get(k, math.nan) for k in
                                    ('field_x', 'field_y', 'repulsion_magnitude', 'tangent_weight',
                                     'fallback_count', 'fallback_reason'))))

    def tick(self):
        """One timer owns normal commands; any missing sensor commands zero."""
        now = time.monotonic()
        dt = self.cfg.control_period if self.last_tick is None else now-self.last_tick
        self.last_tick = now
        if self.fatal:
            self.set_status(self.fatal)
            self.publish(0., 0.)
            return
        problems = []
        for kind, received in (('odom', self.odom_received), ('scan', self.scan_received)):
            if not self.fresh(received, now):
                reason = self.sensor_problems[kind]
                if not reason:
                    reason = f'no new valid data for >{self.cfg.sensor_timeout:g}s'
                problems.append(f'{kind}: {reason}')
        if dt <= 0 or dt > self.cfg.sensor_timeout:
            problems.append('control loop delayed or invalid interval')
        if problems:
            self.last_problem = '; '.join(problems)
            self.nav.pause()
            self.follower.previous_v = 0.
            self.set_status('SENSOR_STOP')
            self.publish(0., 0.)
            return
        if self.mode == 'follow':
            v, w = self.follower.step(self.pose, self.scan, now, dt)
            self.set_status(self.follower.state)
        else:
            v, w = self.nav.step(self.pose, self.scan, now, dt, self.mode)
            self.set_status(self.nav.state)
        self.publish(v, w)

    def stop_callback(self, request, response):
        """Latch a software stop until an explicit reset service call."""
        self.fatal = 'USER_STOP_RESET_REQUIRED'
        self.publish(0., 0.)
        response.success, response.message = True, 'Stopped; call lab3/reset to restart.'
        return response

    def reset_callback(self, request, response):
        """Stop, then capture a NEW start pose on the next odometry message."""
        self.publish(0., 0.)
        self.origin = self.pose = self.scan = None
        self.odom_received = self.scan_received = None
        self.fatal = self.last_problem = ''
        self.sensor_problems = {'odom': 'waiting for first valid message',
                                'scan': 'waiting for first valid scan and scan-to-base TF'}
        self.nav, self.follower = make_navigator(self.cfg), Follower(self.cfg)
        self.last_tick = None
        response.success = True
        response.message = 'Reset; next fresh odometry defines a NEW goal origin / target acquisition.'
        return response

    def close(self):
        """Stop motion on shutdown and close the CSV."""
        self.timer.cancel()
        self.publish(0., 0.)
        if self.log_file:
            self.log_file.close()


def main(args=None):
    """Run until interrupted, publishing zero on normal shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Lab3Controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
