"""Circle: independent target, vector-field steering and phase speed control."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64, String

from py_amr_ttb.lab2.circle_config import CircleConfig
from py_amr_ttb.lab2.circle_odom import (
    odometry_is_fresh, read_planar_pose, relative_pose,
)
from py_amr_ttb.lab2.pure_pursuit import (
    CircleProgress, TimedCircleTarget, circle_tracking_command,
)


class Lab2Circle(Node):
    """Use time for target motion; odometry updates only robot state."""

    def __init__(self):
        super().__init__('lab2_circle')
        self.config = CircleConfig
        defaults = {
            'radius': self.config.CIRCLE_RADIUS,
<<<<<<< HEAD
=======
            'speed': self.config.CIRCLE_SPEED,
>>>>>>> 77d1cce42b71b4269be683b4b912eafb5673ea7d
            'target_speed': self.config.CIRCLE_TARGET_SPEED,
            'lookahead': self.config.CIRCLE_LOOKAHEAD,
            'laps': self.config.CIRCLE_LAPS,
            'duration': self.config.CIRCLE_DURATION,
            'direction': self.config.CIRCLE_DIRECTION,
            'robot_namespace': self.config.ROBOT_NAMESPACE,
        }
        settings = {key: self.declare_parameter(key, value).value
                    for key, value in defaults.items()}
        self.target = TimedCircleTarget(
            settings['radius'], settings['target_speed'], settings['lookahead'],
            settings['direction'], settings['laps'], settings['duration'])
        self.path = self.target.path
<<<<<<< HEAD
=======
        self.speed = settings['speed']
        if (not math.isfinite(self.speed)
                or not 0 <= self.speed <= self.config.NAV_MAX_LINEAR_SPEED):
            raise ValueError('speed must be between 0 and 0.20 m/s')
>>>>>>> 77d1cce42b71b4269be683b4b912eafb5673ea7d
        prefix = settings['robot_namespace'].rstrip('/')
        if not prefix.startswith('/') or not prefix:
            raise ValueError('robot_namespace must be absolute, e.g. /TTB10')
        self.cmd_topic, self.odom_topic = prefix + '/cmd_vel', prefix + '/odom'
        self.diagnostic_prefix = prefix + '/lab2_circle'
        self.origin = None
        self.started = None
        self.last_odom_time = None
        self.raw_pose = (0.0, 0.0, 0.0)
        self.current_x = self.current_y = self.current_yaw = 0.0
        self.measured_linear = self.measured_angular = float('nan')
        self.progress = CircleProgress(self.path)
        self.goal_reached = False
        self.last_warning = None
        self.last_sample = None
        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sample_publisher = self.create_publisher(
            String, self.diagnostic_prefix + '/sample', 10)
        names = (
            'x', 'y', 'yaw', 'target_x', 'target_y', 'radial_error',
            'progress', 'laps', 'distance_to_start', 'done',
            'linear_command', 'angular_command', 'position_error',
<<<<<<< HEAD
            'target_phase', 'target_laps', 'target_speed', 'robot_speed_limit',
=======
            'target_phase', 'target_laps', 'target_speed', 'robot_speed',
>>>>>>> 77d1cce42b71b4269be683b4b912eafb5673ea7d
            'phase_gap', 'odom_fresh', 'elapsed',
            'measured_linear', 'measured_angular',
        )
        self.plot_publishers = {name: self.create_publisher(
            Float64, self.diagnostic_prefix + '/' + name, 10) for name in names}
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.parameter_callback = self.add_on_set_parameters_callback(
            self.validate_parameters)
        self.timer = self.create_timer(self.config.CONTROL_PERIOD,
                                       self.controller_callback)
        end = (f'{self.target.duration:.1f} seconds' if self.target.duration > 0
               else f'{self.target.laps} TARGET lap(s)')
        self.get_logger().info(
<<<<<<< HEAD
            f'Circle: automatic robot speed (fixed bound '
            f'{self.config.NAV_MAX_LINEAR_SPEED:.3f} m/s), '
            f'target speed={self.target.speed:.3f} m/s, '
            f'R={self.path.radius:.3f} m; end after {end}. Waiting for {self.odom_topic}.')

    def validate_parameters(self, parameters):
        """Allow target speed changes; robot speed is a controller output."""
        fixed = {'radius', 'lookahead', 'laps', 'duration', 'direction', 'robot_namespace'}
        for parameter in parameters:
            if parameter.name == 'speed':
                return SetParametersResult(
                    successful=False, reason='Robot speed is automatic; set target_speed instead')
            if parameter.name in fixed:
                return SetParametersResult(
                    successful=False, reason=parameter.name + ' requires restarting circle')
            if parameter.name == 'target_speed':
=======
            f'Circle: robot speed LIMIT={self.speed:.3f}, target speed={self.target.speed:.3f} m/s, '
            f'R={self.path.radius:.3f} m; end after {end}. Waiting for {self.odom_topic}.')

    def validate_parameters(self, parameters):
        """Allow live speeds; geometry and finish rules require restarting."""
        fixed = {'radius', 'lookahead', 'laps', 'duration', 'direction', 'robot_namespace'}
        for parameter in parameters:
            if parameter.name in fixed:
                return SetParametersResult(
                    successful=False, reason=parameter.name + ' requires restarting circle')
            if parameter.name in ('speed', 'target_speed'):
>>>>>>> 77d1cce42b71b4269be683b4b912eafb5673ea7d
                value = parameter.value
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not math.isfinite(value) or value < 0):
                    return SetParametersResult(
<<<<<<< HEAD
                        successful=False, reason='target_speed must be finite and nonnegative')
=======
                        successful=False, reason='Speeds must be finite and nonnegative')
                if (parameter.name == 'speed'
                        and value > self.config.NAV_MAX_LINEAR_SPEED):
                    return SetParametersResult(
                        successful=False, reason='Robot speed limit is 0.20 m/s')
>>>>>>> 77d1cce42b71b4269be683b4b912eafb5673ea7d
        return SetParametersResult(successful=True)

    def odom_callback(self, message):
        """Update robot position/yaw without changing target phase or its clock."""
        try:
            pose = read_planar_pose(message)
        except ValueError as error:
            self.last_odom_time = None
            self.warn_once(str(error))
            return
        now = time.monotonic()
        if self.origin is None:
            self.origin = pose
            self.started = now
            self.target.start(now)
            self.get_logger().info(
                'Starting pose captured. Independent target clock started.')
        self.raw_pose = pose
        self.current_x, self.current_y, self.current_yaw = relative_pose(
            *pose, *self.origin)
        self.last_odom_time = now
        self.progress.update(self.current_x, self.current_y)
        try:
            self.measured_linear = float(message.twist.twist.linear.x)
            self.measured_angular = float(message.twist.twist.angular.z)
        except (AttributeError, TypeError, ValueError):
            self.measured_linear = self.measured_angular = float('nan')

    def warn_once(self, warning):
        """Suppress unchanged consecutive warnings."""
        if warning != self.last_warning:
            self.get_logger().warning(warning)
            self.last_warning = warning

    def controller_callback(self):
        """Advance the timed target, follow the circle and synchronize phase."""
        command = Twist()
        now = time.monotonic()
        if self.started is None:
            self.publisher.publish(command)
            return
<<<<<<< HEAD
=======
        self.speed = self.get_parameter('speed').value
>>>>>>> 77d1cce42b71b4269be683b4b912eafb5673ea7d
        target_speed = self.get_parameter('target_speed').value
        tx, ty = self.target.advance(now, speed=target_speed)
        fresh = odometry_is_fresh(
            now, self.last_odom_time, self.config.SENSOR_TIMEOUT)
        if self.target.done and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info(
                'Circle experiment completed: target reached its time/lap limit. '
                'Robot stopped. Ctrl+C to exit.')
        if not fresh:
            self.warn_once(
                'Stale/invalid odometry: robot stopped; target clock continues.')
        else:
            self.last_warning = None
<<<<<<< HEAD
            if not self.goal_reached:
                command.linear.x, command.angular.z = circle_tracking_command(
                    self.current_x, self.current_y, self.current_yaw,
                    self.progress.radians, self.target.phase, self.target.speed,
                    self.path, self.target.lookahead,
=======
            if not self.goal_reached and self.speed > 0:
                command.linear.x, command.angular.z = circle_tracking_command(
                    self.current_x, self.current_y, self.current_yaw,
                    self.progress.radians, self.target.phase, self.target.speed,
                    self.path, self.speed, self.target.lookahead,
>>>>>>> 77d1cce42b71b4269be683b4b912eafb5673ea7d
                    self.config.NAV_MAX_LINEAR_SPEED,
                    self.config.NAV_MAX_ANGULAR_SPEED)
        self.publisher.publish(command)
        nan = float('nan')
        x, y, yaw = self.current_x, self.current_y, self.current_yaw
        sample = {
            'elapsed_s': max(0.0, now - self.started),
            'odom_x_m': self.raw_pose[0] if fresh else nan,
            'odom_y_m': self.raw_pose[1] if fresh else nan,
            'odom_yaw_rad': self.raw_pose[2] if fresh else nan,
            'x_m': x if fresh else nan, 'y_m': y if fresh else nan,
            'yaw_rad': yaw if fresh else nan,
            'target_x_m': tx, 'target_y_m': ty,
            'target_phase_rad': self.target.phase,
            'target_laps': self.target.completed_laps,
            'robot_phase_rad': self.progress.radians if fresh else nan,
            'position_error_m': math.hypot(tx - x, ty - y) if fresh else nan,
            'radial_error_m': self.path.radial_error(x, y) if fresh else nan,
            'phase_gap_rad': self.target.phase - self.progress.radians if fresh else nan,
            'linear_command_mps': command.linear.x,
            'angular_command_radps': command.angular.z,
            'measured_linear_mps': self.measured_linear if fresh else nan,
            'measured_angular_radps': self.measured_angular if fresh else nan,
            'target_speed_mps': 0.0 if self.goal_reached else self.target.speed,
<<<<<<< HEAD
            'robot_speed_limit_mps': self.config.NAV_MAX_LINEAR_SPEED,
=======
            'robot_speed_mps': 0.0 if self.goal_reached else self.speed,
>>>>>>> 77d1cce42b71b4269be683b4b912eafb5673ea7d
            'radius_m': self.path.radius, 'direction': self.path.direction,
            'lookahead_m': self.target.lookahead,
            'requested_laps': self.target.laps,
            'duration_limit_s': self.target.duration,
            'odom_age_s': (now - self.last_odom_time
                           if self.last_odom_time is not None else nan),
            'odom_fresh': int(fresh), 'done': int(self.goal_reached),
            'data_source': 'robot_odometry',
        }
        self.last_sample = sample
        self.publish_diagnostics(sample)

    def publish_diagnostics(self, sample):
        """Keep old scalar topic names; add error/target paired telemetry."""
        fields = {
            'x': 'x_m', 'y': 'y_m', 'yaw': 'yaw_rad',
            'target_x': 'target_x_m', 'target_y': 'target_y_m',
            'radial_error': 'radial_error_m', 'progress': 'robot_phase_rad',
            'done': 'done', 'linear_command': 'linear_command_mps',
            'angular_command': 'angular_command_radps',
            'position_error': 'position_error_m',
            'target_phase': 'target_phase_rad', 'target_laps': 'target_laps',
<<<<<<< HEAD
            'target_speed': 'target_speed_mps',
            'robot_speed_limit': 'robot_speed_limit_mps',
=======
            'target_speed': 'target_speed_mps', 'robot_speed': 'robot_speed_mps',
>>>>>>> 77d1cce42b71b4269be683b4b912eafb5673ea7d
            'phase_gap': 'phase_gap_rad', 'odom_fresh': 'odom_fresh',
            'elapsed': 'elapsed_s', 'measured_linear': 'measured_linear_mps',
            'measured_angular': 'measured_angular_radps',
        }
        for name, field in fields.items():
            self.plot_publishers[name].publish(Float64(data=float(sample[field])))
        self.plot_publishers['laps'].publish(
            Float64(data=sample['robot_phase_rad'] / math.tau))
        self.plot_publishers['distance_to_start'].publish(
            Float64(data=math.hypot(sample['x_m'], sample['y_m'])))
        serializable = {
            key: None if isinstance(value, (int, float)) and not math.isfinite(value) else value
            for key, value in sample.items()
        }
        self.sample_publisher.publish(
            String(data=json.dumps(serializable, allow_nan=False)))


def main(args=None):
    """Run circle with zero velocity on normal Ctrl+C shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Lab2Circle()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.timer.cancel()
            if rclpy.ok():
                node.publisher.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
