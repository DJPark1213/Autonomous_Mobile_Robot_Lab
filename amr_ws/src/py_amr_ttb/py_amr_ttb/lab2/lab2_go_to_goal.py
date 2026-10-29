"""Reach (1.5, 1.5) in the fixed starting robot frame."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from py_amr_ttb.lab_config import LabConfig
from py_amr_ttb.odom_utils import (
    normalize_angle, odometry_is_fresh, read_planar_pose, relative_pose,
)
from py_amr_ttb.lab2.motion_control import GoalController


class Lab2GoToGoal(Node):
    """Reach (1.5, 1.5) in the fixed starting robot frame."""

    def __init__(self):
        super().__init__('lab2_go_to_goal')
        # Per-node settings: experiments must not change square/star defaults.
        self.config = LabConfig()
        for name, attribute in (
            ('linear_gain', 'NAV_LINEAR_GAIN'),
            ('angular_gain', 'NAV_ANGULAR_GAIN'),
            ('max_linear_speed', 'NAV_MAX_LINEAR_SPEED'),
            ('max_angular_speed', 'NAV_MAX_ANGULAR_SPEED'),
            ('position_tolerance', 'NAV_POSITION_TOLERANCE'),
            ('yaw_tolerance', 'NAV_YAW_TOLERANCE'),
        ):
            value = self.declare_parameter(name, getattr(self.config, attribute)).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            if name == 'yaw_tolerance' and value > math.pi:
                raise ValueError('yaw_tolerance must be <= pi radians')
            setattr(self.config, attribute, value)
        pid_settings = {}
        for name, default in {'linear_ki': 0.01, 'linear_kd': 0.02,
                              'angular_ki': 0.01, 'angular_kd': 0.03,
                              'align_enter': max(0.20, 2 * self.config.NAV_YAW_TOLERANCE)}.items():
            value = self.declare_parameter(name, default).value
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
            pid_settings[name] = value
        self.control = GoalController(self.config, **pid_settings)
        self.last_control_time = None
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.last_odom_time = None
        self.last_warning = None
        self.goal_reached = False
        self.target_x = self.config.GOAL_X_OFFSET
        self.target_y = self.config.GOAL_Y_OFFSET
        self.publisher = self.create_publisher(Twist, self.config.CMD_VEL_TOPIC, 10)
        # Relative pose diagnostics; rqt_plot can read each Float64 /data field.
        self.plot_publishers = {
            name: self.create_publisher(
                Float64, self.config.ROBOT_NAMESPACE + '/lab2_go_to_goal/' + name, 10,
            ) for name in (
                'x', 'y', 'yaw', 'distance', 'error_x', 'error_y',
                'heading_error', 'remaining_along_route',
                'command_linear', 'command_angular',
                'measured_linear', 'measured_angular',
            )
        }
        self.odom_subscription = self.create_subscription(
            Odometry, self.config.ODOM_TOPIC, self.odom_callback, qos_profile_sensor_data,
        )
        self.timer = self.create_timer(self.config.CONTROL_PERIOD, self.controller_callback)
        self.get_logger().info('lab2_go_to_goal: waiting for starting odometry.')

    def odom_callback(self, message):
        """Accept a valid pose and express it in the fixed starting frame."""
        try:
            x, y, yaw = read_planar_pose(message)
        except ValueError as error:
            self.last_odom_time = None
            self.warn_once(str(error))
            return
        if self.initial_x is None:
            self.initial_x, self.initial_y, self.initial_yaw = x, y, yaw
        self.current_x, self.current_y, self.current_yaw = relative_pose(
            x, y, yaw, self.initial_x, self.initial_y, self.initial_yaw,
        )
        self.last_odom_time = time.monotonic()
        target_x, target_y = (self.target_x, self.target_y)
        dx, dy = target_x - self.current_x, target_y - self.current_y
        distance = math.hypot(dx, dy)
        route_length = math.hypot(target_x, target_y)
        diagnostics = {
            'x': self.current_x, 'y': self.current_y, 'yaw': self.current_yaw,
            'distance': distance, 'error_x': dx, 'error_y': dy,
            'heading_error': normalize_angle(math.atan2(dy, dx) - self.current_yaw),
            'remaining_along_route': (
                (dx * target_x + dy * target_y) / route_length if route_length else 0.0
            ),
            'measured_linear': message.twist.twist.linear.x,
            'measured_angular': message.twist.twist.angular.z,
        }
        for name, value in diagnostics.items():
            self.plot_publishers[name].publish(Float64(data=float(value)))

    def publish_command(self, command):
        """Publish the actual selected command, including all stop conditions."""
        self.publisher.publish(command)
        self.plot_publishers['command_linear'].publish(Float64(data=command.linear.x))
        self.plot_publishers['command_angular'].publish(Float64(data=command.angular.z))

    def warn_once(self, warning):
        """Log a changed odometry warning without flooding the console."""
        if warning != self.last_warning:
            self.get_logger().warning(warning)
            self.last_warning = warning

    def controller_callback(self):
        """Publish a command every tick, including a stop on missing/stale pose."""
        command = Twist()
        now = time.monotonic()
        dt = self.config.CONTROL_PERIOD if self.last_control_time is None else now - self.last_control_time
        self.last_control_time = now
        if dt <= 0 or dt > self.config.SENSOR_TIMEOUT:
            self.control.reset()
            dt = self.config.CONTROL_PERIOD
        if self.goal_reached:
            self.control.reset()
            self.publish_command(command)
            return
        if not odometry_is_fresh(
            time.monotonic(), self.last_odom_time, self.config.SENSOR_TIMEOUT,
        ):
            self.control.reset()
            self.warn_once('Missing, invalid, or stale odometry: stopping.')
            self.publish_command(command)
            return
        self.last_warning = None
        target_x, target_y = (self.target_x, self.target_y)
        linear, angular, reached = self.control.update(
            self.current_x, self.current_y, self.current_yaw,
            target_x, target_y, dt,
        )
        command.linear.x = linear
        command.angular.z = angular
        if reached:
            self.goal_reached = True
            self.get_logger().info('Goal reached: stopped.')
        self.publish_command(command)


def main(args=None):
    """Run the controller and send zero velocity on normal shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Lab2GoToGoal()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.timer.cancel()
            if rclpy.ok():
                node.publish_command(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
