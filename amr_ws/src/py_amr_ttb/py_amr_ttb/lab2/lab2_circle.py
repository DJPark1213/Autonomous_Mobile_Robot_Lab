"""Circle: independent moving target and robot pursuit, original interfaces."""

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
    CircleProgress, TimedCircleTarget, pursuit_command,
)


class Lab2Circle(Node):
    """Use time for target motion; odometry updates only robot state."""

    def __init__(self):
        super().__init__('lab2_circle')
        self.config = CircleConfig
        defaults = {
            'radius': self.config.CIRCLE_RADIUS,
            'speed': self.config.CIRCLE_SPEED,
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
        self.speed = settings['speed']
        if (not math.isfinite(self.speed)
                or not 0 <= self.speed <= self.config.NAV_MAX_LINEAR_SPEED):
            raise ValueError('speed must be between 0 and 0.20 m/s')
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
            'target_phase', 'target_laps', 'target_speed', 'robot_speed',
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
            f'Circle: robot speed={self.speed:.3f}, target speed={self.target.speed:.3f} m/s, '
            f'R={self.path.radius:.3f} m; end after {end}. Waiting for {self.odom_topic}.')

    def validate_parameters(self, parameters):
        """Allow live speeds; geometry and finish rules require restarting."""
        fixed = {'radius', 'lookahead', 'laps', 'duration', 'direction', 'robot_namespace'}
        for parameter in parameters:
            if parameter.name in fixed:
                return SetParametersResult(
