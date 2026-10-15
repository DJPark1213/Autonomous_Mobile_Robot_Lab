"""Follow a circle with odometry-feedback pure pursuit and measured lap stopping."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from py_amr_ttb.lab2.circle_config import CircleConfig
from py_amr_ttb.lab2.pure_pursuit import CirclePath, CircleProgress, pursuit_command
from py_amr_ttb.lab2.circle_odom import odometry_is_fresh, read_planar_pose, relative_pose


class Lab2Circle(Node):
    """Track a continuous circle in the fixed starting robot frame."""

    def __init__(self):
        super().__init__('lab2_circle')
        self.config = CircleConfig
        defaults = {
            'radius': self.config.CIRCLE_RADIUS,
            'speed': self.config.CIRCLE_SPEED,
            'lookahead': self.config.CIRCLE_LOOKAHEAD,
            'laps': self.config.CIRCLE_LAPS,
            'direction': self.config.CIRCLE_DIRECTION,
            'robot_namespace': self.config.ROBOT_NAMESPACE,
        }
        settings = {name: self.declare_parameter(name, default).value
                    for name, default in defaults.items()}
        self.path = CirclePath(settings['radius'], settings['direction'])
        self.speed = settings['speed']
        self.lookahead = settings['lookahead']
        self.laps = settings['laps']
        if not math.isfinite(self.speed) or not 0 < self.speed <= self.config.NAV_MAX_LINEAR_SPEED:
            raise ValueError('speed must be positive and <= NAV_MAX_LINEAR_SPEED (0.20 m/s)')
        if not math.isfinite(self.lookahead) or not 0 < self.lookahead < 2 * self.path.radius:
            raise ValueError('lookahead must be positive and less than 2 * radius')
        if isinstance(self.laps, bool) or not isinstance(self.laps, int) or self.laps < 1:
            raise ValueError('laps must be a positive integer')
        prefix = settings['robot_namespace'].rstrip('/')
        if not prefix.startswith('/') or prefix == '':
            raise ValueError('robot_namespace must be absolute, e.g. /TTB10')
        self.cmd_topic = prefix + '/cmd_vel'
        self.odom_topic = prefix + '/odom'
        self.progress = CircleProgress(self.path)
        self.origin = None
        self.current_x = self.current_y = self.current_yaw = 0.0
        self.last_odom_time = None
        self.last_warning = None
        self.goal_reached = False
        self.tracking_failed = False
        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.plot_publishers = {
            name: self.create_publisher(Float64, prefix + '/lab2_circle/' + name, 10)
            for name in ('x', 'y', 'yaw', 'target_x', 'target_y', 'radial_error',
                         'progress', 'laps', 'distance_to_start', 'done',
                         'linear_command', 'angular_command')
        }
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(self.config.CONTROL_PERIOD, self.controller_callback)
        direction = 'CCW (left)' if self.path.direction == 1 else 'CW (right)'
        self.get_logger().info(
            f'Circle: radius={self.path.radius:.3f} m, speed={self.speed:.3f} m/s, '
            f'lookahead={self.lookahead:.3f} m, {self.laps} lap(s), {direction}. '
            f'Waiting for {self.odom_topic}.')

    def odom_callback(self, message):
        """Capture one fixed origin; update position and directed path progress."""
        try:
            pose = read_planar_pose(message)
        except ValueError as error:
            self.last_odom_time = None
            self.warn_once(str(error))
            return
        if self.origin is None:
            self.origin = pose
            self.get_logger().info('Starting pose captured. Circle tracking begins.')
        self.current_x, self.current_y, self.current_yaw = relative_pose(*pose, *self.origin)
        self.last_odom_time = time.monotonic()
        if not self.goal_reached:
            self.progress.update(self.current_x, self.current_y)

    def warn_once(self, warning):
        """Report a changed warning once."""
        if warning != self.last_warning:
            self.get_logger().warning(warning)
            self.last_warning = warning

    def publish_diagnostics(self, command):
        """Publish starting-frame trajectory and pursuit tracking diagnostics."""
        x, y = self.current_x, self.current_y
        tx, ty = self.path.target(x, y, self.lookahead)
        values = {
            'x': x, 'y': y, 'yaw': self.current_yaw, 'target_x': tx, 'target_y': ty,
            'radial_error': self.path.radial_error(x, y),
            'progress': self.progress.radians,
            'laps': self.progress.radians / math.tau,
            'distance_to_start': math.hypot(x, y), 'done': float(self.goal_reached),
            'linear_command': command.linear.x, 'angular_command': command.angular.z,
        }
        for name, value in values.items():
            self.plot_publishers[name].publish(Float64(data=float(value)))

    def controller_callback(self):
        """Publish exactly one velocity per tick; completion remains stopped."""
        command = Twist()
        fresh = odometry_is_fresh(
            time.monotonic(), self.last_odom_time, self.config.SENSOR_TIMEOUT)
        if self.goal_reached or self.tracking_failed:
            pass
        elif not fresh:
            self.warn_once('Missing, invalid, or stale odometry: stopping.')
        else:
            self.last_warning = None
            x, y, yaw = self.current_x, self.current_y, self.current_yaw
            completed = (
                self.progress.radians >= math.tau * self.laps
                and math.hypot(x, y) <= self.config.NAV_POSITION_TOLERANCE
                and abs(yaw) <= self.config.CIRCLE_FINISH_YAW_TOLERANCE
            )
            if completed:
                self.goal_reached = True
                self.get_logger().info(
                    f'Circle completed: {self.progress.radians / math.tau:.3f} laps, '
                    f'closure error={math.hypot(x, y):.3f} m. Stopped. Ctrl+C to exit.')
            elif abs(self.path.radial_error(x, y)) > self.config.CIRCLE_MAX_RADIAL_ERROR:
                self.tracking_failed = True
                self.warn_once('Circle tracking error > 0.35 m: stopped. Check pose and relaunch.')
            else:
                tx, ty = self.path.target(x, y, self.lookahead)
                command.linear.x, command.angular.z = pursuit_command(
                    x, y, yaw, tx, ty, self.speed, self.config.NAV_MAX_LINEAR_SPEED,
                    self.config.NAV_MAX_ANGULAR_SPEED)
        self.publisher.publish(command)
        if fresh:
            self.publish_diagnostics(command)


def main(args=None):
    """Run the node, sending zero velocity while the ROS context is available."""
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
