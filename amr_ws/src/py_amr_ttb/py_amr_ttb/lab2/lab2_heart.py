"""Follow a heart with pure pursuit, stopping at the notch and final tip."""

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
from py_amr_ttb.lab2.navigation import HeartPath
from py_amr_ttb.lab2.pure_pursuit import pursuit_command
from py_amr_ttb.odom_utils import (
    normalize_angle, odometry_is_fresh, read_planar_pose, relative_pose,
)


class Lab2Heart(Node):
    """Track the left then right lobe in one fixed starting robot frame."""

    def __init__(self):
        super().__init__('lab2_heart')
        self.config = LabConfig
        self.width = self.declare_parameter('width', self.config.HEART_WIDTH).value
        self.speed = self.declare_parameter('speed', self.config.HEART_SPEED).value
        self.lookahead = self.declare_parameter('lookahead', self.config.HEART_LOOKAHEAD).value
        self.path = HeartPath(self.width)
        if not math.isfinite(self.speed) or not 0 < self.speed <= self.config.NAV_MAX_LINEAR_SPEED:
            raise ValueError('heart speed must be positive and <= NAV_MAX_LINEAR_SPEED')
        if not math.isfinite(self.lookahead) or not 0 < self.lookahead < .4 * self.width:
            raise ValueError('heart lookahead must be positive and < 0.4 * width')
        self.tolerance = self.config.HEART_POSITION_TOLERANCE
        if not 0 < self.tolerance < self.width / 10:
            raise ValueError('heart position tolerance must be positive and < width/10')
        if (not math.isfinite(self.config.HEART_CUSP_STOP_TIME)
                or self.config.HEART_CUSP_STOP_TIME < 0):
            raise ValueError('heart cusp stop time must be finite and nonnegative')
        self.origin = None
        self.current_x = self.current_y = self.current_yaw = 0.0
        self.last_odom_time = None
        self.last_warning = None
        self.goal_reached = False
        self.tracking_failed = False
        self.state = 'ALIGN'
        self.stop_until = None
        self.path_error = 0.0
        self.target_x = self.target_y = 0.0
        self.publisher = self.create_publisher(Twist, self.config.CMD_VEL_TOPIC, 10)
        self.plot_publishers = {
            name: self.create_publisher(
                Float64, self.config.ROBOT_NAMESPACE + '/lab2_heart/' + name, 10,
            ) for name in ('x', 'y', 'yaw', 'target_x', 'target_y', 'progress',
                           'section', 'path_error', 'distance', 'done',
                           'linear_command', 'angular_command')
        }
        self.odom_subscription = self.create_subscription(
            Odometry, self.config.ODOM_TOPIC, self.odom_callback, qos_profile_sensor_data,
        )
        self.timer = self.create_timer(self.config.CONTROL_PERIOD, self.controller_callback)
        self.get_logger().info(
            f'Heart: width={self.width:.2f} m, speed={self.speed:.2f} m/s, '
            f'lookahead={self.lookahead:.2f} m. Waiting for odometry.')

    def odom_callback(self, message):
        """Validate odometry and update pose without resetting route progress."""
        try:
            pose = read_planar_pose(message)
        except ValueError as error:
            self.last_odom_time = None
            self.warn_once(str(error))
            return
        if self.origin is None:
            self.origin = pose
            self.get_logger().info('Starting pose captured; aligning with the left lobe.')
        self.current_x, self.current_y, self.current_yaw = relative_pose(*pose, *self.origin)
        self.last_odom_time = time.monotonic()

    def warn_once(self, warning):
        """Log changed warnings once."""
        if warning != self.last_warning:
            self.get_logger().warning(warning)
            self.last_warning = warning

    def publish_diagnostics(self, command):
        """Publish measured pose, ordered progress and command diagnostics."""
        values = {
            'x': self.current_x, 'y': self.current_y, 'yaw': self.current_yaw,
            'target_x': self.target_x, 'target_y': self.target_y,
            'progress': 1.0 if self.goal_reached else self.path.progress,
            'section': self.path.section, 'path_error': self.path_error,
            'distance': math.dist((self.current_x, self.current_y), self.path.endpoint),
            'done': float(self.goal_reached),
            'linear_command': command.linear.x, 'angular_command': command.angular.z,
        }
        for name, value in values.items():
            self.plot_publishers[name].publish(Float64(data=float(value)))

    def controller_callback(self):
        """Publish one bounded command per tick; latch path failure and completion."""
        command = Twist()
        now = time.monotonic()
        fresh = odometry_is_fresh(now, self.last_odom_time, self.config.SENSOR_TIMEOUT)
        if self.goal_reached or self.tracking_failed:
            pass
        elif not fresh:
            self.warn_once('Missing, invalid, or stale odometry: stopping.')
        else:
            self.last_warning = None
            x, y, yaw = self.current_x, self.current_y, self.current_yaw
            if self.state == 'CUSP_STOP':
                if now >= self.stop_until:
                    self.path.advance_section()
                    self.state = 'ALIGN'
                    self.get_logger().info('Notch stop complete; aligning with the right lobe.')
            elif self.state == 'ALIGN':
                # Use an outgoing secant so a zero derivative at the cusp is harmless.
                a = self.path.point_at(0.0)
                b = self.path.point_at(min(self.lookahead / 3, self.width / 30))
                error = normalize_angle(math.atan2(b[1] - a[1], b[0] - a[0]) - yaw)
                if abs(error) <= self.config.NAV_YAW_TOLERANCE:
                    self.state = 'FOLLOW'
                else:
                    command.angular.z = max(
                        -self.config.NAV_MAX_ANGULAR_SPEED,
                        min(self.config.NAV_MAX_ANGULAR_SPEED,
                            self.config.NAV_ANGULAR_GAIN * error),
                    )
            else:
                self.path_error = self.path.update(x, y, 2 * self.lookahead)
                if self.path_error > self.config.HEART_MAX_PATH_ERROR:
                    self.tracking_failed = True
                    self.state = 'FAILED'
                    self.warn_once('Heart path error too large: stopped. Check pose and relaunch.')
                elif self.path.section_reached(x, y, self.tolerance):
                    if self.path.section == 0:
                        self.state = 'CUSP_STOP'
                        self.stop_until = now + self.config.HEART_CUSP_STOP_TIME
                        self.get_logger().info('Heart notch reached: stopping before right lobe.')
                    else:
                        self.goal_reached = True
                        self.state = 'DONE'
                        self.get_logger().info('Heart completed: returned to bottom tip. Stopped.')
                else:
                    self.target_x, self.target_y = self.path.target(self.lookahead)
                    approach = max(self.path.remaining, math.dist((x, y), self.path.endpoint))
                    speed = min(self.speed, self.config.NAV_LINEAR_GAIN * approach)
                    command.linear.x, command.angular.z = pursuit_command(
                        x, y, yaw, self.target_x, self.target_y, speed,
                        self.config.NAV_MAX_LINEAR_SPEED, self.config.NAV_MAX_ANGULAR_SPEED,
                    )
        self.publisher.publish(command)
        if fresh:
            self.publish_diagnostics(command)


def main(args=None):
    """Run the heart task and send a zero command on normal shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Lab2Heart()
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
