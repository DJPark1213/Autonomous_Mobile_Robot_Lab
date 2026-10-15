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
    odometry_is_fresh, read_planar_pose, relative_pose,
)
from py_amr_ttb.lab2.navigation import waypoint_command


class Lab2GoToGoal(Node):
    """Reach (1.5, 1.5) in the fixed starting robot frame."""

    def __init__(self):
        super().__init__('lab2_go_to_goal')
        self.config = LabConfig
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
            ) for name in ('x', 'y', 'yaw', 'distance')
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
        distance = math.hypot(target_x - self.current_x, target_y - self.current_y)
        for name, value in zip(
            ('x', 'y', 'yaw', 'distance'),
            (self.current_x, self.current_y, self.current_yaw, distance),
        ):
            self.plot_publishers[name].publish(Float64(data=float(value)))

    def warn_once(self, warning):
        """Log a changed odometry warning without flooding the console."""
        if warning != self.last_warning:
            self.get_logger().warning(warning)
            self.last_warning = warning

    def controller_callback(self):
        """Publish a command every tick, including a stop on missing/stale pose."""
        command = Twist()
        if self.goal_reached:
            self.publisher.publish(command)
            return
        if not odometry_is_fresh(
            time.monotonic(), self.last_odom_time, self.config.SENSOR_TIMEOUT,
        ):
            self.warn_once('Missing, invalid, or stale odometry: stopping.')
            self.publisher.publish(command)
            return
        self.last_warning = None
        target_x, target_y = (self.target_x, self.target_y)
        linear, angular, reached = waypoint_command(
            self.current_x, self.current_y, self.current_yaw,
            target_x, target_y, self.config,
        )
        command.linear.x = linear
        command.angular.z = angular
        if reached:
            self.goal_reached = True
            self.get_logger().info('Goal reached: stopped.')
        self.publisher.publish(command)


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
                node.publisher.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
