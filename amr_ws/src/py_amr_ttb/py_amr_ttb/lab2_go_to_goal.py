"""Go to a relative pose using odometry feedback."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from py_amr_ttb.lab_config import LabConfig


class Lab2GoToGoal(Node):
    """Move to (1.5, 1.5) relative to the first odometry pose."""

    def __init__(self):
        super().__init__('lab2_go_to_goal')

        self.config = LabConfig
        self.initial_x = None
        self.initial_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False
        self.last_warning = None

        self.target_x = self.config.GOAL_X_OFFSET
        self.target_y = self.config.GOAL_Y_OFFSET
        self.position_tolerance = 0.05
        self.yaw_tolerance = 0.10
        self.max_linear_speed = 0.20
        self.max_angular_speed = 0.40
        self.linear_gain = 1.0
        self.angular_gain = 2.0

        self.publisher = self.create_publisher(
            Twist,
            self.config.CMD_VEL_TOPIC,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.config.ODOM_TOPIC,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(
            self.config.CONTROL_PERIOD,
            self.controller_callback,
        )

        self.get_logger().info(
            'Lab 2: going to (1.5, 1.5) relative to the initial pose.'
        )

    def odom_callback(self, message):
        """Record the latest pose and publish it for rqt_plot."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

    def controller_callback(self):
        """Compute and publish velocity until the relative goal is reached."""
        command = Twist()
        

        return command


def main(args=None):
    """Start the relative-position controller."""
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
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
