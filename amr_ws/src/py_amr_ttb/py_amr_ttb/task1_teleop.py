"""Task 1: standalone joystick teleoperation."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy

from py_amr_ttb.lab_config import LabConfig


class Task1Teleop(Node):
    """Publish joystick commands at a fixed control rate."""

    def __init__(self):
        super().__init__('task1_teleop')

        self.config = LabConfig
        self.axes = []
        self.last_joy_time = None

        self.publisher = self.create_publisher(
            Twist,
            self.config.CMD_VEL_TOPIC,
            10,
        )

        self.joy_subscription = self.create_subscription(
            Joy,
            self.config.JOY_TOPIC,
            self.joy_callback,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(
            self.config.CONTROL_PERIOD,
            self.controller_callback,
        )

        self.get_logger().info(
            'Task 1: joystick teleoperation ready.'
        )

    def joy_callback(self, message):
        """Store the latest axes and receipt time."""
        self.axes = list(message.axes)
        self.last_joy_time = time.monotonic()

    def controller_callback(self):
        """Publish joystick motion or zero when input is unusable."""
        command = Twist()

        fresh = (
            self.last_joy_time is not None
            and 0.0 <= time.monotonic() - self.last_joy_time
            <= self.config.JOYSTICK_TIMEOUT
        )

        forward_axis = self.config.FORWARD_AXIS
        turn_axis = self.config.TURN_AXIS

        axes_present = (
            0 <= forward_axis < len(self.axes)
            and 0 <= turn_axis < len(self.axes)
        )

        if fresh and axes_present:
            forward = float(self.axes[forward_axis])
            turn = float(self.axes[turn_axis])

            if math.isfinite(forward) and math.isfinite(turn):
                command.linear.x = (
                    self.config.TELEOP_LINEAR_SCALE * forward
                )

                command.angular.z = (
                    self.config.TELEOP_ANGULAR_SCALE * turn
                )

        self.publisher.publish(command)


def main(args=None):
    """Start teleoperation and attempt a stop on Ctrl+C."""
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

    node = None

    try:
        node = Task1Teleop()
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