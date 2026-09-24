"""Run Task 3 independently using odometry feedback."""

import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from py_amr_ttb.lab_config import LabConfig
from py_amr_ttb.pid_speed_controller import PidSpeedController


class LabOnePid(Node):

    def __init__(self):
        super().__init__('lab_one_pid')

        self.config = LabConfig
        self.pid = PidSpeedController(self.config)

        self.target_speed = 0.3
        self.measured_speed = None
        self.last_odom_time = None

        self.publisher = self.create_publisher(
            Twist,
            self.config.CMD_VEL_TOPIC,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            self.config.ODOM_TOPIC,
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            self.config.CONTROL_PERIOD,
            self.controller_callback
        )

    def odom_callback(self, message):
        self.measured_speed = message.twist.twist.linear.x
        self.last_odom_time = time.monotonic()

    def controller_callback(self):
        command = Twist()

        fresh = (
            self.last_odom_time is not None
            and time.monotonic() - self.last_odom_time
            <= self.config.SENSOR_TIMEOUT
        )

        if fresh and self.measured_speed is not None:
            command.linear.x = self.pid.update(
                self.target_speed,
                self.measured_speed,
                self.config.CONTROL_PERIOD
            )
        else:
            self.pid.reset()

        self.publisher.publish(command)


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = None

    try:
        node = LabOnePid()
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
