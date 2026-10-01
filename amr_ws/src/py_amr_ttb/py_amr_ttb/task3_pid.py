"""Task 3: standalone PID speed control."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from py_amr_ttb.lab_config import LabConfig
from py_amr_ttb.pid_speed_controller import PidSpeedController


class Task3Pid(Node):
    """Maintain the configured target using odometry feedback."""

    def __init__(self):
        super().__init__('task3_pid')

        self.config = LabConfig
        self.pid = PidSpeedController(self.config)

        self.target_speed = self.config.PID_TARGET_SPEED
        self.measured_speed = None
        self.last_odom_time = None
        self.last_warning = None

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
            f'Task 3: PID target = {self.target_speed:.2f} m/s.'
        )

    def odom_callback(self, message):
        """Store finite forward speed and receipt time."""
        speed = float(message.twist.twist.linear.x)

        if math.isfinite(speed):
            self.measured_speed = speed
            self.last_odom_time = time.monotonic()
        else:
            self.measured_speed = None
            self.last_odom_time = None

    def controller_callback(self):
        """Publish a PID command or stop when odometry is unavailable."""
        command = Twist()
        warning = None

        fresh = (
            self.last_odom_time is not None
            and 0.0 <= time.monotonic() - self.last_odom_time
            <= self.config.SENSOR_TIMEOUT
        )

        if fresh and self.measured_speed is not None:
            command.linear.x = self.pid.update(
                self.target_speed,
                self.measured_speed,
                self.config.CONTROL_PERIOD,
            )
        else:
            self.pid.reset()
            warning = 'Odometry is missing/stale: stopped.'

        self.publisher.publish(command)

        if warning and warning != self.last_warning:
            self.get_logger().warning(warning)

        self.last_warning = warning


def main(args=None):
    """Start PID control and attempt a stop on Ctrl+C."""
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

    node = None

    try:
        node = Task3Pid()
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