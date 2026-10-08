"""Task 2: standalone autonomous wandering."""

import rclpy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from py_amr_ttb.ir_sensor import IrSensorState
from py_amr_ttb.lab_config import LabConfig
from py_amr_ttb.wander_behavior import WanderBehavior


class Task2Wander(Node):
    """Start wandering once usable IR messages arrive."""

    def __init__(self):
        super().__init__('task2_wander')

        self.config = LabConfig
        self.ir_sensor = IrSensorState(self.config)
        self.wander = WanderBehavior(self.config)
        self.last_warning = None

        self.publisher = self.create_publisher(
            Twist,
            self.config.CMD_VEL_TOPIC,
            10,
        )

        self.ir_subscription = self.create_subscription(
            IrIntensityVector,
            self.config.IR_TOPIC,
            self.ir_callback,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(
            self.config.CONTROL_PERIOD,
            self.controller_callback,
        )

        self.get_logger().info(
            'Task 2: waiting for IR sensor data.'
        )

    def now_seconds(self):
        """Return ROS clock time in seconds."""
        return (
            self.get_clock().now().nanoseconds
            / 1_000_000_000.0
        )

    def ir_callback(self, message):
        """Store the latest IR readings."""
        self.ir_sensor.update(
            message,
            self.now_seconds(),
        )

    def controller_callback(self):
        """Publish exactly one wandering command."""
        command, warning = self.wander.command(
            self.now_seconds(),
            self.ir_sensor,
        )

        self.publisher.publish(command)

        if warning and warning != self.last_warning:
            self.get_logger().warning(warning)

        self.last_warning = warning


def main(args=None):
    """Start wandering and attempt a stop on Ctrl+C."""
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

    node = None

    try:
        node = Task2Wander()
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