"""Run Task 2 autonomous wandering."""

import rclpy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from py_amr_ttb.ir_sensor import IrSensorState
from py_amr_ttb.lab_config import LabConfig
from py_amr_ttb.wander_behavior import WanderBehavior


class LabOneController(Node):
    """Run autonomous wandering immediately after startup."""

    def __init__(self):
        super().__init__('lab_one_controller')

        self.config = LabConfig
        self.ir_sensor = IrSensorState(self.config)
        self.wander = WanderBehavior(self.config)
        self.last_error = None

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
            'Task 2 started. Waiting for IR sensor data.'
        )

    def now_seconds(self):
        """Return the current ROS clock time in seconds."""
        return (
            self.get_clock().now().nanoseconds
            / 1_000_000_000.0
        )

    def ir_callback(self, message):
        """Store the newest IR sensor readings."""
        error = self.ir_sensor.update(
            message,
            self.now_seconds(),
        )

        if error is not None:
            self.get_logger().error(error)

    def controller_callback(self):
        """Calculate and publish one wandering command."""
        command, error = self.wander.command(
            self.now_seconds(),
            self.ir_sensor,
        )

        if error != self.last_error:
            if error is not None:
                self.get_logger().warn(error)
            else:
                self.get_logger().info(
                    'IR data received. Wandering is active.'
                )

            self.last_error = error

        self.publisher.publish(command)

    def stop_robot(self):
        """Publish a zero velocity command."""
        self.publisher.publish(Twist())


def main(args=None):
    """Start Task 2."""
    rclpy.init(args=args)
    controller = LabOneController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.timer.cancel()
        controller.stop_robot()
        controller.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
