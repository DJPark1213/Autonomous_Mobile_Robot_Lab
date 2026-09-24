from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from py_amr_ttb.ir_sensor import IrSensorState
from py_amr_ttb.lab_config import LabConfig
from py_amr_ttb.wander_behavior import WanderBehavior


class Mode(Enum):
    """Modes currently available in the basic controller."""

    STOP = 'stop'
    WANDER = 'wander'


class LabOneController(Node):
    """Hold behavior objects and document the future ROS connections."""

    def __init__(self):
        super().__init__('lab_one_controller')

        self.config = LabConfig
        self.ir_sensor = IrSensorState(self.config)
        self.wander = WanderBehavior(self.config)
        self.selected_mode = Mode.STOP

        # TODO Task 4: enable these ROS connections during integration:
        #
        # from irobot_create_msgs.msg import IrIntensityVector
        # from rclpy.qos import qos_profile_sensor_data
        #
        # self.publisher = self.create_publisher(
        #     Twist, self.config.CMD_VEL_TOPIC, 10,
        # )
        # self.ir_subscription = self.create_subscription(
        #     IrIntensityVector,
        #     self.config.IR_TOPIC,
        #     self.ir_callback,
        #     qos_profile_sensor_data,
        # )
        # self.timer = self.create_timer(
        #     self.config.CONTROL_PERIOD,
        #     self.controller_callback,
        # )
        #
        # The timer callback should select exactly one behavior command and
        # publish exactly once. Keep the node in STOP until L1 selects WANDER.

        # TODO Task 4: connect Joy and odometry inputs to the controller.
        # TODO: implement L1/L2 selection, cruise buttons, and R1 override.
        self.get_logger().info(
            'Integration scaffold only: no subscriptions or publisher active.'
        )

    def now_seconds(self):
        """Return the current ROS clock time in seconds."""
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def ir_callback(self, message):
        """Store IR data after Task 4 connects the future subscription."""
        return self.ir_sensor.update(message, self.now_seconds())

    def wander_command(self, now=None):
        """Make the Part 2 command available to the future timer callback."""
        if now is None:
            now = self.now_seconds()
        return self.wander.command(now, self.ir_sensor)

    def controller_callback(self):
        """Return the selected command; publishing is intentionally pending."""
        if self.selected_mode is Mode.WANDER:
            return self.wander_command()
        return Twist(), None

    def joy_callback(self, message):
        """TODO: store joystick axes/buttons and detect button edges."""
        # Handle joystick input here; no separate joystick module is needed.
        pass

    def pressed(self, index):
        """TODO: report whether a joystick button is held."""
        return False

    def newly_pressed(self, index):
        """TODO: detect a released-to-pressed button transition."""
        return False

    def teleop_command(self, now):
        """TODO: use LabConfig axes/scales and R1 override; zero for now."""
        return Twist(), 'Task 4: joystick integration is not implemented.'


def main(args=None):
    """Start the inert integration scaffold in a sourced ROS 2 environment."""
    rclpy.init(args=args)
    controller = LabOneController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
