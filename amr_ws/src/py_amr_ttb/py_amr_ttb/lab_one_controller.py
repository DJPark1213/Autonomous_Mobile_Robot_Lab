"""Task 4 TODO: combine teleoperation, wandering, and cruise control."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class LabOneController(Node):
    """Launchable template with no subscriptions or command publisher."""

    def __init__(self):
        super().__init__('lab_one_controller')
        # TODO: connect Joy, odometry, and IR inputs to the behavior modules.
        # TODO: implement L1/L2 selection, cruise buttons, and R1 override.
        # TODO: add one final command publisher and stop/timeout handling.
        self.get_logger().info('Task 4 template: integration is pending.')

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
    """Start the idle template node in a sourced ROS 2 environment."""
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
