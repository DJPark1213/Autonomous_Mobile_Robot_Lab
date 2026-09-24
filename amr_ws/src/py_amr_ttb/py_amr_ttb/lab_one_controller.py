from enum import Enum
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector
"""Run Task 2 autonomous wandering."""

import rclpy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from py_amr_ttb.ir_sensor import IrSensorState
from py_amr_ttb.lab_config import LabConfig
from py_amr_ttb.pid_speed_controller import PidSpeedController
from py_amr_ttb.wander_behavior import WanderBehavior


class Mode(Enum):
    """Persistent mode retained across a temporary R1 override."""

    STOP = 'stop'
    WANDER = 'wander'
    CRUISE = 'cruise'


class LabOneController(Node):
    """Q4: select wandering, cruise control, or temporary teleoperation."""

    def __init__(self):
        super().__init__('lab_one_controller')

        self.config = LabConfig

        # Use the existing Q2 sensor and wandering code.
        self.ir_sensor = IrSensorState(self.config)
        self.wander = WanderBehavior(self.config)

        # Start stopped until the user chooses a mode.
        self.selected_mode = Mode.STOP

        # Store current and previous joystick inputs.
        self.axes = []
        self.buttons = []
        self.previous_buttons = []
        self.last_joy_time = None

        # Cruise-control target and measured forward speed.
        self.target_speed = 0.3
        self.measured_speed = 0.0
        self.last_odom_time = None

        self.pid = PidSpeedController(self.config)

        self.last_control_time = self.now_seconds()
        self.last_warning = None

        # Only the controller timer publishes velocity commands.
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

        self.joy_subscription = self.create_subscription(
            Joy,
            self.config.JOY_TOPIC,
            self.joy_callback,
            qos_profile_sensor_data,
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
            'Ready: L1=WANDER, L2=CRUISE, hold R1=TELEOP.'
        )

    def now_seconds(self):
        """Return ROS time in seconds."""
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def reset_pid(self):
        """Clear stored error when changing modes or target speeds."""
        self.pid.reset()

    def pressed(self, index):
        """True when this button is currently held."""
        return (
            0 <= index < len(self.buttons)
            and bool(self.buttons[index])
        )

    def newly_pressed(self, index):
        """True only on the transition from released to pressed."""
        was_pressed = (
            0 <= index < len(self.previous_buttons)
            and bool(self.previous_buttons[index])
        )
        return self.pressed(index) and not was_pressed

    def joy_callback(self, message):
        """Store joystick input and handle mode/speed selections."""

        # Save old buttons before replacing them with new buttons.
        self.previous_buttons = self.buttons
        self.buttons = list(message.buttons)
        self.axes = list(message.axes)
        self.last_joy_time = self.now_seconds()

        # R1 temporarily overrides the mode; it does not replace it.
        # Ignore mode/speed selections while R1 is held.
        if self.pressed(self.config.BUTTON_R1):
            self.reset_pid()
            return

        if self.newly_pressed(self.config.BUTTON_L1):
            self.selected_mode = Mode.WANDER
            self.wander.reset()
            self.reset_pid()
            self.get_logger().info('Mode: WANDER')

        elif self.newly_pressed(self.config.BUTTON_L2):
            self.selected_mode = Mode.CRUISE
            self.reset_pid()
            self.get_logger().info('Mode: CRUISE')

        # Face buttons change the target only in cruise mode.
        if self.selected_mode is Mode.CRUISE:
            new_target = self.target_speed

            if self.newly_pressed(self.config.BUTTON_CROSS):
                new_target = 0.0
            elif self.newly_pressed(self.config.BUTTON_SQUARE):
                new_target = 0.1
            elif self.newly_pressed(self.config.BUTTON_TRIANGLE):
                new_target = 0.2
            elif self.newly_pressed(self.config.BUTTON_CIRCLE):
                new_target = 0.4

            if new_target != self.target_speed:
                self.target_speed = new_target
                self.reset_pid()
                self.get_logger().info(
                    f'Cruise target: {self.target_speed:.1f} m/s'
                )

    def ir_callback(self, message):
        """Pass the latest IR readings to the existing Q2 helper."""
        error = self.ir_sensor.update(message, self.now_seconds())
        if error is not None:
            self.get_logger().warning(error)

    def odom_callback(self, message):
        """Read actual forward speed for PID feedback."""
        speed = float(message.twist.twist.linear.x)

        if math.isfinite(speed):
            self.measured_speed = speed
            self.last_odom_time = self.now_seconds()
        else:
            self.last_odom_time = None

    def wander_command(self, now):
        """Use the existing Q2 wandering behavior."""
        return self.wander.command(now, self.ir_sensor)

    def teleop_command(self, now):
        """Convert joystick axes into forward and turning commands."""
        command = Twist()

        if self.last_joy_time is None:
            return command, 'Waiting for joystick data.'

        age = now - self.last_joy_time
        if not 0.0 <= age <= self.config.JOYSTICK_TIMEOUT:
            return command, 'Joystick data is stale: stopped.'

        forward_axis = self.config.FORWARD_AXIS
        turn_axis = self.config.TURN_AXIS

        if not (
            0 <= forward_axis < len(self.axes)
            and 0 <= turn_axis < len(self.axes)
        ):
            return command, 'Configured joystick axes are missing.'

        forward = float(self.axes[forward_axis])
        turn = float(self.axes[turn_axis])

        if not (math.isfinite(forward) and math.isfinite(turn)):
            return command, 'Invalid joystick axes: stopped.'

        command.linear.x = self.config.TELEOP_LINEAR_SCALE * forward
        command.angular.z = self.config.TELEOP_ANGULAR_SCALE * turn

        return command, None

    def cruise_command(self, now, dt):
        """Use odometry feedback to maintain the selected forward speed."""
        command = Twist()

        # X must stop immediately, including any stored PID correction.
        if self.target_speed == 0.0:
            self.reset_pid()
            return command, None

        if self.last_odom_time is None:
            self.reset_pid()
            return command, 'Waiting for odometry.'

        age = now - self.last_odom_time
        if not 0.0 <= age <= self.config.SENSOR_TIMEOUT:
            self.reset_pid()
            return command, 'Odometry is stale: stopped.'

        command.linear.x = self.pid.update(
            self.target_speed, self.measured_speed, dt,
        )
        return command, None

    def controller_callback(self):
        """Choose exactly one behavior and publish exactly once."""
        now = self.now_seconds()
        dt = now - self.last_control_time
        self.last_control_time = now

        # Reset PID timing after a clock jump or long interruption.
        if dt <= 0.0 or dt > self.config.SENSOR_TIMEOUT:
            self.reset_pid()
            dt = self.config.CONTROL_PERIOD

        # Highest priority: R1 joystick override.
        if self.pressed(self.config.BUTTON_R1):
            self.reset_pid()
            command, warning = self.teleop_command(now)

        elif self.selected_mode is Mode.WANDER:
            command, warning = self.wander_command(now)

        elif self.selected_mode is Mode.CRUISE:
            command, warning = self.cruise_command(now, dt)

        else:
            command, warning = Twist(), None

        self.publisher.publish(command)

        # Report a warning once instead of printing it every timer tick.
        if warning and warning != self.last_warning:
            self.get_logger().warning(warning)
        self.last_warning = warning


def main(args=None):
    """Run the combined controller and publish zero on shutdown."""
    rclpy.init(args=args)
    controller = LabOneController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            controller.publisher.publish(Twist())
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()