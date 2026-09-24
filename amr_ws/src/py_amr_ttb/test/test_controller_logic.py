"""Offline behavior checks with ROS interface doubles, not transport tests."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


class Twist:
    """Represent the velocity fields used by the controller."""

    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


class Node:
    """Capture subscriptions and outputs without a ROS installation."""

    def __init__(self, name):
        self.time = 10.0
        self.commands = []
        self.subscriptions = []
        self.logs = []

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(self.time * 1_000_000_000),
        ))

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append, warning=self.logs.append)

    def create_publisher(self, message_type, topic, depth):
        self.command_topic = topic
        return SimpleNamespace(publish=self.commands.append)

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions.append((topic, callback))
        return callback

    def create_timer(self, period, callback):
        return SimpleNamespace(period=period, callback=callback)

    def destroy_node(self):
        pass


def load_modules():
    """Load real project modules with temporary, isolated ROS doubles."""
    modules = {}
    for name in (
        'rclpy', 'rclpy.node', 'rclpy.qos', 'geometry_msgs',
        'geometry_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg',
        'nav_msgs', 'nav_msgs.msg', 'irobot_create_msgs',
        'irobot_create_msgs.msg', 'py_amr_ttb',
    ):
        modules[name] = ModuleType(name)
    modules['rclpy.node'].Node = Node
    modules['rclpy.qos'].qos_profile_sensor_data = object()
    modules['geometry_msgs.msg'].Twist = Twist
    modules['sensor_msgs.msg'].Joy = object
    modules['nav_msgs.msg'].Odometry = object
    modules['irobot_create_msgs.msg'].IrIntensityVector = object
    modules['rclpy'].init = lambda args=None: None
    modules['rclpy'].spin = lambda node: None
    modules['rclpy'].ok = lambda: True
    modules['rclpy'].shutdown = lambda: None
    source = Path(__file__).resolve().parents[1] / 'py_amr_ttb'
    with patch.dict(sys.modules, modules):
        for name in (
            'lab_config', 'ir_sensor', 'wander_behavior',
            'pid_speed_controller', 'lab_one_controller',
        ):
            key = 'py_amr_ttb.' + name
            spec = importlib.util.spec_from_file_location(
                key, source / (name + '.py'),
            )
            module = importlib.util.module_from_spec(spec)
            sys.modules[key] = module
            spec.loader.exec_module(module)
            modules[name] = module
    return modules


class ControllerTests(unittest.TestCase):
    """Exercise the pasted logic against the real IR and wander modules."""

    @classmethod
    def setUpClass(cls):
        cls.modules = load_modules()

    def setUp(self):
        controller = self.modules['lab_one_controller']
        self.node = controller.LabOneController()
        self.mode = controller.Mode
        self.config = self.node.config

    def joy(self, *pressed, axes=None):
        buttons = [0] * 8
        for index in pressed:
            buttons[index] = 1
        self.node.joy_callback(SimpleNamespace(
            buttons=buttons, axes=axes if axes is not None else [0.0] * 5,
        ))

    def odom(self, speed):
        self.node.odom_callback(SimpleNamespace(twist=SimpleNamespace(
            twist=SimpleNamespace(linear=SimpleNamespace(x=speed)),
        )))

    def ir(self, front=0):
        values = [0, front, front, front, front, 0]
        self.node.ir_callback(SimpleNamespace(readings=[
            SimpleNamespace(value=value) for value in values
        ]))

    def tick(self, seconds=0.1):
        self.node.time += seconds
        before = len(self.node.commands)
        self.node.controller_callback()
        self.assertEqual(len(self.node.commands), before + 1)
        return self.node.commands[-1]

    def assert_stopped(self, command):
        self.assertEqual(list(vars(command.linear).values()), [0.0] * 3)
        self.assertEqual(list(vars(command.angular).values()), [0.0] * 3)

    def test_startup_is_stopped_and_topics_are_connected(self):
        self.assertIs(self.node.selected_mode, self.mode.STOP)
        self.assert_stopped(self.tick())
        self.assertEqual(self.node.command_topic, self.config.CMD_VEL_TOPIC)
        self.assertEqual({t for t, _ in self.node.subscriptions}, {
            self.config.JOY_TOPIC,
            self.config.IR_TOPIC,
            self.config.ODOM_TOPIC,
        })

    def test_wander_uses_existing_ir_and_stops_before_turning(self):
        self.ir()
        self.joy(self.config.BUTTON_L1)
        self.assertEqual(
            self.tick().linear.x, self.config.WANDER_FORWARD_SPEED,
        )
        self.ir(100)
        self.assert_stopped(self.tick())
        self.assertEqual(
            abs(self.tick().angular.z), self.config.WANDER_TURN_SPEED,
        )

    def test_wander_stops_for_bad_ir(self):
        self.joy(self.config.BUTTON_L1)
        self.assert_stopped(self.tick())
        self.ir()
        self.assert_stopped(self.tick(0.6))
        self.node.ir_callback(SimpleNamespace(readings=[]))
        self.assert_stopped(self.tick())

    def test_cruise_initial_target_and_feedforward_pid(self):
        self.joy(self.config.BUTTON_L2)
        self.odom(0.0)
        self.assertAlmostEqual(self.tick().linear.x, 0.33003)

    def test_cruise_buttons_and_x_clear_pid(self):
        self.joy(self.config.BUTTON_L2)
        for button, speed in (
            (self.config.BUTTON_SQUARE, 0.1),
            (self.config.BUTTON_TRIANGLE, 0.2),
            (self.config.BUTTON_CIRCLE, 0.4),
            (self.config.BUTTON_CROSS, 0.0),
        ):
            self.joy()
            self.joy(button)
            self.assertEqual(self.node.target_speed, speed)
            self.assertEqual(self.node.pid.integral, 0.0)
        self.assert_stopped(self.tick())

    def test_face_buttons_do_not_change_wander_target(self):
        self.joy(self.config.BUTTON_L1)
        self.joy(self.config.BUTTON_CIRCLE)
        self.assertEqual(self.node.target_speed, 0.3)

    def test_held_mode_button_does_not_repeat_reset(self):
        self.joy(self.config.BUTTON_L2)
        self.node.pid.integral = 0.25
        self.joy(self.config.BUTTON_L2)
        self.assertEqual(self.node.pid.integral, 0.25)

    def test_r1_overrides_and_release_returns_to_cruise(self):
        self.joy(self.config.BUTTON_L2)
        self.joy(
            self.config.BUTTON_R1, self.config.BUTTON_L1,
            self.config.BUTTON_CIRCLE, axes=[0, 0.8, 0, -0.5, 0],
        )
        command = self.tick()
        self.assertAlmostEqual(command.linear.x, 0.4)
        self.assertAlmostEqual(command.angular.z, -0.1)
        self.assertIs(self.node.selected_mode, self.mode.CRUISE)
        self.assertEqual(self.node.target_speed, 0.3)
        self.joy()
        self.odom(0.3)
        self.assertAlmostEqual(self.tick().linear.x, 0.3)

    def test_override_returns_to_wander(self):
        self.joy(self.config.BUTTON_L1)
        self.joy(self.config.BUTTON_R1)
        self.assert_stopped(self.tick())
        self.joy()
        self.ir()
        self.assertEqual(
            self.tick().linear.x, self.config.WANDER_FORWARD_SPEED,
        )

    def test_stale_held_override_stays_stopped(self):
        self.joy(self.config.BUTTON_L2)
        self.joy(self.config.BUTTON_R1, axes=[0, 1, 0, 0, 0])
        self.odom(0.3)
        self.assert_stopped(self.tick(0.6))
        self.assertIs(self.node.selected_mode, self.mode.CRUISE)

    def test_short_and_nonfinite_axes_stop_override(self):
        for axes in ([], [0, float('nan'), 0, 0, 0]):
            self.joy(self.config.BUTTON_R1, axes=axes)
            self.assert_stopped(self.tick())

    def test_cruise_stops_on_bad_or_stale_odometry(self):
        self.joy(self.config.BUTTON_L2)
        self.assert_stopped(self.tick())
        self.odom(0.1)
        self.assert_stopped(self.tick(0.6))
        self.odom(float('nan'))
        self.assert_stopped(self.tick())

    def test_clock_jump_resets_pid_timing(self):
        self.joy(self.config.BUTTON_L2)
        self.node.pid.previous_error = 100.0
        self.node.time -= 2.0
        self.odom(0.0)
        self.assertAlmostEqual(self.tick().linear.x, 0.33003)

    def test_pid_saturation_does_not_accumulate_error(self):
        pid = self.node.pid
        self.assertEqual(pid.update(0.4, -10.0, 0.1), 0.5)
        self.assertEqual(pid.integral, 0.0)
        pid.reset()
        self.assertEqual(pid.update(0.1, 10.0, 0.1), 0.0)
        self.assertEqual(pid.integral, 0.0)

    def test_pid_stop_and_bad_time_reset_history(self):
        for setpoint, measurement, dt in (
            (0, 0.3, 0.1), (0.3, 0, 0), (0.3, 0, -1),
            (0.3, float('nan'), 0.1), (0.3, 0, float('nan')),
        ):
            self.node.pid.integral = 0.5
            self.assertEqual(
                self.node.pid.update(setpoint, measurement, dt), 0,
            )
            self.assertEqual(self.node.pid.integral, 0.0)
            self.assertIsNone(self.node.pid.previous_error)

    def test_shutdown_publishes_zero(self):
        module = self.modules['lab_one_controller']
        with patch.object(module, 'LabOneController', return_value=self.node):
            module.main()
        self.assert_stopped(self.node.commands[-1])


if __name__ == '__main__':
    unittest.main()
