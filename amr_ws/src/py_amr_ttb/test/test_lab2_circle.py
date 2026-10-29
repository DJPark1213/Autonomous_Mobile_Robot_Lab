"""Test pursuit geometry and real circle-node callbacks with ideal kinematics."""

import importlib
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from py_amr_ttb.lab2.pure_pursuit import (  # noqa: E402
    CirclePath, CircleProgress, pursuit_command, moving_goal_command,
)


class Twist:
    """Capture velocity commands without ROS."""

    def __init__(self):
        self.linear = NS(x=0.0)
        self.angular = NS(z=0.0)


class Node:
    """Implement the node interfaces used by the circle controller."""

    overrides = {}

    def __init__(self, name):
        self.outputs = {}
        self.parameters = {}

    def declare_parameter(self, name, default):
        self.parameters[name] = NS(value=self.overrides.get(name, default))
        return self.parameters[name]

    def get_parameter(self, name):
        return self.parameters[name]

    def add_on_set_parameters_callback(self, callback):
        return callback

    def create_publisher(self, kind, topic, depth):
        self.outputs[topic] = []
        return NS(publish=self.outputs[topic].append)

    def create_subscription(self, *args):
        return args

    def create_timer(self, period, callback):
        return NS(cancel=lambda: None)

    def get_logger(self):
        return NS(info=lambda message: None, warning=lambda message: None)


def odom(x=0.0, y=0.0, yaw=0.0, speed=0.0):
    """Make a planar pose fixture."""
    return NS(twist=NS(twist=NS(linear=NS(x=speed))), pose=NS(pose=NS(
        position=NS(x=x, y=y),
        orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
    )))


class CircleTests(unittest.TestCase):
    """Check stopping and convergence; simulations are not robot measurements."""

    @classmethod
    def setUpClass(cls):
        modules = {name: ModuleType(name) for name in (
            'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.signals',
            'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
            'std_msgs', 'std_msgs.msg', 'rcl_interfaces', 'rcl_interfaces.msg',
        )}
        modules['rclpy.node'].Node = Node
        modules['rclpy.qos'].qos_profile_sensor_data = object()
        modules['rclpy.signals'].SignalHandlerOptions = NS(NO=0)
        modules['geometry_msgs.msg'].Twist = Twist
        modules['nav_msgs.msg'].Odometry = object
        modules['std_msgs.msg'].Float64 = NS
        modules['std_msgs.msg'].String = NS
        modules['rcl_interfaces.msg'].SetParametersResult = NS
        with patch.dict(sys.modules, modules):
            cls.controller = importlib.import_module('py_amr_ttb.lab2.lab2_circle').Lab2Circle

    def command(self, node):
        return node.outputs[node.cmd_topic][-1]

    def assert_stopped(self, node):
        command = self.command(node)
        self.assertEqual((command.linear.x, command.angular.z), (0.0, 0.0))

    def test_circle_intersection_and_curvature_in_both_directions(self):
        for direction in (-1, 1):
            circle = CirclePath(.75, direction)
            for phase in (0, 1.3, 3.14, 5.8):
                x, y = circle.point(phase)
                tx, ty = circle.target(x, y, .30)
                self.assertAlmostEqual(math.hypot(tx - x, ty - y), .30)
                v, w = pursuit_command(x, y, direction * phase, tx, ty, .12, .2, .4)
                self.assertAlmostEqual(w / v, direction / .75)

    def test_progress_crosses_branch_cut_and_subtracts_reverse(self):
        for direction in (-1, 1):
            path = CirclePath(.75, direction)
            progress = CircleProgress(path)
            for i in range(1001):
                progress.update(*path.point(i * 4 * math.pi / 1000))
            self.assertAlmostEqual(progress.radians, 4 * math.pi)
            for i in range(101):
                progress.update(*path.point(4 * math.pi - i * .01))
            self.assertAlmostEqual(progress.radians, 4 * math.pi - 1)

    def test_missing_stale_invalid_recovery_and_completion_not_at_start(self):
        with patch('time.monotonic', return_value=10) as clock:
            node = self.controller()
            node.controller_callback()
            self.assert_stopped(node)
            node.odom_callback(odom())
            node.controller_callback()
            self.assertFalse(node.goal_reached)
            self.assertGreater(self.command(node).linear.x, 0)
            with patch('time.monotonic', return_value=11):
                node.controller_callback()
                self.assert_stopped(node)
            clock.return_value = 11.1
            node.odom_callback(odom(float('nan')))
            node.controller_callback()
            self.assert_stopped(node)
            node.odom_callback(odom())
            node.controller_callback()
            self.assertGreater(self.command(node).linear.x, 0)
            # Rotating in place is not a completed lap.
            for i in range(100):
                node.odom_callback(odom(0, 0, i * .1))
                node.controller_callback()
            self.assertFalse(node.goal_reached)
            self.assertEqual(node.progress.radians, 0)

    def test_rotated_translated_start_one_and_two_laps_with_disturbance(self):
        for direction, laps, radius, lookahead in ((1, 1, .75, .30), (-1, 2, .50, .25)):
            with self.subTest(direction=direction), patch('time.monotonic', return_value=10) as clock, \
                    patch.object(Node, 'overrides', {'direction': direction, 'laps': laps,
                                                    'radius': radius, 'lookahead': lookahead}):
                node = self.controller()
                x, y, yaw = 8.0, -3.0, 1.2
                measured = 0.0
                for step in range(3000):
                    clock.return_value = 10 + step * .1
                    node.odom_callback(odom(x, y, yaw, measured))
                    node.controller_callback()
                    cmd = self.command(node)
                    self.assertLessEqual(abs(cmd.linear.x), .20)
                    self.assertLessEqual(abs(cmd.angular.z), .40 + 1e-12)
                    if node.goal_reached:
                        break
                    measured = cmd.linear.x
                    # Exact integration of the ideal constant-twist step.
                    dt = .1
                    if abs(cmd.angular.z) > 1e-10:
                        ratio = cmd.linear.x / cmd.angular.z
                        x += ratio * (math.sin(yaw + dt * cmd.angular.z) - math.sin(yaw))
                        y -= ratio * (math.cos(yaw + dt * cmd.angular.z) - math.cos(yaw))
                    else:
                        x += dt * cmd.linear.x * math.cos(yaw)
                        y += dt * cmd.linear.x * math.sin(yaw)
                    yaw += dt * cmd.angular.z
                    if step == 150:
                        x += .06
                        y -= .04
                self.assertTrue(node.goal_reached, 'did not finish circle')
                self.assertTrue(node.target.done)
                tx, ty = node.path.point(node.target.phase)
                self.assertLessEqual(math.hypot(tx-node.current_x, ty-node.current_y), .05)
                self.assert_stopped(node)
                node.controller_callback()
                self.assert_stopped(node)

    def test_off_circle_robot_pursues_goal_without_path_error_stop(self):
        with patch('time.monotonic', return_value=10):
            node = self.controller()
            node.odom_callback(odom())
            node.odom_callback(odom(2, 0, math.pi))
            node.controller_callback()
            self.assertFalse(node.goal_reached)
            self.assertGreater(self.command(node).linear.x, 0)

    def test_independent_live_speeds_and_stale_target_clock(self):
        with patch('time.monotonic', return_value=10) as clock:
            node = self.controller()
            node.odom_callback(odom())
            node.parameters['robot_speed'].value = 0.0
            node.controller_callback()
            self.assert_stopped(node)
            phase = node.target.phase
            clock.return_value = 11
            node.controller_callback()
            self.assert_stopped(node)
            self.assertAlmostEqual(node.target.phase-phase, .12/.75)
            node.parameters['robot_speed'].value = .08
            node.parameters['target_speed'].value = 0.0
            node.odom_callback(odom())
            node.controller_callback()
            frozen = node.target.phase
            clock.return_value = 11.1
            node.odom_callback(odom())
            node.controller_callback()
            self.assertAlmostEqual(node.target.phase, frozen)
            self.assertGreater(self.command(node).linear.x, 0)
            self.assertLessEqual(self.command(node).linear.x, .08)

    def test_target_completion_waits_for_robot_then_latches_stop(self):
        with patch.object(Node, 'overrides', {'duration': .1}), \
                patch('time.monotonic', return_value=10) as clock:
            node = self.controller()
            node.odom_callback(odom())
            clock.return_value = 10.2
            node.odom_callback(odom())
            node.controller_callback()
            self.assertTrue(node.target.done)
            self.assertFalse(node.goal_reached)
            self.assertGreater(self.command(node).linear.x, 0)
            tx, ty = node.path.point(node.target.phase)
            node.odom_callback(odom(tx, ty))
            node.controller_callback()
            self.assertTrue(node.goal_reached)
            self.assert_stopped(node)
            node.odom_callback(odom(tx+.2, ty))
            node.controller_callback()
            self.assert_stopped(node)

    def test_follower_uses_only_goal_position_and_independent_speed(self):
        v, w = moving_goal_command(0, 0, 0, 1, 1, .1)
        self.assertGreater(v, 0)
        self.assertGreater(w, 0)
        faster, same_turn = moving_goal_command(0, 0, 0, 1, 1, .2)
        self.assertAlmostEqual(faster, 2*v)
        self.assertEqual(w, same_turn)
        self.assertEqual(moving_goal_command(0, 0, 0, 1, 1, 0), (0, 0))
        self.assertEqual(moving_goal_command(1, 1, 0, 1, 1, .1), (0, 0))
        self.assertEqual(moving_goal_command(0, 0, 0, -1, 0, .1)[0], 0)

    def test_live_speed_validation(self):
        node = self.controller()
        for name in ('robot_speed', 'target_speed'):
            for bad in (-1, float('nan'), float('inf'), True):
                self.assertFalse(node.validate_parameters([NS(name=name, value=bad)]).successful)
            self.assertTrue(node.validate_parameters([NS(name=name, value=0.0)]).successful)
        self.assertFalse(node.validate_parameters([NS(name='robot_speed', value=.21)]).successful)
        self.assertTrue(node.validate_parameters([NS(name='target_speed', value=.3)]).successful)

    def test_speed_limit_preserves_curvature_and_bad_parameters_fail(self):
        v, w = pursuit_command(0, 0, 0, .05, .05, .2, .2, .4)
        self.assertLess(v, .2)
        self.assertAlmostEqual(w, .4)
        self.assertAlmostEqual(w / v, 20)
        for overrides in ({'radius': 0.0}, {'robot_speed': float('nan')}, {'robot_speed': .21}, {'target_speed': -1}, {'lookahead': 2.0},
                          {'laps': 0}, {'direction': 0}):
            with self.subTest(overrides=overrides), patch.object(Node, 'overrides', overrides):
                with self.assertRaises(ValueError):
                    self.controller()


if __name__ == '__main__':
    unittest.main()
