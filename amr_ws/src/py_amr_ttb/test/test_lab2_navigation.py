"""Exercise real Lab 2 callbacks with ROS doubles and kinematic simulation."""

import importlib
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch


class Twist:
    """Represent the command fields used by the nodes."""

    def __init__(self):
        self.linear = NS(x=0.0)
        self.angular = NS(z=0.0)


class Node:
    """Capture each topic independently without a ROS installation."""

    def __init__(self, name):
        self.outputs = {}

    def create_publisher(self, kind, topic, depth):
        self.outputs[topic] = []
        return NS(publish=self.outputs[topic].append)

    def create_subscription(self, *args):
        return args

    def declare_parameter(self, name, default):
        return NS(value=default)

    def create_timer(self, period, callback):
        return NS(cancel=lambda: None)

    def get_logger(self):
        return NS(info=lambda message: None, warning=lambda message: None)


def odom(x=0.0, y=0.0, yaw=0.0):
    """Create a planar unit-quaternion pose."""
    return NS(twist=NS(twist=NS(linear=NS(x=0.0), angular=NS(z=0.0))),
              pose=NS(pose=NS(
        position=NS(x=x, y=y),
        orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
    )))


class NavigationTests(unittest.TestCase):
    """Verify arrival, frame conversion, command limits, and sensor stops."""

    @classmethod
    def setUpClass(cls):
        source = str(Path(__file__).parents[1])
        modules = {name: ModuleType(name) for name in (
            'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.signals',
            'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
            'std_msgs', 'std_msgs.msg',
        )}
        modules['rclpy.node'].Node = Node
        modules['rclpy.qos'].qos_profile_sensor_data = object()
        modules['rclpy.signals'].SignalHandlerOptions = NS(NO=0)
        modules['geometry_msgs.msg'].Twist = Twist
        modules['nav_msgs.msg'].Odometry = object
        modules['std_msgs.msg'].Float64 = NS
        # Remove imported project modules after loading; do not contaminate Lab 1 tests.
        with patch.dict(sys.modules, modules), patch.object(sys, 'path', [source] + sys.path):
            cls.nodes = [getattr(importlib.import_module('py_amr_ttb.lab2.' + module), name)
                         for module, name in (
                             ('lab2_go_to_goal', 'Lab2GoToGoal'),
                             ('lab2_square', 'Lab2Square'), ('lab2_star', 'Lab2Star'),
                         )]

    def command(self, node):
        return node.outputs[node.config.CMD_VEL_TOPIC][-1]

    def assert_stopped(self, node):
        cmd = self.command(node)
        self.assertEqual((cmd.linear.x, cmd.angular.z), (0.0, 0.0))

    def test_missing_stale_invalid_and_recovered_odometry(self):
        for cls in self.nodes:
            with self.subTest(node=cls.__name__), patch('time.monotonic', return_value=10):
                node = cls()
                node.controller_callback()
                self.assert_stopped(node)
                node.odom_callback(odom())
                node.controller_callback()
                command = self.command(node)
                self.assertNotEqual((command.linear.x, command.angular.z), (0, 0))
                with patch('time.monotonic', return_value=11):
                    node.controller_callback()
                    self.assert_stopped(node)
                bad = odom()
                bad.pose.pose.orientation.w = 0
                node.odom_callback(bad)
                node.controller_callback()
                self.assert_stopped(node)
                node.odom_callback(odom(float('nan')))
                self.assertIsNone(node.last_odom_time)
                node.odom_callback(odom())
                node.controller_callback()
                command = self.command(node)
                self.assertNotEqual((command.linear.x, command.angular.z), (0, 0))

    def test_all_routes_complete_from_rotated_translated_start(self):
        # Ideal differential-drive model, not a physical robot result.
        for cls in self.nodes:
            with self.subTest(node=cls.__name__), patch('time.monotonic', return_value=10):
                node = cls()
                x, y, yaw = 8.0, -3.0, 1.2
                visited = []
                for step in range(6000):
                    node.odom_callback(odom(x, y, yaw))
                    node.controller_callback()
                    cmd = self.command(node)
                    self.assertLessEqual(abs(cmd.linear.x), node.config.NAV_MAX_LINEAR_SPEED)
                    self.assertLessEqual(abs(cmd.angular.z), node.config.NAV_MAX_ANGULAR_SPEED)
                    if hasattr(node, 'waypoint_index') and node.waypoint_index not in visited:
                        visited.append(node.waypoint_index)
                    if node.goal_reached:
                        break
                    x += .1 * cmd.linear.x * math.cos(yaw)
                    y += .1 * cmd.linear.x * math.sin(yaw)
                    yaw += .1 * cmd.angular.z
                self.assertTrue(node.goal_reached, f'{cls.__name__} failed to finish')
                self.assert_stopped(node)
                if hasattr(node, 'waypoints'):
                    self.assertEqual(visited, list(range(len(node.waypoints) + 1)))
                node.controller_callback()
                self.assert_stopped(node)

    def test_goal_parameters_are_local_and_invalid_values_rejected(self):
        cls = self.nodes[0]
        original = cls().config.NAV_MAX_LINEAR_SPEED
        with patch.object(Node, 'declare_parameter',
                          side_effect=lambda name, default: NS(
                              value=0.15 if name == 'max_linear_speed' else default)):
            node = cls()
            self.assertEqual(node.config.NAV_MAX_LINEAR_SPEED, 0.15)
        self.assertEqual(cls().config.NAV_MAX_LINEAR_SPEED, original)
        for bad in (0.0, -1.0, float('nan'), float('inf')):
            with patch.object(Node, 'declare_parameter', return_value=NS(value=bad)):
                with self.assertRaises(ValueError):
                    cls()

    def test_goal_overshoot_diagnostics_and_latched_stop(self):
        node = self.nodes[0]()
        prefix = node.config.ROBOT_NAMESPACE + '/lab2_go_to_goal/'
        with patch('time.monotonic', return_value=10):
            node.odom_callback(odom())
            node.odom_callback(odom(1.5, 1.5))
            node.controller_callback()
            self.assertTrue(node.goal_reached)
            # Residual motion after arrival remains visible, without restarting.
            node.odom_callback(odom(1.6, 1.6))
            node.controller_callback()
        self.assertLess(node.outputs[prefix + 'remaining_along_route'][-1].data, 0)
        self.assertGreater(node.outputs[prefix + 'distance'][-1].data, 0)
        self.assertEqual(node.outputs[prefix + 'command_linear'][-1].data, 0)
        self.assert_stopped(node)


if __name__ == '__main__':
    unittest.main()
