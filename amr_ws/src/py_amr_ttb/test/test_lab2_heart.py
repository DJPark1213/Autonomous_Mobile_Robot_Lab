"""Heart geometry and real controller tests with ideal differential-drive motion."""

import importlib
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from test_lab2_navigation import Node, Twist, odom


class HeartNode(Node):
    """Extend the existing ROS double with parameter and shutdown interfaces."""

    overrides = {}

    def declare_parameter(self, name, default):
        return NS(value=self.overrides.get(name, default))

    def destroy_node(self):
        self.destroyed = True


class HeartTests(unittest.TestCase):
    """Verify sharp points, ordered completion, curved commands and data stops."""

    @classmethod
    def setUpClass(cls):
        source = str(Path(__file__).resolve().parents[1])
        modules = {name: ModuleType(name) for name in (
            'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.signals',
            'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
            'std_msgs', 'std_msgs.msg',
        )}
        modules['rclpy.node'].Node = HeartNode
        modules['rclpy.qos'].qos_profile_sensor_data = object()
        modules['rclpy.signals'].SignalHandlerOptions = NS(NO=0)
        modules['geometry_msgs.msg'].Twist = Twist
        modules['nav_msgs.msg'].Odometry = object
        modules['std_msgs.msg'].Float64 = NS
        with patch.dict(sys.modules, modules), patch.object(sys, 'path', [source] + sys.path):
            cls.module = importlib.import_module('py_amr_ttb.lab2.lab2_heart')
            cls.geometry = importlib.import_module('py_amr_ttb.lab2.navigation')

    def setUp(self):
        HeartNode.overrides = {}

    def command(self, node):
        return node.outputs[node.config.CMD_VEL_TOPIC][-1]

    def assert_stopped(self, node):
        command = self.command(node)
        self.assertEqual((command.linear.x, command.angular.z), (0.0, 0.0))

    def test_geometry_width_closure_notch_and_near_uniform_spacing(self):
        points = self.geometry.heart_points(1.0)
        self.assertEqual(points[0], (0, 0))
        self.assertEqual(points[-1], (0, 0))
        self.assertEqual(points[200], (22 / 32, 0))
        self.assertAlmostEqual(max(y for x, y in points) - min(y for x, y in points),
                               1.0, delta=.001)
        self.assertTrue(all(y >= -1e-12 for x, y in points[:201]))
        self.assertTrue(all(y <= 1e-12 for x, y in points[200:]))
        distances = [math.dist(a, b) for a, b in zip(points, points[1:])]
        self.assertLess(max(distances) / min(distances), 1.02)

    def test_progress_does_not_jump_lobes_or_complete_at_start(self):
        path = self.geometry.HeartPath(1.0)
        path.update(0, 0, .24)
        self.assertFalse(path.section_reached(0, 0, .03))
        self.assertEqual(path.section, 0)
        for point in path.sections[0]:
            path.update(*point, .24)
        self.assertTrue(path.section_reached(*path.endpoint, .03))
        self.assertEqual(path.target(.12), path.endpoint)
        self.assertAlmostEqual(path.progress, .5)
        path.advance_section()
        self.assertFalse(path.section_reached(0, 0, .03))
        for point in path.sections[1]:
            path.update(*point, .24)
        self.assertTrue(path.section_reached(0, 0, .03))
        self.assertAlmostEqual(path.progress, 1.0)

    def test_invalid_geometry_and_controller_parameters_rejected(self):
        for width in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                self.geometry.heart_points(width)
        for overrides in ({'speed': .5}, {'lookahead': .5}, {'width': .2}):
            HeartNode.overrides = overrides
            with self.assertRaises(ValueError):
                self.module.Lab2Heart()

    def test_sensor_stops_recovery_and_latched_tracking_failure(self):
        with patch('time.monotonic', return_value=10):
            node = self.module.Lab2Heart()
            node.controller_callback()
            self.assert_stopped(node)
            node.odom_callback(odom())
            node.controller_callback()
            self.assertFalse(node.goal_reached)
            with patch('time.monotonic', return_value=11):
                node.controller_callback()
                self.assert_stopped(node)
            invalid = odom(float('nan'))
            node.odom_callback(invalid)
            node.controller_callback()
            self.assert_stopped(node)
            node.odom_callback(odom())
            node.controller_callback()
            a = node.path.point_at(0)
            b = node.path.point_at(min(node.lookahead / 3, node.width / 30))
            aligned = math.atan2(b[1] - a[1], b[0] - a[0])
            node.odom_callback(odom(0, 0, aligned))
            node.controller_callback()
            node.controller_callback()
            self.assertGreater(self.command(node).linear.x, 0)
            node.odom_callback(odom(2, 2))
            node.controller_callback()
            self.assertTrue(node.tracking_failed)
            self.assert_stopped(node)
            node.odom_callback(odom())
            node.controller_callback()
            self.assert_stopped(node)

    def test_curved_motion_notch_dwell_and_completion_with_rotated_origin(self):
        for width, lookahead in ((1.0, .12), (.6, .10), (1.4, .15)):
            with self.subTest(width=width):
                HeartNode.overrides = {'width': width, 'lookahead': lookahead}
                node = self.module.Lab2Heart()
                x, y, yaw = 4.0, -2.0, 1.1
                clock = [0.0]
                states = []
                notch_ticks = 0
                curved_ticks = 0
                with patch('time.monotonic', side_effect=lambda: clock[0]):
                    for step in range(12000):
                        clock[0] = step * .1
                        if step == 120:
                            # A small displacement requires actual feedback correction.
                            x += .02
                            y -= .015
                        if step == 200:
                            progress = node.path.progress
                            clock[0] += 1
                            node.controller_callback()
                            self.assert_stopped(node)
                            self.assertEqual(node.path.progress, progress)
                            clock[0] -= 1
                        node.odom_callback(odom(x, y, yaw))
                        node.controller_callback()
                        command = self.command(node)
                        self.assertLessEqual(abs(command.linear.x), node.speed)
                        self.assertLessEqual(abs(command.angular.z), .4 + 1e-12)
                        if not states or states[-1] != node.state:
                            states.append(node.state)
                        if node.state == 'CUSP_STOP':
                            notch_ticks += 1
                            self.assert_stopped(node)
                        if command.linear.x > 0 and abs(command.angular.z) > .01:
                            curved_ticks += 1
                        if node.goal_reached or node.tracking_failed:
                            break
                        x += .1 * command.linear.x * math.cos(yaw)
                        y += .1 * command.linear.x * math.sin(yaw)
                        yaw += .1 * command.angular.z
                    self.assertTrue(node.goal_reached, (node.state, node.path.progress,
                                                        node.path_error, node.path.remaining))
                    self.assertEqual(states, ['ALIGN', 'FOLLOW', 'CUSP_STOP',
                                              'ALIGN', 'FOLLOW', 'DONE'])
                    self.assertGreaterEqual(notch_ticks, 3)
                    self.assertGreater(curved_ticks, 30)
                    self.assertLessEqual(math.hypot(node.current_x, node.current_y), .03)
                    self.assertGreater(node.path.progress, .97)
                    self.assert_stopped(node)
                    node.controller_callback()
                    self.assert_stopped(node)

    def test_normal_shutdown_publishes_zero(self):
        node = self.module.Lab2Heart()
        ros = self.module.rclpy
        with patch.object(self.module, 'Lab2Heart', return_value=node), \
                patch.object(ros, 'init', create=True), \
                patch.object(ros, 'spin', side_effect=KeyboardInterrupt, create=True), \
                patch.object(ros, 'ok', return_value=True, create=True), \
                patch.object(ros, 'shutdown', create=True):
            self.module.main()
        self.assert_stopped(node)
        self.assertTrue(node.destroyed)


if __name__ == '__main__':
    unittest.main()
