"""Check PID saturation, heading wrap, recovery hysteresis and speed restart."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from py_amr_ttb.lab2.pid import PID
from py_amr_ttb.lab2.motion_control import (
    AlignmentRecovery, CircleSpeedController, GoalController,
)
from py_amr_ttb.lab_config import LabConfig


class PIDTests(unittest.TestCase):
    def test_integral_derivative_and_reset(self):
        pid = PID(1, 1, .1)
        self.assertAlmostEqual(pid.update(.2, .1, -1, 1), .22)
        self.assertAlmostEqual(pid.update(.3, .1, -1, 1), .45)
        pid.reset()
        self.assertEqual(pid.integral, 0)
        self.assertIsNone(pid.previous_error)
        self.assertEqual(pid.update(.2, 0, -1, 1), 0)

    def test_antiwindup_at_both_limits_and_unwinding(self):
        pid = PID(1, 1, 0)
        for _ in range(100):
            self.assertEqual(pid.update(10, .1, 0, .2), .2)
        self.assertEqual(pid.integral, 0)
        for _ in range(100):
            self.assertEqual(pid.update(-10, .1, 0, .2), 0)
        self.assertEqual(pid.integral, 0)
        pid.integral = .1
        pid.update(-.01, .1, 0, .2)
        self.assertLess(pid.integral, .1)

    def test_heading_derivative_wraps_at_pi(self):
        pid = PID(0, 0, 1, angular=True)
        pid.update(math.pi - .01, .1, -100, 100)
        self.assertAlmostEqual(pid.update(-math.pi + .01, .1, -100, 100), .2)

    def test_recovery_hysteresis(self):
        recovery = AlignmentRecovery(1.2, .35)
        self.assertEqual(recovery.update(1.3), 'ALIGN')
        self.assertEqual(recovery.update(1.1), 'ALIGN')
        self.assertEqual(recovery.update(.34), 'FOLLOW')
        self.assertEqual(recovery.update(.5), 'FOLLOW')
        self.assertEqual(recovery.update(-1.3), 'ALIGN')
        recovery.reset()
        self.assertEqual(recovery.state, 'STOP')

    def test_speed_feedback_ramp_and_curvature_limit(self):
        control = CircleSpeedController(.1, .01, .005, .15)
        previous = 0
        for _ in range(100):
            speed = control.update(.12, 0, 1 / .75, .1, .2, .4)
            self.assertLessEqual(speed - previous, .015 + 1e-12)
            previous = speed
        self.assertGreater(speed, .12)  # Correct measured underspeed.
        speed = control.update(.12, 0, 20, .1, .2, .4)
        self.assertLessEqual(speed * 20, .4)
        control.reset()
        self.assertEqual(control.pid.integral, 0)
        self.assertAlmostEqual(control.update(.12, 0, 1, .1, .2, .4), .015)

    def test_goal_turning_and_arrival_reset_distance_pid(self):
        control = GoalController(LabConfig(), .01, .02, .01, .03, .2)
        v, w, reached = control.update(0, 0, math.pi, 1, 0, .1)
        self.assertEqual(v, 0)
        self.assertNotEqual(w, 0)
        self.assertFalse(reached)
        self.assertEqual(control.distance_pid.integral, 0)
        v, _, _ = control.update(0, 0, 0, .1, 0, .1)
        self.assertGreater(v, 0)
        self.assertGreater(control.distance_pid.integral, 0)
        self.assertEqual(control.update(.1, 0, 0, .1, 0, .1), (0, 0, True))
        self.assertEqual(control.distance_pid.integral, 0)
        self.assertIsNone(control.heading_pid.previous_error)


if __name__ == '__main__':
    unittest.main()
