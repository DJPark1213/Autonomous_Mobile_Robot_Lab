"""Behavioral regressions for unknown-space navigation and reuse of learned runs."""

from dataclasses import replace
import math
import json
import multiprocessing as mp
from pathlib import Path
from queue import Queue
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from py_amr_ttb.lab_config import Lab3Config
from py_amr_ttb.lab3.geometry import (
    process_scan, observed, observed_travel, compose, inverse_points, circles_clear,
    circles_observed, scan_at_pose, transform_points,
)
from py_amr_ttb.lab3.mapping import OccupancyMap
from py_amr_ttb.lab3.memory import RunMemory
from py_amr_ttb.lab3.motion import MotionController
from py_amr_ttb.lab3.trajectory import rollout, approach, evaluate_trajectory, capsule_clearance
from py_amr_ttb.lab3.replay import optimize_route
from py_amr_ttb.lab3.simulate import simulate, raycast, course, plant, wall_distance
from py_amr_ttb.lab3.state import StateTracker, PoseEstimator
from py_amr_ttb.lab3.types import RobotState, Target, Plan
from py_amr_ttb.lab3.worker import NavigationEngine, Work, put_latest, take_latest, worker_main


def scan(config, values=None, pose=(0., 0., 0.), sequence=0, stamp=0.):
    return process_scan(np.full(360, np.inf) if values is None else values,
                        -math.pi, math.pi/180, .03, 6., config, pose=pose,
                        sequence=sequence, stamp=stamp, received=stamp)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Lab3Config()

    def test_completed_but_old_plan_still_expires(self):
        state = RobotState((0., 0., 0.), 0., 0., 3., 3.)
        plan = Plan(Target((1., 0.)), (0., 0., 0.), 0.,
                    diagnostics={'completed_at': 3.})
        result = MotionController(self.cfg).step_plan(
            state, scan(self.cfg, stamp=3.), plan, 3., .05)
        self.assertEqual(result.reason, 'WAITING_FOR_FRESH_PLAN')


    def test_history_combines_complementary_coverage(self):
        values = np.full(360, 3.)
        values[175:186] = np.nan
        current = scan(self.cfg, values, stamp=.2)
        previous = scan(self.cfg, values, pose=(0., 0., math.pi/2), stamp=.1)
        combined = replace(current, history=(previous,))
        self.assertFalse(circles_observed(current, [[.4, 0.]], .22)[0])
        self.assertTrue(circles_observed(combined, [[.4, 0.]], .22)[0])

    def test_history_does_not_override_current_obstacle(self):
        values = np.full(360, 3.)
        values[180] = .4
        current = replace(scan(self.cfg, values), history=(scan(self.cfg),))
        self.assertFalse(circles_observed(current, [[.4, 0.]], .22)[0])

    def test_footprint_can_require_coverage_from_both_scans(self):
        first = np.full(360, 3.)
        second = first.copy()
        first[170:174] = np.nan
        second[187:191] = np.nan
        current, previous = scan(self.cfg, first), scan(self.cfg, second)
        self.assertFalse(circles_observed(current, [[.5, 0.]], .22)[0])
        self.assertFalse(circles_observed(previous, [[.5, 0.]], .22)[0])
        self.assertTrue(circles_observed(replace(current, history=(previous,)),
                                         [[.5, 0.]], .22)[0])

    def test_recent_obstacle_is_retained(self):
        values = np.full(360, 3.)
        values[180] = .4
        combined = replace(scan(self.cfg), history=(scan(self.cfg, values),))
        self.assertFalse(circles_observed(combined, [[.4, 0.]], .22)[0])

    def test_expired_history_cannot_authorize_motion(self):
        values = np.full(360, 3.)
        values[175:186] = np.nan
        current = replace(scan(self.cfg, values, stamp=1.),
                          history=(scan(self.cfg, stamp=0.),))
        state = RobotState((0., 0., 0.), 0., 0., 1., 1.)
        self.assertFalse(MotionController(self.cfg).safe(state, current, .2, 0., 1.)[0])


    def test_rear_unknown_does_not_trap_starting_footprint(self):
        values = np.full(360, np.inf)
        values[:4] = np.nan
        values[-4:] = np.nan
        result = scan(self.cfg, values)
        self.assertGreater(observed_travel(result, 0., self.cfg.radius, 1.5), 1.4)
        self.assertEqual(observed_travel(result, math.pi, self.cfg.radius, 1.5), 0.)

    def test_occupied_coverage_still_rejects_obstacle_hits(self):
        values = np.full(360, np.inf)
        values[180] = self.cfg.radius / 2
        result = scan(self.cfg, values)
        self.assertFalse(circles_observed(result, [[0., 0.]], self.cfg.radius,
                                         occupied_radius=self.cfg.radius)[0])

    def test_no_return_is_observed_but_nan_is_unknown(self):
        values = np.full(360, np.inf)
        values[178:183] = np.nan
        s = scan(self.cfg, values)
        self.assertFalse(s.hits.any())
        self.assertFalse(observed(s, [[1., 0.]])[0])
        self.assertTrue(observed(s, [[0., 1.]])[0])
        self.assertEqual(observed_travel(s, 0., self.cfg.radius, 1.5), 0.)
        self.assertGreater(observed_travel(scan(self.cfg), 0., self.cfg.radius, 1.5), 1.4)

    def test_reject_partial_scan_and_bad_ranges(self):
        for values in (np.full(360, np.nan), np.zeros(360), np.full(360, -np.inf)):
            with self.assertRaises(ValueError):
                scan(self.cfg, values)
        with self.assertRaises(ValueError):
            process_scan([2.]*90, 0., math.pi/180, .03, 6., self.cfg)

    def test_scan_range_boundaries_preserve_known_and_hit_meanings(self):
        ranges = np.full(360, np.inf)
        ranges[:9] = [.02, .03, 1., 6., 7., 8., 8.1, np.nan, -np.inf]
        result = process_scan(
            ranges, -math.pi, math.pi / 180, .03, 8., self.cfg,
        )
        np.testing.assert_array_equal(
            result.known[:9], [False, True, True, True, True, True, False, False, False],
        )
        np.testing.assert_array_equal(
            result.hits[:9], [False, True, True, False, False, False, False, False, False],
        )
        np.testing.assert_array_equal(result.ranges[:9], [6., .03, 1., 6., 6., 6., 6., 6., 6.])
        self.assertTrue(np.isfinite(result.ranges).all())

    def test_inverted_offset_laser_keeps_hits_paired_with_sorted_bearings(self):
        ranges = np.full(360, np.inf)
        ranges[[0, 90, 180, 270]] = [1., 2., 3., 4.]
        mounting = (.1, -.2, math.pi / 2)
        inverted = process_scan(
            ranges, 0., math.pi / 180, .03, 8., self.cfg,
            transform=mounting, angle_sign=-1.,
        )
        clockwise = process_scan(
            ranges, 0., -math.pi / 180, .03, 8., self.cfg,
            transform=mounting,
        )
        self.assertTrue(np.all(np.diff(inverted.angles) >= 0))
        np.testing.assert_allclose(
            inverted.points, [[-3.9, -.2], [.1, -3.2], [2.1, -.2], [.1, .8]], atol=1e-12,
        )
        np.testing.assert_array_equal(clockwise.points, inverted.points)

    def test_reexpressing_scan_preserves_fixed_obstacles_and_acquisition_time(self):
        ranges = np.full(360, np.inf)
        ranges[[0, 90, 180]] = [1., 2., 3.]
        original = process_scan(
            ranges, -math.pi, math.pi / 180, .03, 6., self.cfg,
            pose=(2., 1., math.pi / 2), transform=(.1, -.2, 0.),
            stamp=10., received=12., sequence=7,
        )
        saved_ranges = original.ranges.copy()
        moved = scan_at_pose(original, (3., 2., math.pi))
        before = transform_points(original.points, original.pose)
        after = transform_points(moved.points, moved.pose)
        before = before[np.lexsort((before[:, 1], before[:, 0]))]
        after = after[np.lexsort((after[:, 1], after[:, 0]))]
        np.testing.assert_allclose(after, before, atol=1e-12)
        np.testing.assert_array_equal(original.ranges, saved_ranges)
        self.assertEqual((moved.stamp, moved.received, moved.sequence), (10., 12., 7))

    def test_footprints_use_each_radius_and_the_laser_origin(self):
        result = process_scan(
            np.full(360, np.inf), -math.pi, math.pi / 180, .03, 6., self.cfg,
            transform=(.3, -.1, 0.),
        )
        clear = circles_observed(result, [[6., -.1], [6., -.1]], [.2, .4])
        np.testing.assert_array_equal(clear, [True, False])

    def test_single_unknown_beam_cannot_hide_between_footprint_samples(self):
        values = np.full(360, np.inf)
        values[185] = np.nan
        self.assertFalse(circles_clear(scan(self.cfg, values), [[0., 0.]], self.cfg.radius))

    def test_footprint_prevents_passing_through_narrow_gap(self):
        walls = [((.8, -2.), (.8, -.18)), ((.8, .18), (.8, 2.))]
        s = scan(self.cfg, raycast(walls, (0., 0., 0.)))
        self.assertLess(observed_travel(s, 0., self.cfg.radius, 1.5), .8)

    def test_transform_roundtrip_includes_rotation(self):
        pose = compose((3., 4., math.pi/2), (2., 0., .1))
        np.testing.assert_allclose(pose, (3., 6., math.pi/2+.1))
        np.testing.assert_allclose(inverse_points([[3., 6.]], (3., 4., math.pi/2)), [[2., 0.]], atol=1e-10)

    def test_config_rejects_invalid_motion_and_search_limits(self):
        for values in ({'max_linear': .47}, {'max_angular': 2.47}, {'linear_braking': 0.}, {'goal_x': 100.},
                       {'local_spacing': .001}, {'angular_response_fraction': 2.},
                       {'trajectory_alignment_weight': -.1},
                       {'trajectory_speed_weight': float('nan')},
                       {'trajectory_steering_weight': float('inf')}):
            with self.assertRaises(ValueError):
                Lab3Config(**values)


class StateAndMotionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Lab3Config()


    def test_safe_command_respects_measured_acceleration(self):
        state = RobotState((0., 0., 0.), .1, .2, 0., 0.)
        plan = Plan(Target((2., 1.)), (0., 0., 0.), 0., command=(.3, 1.))
        result = MotionController(self.cfg).step_plan(state, scan(self.cfg), plan, 0., .05, (state.v, state.w))
        self.assertGreater(result.v, state.v)
        self.assertLessEqual(result.v-state.v, self.cfg.linear_acceleration*.05+1e-9)
        self.assertLessEqual(result.w-state.w, self.cfg.angular_acceleration*.05+1e-9)
        self.assertFalse(result.blocked)

    def test_command_ramp_advances_even_when_odometry_stays_at_rest(self):
        controller = MotionController(self.cfg)
        state = RobotState((0., 0., 0.), 0., 0., 0., 0.)
        plan = Plan(Target((2., 0.)), (0., 0., 0.), 0., command=(.3, 0.))
        previous = (0., 0.)
        for tick in range(1, 6):
            result = controller.step_plan(state, scan(self.cfg), plan, tick*.05, .05, previous)
            previous = (result.v, result.w)
            self.assertAlmostEqual(result.v, tick*.02)
        stopped = controller.step_plan(state, scan(self.cfg), None, .3, .05, previous)
        self.assertEqual((stopped.v, stopped.w), (0., 0.))

    def test_startup_crosses_deadband_after_delay_and_recovers_from_plan_gap(self):
        controller = MotionController(self.cfg)
        previous = (0., 0.)
        pending = []
        velocity = 0.
        position = 0.
        peak_command = 0.
        for tick in range(100):
            now = tick*.05
            state = RobotState((position, 0., 0.), velocity, 0., now, now)
            # A delayed worker leaves a brief gap after initial forward progress.
            plan = None if 40 <= tick < 45 else Plan(
                Target((3., 0.)), (0., 0., 0.), now, command=(.3, 0.))
            sensor = scan(self.cfg, pose=state.pose, stamp=now)
            result = controller.step_plan(state, sensor, plan, now, .05, previous)
            previous = (result.v, result.w)
            if plan is None:
                self.assertEqual(previous, (0., 0.))
            peak_command = max(peak_command, result.v)
            pending.append(result.v)
            delayed = pending.pop(0) if len(pending) > 4 else 0.
            desired = delayed if abs(delayed) >= .06 else 0.
            velocity += max(-.025, min(.0175, desired-velocity))
            position += velocity*.05
        self.assertGreater(peak_command, .1)
        self.assertGreater(position, .5)
        self.assertGreater(velocity, .1)

    def test_selected_zero_command_reports_planner_hold(self):
        state = RobotState((0., 0., 0.), 0., 0., 0., 0.)
        plan = Plan(Target((2., 0.)), (0., 0., 0.), 0., command=(0., 0.))
        result = MotionController(self.cfg).step_plan(state, scan(self.cfg), plan, 0., .05)
        self.assertEqual(result.reason, 'PLANNER_HOLD')
        self.assertEqual(result.diagnostics['requested_command'], (0., 0.))

    def test_stale_scan_stops_valid_plan(self):
        state = RobotState((0., 0., 0.), .2, 0., 1., 1.)
        plan = Plan(Target((2., 0.)), (0., 0., 0.), 1., command=(.3, 0.))
        result = MotionController(self.cfg).step_plan(state, scan(self.cfg), plan, 1., .05)
        self.assertEqual((result.v, result.w), (0., 0.))
        self.assertEqual(result.reason, 'SENSOR_STOP')

    def test_missing_motion_segment_does_not_replan_in_controller(self):
        state = RobotState((0., 0., 0.), 0., 0., 0., 0.)
        plan = Plan(Target((2., 0.)), (0., 0., 0.), 0.)
        result = MotionController(self.cfg).step_plan(state, scan(self.cfg), plan, 0., .05)
        self.assertEqual((result.v, result.w), (0., 0.))

    def test_invalid_command_fails_closed(self):
        state = RobotState((0., 0., 0.), 0., 0., 0., 0.)
        for command in ((float('nan'), 0.), (1., 0.), (0., 20.)):
            plan = Plan(Target((2., 0.)), (0., 0., 0.), 0., command=command)
            result = MotionController(self.cfg).step_plan(state, scan(self.cfg), plan, 0., .05)
            self.assertEqual(result.reason, 'INVALID_COMMAND')
            self.assertEqual((result.v, result.w), (0., 0.))

    def test_appearing_obstacle_overrides_unexpired_plan(self):
        state = RobotState((0., 0., 0.), .25, 0., 0., 0.)
        plan = Plan(Target((2., 0.)), (0., 0., 0.), 0., command=(.3, 0.))
        sensor = scan(self.cfg, raycast([((.3, -1.), (.3, 1.))], state.pose))
        result = MotionController(self.cfg).step_plan(state, sensor, plan, 0., .05, (state.v, state.w))
        self.assertTrue(result.blocked)
        self.assertEqual((result.v, result.w), (0., 0.))

    def test_expiry_is_bounded_by_sensor_and_segment_lifetime(self):
        state = RobotState((0., 0., 0.), 0., 0., .1, .1)
        plan = Plan(Target((2., 0.)), (0., 0., 0.), 0., command=(.3, 0.))
        result = MotionController(self.cfg).step_plan(state, scan(self.cfg, stamp=.1), plan, .1, .05)
        self.assertLessEqual(result.diagnostics['valid_until'], 2*self.cfg.map_period)

    def test_local_route_goes_around_obstacle(self):
        from py_amr_ttb.lab3.local_planner import TrajectoryPlanner
        state = RobotState((0., 0., 0.), 0., 0., 0., 0.)
        sensor = scan(self.cfg, raycast([((1., -.4), (1., .4))], state.pose))
        point, path = TrajectoryPlanner(self.cfg).guide(state, sensor, (2., 0.))
        self.assertGreater(len(path), 2)
        self.assertTrue(any(abs(p[1]) > .4 for p in path))
        self.assertGreater(point[0], 0.)

    def test_replay_stall_keeps_detour_across_planning_updates(self):
        from py_amr_ttb.lab3.planner import Planner
        cfg = replace(self.cfg, goal_x=2.)
        planner = Planner(cfg, RunMemory(cfg), replay_route=[(0., 0.), (2., 0.)])
        state = RobotState((0., 0., 0.), 0., 0., 0., 0.)
        planner.step(state, scan(cfg), None, 0.)
        first = planner.step(replace(state, received=6.), scan(cfg, stamp=6.), None, 6.)
        second = planner.step(replace(state, received=6.2), scan(cfg, stamp=6.2), None, 6.2)
        self.assertEqual(first.target.mode, 'DETOUR')
        self.assertEqual(second.target.mode, 'DETOUR')
        self.assertEqual(second.target.point, first.target.point)

    def test_planned_command_has_verified_stopping_sweep(self):
        from py_amr_ttb.lab3.local_planner import TrajectoryPlanner
        from py_amr_ttb.lab3.trajectory import evaluate_trajectory
        state = RobotState((0., 0., 0.), .1, 0., 0., 0.)
        sensor = scan(self.cfg)
        command, details = TrajectoryPlanner(self.cfg).choose(state, sensor, (2., .4), 0.)
        self.assertGreater(command[0], 0.)
        self.assertTrue(evaluate_trajectory(state, sensor, *command,
                        replace(self.cfg, prediction_horizon=.7), 0.)[0])

    def test_nearby_guide_does_not_turn_moving_robot_back(self):
        from py_amr_ttb.lab3.local_planner import TrajectoryPlanner
        state = RobotState((0., 0., 0.), .3, 0., 0., 0.)
        planner = TrajectoryPlanner(self.cfg)
        planner.previous = (.3, 0.)
        with patch.object(planner, 'guide', return_value=((.3, 0.), [[0., 0.], [.3, 0.], [2., 0.]])):
            command, _ = planner.choose(state, scan(self.cfg), (2., 0.), 0.)
        self.assertGreaterEqual(command[0], .29)
        self.assertAlmostEqual(command[1], 0.)

    def test_parallel_close_wall_allows_fast_straight_travel(self):
        from py_amr_ttb.lab3.local_planner import TrajectoryPlanner
        state = RobotState((0., 0., 0.), .3, 0., 0., 0.)
        walls = [((-.5, -.23), (4., -.23)), ((-.5, .23), (4., .23))]
        sensor = scan(self.cfg, raycast(walls, state.pose))
        command, _ = TrajectoryPlanner(self.cfg).choose(state, sensor, (3., 0.), 0.)
        self.assertAlmostEqual(command[0], self.cfg.max_linear)
        self.assertAlmostEqual(command[1], 0.)

    def test_slower_steering_rejects_nominally_clear_turn(self):
        from py_amr_ttb.lab3.trajectory import evaluate_trajectory
        state = RobotState((0., 0., 0.), .25, 0., 0., 0.)
        sensor = scan(self.cfg, raycast([((.4, -.085), (.4, -.065))], state.pose))
        nominal = replace(self.cfg, angular_response_fraction=1.)
        self.assertTrue(evaluate_trajectory(state, sensor, .3, 1.4, nominal, 0.)[0])
        checks = {}
        self.assertFalse(evaluate_trajectory(state, sensor, .3, 1.4, self.cfg, 0., checks)[0])
        self.assertFalse(checks['slower_steering_clear'])

    def test_route_through_unseen_space_cannot_authorize_forward_motion(self):
        from py_amr_ttb.lab3.local_planner import TrajectoryPlanner
        state = RobotState((0., 0., 0.), 0., 0., 0., 0.)
        values = np.full(360, np.inf)
        values[160:200] = np.nan
        sensor = scan(self.cfg, values)
        command, _ = TrajectoryPlanner(self.cfg).choose(state, sensor, (2., 0.), 0.)
        # The route guide can suggest a direction, but cannot certify an unseen sweep.
        self.assertLessEqual(command[0], 0.)

    def test_stale_odometry_stops_fresh_scan_and_plan(self):
        state = RobotState((0., 0., 0.), 0., 0., 0., 0.)
        plan = Plan(Target((2., 0.)), (0., 0., 0.), 1., command=(.3, 0.))
        result = MotionController(self.cfg).step_plan(state, scan(self.cfg, stamp=1.), plan, 1., .05)
        self.assertEqual(result.reason, 'SENSOR_STOP')
        self.assertEqual((result.v, result.w), (0., 0.))

    def test_capture_keeps_the_rejected_cycle(self):
        from py_amr_ttb.lab3.telemetry import Telemetry
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            telemetry = Telemetry(directory, 'learn')
            state = RobotState((1., 2., 0.), 0., 0., 5., 5.)
            path = telemetry.capture('TRANSLATION_COVERAGE', dict(now=5., state=state))
            state = replace(state, pose=(9., 9., 0.), stamp=6.)
            telemetry.close()
            captured = json.loads(path.read_text())
            self.assertEqual(captured['now'], 5.)
            self.assertEqual(captured['state']['pose'], [1., 2., 0.])


    def test_acquisition_interpolation_and_signed_twist(self):
        state = StateTracker(self.cfg)
        state.update((3., 4., math.pi/2), -.1, -.2, 10., 0.)
        latest = state.update((3., 3.99, math.pi/2-.02), -.1, -.2, 10.1, .1)
        self.assertEqual((latest.v, latest.w), (-.1, -.2))
        self.assertAlmostEqual(state.pose_at(10.05)[0], -.005)
        self.assertAlmostEqual(state.pose_at(10.05)[2], -.01)
        with self.assertRaises(ValueError):
            state.pose_at(11.)
        with self.assertRaises(ValueError):
            state.update((30., 4., 0.), 0., 0., 10.2, .2)

    def test_velocity_reversal_has_residual_motion(self):
        state = RobotState((0., 0., 0.), .3, .8, 0., 0.)
        poses = rollout(state, 0., -1., self.cfg)
        self.assertGreater(poses[1][0], 0.)
        self.assertGreater(poses[1][2], 0.)
        self.assertTrue(any(b[2] < a[2] for a, b in zip(poses[4:], poses[5:])))


    def test_braking_sweep_rejects_obstacle_beyond_current_footprint(self):
        state = RobotState((0., 0., 0.), .3, 0., 0., 0.)
        s = scan(self.cfg, raycast([((.34, -2.), (.34, 2.))], state.pose))
        controller = MotionController(self.cfg)
        self.assertFalse(controller.safe(state, s, .3, 0., 0.)[0])
        result = controller.step_plan(state, s, Plan(Target((2., 0.)), (0., 0., 0.), 0., command=(.3, 0.)), 0., .05)
        self.assertLess(result.v, .3)


    def test_late_worker_output_cannot_move_robot(self):
        state = RobotState((0., 0., 0.), 0., 0., 10., 10.)
        plan = Plan(Target((2., 0.)), (0., 0., 0.), 0.)
        result = MotionController(self.cfg).step_plan(state, scan(self.cfg, stamp=10.), plan, 10., .05)
        self.assertEqual((result.v, result.w), (0., 0.))
        self.assertEqual(result.reason, 'WAITING_FOR_FRESH_PLAN')

    def test_filter_predicts_uncertainty_and_rejects_bad_correction(self):
        estimator = PoseEstimator(self.cfg)
        estimator.predict((0., 0., 0.))
        before = np.trace(estimator.covariance)
        estimator.predict((1., 0., .1))
        self.assertGreater(np.trace(estimator.covariance), before)
        predicted = estimator.pose
        self.assertFalse(estimator.correct((50., 0., 0.)))
        self.assertEqual(estimator.pose, predicted)
        variance = np.trace(estimator.covariance)
        self.assertTrue(estimator.correct((1.02, .01, .11)))
        self.assertLess(np.trace(estimator.covariance), variance)
        self.assertTrue(np.all(np.linalg.eigvalsh(estimator.covariance) > 0))


class PreparedCoverageTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Lab3Config()

    def test_small_bracketed_gap_only_supports_near_field(self):
        from py_amr_ttb.lab3.coverage import prepare_coverage
        ranges = np.full(360, 3.)
        ranges[179:181] = np.nan
        sensor = scan(self.cfg, ranges)
        centers = [[.35, 0.], [.6, 0.]]
        strict = replace(sensor, coverage=prepare_coverage((sensor,)))
        tolerant = replace(sensor, coverage=prepare_coverage((sensor,), bridge_small_gaps=True))
        np.testing.assert_array_equal(circles_observed(strict, centers, .005), [False, False])
        np.testing.assert_array_equal(circles_observed(tolerant, centers, .005), [True, False])
        self.assertEqual(np.count_nonzero(~sensor.known), 2)

    def test_gap_inference_respects_short_returns_and_large_gaps(self):
        from py_amr_ttb.lab3.coverage import prepare_coverage
        ranges = np.full(360, 3.)
        ranges[178] = .25
        ranges[179:181] = np.nan
        sensor = scan(self.cfg, ranges)
        tolerant = replace(sensor, coverage=prepare_coverage((sensor,), bridge_small_gaps=True))
        self.assertFalse(circles_observed(tolerant, [[.35, 0.]], .005)[0])
        np.testing.assert_array_equal(tolerant.coverage.sources[0].hits, sensor.points)

        ranges[175:186] = np.nan
        sensor = scan(self.cfg, ranges)
        tolerant = replace(sensor, coverage=prepare_coverage((sensor,), bridge_small_gaps=True))
        self.assertFalse(circles_observed(tolerant, [[.35, 0.]], .005)[0])

    def test_prepared_circle_bounds_are_conservative(self):
        from py_amr_ttb.lab3.coverage import prepare_coverage
        rng = np.random.default_rng(12)
        values = rng.uniform(.3, 3., 360)
        values[::19] = np.nan
        sensor = scan(self.cfg, values, pose=(2., -1., 2.9))
        points = rng.uniform(-1., 1., (400, 2))
        radii = rng.uniform(.001, .25, 400)
        source = prepare_coverage((sensor,)).sources[0]
        fast = source.covers(transform_points(points, sensor.pose), radii)
        original = circles_observed(sensor, points, radii)
        self.assertFalse(np.any(fast & ~original))

    def prepared(self, current, previous=(), now=0.):
        from py_amr_ttb.lab3.coverage import prepare_coverage
        return replace(current, coverage=prepare_coverage((current, *previous)),
                       coverage_time=now, coverage_timeout=.4)

    def test_unknown_wedge_is_not_filled(self):
        values = np.full(360, 3.)
        values[175:186] = np.nan
        sensor = self.prepared(scan(self.cfg, values))
        self.assertFalse(circles_observed(sensor, [[.5, 0.]], .22)[0])

    def test_expiry_removes_old_coverage(self):
        values = np.full(360, 3.)
        values[175:186] = np.nan
        current = scan(self.cfg, values, stamp=.3)
        sensor = self.prepared(current, (scan(self.cfg, stamp=0.),), now=.3)
        self.assertTrue(circles_observed(sensor, [[.5, 0.]], .22)[0])
        self.assertFalse(circles_observed(replace(sensor, coverage_time=.5), [[.5, 0.]], .22)[0])
        self.assertFalse(circles_observed(replace(sensor, coverage_time=.8), [[0., 0.]], .22)[0])

    def test_new_obstacle_overrides_prepared_free_space(self):
        free = scan(self.cfg)
        values = np.full(360, 3.)
        values[180] = .4
        current = scan(self.cfg, values, stamp=.1)
        prepared = self.prepared(free).coverage
        current = replace(current, coverage=prepared, coverage_time=.1)
        self.assertFalse(circles_observed(current, [[.4, 0.]], .22)[0])

    def test_transformed_query_and_pickled_result(self):
        import pickle
        sensor = self.prepared(scan(self.cfg, pose=(2., 3., math.pi/2)))
        shifted = scan_at_pose(sensor, (2., 3.2, math.pi/2))
        shifted = replace(shifted, coverage=pickle.loads(pickle.dumps(shifted.coverage)))
        self.assertTrue(circles_observed(shifted, [[.5, 0.]], .22)[0])


class MappingAndReplayTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Lab3Config(goal_x=3.4, alignment_enabled=False)


    def test_duplicate_scan_does_not_amplify_evidence(self):
        grid = OccupancyMap(self.cfg)
        s = scan(self.cfg, raycast(course('pocket')[0], (0., 0., 0.)))
        self.assertTrue(grid.integrate(s, s.pose))
        before = grid.evidence.copy()
        self.assertFalse(grid.integrate(s, s.pose))
        np.testing.assert_array_equal(before, grid.evidence)

    def test_unknown_or_wall_crossing_shortcut_is_rejected(self):
        grid = OccupancyMap(self.cfg)
        self.assertFalse(grid.segment_clear((0., 0.), (1., 0.)))
        grid.observed[:] = True
        grid.evidence[:] = -4.
        self.assertTrue(grid.segment_clear((0., 0.), (1., 0.)))
        x, y = grid.cell((.5, 0.))
        grid.evidence[y, x] = 4.
        grid._layers = None
        self.assertFalse(grid.segment_clear((0., 0.), (1., 0.)))

    def test_completed_run_roundtrip_and_incompatible_config(self):
        memory, grid = RunMemory(self.cfg), OccupancyMap(self.cfg)
        memory.route = [(0., 0., 0., 0.), (1., 0., 0., 5.)]
        memory.complete = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'run.npz'
            memory.save(path, grid)
            loaded, restored = RunMemory.load(path, self.cfg)
            self.assertTrue(loaded.complete)
            self.assertEqual(len(loaded.route), 2)
            np.testing.assert_array_equal(restored.observed, grid.observed)
            with self.assertRaises(ValueError):
                RunMemory.load(path, replace(self.cfg, goal_x=4.))

    def test_incomplete_run_is_not_replayable(self):
        with tempfile.TemporaryDirectory() as directory:
            config = replace(self.cfg, run_file=str(Path(directory)/'run.npz'))
            RunMemory(config).save(config.run_file, OccupancyMap(config))
            with self.assertRaises(ValueError):
                NavigationEngine(replace(config, run_mode='replay'))

    def test_ambiguous_alignment_fails_closed(self):
        grid = OccupancyMap(self.cfg)
        pose, quality = grid.align(scan(self.cfg), (0., 0., 0.), initial=True)
        self.assertFalse(quality['accepted'])
        self.assertEqual(pose, (0., 0., 0.))

    def test_loop_can_be_trimmed_only_in_observed_free_space(self):
        grid = OccupancyMap(self.cfg)
        grid.observed[:] = True
        grid.evidence[:] = -4.
        route = [(0., 0.), (0., 1.), (1., 1.), (0., 1.), (0., 0.), (2., 0.)]
        result = optimize_route(route, grid, self.cfg)
        np.testing.assert_allclose(result, [(0., 0.), (2., 0.)])

    def test_latest_queue_is_bounded(self):
        queue = Queue(1)
        put_latest(queue, 'old')
        put_latest(queue, 'new')
        self.assertEqual(take_latest(queue), 'new')
        self.assertIsNone(take_latest(queue))

    def test_spawned_worker_produces_plan_and_saves_on_shutdown(self):
        with tempfile.TemporaryDirectory() as directory:
            config = replace(self.cfg, run_file=str(Path(directory)/'run.npz'))
            completed = RunMemory(config)
            completed.complete = True
            completed.save(config.run_file, OccupancyMap(config))
            original = Path(config.run_file).read_bytes()
            context = mp.get_context('spawn')
            incoming, outgoing = context.Queue(1), context.Queue(1)
            stopping, save = context.Event(), context.Event()
            process = context.Process(target=worker_main, args=(config, incoming, outgoing, stopping, save))
            process.start()
            try:
                now = time.monotonic()
                incoming.put(Work(RobotState((0., 0., 0.), 0., 0., now, now), scan(config, stamp=now), now))
                plan = outgoing.get(timeout=10.)
                self.assertEqual(plan.status, 'RUNNING')
                self.assertIsNotNone(plan.target)
            finally:
                stopping.set()
                process.join(timeout=5.)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=2.)
                incoming.close()
                outgoing.close()
            self.assertEqual(process.exitcode, 0)
            self.assertEqual(Path(config.run_file).read_bytes(), original)
            self.assertTrue(Path(directory, 'run.incomplete.npz').is_file())


class ClearanceRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Lab3Config()
        self.state = RobotState((0., 0., 0.), 0., 0., 0., 0.)

    def wall_scan(self, walls):
        return scan(self.cfg, raycast(walls, self.state.pose))

    def test_parallel_wall_does_not_need_sideways_travel_padding(self):
        observed = self.wall_scan([((-.8, .191), (2., .191))])
        clear, _ = evaluate_trajectory(self.state, observed, .08, 0., self.cfg, 0.)
        self.assertTrue(clear)

    def test_margin_escape_increases_clearance_but_toward_wall_stops(self):
        observed = self.wall_scan([((.185, -.8), (.185, .8))])
        details = {}
        clear, _ = evaluate_trajectory(self.state, observed, -.05, 0., self.cfg, 0., details)
        self.assertTrue(clear)
        self.assertTrue(details['margin_escape'])
        self.assertGreater(details['escape_gain'], .001)
        self.assertFalse(evaluate_trajectory(self.state, observed, .05, 0., self.cfg, 0.)[0])
        self.assertFalse(evaluate_trajectory(self.state, observed, -.2, 0., self.cfg, 0.)[0])

    def test_escape_cannot_use_unknown_rear_space(self):
        observed = self.wall_scan([((.185, -.8), (.185, .8))])
        observed.known[np.abs(observed.angles) > 1.5] = False
        self.assertFalse(evaluate_trajectory(self.state, observed, -.05, 0., self.cfg, 0.)[0])

    def test_escape_does_not_approach_opposing_obstacle(self):
        observed = self.wall_scan([((.185, -.8), (.185, .8)), ((-.195, -.8), (-.195, .8))])
        self.assertFalse(evaluate_trajectory(self.state, observed, -.05, 0., self.cfg, 0.)[0])

    def test_body_overlap_never_authorizes_escape(self):
        observed = self.wall_scan([((.175, -.8), (.175, .8))])
        self.assertFalse(evaluate_trajectory(self.state, observed, -.05, 0., self.cfg, 0.)[0])

    def test_capsule_detects_hit_between_samples(self):
        distances = capsule_clearance(np.array([[0.,0.],[1.,0.]]), np.array([[.5,.1]]))
        self.assertAlmostEqual(float(distances[0,0]), .1)

    def test_exhausted_rotation_prefers_clear_escape_over_holding(self):
        from py_amr_ttb.lab3.local_planner import TrajectoryPlanner
        observed = self.wall_scan([((.185, -.8), (.185, .8))])
        command, details = TrajectoryPlanner(self.cfg).choose(
            self.state, observed, (1., 0.), 0., allow_rotation=False)
        self.assertLess(command[0], 0.)
        self.assertTrue(details['clearance']['margin_escape'])
        self.assertTrue(details['rotation_limited'])

    def test_turn_budget_survives_objective_changes(self):
        from py_amr_ttb.lab3.planner import Planner
        planner = Planner(self.cfg, RunMemory(self.cfg))
        observed = scan(self.cfg)
        with patch.object(planner.motion, 'choose', return_value=((0.,0.), {})) as choose:
            for i in range(16):
                state = replace(self.state, pose=(0.,0.,i*.45))
                planner.detour = (1., .1*i)
                planner.step(state, observed, None, i*.2)
            self.assertFalse(choose.call_args.kwargs['allow_rotation'])
            planner.step(replace(state, pose=(.2,0.,3.6)), observed, None, 2.)
            self.assertTrue(choose.call_args.kwargs['allow_rotation'])


class CourseTests(unittest.TestCase):
    def test_mixed_diamond_wall_mazes_complete(self):
        for name in ('maze_s_bend', 'maze_chicane', 'maze_offset_gate', 'maze_double_bend',
                     'example_course'):
            with self.subTest(course=name), tempfile.TemporaryDirectory() as directory:
                _, goal = course(name)
                cfg = Lab3Config(goal_x=goal[0], goal_y=goal[1], alignment_enabled=False,
                                 run_file=str(Path(directory)/'run.npz'))
                result = simulate(name, cfg, seconds=60.)
                self.assertTrue(result['completed'])
                self.assertGreaterEqual(result['minimum_clearance'], cfg.radius)

    def test_diamond_enters_narrow_passage_without_reverse_or_stationary_turn(self):
        for name in ('diamond42', 'diamond40'):
            with self.subTest(course=name), tempfile.TemporaryDirectory() as directory:
                _, goal = course(name)
                cfg = Lab3Config(goal_x=goal[0], goal_y=goal[1], alignment_enabled=False,
                                 run_file=str(Path(directory)/'run.npz'))
                result = simulate(name, cfg, seconds=35.)
                self.assertTrue(result['completed'])
                self.assertGreaterEqual(result['minimum_clearance'], cfg.radius)
                samples = np.asarray(result['samples'])
                self.assertGreaterEqual(float(samples[:, 4].min()), -.005)
                stationary_turns = (abs(samples[:, 4]) < .02) & (abs(samples[:, 5]) > .1)
                self.assertFalse(stationary_turns.any())

    def test_narrow_corridor_and_staggered_openings_complete(self):
        for name in ('corridor', 'staggered'):
            with self.subTest(course=name), tempfile.TemporaryDirectory() as directory:
                _, goal = course(name)
                cfg = Lab3Config(goal_x=goal[0], goal_y=goal[1], alignment_enabled=False,
                                 run_file=str(Path(directory)/'run.npz'))
                first = simulate(name, cfg, seconds=30.)
                self.assertTrue(first['completed'])
                self.assertGreaterEqual(first['minimum_clearance'], cfg.radius)
                self.assertGreater(first['peak_speed'], .29)
                self.assertNotIn('BACKTRACK', first['modes'])
                if name == 'corridor':
                    self.assertNotIn('ALIGN', first['reasons'])
                    self.assertLess(first['elapsed'], 9.)
                second = simulate(name, replace(cfg, run_mode='replay'), seconds=30.)
                self.assertTrue(second['completed'])
                self.assertGreaterEqual(second['minimum_clearance'], cfg.radius)
                if name == 'corridor':
                    self.assertLessEqual(second['elapsed'], first['elapsed'])
                else:
                    # A direct first run may already be as short as its replay.
                    self.assertLessEqual(second['elapsed'], 1.10*first['elapsed'])

    def test_offset_42cm_passage_with_and_without_protruding_wall(self):
        from py_amr_ttb.lab3.simulate import passage_course
        for obstacle in ('empty', 'extruded_wall'):
            with self.subTest(obstacle=obstacle), tempfile.TemporaryDirectory() as directory:
                layout = passage_course(obstacle)
                cfg = Lab3Config(goal_x=layout[1][0], goal_y=layout[1][1],
                                 alignment_enabled=False, run_file=str(Path(directory)/'passage.npz'))
                with patch('py_amr_ttb.lab3.simulate.course', return_value=layout):
                    result = simulate(config=cfg, seconds=45.)
                self.assertTrue(result['completed'])
                self.assertGreaterEqual(result['minimum_clearance'], cfg.radius)
                self.assertNotIn('BACKTRACK', result['modes'])

    def test_default_pose_filter_completes_obstacle_course(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg = Lab3Config(goal_x=3., run_file=str(Path(directory)/'run.npz'))
            result = simulate('obstacle', cfg, seconds=45.)
            self.assertTrue(result['completed'])
            self.assertGreater(result['minimum_clearance'], cfg.radius)

    def test_obstacle_and_dead_end_complete_with_efficient_replay(self):
        for name in ('straight', 'obstacle', 'pocket', 'corner'):
            with self.subTest(course=name), tempfile.TemporaryDirectory() as directory:
                _, goal = course(name)
                cfg = Lab3Config(goal_x=goal[0], goal_y=goal[1], alignment_enabled=False,
                                 run_file=str(Path(directory)/'run.npz'))
                first = simulate(name, cfg, seconds=45.)
                self.assertTrue(first['completed'])
                self.assertGreater(first['minimum_clearance'], cfg.radius)
                self.assertGreater(first['peak_speed'], .29)
                if name != 'straight':
                    second = simulate(name, replace(cfg, run_mode='replay'), seconds=45.)
                    self.assertTrue(second['completed'])
                    self.assertGreater(second['minimum_clearance'], cfg.radius)
                    # A direct first run may already be as short as its replay.
                    self.assertLessEqual(second['elapsed'], 1.10*first['elapsed'])


if __name__ == '__main__':
    unittest.main()
