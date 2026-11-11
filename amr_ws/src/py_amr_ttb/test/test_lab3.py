"""Behavioral regressions for unknown-space navigation and reuse of learned runs."""

from dataclasses import replace
import math
import multiprocessing as mp
from pathlib import Path
from queue import Queue
import sys
import tempfile
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from py_amr_ttb.lab_config import Lab3Config
from py_amr_ttb.lab3.geometry import (
    process_scan, observed, observed_travel, compose, inverse_points, circles_clear,
    circles_observed, scan_at_pose, transform_points,
)
from py_amr_ttb.lab3.mapping import OccupancyMap
from py_amr_ttb.lab3.memory import RunMemory
from py_amr_ttb.lab3.motion import MotionController, rollout
from py_amr_ttb.lab3.replay import optimize_route, speed_profile
from py_amr_ttb.lab3.simulate import simulate, raycast, course
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

    def test_stationary_speed_noise_does_not_block_alignment(self):
        values = np.full(360, 2.)
        values[120:123] = np.nan
        values[238:241] = np.nan
        result = process_scan(values, -math.pi, math.pi/180, .15, 12., self.cfg,
                              transform=(-.04, 0., 0.), received=0.)
        state = RobotState((0., 0., 0.), .0000823, .000935, 0., 0.)
        controller = MotionController(self.cfg)
        self.assertTrue(controller.safe(state, result, 0., 0., .08)[0])
        self.assertTrue(controller.safe(state, result, 0., .09, .08)[0])
        self.assertFalse(controller.safe(state, result, .2, 0., .08)[0])

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

    def test_config_allows_zero_weights_but_rejects_invalid_motion_limits(self):
        self.assertEqual(Lab3Config(weight_goal=0.).weight_goal, 0.)
        for values in ({'max_linear': .4}, {'linear_braking': 0.}, {'goal_x': 100.},
                       {'front_sections': 1000}, {'revisit_min_age': 20.}):
            with self.assertRaises(ValueError):
                Lab3Config(**values)


class StateAndMotionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Lab3Config()

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

    def test_stale_sensors_stop_even_with_clear_target(self):
        state = RobotState((0., 0., 0.), .3, 0., 0., 0.)
        result = MotionController(self.cfg).step(state, scan(self.cfg), Target((2., 0.)), 1., .05)
        self.assertEqual((result.v, result.w, result.reason), (0., 0., 'SENSOR_STOP'))

    def test_braking_sweep_rejects_obstacle_beyond_current_footprint(self):
        state = RobotState((0., 0., 0.), .3, 0., 0., 0.)
        s = scan(self.cfg, raycast([((.34, -2.), (.34, 2.))], state.pose))
        controller = MotionController(self.cfg)
        self.assertFalse(controller.safe(state, s, .3, 0., 0.)[0])
        result = controller.step(state, s, Target((2., 0.)), 0., .05)
        self.assertLess(result.v, .3)

    def test_stationary_robot_can_retreat_near_wall(self):
        state = RobotState((0., 0., 0.), 0., 0., 0., 0.)
        s = scan(self.cfg, raycast([((.225, -2.), (.225, 2.))], state.pose))
        result = MotionController(self.cfg).step(state, s, Target((-.5, 0.), reverse=True), 0., .05)
        self.assertLess(result.v, 0.)

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

    def test_closed_cap_and_unseen_continuation_are_different(self):
        grid = OccupancyMap(self.cfg)
        walls = course('pocket')[0]
        s = scan(self.cfg, raycast(walls, (.8, 0., 0.)), pose=(.8, 0., 0.))
        grid.integrate(s, s.pose)
        self.assertTrue(grid.closed_ahead((.8, 0.), 0.))
        self.assertFalse(grid.closed_ahead((.8, 0.), math.pi))
        grid.observed[:, :] = False
        grid._layers = None
        self.assertFalse(grid.closed_ahead((.8, 0.), 0.))

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
        self.assertEqual(speed_profile(result, self.cfg)[-1], 0.)

    def test_latest_queue_is_bounded(self):
        queue = Queue(1)
        put_latest(queue, 'old')
        put_latest(queue, 'new')
        self.assertEqual(take_latest(queue), 'new')
        self.assertIsNone(take_latest(queue))

    def test_spawned_worker_produces_plan_and_saves_on_shutdown(self):
        with tempfile.TemporaryDirectory() as directory:
            config = replace(self.cfg, run_file=str(Path(directory)/'run.npz'))
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
            self.assertTrue(Path(config.run_file).is_file())


class CourseTests(unittest.TestCase):
    def test_default_pose_filter_completes_obstacle_course(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg = Lab3Config(goal_x=3., run_file=str(Path(directory)/'run.npz'))
            result = simulate('obstacle', cfg, seconds=45.)
            self.assertTrue(result['completed'])
            self.assertGreater(result['minimum_clearance'], cfg.radius)

    def test_obstacle_and_dead_end_complete_and_replay_improves_time(self):
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
                    self.assertLess(second['elapsed'], first['elapsed'])
                if name == 'pocket':
                    self.assertGreater(first['modes'].get('BACKTRACK', 0), 0)


if __name__ == '__main__':
    unittest.main()
