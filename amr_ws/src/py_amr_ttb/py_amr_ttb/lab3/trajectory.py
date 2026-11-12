"""Shared acceleration, stopping prediction and swept-footprint validation.

All positions use starting odometry until footprint checks transform them into
the scan frame. Evaluation is independent of target selection and controller state.
"""

import math
from dataclasses import replace

import numpy as np

from py_amr_ttb.odom_utils import normalize_angle
from .geometry import inverse_points, circles_clear, transform_points


def approach(current, desired, acceleration, braking, dt):
    """Assuming an instant speed change would underestimate travel
    while braking or reversing.

    Advance current toward desired over dt using acceleration or braking limits.
    Units are m/s and m/s squared, or rad/s and rad/s squared. Reversal reaches zero
    before accelerating the other way. Return the next signed speed.
    """
    reversing = current * desired < 0
    slowing_down = abs(desired) < abs(current)
    rate_limit = braking if reversing or slowing_down else acceleration
    maximum_change = rate_limit * dt
    change = max(-maximum_change, min(desired-current, maximum_change))
    next_speed = current + change

    if reversing and current * next_speed < 0:
        return 0.0
    return next_speed


def rollout(state, v_command, w_command, config, age=0.0):
    """A robot can avoid contact during a command yet collide afterward
    because it has no room to stop.

    Return starting-odometry poses through latency, the command horizon and braking
    to rest, beginning at state.pose and its measured speeds. v_command is m/s,
    w_command is rad/s, and age adds latency in seconds. Accuracy depends on
    configured response limits.
    """
    dt = config.prediction_step
    latency = config.command_latency + max(0.0, age)
    command_end = latency + config.prediction_horizon
    linear_stop_time = max(abs(state.v), abs(v_command)) / config.linear_braking
    angular_stop_time = max(abs(state.w), abs(w_command)) / config.angular_braking
    prediction_end = command_end + max(linear_stop_time, angular_stop_time) + dt

    x, y, yaw = state.pose
    linear_speed, angular_speed = state.v, state.w
    poses = [(x, y, yaw)]
    for elapsed in np.arange(dt, prediction_end + dt / 2, dt):
        # A new command cannot change motion before its communication/response delay.
        if elapsed > latency:
            # The path must leave braking room after the command horizon ends.
            if elapsed <= command_end:
                target_linear, target_angular = v_command, w_command
            else:
                target_linear, target_angular = 0.0, 0.0
            linear_speed = approach(
                linear_speed, target_linear,
                config.linear_acceleration, config.linear_braking, dt,
            )
            angular_speed = approach(
                angular_speed, target_angular,
                config.angular_acceleration, config.angular_braking, dt,
            )

        midpoint_heading = yaw + angular_speed * dt / 2
        x += linear_speed * math.cos(midpoint_heading) * dt
        y += linear_speed * math.sin(midpoint_heading) * dt
        yaw = normalize_angle(yaw + angular_speed * dt)
        poses.append((x, y, yaw))
    return poses


def capsule_clearance(positions, hits):
    """Measure each obstacle against the complete segment, including endpoints."""
    delta = np.diff(positions, axis=0)
    offset = hits[None] - positions[:-1, None]
    length2 = np.sum(delta*delta, axis=1)
    fraction = np.clip(np.sum(offset*delta[:, None], axis=2) /
                       np.maximum(length2[:, None], 1e-20), 0., 1.)
    return np.linalg.norm(offset-fraction[..., None]*delta[:, None], axis=2)


def sweep_clear(trajectory, scan, config, state, v, diagnostics):
    """Keep the physical body clear while permitting bounded margin recovery.

    Near hits must never get closer during an escape, including its braking
    sweep. Other hits retain the normal margin. Pure rotation preserves a
    circular footprint; the planner separately limits unproductive turning.
    """
    positions = inverse_points(np.asarray(trajectory)[:, :2], scan.pose)
    hits = [scan.points]
    if scan.coverage is not None:
        selected = scan.coverage.select(scan.coverage_time, scan.coverage_timeout)
        if selected is None:
            diagnostics['rejection'] = 'COVERAGE_EXPIRED'
            return False
        hits += [inverse_points(source.hits, scan.pose) for source in selected]
    else:
        hits += [inverse_points(transform_points(old.points, old.pose), scan.pose)
                 for old in scan.history]
    hits = np.concatenate(hits)
    if not len(hits):
        return True
    distances = capsule_clearance(positions, hits)
    # The rollout uses midpoint integration. Bound deviation from each chord
    # by arc sagitta plus its chord-length integration error.
    turn = np.abs(np.arctan2(np.sin(np.diff(np.asarray(trajectory)[:, 2])),
                            np.cos(np.diff(np.asarray(trajectory)[:, 2]))))
    lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    error = lengths*(turn/8+turn*turn/24)
    distances -= error[:, None]
    initial = np.linalg.norm(hits-positions[0], axis=1)
    minimum = float(distances.min())
    diagnostics.update(initial_clearance=float(initial.min()), minimum_clearance=minimum)
    if minimum <= config.robot_radius:
        diagnostics['rejection'] = 'BODY_COLLISION'
        return False
    endpoint_distances = np.linalg.norm(positions[:, None]-hits[None], axis=2)
    # Reserve closing motion toward the nearest observed surface, rather than
    # toward individual beams farther along a parallel wall.
    closing_speed = np.maximum(0., -np.diff(endpoint_distances.min(axis=1)) /
                               config.prediction_step)[:, None]
    required = np.broadcast_to(config.radius+config.command_latency*closing_speed, distances.shape)
    near = initial < config.radius
    if not np.any(near):
        clear = bool(np.all(distances > required))
        if not clear:
            diagnostics['rejection'] = 'CLEARANCE_MARGIN'
        return clear
    travel = float(lengths.sum())
    nondecreasing = np.all(distances[:, near] >= endpoint_distances[:-1, near]-1e-7)
    other_clear = not np.any(distances[:, ~near] <= required[:, ~near])
    gain = float(endpoint_distances[-1].min()-initial.min())
    rotation = travel < 1e-5
    bounded = abs(v) <= .08 and abs(state.v) <= .10 and travel <= .10
    clear = bounded and nondecreasing and other_clear and (rotation or gain >= .001)
    diagnostics.update(margin_escape=bool(clear), escape_gain=gain, escape_travel=travel)
    if not clear:
        diagnostics['rejection'] = 'ESCAPE_NOT_IMPROVING'
    return bool(clear)


def visibility_scan(scan):
    """Separate visibility evidence from collision hits without erasing ranges."""
    coverage = scan.coverage
    if coverage is not None:
        coverage = replace(coverage, sources=tuple(
            replace(source, hits=np.empty((0, 2))) for source in coverage.sources))
    return replace(scan, hits=np.zeros_like(scan.hits, dtype=bool), coverage=coverage,
                   history=tuple(visibility_scan(old) for old in scan.history))


def _evaluate_response(state, scan, v, w, config, now, diagnostics=None):
    """Obstacles can lie between predicted positions or along the
    later braking path, not just at a command's endpoint.

    Return (is_clear, trajectory). Predict through a complete stop, convert
    positions into the scan frame, check obstacle capsules, then certify body
    visibility with endpoint and midpoint disks. This is the low-speed fallback;
    clearance depends on scan, pose and response-model accuracy.
    """
    scan_age = max(0.0, now - scan.received)
    if scan.coverage is not None:
        scan = replace(scan, coverage_time=now, coverage_timeout=config.sensor_timeout)
    history = tuple(previous for previous in scan.history
                    if 0 <= now-previous.received <= config.sensor_timeout)
    if len(history) != len(scan.history):
        scan = replace(scan, history=history)
    trajectory = rollout(state, v, w, config, scan_age)
    odometry_positions = np.asarray(trajectory)[:, :2]
    scan_frame_positions = inverse_points(odometry_positions, scan.pose)
    details = diagnostics if diagnostics is not None else {}
    if not sweep_clear(trajectory, scan, config, state, v, details):
        return False, trajectory
    # Endpoint disks cover the capsule caps. Midpoint disks cover its rectangular
    # strip with only quadratic sideways enlargement. Visibility needs the body,
    # while sweep_clear independently enforces the obstacle clearance margin.
    segment_midpoints = (scan_frame_positions[:-1]+scan_frame_positions[1:])/2
    lengths = np.linalg.norm(np.diff(scan_frame_positions, axis=0), axis=1)
    turn = np.abs(np.arctan2(np.sin(np.diff(np.asarray(trajectory)[:, 2])),
                            np.cos(np.diff(np.asarray(trajectory)[:, 2]))))
    error = lengths*(turn/8+turn*turn/24)
    radii = np.sqrt((config.robot_radius+error)**2+(lengths/2)**2)
    footprints = np.unique(np.vstack((
        np.column_stack((scan_frame_positions, np.full(len(scan_frame_positions), config.robot_radius))),
        np.column_stack((segment_midpoints, radii)))), axis=0)
    is_clear = circles_clear(visibility_scan(scan), footprints[:, :2], footprints[:, 2],
                             occupied_radius=config.robot_radius,
                             occupied_center=scan_frame_positions[0], diagnostics=details)
    if not is_clear:
        details['rejection'] = 'UNOBSERVED_SWEEP'

    return is_clear, trajectory


def _normal_response(state, scan, v, w, config, now, diagnostics):
    """Cover each stopping-path segment with a clearance-inflated midpoint disk."""
    scan = replace(scan, coverage_time=now, coverage_timeout=config.sensor_timeout,
                   history=tuple(old for old in scan.history
                                 if 0 <= now-old.received <= config.sensor_timeout))
    trajectory = rollout(state, v, w, config, max(0., now-scan.received))
    positions = inverse_points(np.asarray(trajectory)[:, :2], scan.pose)
    midpoints = (positions[:-1]+positions[1:])/2
    radii = config.radius+np.linalg.norm(np.diff(positions, axis=0), axis=1)/2
    footprints = np.unique(np.column_stack((midpoints, radii)), axis=0)
    clear = circles_clear(scan, footprints[:, :2], footprints[:, 2],
                          occupied_radius=config.radius, occupied_center=positions[0],
                          diagnostics=diagnostics)
    return clear, trajectory


def evaluate_trajectory(state, scan, v, w, config, now, diagnostics=None):
    """Use normal stopping checks, with bounded low-speed capsule recovery.

    The recovery check never authorizes a physical-body intersection or unknown
    sweep. Both nominal and slower steering responses must pass the same policy.
    """
    details = diagnostics if diagnostics is not None else {}
    clear, trajectory = _normal_response(state, scan, v, w, config, now, details)
    turning = abs(w)+abs(state.w) >= .01
    slower = replace(config, angular_acceleration=config.angular_acceleration*config.angular_response_fraction,
                     angular_braking=config.angular_braking*config.angular_response_fraction)
    if clear and turning:
        slow_details = {}
        clear, _ = _normal_response(state, scan, v, w, slower, now, slow_details)
        details['slower_steering_clear'] = clear
        if not clear:
            details['slower_steering_rejection'] = slow_details
    if clear:
        return True, trajectory
    if abs(v) > .08 or abs(state.v) > .10:
        details['rejection'] = 'NORMAL_SWEEP_BLOCKED'
        return False, trajectory
    recovery = {}
    clear, trajectory = _evaluate_response(state, scan, v, w, config, now, recovery)
    if clear and turning:
        slow_details = {}
        clear, _ = _evaluate_response(state, scan, v, w, slower, now, slow_details)
        recovery['slower_steering_clear'] = clear
        if not clear:
            recovery['rejection'] = 'SLOWER_STEERING_BLOCKED'
            recovery['slower_steering_rejection'] = slow_details
    details.update(recovery)
    details['capsule_recovery'] = bool(clear)
    return clear, trajectory
