"""Shared acceleration, stopping prediction and swept-footprint validation.

All positions use starting odometry until footprint checks transform them into
the scan frame. Evaluation is independent of target selection and controller state.
"""

import math
from dataclasses import replace

import numpy as np

from py_amr_ttb.odom_utils import normalize_angle
from .geometry import inverse_points, circles_clear


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
    change = float(np.clip(desired - current, -maximum_change, maximum_change))
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


def _evaluate_response(state, scan, v, w, config, now, diagnostics=None):
    """Obstacles can lie between predicted positions or along the
    later braking path, not just at a command's endpoint.

    Return (is_clear, trajectory). Predict through a complete stop, convert
    positions into the scan frame, and use enlarged midpoint circles to cover
    each sampled segment. Clearance depends on scan, pose and response-model
    accuracy.
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
    # Endpoint checks can skip obstacles between samples. A midpoint circle
    # enlarged by half the segment length covers the intervening body motion
    # without adding needless travel padding when the robot is stationary.
    segment_midpoints = (scan_frame_positions[:-1] + scan_frame_positions[1:]) / 2
    segment_lengths = np.linalg.norm(np.diff(scan_frame_positions, axis=0), axis=1)
    swept_radii = config.radius + segment_lengths / 2
    # A stopped circular body repeats the same footprint while yaw settles.
    # Exact deduplication preserves every distinct swept region.
    footprints = np.unique(np.column_stack((segment_midpoints, swept_radii)), axis=0)
    segment_midpoints, swept_radii = footprints[:, :2], footprints[:, 2]
    is_clear = circles_clear(scan, segment_midpoints, swept_radii,
                             occupied_radius=config.radius,
                             occupied_center=scan_frame_positions[0], diagnostics=diagnostics)
    return is_clear, trajectory


def evaluate_trajectory(state, scan, v, w, config, now, diagnostics=None):
    """Require nominal and slower steering responses to leave stopping clearance.

    The second response uses angular_response_fraction (0.75 by default).
    It protects turns against moderate tracking lag, without treating the nominal
    path as the only possible motion. Hardware limits still need calibration.
    """
    clear, trajectory = _evaluate_response(state, scan, v, w, config, now, diagnostics)
    if not clear or abs(w)+abs(state.w) < .01:
        return clear, trajectory
    slower = replace(config, angular_acceleration=config.angular_acceleration*config.angular_response_fraction,
                     angular_braking=config.angular_braking*config.angular_response_fraction)
    details = {}
    robust, _ = _evaluate_response(state, scan, v, w, slower, now, details)
    if diagnostics is not None:
        diagnostics['slower_steering_clear'] = robust
        if not robust:
            diagnostics['slower_steering_rejection'] = details
    return robust, trajectory
