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


def evaluate_trajectory(state, scan, v, w, config, now, diagnostics=None):
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



def command_candidates(state, preferred_v, preferred_w, config, dt):
    """Yield the preferred command, then nearby reachable speed/steering pairs.

    Rank alternatives by tracking the preferred motion and limiting changes from
    measured velocity. Every alternative respects one-cycle acceleration/braking
    limits and keeps the requested travel direction.
    """
    yield preferred_v, preferred_w
    braking_v = approach(state.v, 0., config.linear_acceleration, config.linear_braking, dt)
    speeds = (preferred_v, (preferred_v+braking_v)/2, braking_v)
    steering = [preferred_w]
    for desired_w in (0., -config.max_angular, config.max_angular):
        steering.append(approach(state.w, desired_w,
                                 config.angular_acceleration, config.angular_braking, dt))
    alternatives = set()
    for v in speeds:
        for w in steering:
            if (v, w) == (preferred_v, preferred_w) or abs(v) <= .004:
                continue
            if not -config.reverse_speed <= v <= config.max_linear or abs(w) > config.max_angular:
                continue
            if v*preferred_v < 0 or abs(v*w) > config.lateral_acceleration:
                continue
            alternatives.add((v, w))

    def cost(command):
        v, w = command
        tracking = ((v-preferred_v)/config.max_linear)**2
        tracking += ((w-preferred_w)/config.max_angular)**2
        continuity = ((v-state.v)/config.max_linear)**2
        continuity += ((w-state.w)/config.max_angular)**2
        return tracking + .2*continuity, abs(w), w, v

    yield from sorted(alternatives, key=cost)


def arc_obstacle_distance(points, curvature, direction, radius, cap):
    """Conservative obstacle distance along a constant-curvature center path."""
    nearest = float(np.linalg.norm(points, axis=1).min())-radius if len(points) else .025
    spacing_limit = min(.025, max(.002, nearest))
    distances = np.linspace(0., cap, max(2, math.ceil(cap/spacing_limit)+1))
    signed = direction*distances
    if abs(curvature) < 1e-6:
        centers = np.column_stack((signed, np.zeros(len(signed))))
    else:
        centers = np.column_stack((np.sin(curvature*signed)/curvature,
                                   (1-np.cos(curvature*signed))/curvature))
    if not len(points):
        return cap
    spacing = distances[1]-distances[0]
    midpoint = (centers[:-1]+centers[1:])/2
    clearance = np.linalg.norm(midpoint[:, None, :]-points[None, :, :], axis=2)
    blocked = np.flatnonzero(np.any(clearance <= radius+spacing/2, axis=1))
    return float(distances[blocked[0]]) if len(blocked) else cap
