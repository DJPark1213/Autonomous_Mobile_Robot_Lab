"""Continuous motion with acceleration-aware prediction and stopping sweeps."""

import math
from dataclasses import replace
import numpy as np

from py_amr_ttb.odom_utils import normalize_angle
from .geometry import inverse_points, transform_points, circles_clear
from .types import MotionResult


def approach(current, desired, acceleration, braking, dt):
    """Advance a signed speed for dt seconds within acceleration/braking limits.

    Use metres/second and metres/second squared for translation, or the matching
    angular units for rotation. A reversal brakes to zero before accelerating
    in the opposite direction, even if this step could cross zero.
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
    """Predict starting-odometry poses through command delay, motion, and a stop.

    v_command is m/s, w_command is rad/s, and age is extra latency in seconds.
    Prediction begins with measured speeds, preserving residual motion during
    braking or a change of turn direction. The first returned pose is state.pose.
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
        # Until latency expires, continue at the measured speeds.
        if elapsed > latency:
            # Apply the command for the horizon, then predict braking to rest.
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


class MotionController:
    def __init__(self, config):
        self.cfg = config

    def safe(self, state, scan, v, w, now):
        """Check a command's full stopping sweep against the acquisition-frame scan."""
        scan_age = max(0.0, now - scan.received)
        trajectory = rollout(state, v, w, self.cfg, scan_age)
        odometry_positions = np.asarray(trajectory)[:, :2]
        scan_frame_positions = inverse_points(odometry_positions, scan.pose)
        # Cover each segment with its midpoint and half its actual length. A fixed
        # speed-based pad would make a stationary robot unable to retreat near walls.
        segment_midpoints = (scan_frame_positions[:-1] + scan_frame_positions[1:]) / 2
        segment_lengths = np.linalg.norm(np.diff(scan_frame_positions, axis=0), axis=1)
        swept_radii = self.cfg.radius + segment_lengths / 2
        is_clear = circles_clear(scan, segment_midpoints, swept_radii)
        return is_clear, trajectory

    def step_plan(self, state, scan, plan, now, dt):
        """Reject expired decisions and use the transform carried with the target."""
        if plan is None or not 0 <= now-plan.created <= self.cfg.plan_timeout:
            return MotionResult(reason='WAITING_FOR_FRESH_PLAN')
        if plan.target is None:
            return MotionResult(reason=plan.status)
        target = replace(plan.target, point=tuple(inverse_points([plan.target.point], plan.transform)[0]))
        return self.step(state, scan, target, now, dt, plan.diagnostics.get('speed_cap'))

    def step(self, state, scan, target, now, dt, speed_cap=None):
        c = self.cfg
        if (not 0 < dt <= c.sensor_timeout or not 0 <= now-state.received <= c.sensor_timeout
                or not 0 <= now-scan.received <= c.sensor_timeout):
            return MotionResult(reason='SENSOR_STOP', blocked=True)
        if target is None:
            return MotionResult(reason='WAITING')
        if target.mode == 'DONE':
            return MotionResult(reason='DONE')
        local = inverse_points([target.point], state.pose)[0]
        distance = float(np.linalg.norm(local))
        heading = math.atan2(local[1], local[0])
        if target.reverse:
            heading = normalize_angle(heading-math.pi)
        if target.mode == 'FOLLOW_WALL' and len(scan.points):
            # Transform current scan hits into the current robot frame.
            points = inverse_points(transform_points(scan.points, scan.pose), state.pose)
            side = points[:, 1]*target.wall_side > 0
            if side.any():
                candidates = points[side]
                nearest = candidates[np.argmin(np.linalg.norm(candidates, axis=1))]
                separation = np.linalg.norm(nearest)
                normal = -nearest/max(separation, 1e-9)
                tangent = target.wall_side*np.array([-normal[1], normal[0]])
                vector = tangent+np.clip(c.wall_gain*(c.wall_distance-separation), -.7, .7)*normal
                heading = math.atan2(vector[1], vector[0])
        desired_w = float(np.clip(c.heading_gain*heading, -c.max_angular, c.max_angular))
        top = min(c.max_linear, c.max_linear if speed_cap is None else speed_cap)
        reasons = [(top, 'SPEED_LIMIT' if top == c.max_linear else 'ROUTE_BEND')]
        reasons.append((top*max(0., math.cos(heading)), 'HEADING'))
        if abs(desired_w) > .05:
            reasons.append((c.lateral_acceleration/abs(desired_w), 'TURN'))
        # Arrival braking is for the final goal, not each intermediate target.
        if target.mode == 'FINAL':
            reasons.append((math.sqrt(2*c.linear_braking*max(0., distance-c.goal_tolerance)), 'ARRIVAL'))
        desired, reason = min(reasons, key=lambda pair: pair[0])
        if target.reverse:
            desired = -min(desired, c.reverse_speed)
            reason = 'REVERSE'
        if abs(heading) >= c.rotate_threshold:
            desired, reason = 0., 'ALIGN'
            if abs(state.v) > .025:
                desired_w = 0.
                reason = 'BRAKE_FOR_TURN'
        v = approach(state.v, desired, c.linear_acceleration, c.linear_braking, dt)
        w = approach(state.w, desired_w, c.angular_acceleration, c.angular_braking, dt)
        v = float(np.clip(v, -c.reverse_speed, c.max_linear))
        w = float(np.clip(w, -c.max_angular, c.max_angular))
        # Bounded local candidates; all use the measured current state.
        for fraction in (1., .75, .5, .25):
            trial = v*fraction
            safe, trajectory = self.safe(state, scan, trial, w, now)
            if safe and abs(trial) > .004:
                return MotionResult(trial, w, reason if fraction == 1 else 'CLEARANCE', False, trajectory)
        # A stopped robot may align; a moving one first receives a braking command.
        if abs(state.v) < .025:
            safe, trajectory = self.safe(state, scan, 0., w, now)
            if safe and abs(w) > .005:
                return MotionResult(0., w, 'ALIGN', False, trajectory)
        safe, trajectory = self.safe(state, scan, 0., 0., now)
        return MotionResult(0., 0., 'BRAKE_PATH_BLOCKED' if safe else 'EMERGENCY_STOP', True, trajectory)
