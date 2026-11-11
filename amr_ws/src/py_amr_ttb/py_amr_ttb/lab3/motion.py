"""The robot keeps moving during command delay and braking, so a clear
next position alone does not make a command acceptable.

Predict the stopping path from measured speeds and check it against the current scan
before issuing motion toward a target.
"""

import math
from dataclasses import replace
import numpy as np

from py_amr_ttb.odom_utils import normalize_angle
from .geometry import inverse_points, transform_points, circles_clear
from .types import MotionResult


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


class MotionController:
    """A planner's target can remain useful even when new obstacles
    require slowing, turning or stopping before reaching it.

    Choose velocities from current sensing and measured motion, with a stopping-
    sweep check for each candidate.
    """

    def __init__(self, config):
        self.cfg = config

    def safe(self, state, scan, v, w, now, diagnostics=None):
        """Obstacles can lie between predicted positions or along the
        later braking path, not just at a command's endpoint.

        Return (is_clear, trajectory). Predict through a complete stop, convert
        positions into the scan frame, and use enlarged midpoint circles to cover
        each sampled segment. Clearance depends on scan, pose and response-model
        accuracy.
        """
        scan_age = max(0.0, now - scan.received)
        history = tuple(previous for previous in scan.history
                        if 0 <= now-previous.received <= self.cfg.sensor_timeout)
        if len(history) != len(scan.history):
            scan = replace(scan, history=history)
        trajectory = rollout(state, v, w, self.cfg, scan_age)
        odometry_positions = np.asarray(trajectory)[:, :2]
        scan_frame_positions = inverse_points(odometry_positions, scan.pose)
        # Endpoint checks can skip obstacles between samples. A midpoint circle
        # enlarged by half the segment length covers the intervening body motion
        # without adding needless travel padding when the robot is stationary.
        segment_midpoints = (scan_frame_positions[:-1] + scan_frame_positions[1:]) / 2
        segment_lengths = np.linalg.norm(np.diff(scan_frame_positions, axis=0), axis=1)
        swept_radii = self.cfg.radius + segment_lengths / 2
        is_clear = circles_clear(scan, segment_midpoints, swept_radii,
                                 occupied_radius=self.cfg.radius,
                                 occupied_center=scan_frame_positions[0], diagnostics=diagnostics)
        return is_clear, trajectory

    def step_plan(self, state, scan, plan, now, dt):
        """An old decision or a target paired with the wrong map
        correction could steer toward an obsolete location.

        Reject missing or expired plans with zero velocity. Use the plan's own
        transform to express its target in starting odometry before calling step.
        """
        if plan is None or not 0 <= now-plan.created <= self.cfg.plan_timeout:
            return MotionResult(reason='WAITING_FOR_FRESH_PLAN')
        if plan.target is None:
            return MotionResult(reason=plan.status)
        target = replace(plan.target, point=tuple(inverse_points([plan.target.point], plan.transform)[0]))
        return self.step(state, scan, target, now, dt, plan.diagnostics.get('speed_cap'))

    def step(self, state, scan, target, now, dt, speed_cap=None):
        """A reachable target can still require slowing, alignment or a stop.

        Choose a velocity using current measured motion and scan clearance.
        target.point uses starting odometry; v is m/s and w is rad/s. Each return
        ends this decision with a MotionResult for node.py to publish.
        """
        # 1. Old measurements cannot support a new motion prediction.
        # Invalid timing, no target or a completed run requests zero velocity.
        c = self.cfg
        if (
            not 0 < dt <= c.sensor_timeout
            or not 0 <= now-state.received <= c.sensor_timeout
            or not 0 <= now-scan.received <= c.sensor_timeout
        ):
            return MotionResult(reason='SENSOR_STOP', blocked=True)

        if target is None:
            return MotionResult(reason='WAITING')

        if target.mode == 'DONE':
            return MotionResult(reason='DONE')

        # 2. Steering needs the target's distance and angle relative to the robot now.
        local = inverse_points([target.point], state.pose)[0]
        distance = float(np.linalg.norm(local))
        heading = math.atan2(local[1], local[0])

        # Backward travel should align the robot's rear, rather than its front,
        # toward the target. A negative linear speed is selected below.
        if target.reverse:
            heading = normalize_angle(heading-math.pi)

        # 3. Wall following needs a direction along the wall plus a correction
        # toward the desired separation, instead of steering only at the target.
        if target.mode == 'FOLLOW_WALL' and len(scan.points):
            # The robot may have moved since acquisition; compare hits in its frame now.
            points = inverse_points(transform_points(scan.points, scan.pose), state.pose)
            side = points[:, 1]*target.wall_side > 0
            if side.any():
                candidates = points[side]
                nearest = candidates[np.argmin(np.linalg.norm(candidates, axis=1))]
                separation = np.linalg.norm(nearest)

                # normal points away from the nearest hit; tangent runs along the wall.
                # The separation error pushes away when too close and toward it when far.
                normal = -nearest/max(separation, 1e-9)
                tangent = target.wall_side*np.array([-normal[1], normal[0]])
                vector = tangent+np.clip(
                    c.wall_gain*(c.wall_distance-separation), -.7, .7,
                )*normal
                heading = math.atan2(vector[1], vector[0])

        # 4. Larger heading errors request faster turning, up to the angular limit.
        desired_w = float(np.clip(c.heading_gain*heading, -c.max_angular, c.max_angular))

        # 5. Collect linear-speed limits with their reasons, then use the lowest.
        # A replay bend may lower the ordinary cruise-speed ceiling.
        top = min(c.max_linear, c.max_linear if speed_cap is None else speed_cap)
        reasons = [(top, 'SPEED_LIMIT' if top == c.max_linear else 'ROUTE_BEND')]

        # Reduce forward motion while pointing away from the desired direction.
        reasons.append((top*max(0., math.cos(heading)), 'HEADING'))

        # Turning at speed produces lateral acceleration, approximately abs(v * w).
        if abs(desired_w) > .05:
            reasons.append((c.lateral_acceleration/abs(desired_w), 'TURN'))

        # Braking at every intermediate target would turn continuous travel into
        # stop-and-go motion. Only the final goal adds a stopping-distance limit.
        if target.mode == 'FINAL':
            reasons.append((
                math.sqrt(2*c.linear_braking*max(0., distance-c.goal_tolerance)),
                'ARRIVAL',
            ))

        # Selecting the whole pair keeps the limiting speed tied to its explanation.
        desired, reason = min(reasons, key=lambda pair: pair[0])

        # 6. Recovery may reverse slowly; a large heading error requires alignment first.
        if target.reverse:
            desired = -min(desired, c.reverse_speed)
            reason = 'REVERSE'

        if abs(heading) >= c.rotate_threshold:
            desired, reason = 0., 'ALIGN'
            if abs(state.v) > .025:
                # Brake translation before turning so the moving body does not
                # sweep sideways into a nearby obstacle during alignment.
                desired_w = 0.
                reason = 'BRAKE_FOR_TURN'

        # 7. Normal command changes respect acceleration and braking limits.
        # Start from measured speeds, since the previous command may not be achieved yet.
        v = approach(state.v, desired, c.linear_acceleration, c.linear_braking, dt)
        w = approach(state.w, desired_w, c.angular_acceleration, c.angular_braking, dt)
        v = float(np.clip(v, -c.reverse_speed, c.max_linear))
        w = float(np.clip(w, -c.max_angular, c.max_angular))

        # 8. A desirable speed may lack stopping room. Try slower commands with
        # the same turn command; safe() predicts each from the measured current state.
        for fraction in (1., .75, .5, .25):
            trial = v*fraction
            safe, trajectory = self.safe(state, scan, trial, w, now)
            if safe and abs(trial) > .004:
                return MotionResult(
                    trial, w, reason if fraction == 1 else 'CLEARANCE', False, trajectory,
                )

        # 9. If travel failed, try turning in place only when almost stopped.
        # Rotation still needs a clear predicted stopping sweep.
        if abs(state.v) < .025:
            safe, trajectory = self.safe(state, scan, 0., w, now)
            if safe and abs(w) > .005:
                return MotionResult(0., w, 'ALIGN', False, trajectory)

        # 10. No travel or alignment command qualified. Request zero velocity and
        # tell planning it is blocked. Even this stop has residual motion: report
        # EMERGENCY_STOP if the predicted braking path itself fails clearance.
        diagnostics = {}
        safe, trajectory = self.safe(state, scan, 0., 0., now, diagnostics)
        if (safe and abs(state.v) < .005 and diagnostics.get('unknown_beams', 0)):
            observe_w = approach(state.w, .2 if heading >= 0 else -.2,
                                 c.angular_acceleration, c.angular_braking, dt)
            clear, observation_path = self.safe(state, scan, 0., observe_w, now)
            if clear:
                return MotionResult(0., observe_w, 'OBSERVE', False, observation_path, diagnostics)
        return MotionResult(
            0., 0., 'BRAKE_PATH_BLOCKED' if safe else 'EMERGENCY_STOP', True, trajectory,
            diagnostics,
        )
