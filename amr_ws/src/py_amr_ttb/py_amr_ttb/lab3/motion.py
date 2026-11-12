"""The robot keeps moving during command delay and braking, so a clear
next position alone does not make a command acceptable.

Predict the stopping path from measured speeds and check it against the current scan
before issuing motion toward a target.
"""

import math
from dataclasses import replace
import numpy as np

from py_amr_ttb.odom_utils import normalize_angle
from .geometry import inverse_points, scan_at_pose, free_distances
from .types import MotionResult
# Keep these imports available to existing callers of motion.approach/rollout.
from .trajectory import approach, rollout, evaluate_trajectory, command_candidates


class MotionController:
    """A planner's target can remain useful even when new obstacles
    require slowing, turning or stopping before reaching it.

    Choose velocities from current sensing and measured motion, with a stopping-
    sweep check for each candidate.
    """

    def __init__(self, config):
        self.cfg = config
        self.aligning = False
        self.settling = False
        self.turn_anchor = None
        self.previous_yaw = None
        self.turn_without_progress = 0.

    def safe(self, state, scan, v, w, now, diagnostics=None):
        """Validate a command through braking using the shared trajectory evaluator."""
        return evaluate_trajectory(state, scan, v, w, self.cfg, now, diagnostics)

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
        # Exclude evidence that cannot survive the publication budget before
        # evaluating a command. Direct geometric queries retain their full history.
        usable_age = self.cfg.sensor_timeout-self.cfg.control_period
        history = tuple(previous for previous in scan.history
                        if 0 <= now-previous.received <= usable_age)
        coverage = scan.coverage
        if coverage is not None:
            eligible = [(source, receipt) for source, receipt in
                        zip(coverage.sources, coverage.receipts)
                        if 0 <= now-receipt <= usable_age]
            coverage = replace(coverage,
                               sources=tuple(source for source, _ in eligible),
                               receipts=tuple(receipt for _, receipt in eligible))
        scan = replace(scan, history=history, coverage=coverage)
        result = self.step(state, scan, target, now, dt, plan.diagnostics.get('speed_cap'))
        if result.v or result.w:
            timeout = self.cfg.sensor_timeout
            receipts = scan.coverage.receipts if scan.coverage is not None else (
                previous.received for previous in scan.history)
            sensor_receipts = [state.received, scan.received]
            sensor_receipts.extend(receipt for receipt in receipts
                                   if 0 <= now-receipt <= timeout-self.cfg.control_period)
            sensor_expiry = min(sensor_receipts)+timeout
            result.diagnostics['valid_until'] = min(sensor_expiry, plan.created+self.cfg.plan_timeout)
        return result

    def step(self, state, scan, target, now, dt, speed_cap=None):
        """A reachable target can still require slowing, alignment or a stop.

        Choose a velocity using current measured motion and scan clearance.
        target.point uses starting odometry; v is m/s and w is rad/s. Each return
        ends this decision with a MotionResult for node.py to publish.
        """
        # Old measurements cannot support a new motion prediction.
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
        # Waypoint changes must not renew the turn allowance. Measure motion
        # in odometry so map corrections cannot count as translation.
        position = state.pose[:2]
        yaw = state.pose[2]
        if self.turn_anchor is None or math.dist(position, self.turn_anchor) >= .05:
            self.turn_anchor = tuple(position)
            self.turn_without_progress = 0.
        else:
            self.turn_without_progress += abs(normalize_angle(yaw-self.previous_yaw))
        self.previous_yaw = yaw
        if self.turn_without_progress >= 2*math.pi:
            return MotionResult(reason='TURN_WITHOUT_PROGRESS', diagnostics={
                'turned_radians': self.turn_without_progress,
                'displacement': math.dist(position, self.turn_anchor),
            })
        # Steering needs the target's distance and angle relative to the robot now.
        local = inverse_points([target.point], state.pose)[0]
        distance = float(np.linalg.norm(local))
        # A reached point has no useful bearing. Brake until navigation advances it.
        arrival = target.arrival_distance(
            c.goal_tolerance if target.mode == 'FINAL' else c.target_reached)
        if distance <= arrival:
            self.aligning = self.settling = False
            v = approach(state.v, 0., c.linear_acceleration, c.linear_braking, dt)
            w = approach(state.w, 0., c.angular_acceleration, c.angular_braking, dt)
            details = dict(target_distance=distance)
            clear, trajectory = self.safe(state, scan, v, w, now, details)
            return MotionResult(v if clear else 0., w if clear else 0.,
                                'TARGET_REACHED' if clear else 'EMERGENCY_STOP',
                                not clear, trajectory, details)
        heading = math.atan2(local[1], local[0])

        # Backward travel should align the robot's rear, rather than its front,
        # toward the target. A negative linear speed is selected below.
        if target.reverse:
            heading = normalize_angle(heading-math.pi)

        # Larger heading errors request faster turning, up to the angular limit.
        desired_w = float(np.clip(c.heading_gain*heading, -c.max_angular, c.max_angular))
        forward_recovery = target.mode in ('FOLLOW_WALL', 'FORWARD_CORRECTION', 'RECOVERY_ADVANCE')
        steer_while_moving = forward_recovery and not target.reverse and abs(heading) <= math.pi/4
        was_aligning = self.aligning
        self.aligning = not steer_while_moving and abs(heading) >= (
            c.alignment_tolerance if was_aligning else c.rotate_threshold)
        self.settling = not steer_while_moving and not self.aligning and (
            was_aligning or self.settling)
        details = dict(heading_error=heading, target_distance=distance,
                       translation_rejection=None)
        # Collect linear-speed limits with their reasons, then use the lowest.
        # A replay bend may lower the ordinary cruise-speed ceiling.
        top = min(c.max_linear, c.max_linear if speed_cap is None else speed_cap)
        reasons = [(top, 'SPEED_LIMIT' if top == c.max_linear else 'ROUTE_BEND')]

        # Reduce forward motion while pointing away from the desired direction.
        if target.mode != 'FOLLOW_WALL':
            reasons.append((top*max(0., math.cos(heading)), 'HEADING'))

        # Turning at speed produces lateral acceleration, approximately abs(v * w).
        if abs(desired_w) > .05:
            reasons.append((c.lateral_acceleration/abs(desired_w), 'TURN'))

        # Final goals and short approaches cap speed by the remaining stopping distance.
        if target.mode in ('FINAL', 'APPROACH', 'RECOVERY_REVERSE', 'RECOVERY_ADVANCE'):
            reasons.append((
                math.sqrt(2*c.linear_braking*max(0., distance-target.arrival_distance(c.goal_tolerance))),
                'ARRIVAL',
            ))

        # Anticipate an obstacle during near-straight travel. Reserve maneuvering
        # room plus the command horizon, latency and braking distance.
        if not self.aligning and abs(desired_w) <= .15:
            local_scan = scan_at_pose(scan, state.pose)
            direction = math.pi if target.reverse else 0.
            delay = c.command_latency+max(0., now-scan.received)+c.prediction_horizon
            lookahead = top*delay+top*top/(2*c.linear_braking)+c.target_reserve
            room = float(free_distances(
                local_scan.points, [direction], c.radius, lookahead)[0])
            available = max(0., room-c.target_reserve)
            braking_delay = c.linear_braking*delay
            clearance_speed = math.sqrt(braking_delay**2+2*c.linear_braking*available)-braking_delay
            reasons.append((clearance_speed, 'OBSTACLE_APPROACH'))
            details.update(obstacle_travel=room, obstacle_speed_cap=clearance_speed)

        # Selecting the whole pair keeps the limiting speed tied to its explanation.
        desired, reason = min(reasons, key=lambda pair: pair[0])

        # Recovery may reverse slowly; a large heading error requires alignment first.
        if target.reverse:
            desired = -min(desired, c.reverse_speed)
            reason = 'REVERSE'

        if self.aligning:
            desired, reason = 0., 'ALIGN'
            if abs(state.v) > .025:
                # Brake translation before turning so the moving body does not
                # sweep sideways into a nearby obstacle during alignment.
                desired_w = 0.
                reason = 'BRAKE_FOR_TURN'

        # After stationary alignment, check residual rotation before accelerating.
        if self.settling and abs(state.w) > c.angular_settle_speed:
            clear, trajectory = self.safe(state, scan, 0., 0., now, details)
            return MotionResult(reason='BRAKE_ROTATION' if clear else 'EMERGENCY_STOP',
                                blocked=not clear, trajectory=trajectory, diagnostics=details)
        self.settling = False

        # Normal command changes respect acceleration and braking limits.
        # Start from measured speeds, since the previous command may not be achieved yet.
        v = approach(state.v, desired, c.linear_acceleration, c.linear_braking, dt)
        w = approach(state.w, desired_w, c.angular_acceleration, c.angular_braking, dt)
        v = float(np.clip(v, -c.reverse_speed, c.max_linear))
        w = float(np.clip(w, -c.max_angular, c.max_angular))

        # Try slower translation when the requested stopping sweep is blocked.
        if self.aligning:
            clear, trajectory = self.safe(state, scan, v, w, now, details)
            return MotionResult(v if clear else 0., w if clear else 0.,
                                reason if clear else 'ALIGN_BLOCKED', not clear, trajectory, details)

        attempts = []
        details['attempted_commands'] = attempts
        # The preferred command keeps clear-space tracking inexpensive.
        # If blocked, search reachable speed/steering pairs in preference order.
        for candidate_v, candidate_w in command_candidates(state, v, w, c, dt):
            checks = {}
            safe, trajectory = self.safe(state, scan, candidate_v, candidate_w, now, checks)
            attempts.append(dict(v=candidate_v, w=candidate_w, clear=safe, **checks))
            details.update(checks)
            if safe:
                if abs(candidate_v) <= .004 and reason == 'OBSTACLE_APPROACH':
                    details['translation_rejection'] = 'OBSTACLE'
                    stop_details = {}
                    stop_clear, stop_trajectory = self.safe(
                        state, scan, 0., 0., now, stop_details)
                    details['stop_clearance'] = stop_details
                    return MotionResult(0., 0.,
                                        'TRANSLATION_OBSTACLE' if stop_clear else 'EMERGENCY_STOP',
                                        True, stop_trajectory, details)
                preferred = (candidate_v, candidate_w) == (v, w)
                details['selected_candidate'] = len(attempts)-1
                return MotionResult(candidate_v, candidate_w,
                                    reason if preferred else 'CLEARANCE',
                                    False, trajectory, details)

        # Translation failure is distinct from heading alignment.
        rejection = 'OBSTACLE' if any(
            attempt.get('hit_rejections', 0) for attempt in attempts) else 'COVERAGE'
        details['translation_rejection'] = rejection

        # No travel or alignment command qualified. Request zero velocity and
        # tell planning it is blocked. Even this stop has residual motion: report
        # EMERGENCY_STOP if the predicted braking path itself fails clearance.
        diagnostics = {}
        safe, trajectory = self.safe(state, scan, 0., 0., now, diagnostics)
        return MotionResult(
            0., 0., 'TRANSLATION_'+rejection if safe else 'EMERGENCY_STOP', True, trajectory,
            dict(details, stop_clearance=diagnostics),
        )
