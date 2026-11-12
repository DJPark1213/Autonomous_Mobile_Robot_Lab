"""Validate and execute the planner's motion without selecting another route."""

import math
from dataclasses import replace
from .types import MotionResult
from .trajectory import approach, evaluate_trajectory


class MotionController:
    """Ramp published commands and reject commands whose stopping sweep is unsafe."""

    def __init__(self, config):
        self.cfg = config

    def safe(self, state, scan, v, w, now, diagnostics=None):
        return evaluate_trajectory(state, scan, v, w, self.cfg, now, diagnostics)

    def step_plan(self, state, scan, plan, now, dt, previous_command=(0., 0.)):
        c = self.cfg
        lifetime = min(c.plan_timeout, 2*c.map_period)
        if plan is None or not 0 <= now-plan.created <= lifetime:
            return MotionResult(reason='WAITING_FOR_FRESH_PLAN')
        if (not 0 < dt <= c.sensor_timeout or
                not 0 <= now-state.received <= c.sensor_timeout or
                not 0 <= now-scan.received <= c.sensor_timeout):
            return MotionResult(reason='SENSOR_STOP', blocked=True)
        if plan.command is None or plan.target is None:
            return MotionResult(reason=plan.status)
        desired_v, desired_w = plan.command
        if (not all(math.isfinite(x) for x in plan.command) or
                not -c.reverse_speed <= desired_v <= c.max_linear or abs(desired_w) > c.max_angular):
            return MotionResult(reason='INVALID_COMMAND', blocked=True)
        usable_age = c.sensor_timeout-c.control_period
        history = tuple(previous for previous in scan.history
                        if 0 <= now-previous.received <= usable_age)
        coverage = scan.coverage
        if coverage is not None:
            eligible = [(source, receipt) for source, receipt in zip(coverage.sources, coverage.receipts)
                        if 0 <= now-receipt <= usable_age]
            coverage = replace(coverage, sources=tuple(item[0] for item in eligible),
                               receipts=tuple(item[1] for item in eligible))
        scan = replace(scan, history=history, coverage=coverage)
        v = approach(previous_command[0], desired_v, c.linear_acceleration, c.linear_braking, dt)
        w = approach(previous_command[1], desired_w, c.angular_acceleration, c.angular_braking, dt)
        checks = dict(previous_command=previous_command,
                      requested_command=plan.command, ramped_command=(v, w))
        safe, trajectory = self.safe(state, scan, v, w, now, checks)
        receipts = [state.received, scan.received]
        receipts += list(coverage.receipts) if coverage is not None else [item.received for item in history]
        checks['valid_until'] = min(min(receipts)+c.sensor_timeout, plan.created+lifetime)
        reason = 'TRAJECTORY' if plan.status == 'RUNNING' else plan.status
        if plan.status == 'RUNNING' and desired_v == 0. and desired_w == 0.:
            reason = 'PLANNER_HOLD'
        if not safe:
            reason = 'TRAJECTORY_BLOCKED'
        return MotionResult(v if safe else 0., w if safe else 0.,
                            reason, not safe or plan.status == 'NO_SAFE_TRAJECTORY', trajectory, checks)
