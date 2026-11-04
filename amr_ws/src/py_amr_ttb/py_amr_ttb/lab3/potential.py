import math

import numpy as np

from .core import LectureNavigator, free_distances, guard_command


class PotentialNavigator(LectureNavigator):
    """Follow a continuous field while retaining the baseline's stopping guard."""

    def __init__(self, config):
        super().__init__(config)
        self.behavior = 'FIELD_GO_TO_GOAL'
        self.fallback = None
        self.fallback_count = 0
        self.fallback_reason = ''
        self.motion_anchor = None
        self.motion_since = None
        self.tangent_weight = 0.0
        self.last_wall_direction = -1
        self.switch_ticks = 0
        self.last_switch_time = -math.inf
        self.switch_count = 0

    def pause(self, state='WAITING'):
        """Pause both controllers and restart stall timing after data recovers."""
        self.motion_anchor = self.motion_since = None
        self.switch_ticks = 0
        if self.fallback is not None:
            self.fallback.pause(state)
        return super().pause(state)

    def start_fallback(self, goal, normal, distance, now, reason):
        """Hand the same pose, detour direction and speed to lecture wall following."""
        if not self.wall_direction:
            self.select_wall_direction(goal, normal, distance)
        fallback = LectureNavigator(self.cfg)
        fallback.wall_direction = self.wall_direction
        fallback.behavior = fallback.wall_behavior()
        fallback.entry_distance = distance
        fallback.previous_v = self.previous_v
        fallback.started = self.started
        fallback.last_progress = now
        fallback.best_distance = distance
        self.fallback = fallback
        self.fallback_count += 1
        self.fallback_reason = reason

    def step_fallback(self, pose, scan, now, dt, mode):
        """Return to the field only after the lecture controller clears the obstacle."""
        v, w = self.fallback.step(pose, scan, now, dt, mode)
        self.previous_v = v
        self.diagnostics.update(self.fallback.diagnostics)
        self.diagnostics.update(fallback_count=self.fallback_count,
                                fallback_reason=self.fallback_reason)
        self.behavior = 'FALLBACK_' + self.fallback.behavior
        self.state = 'FALLBACK_' + self.fallback.state
        if self.fallback.done:
            self.done = True
            return self.pause('DONE')
        if self.fallback.failure:
            self.failure = self.fallback.failure
            return self.pause(self.failure)
        if self.fallback.behavior == 'GO_TO_GOAL':
            self.fallback = None
            self.wall_direction = 0
            self.clear_ticks = 0
            self.tangent_weight = 0.0
            self.motion_anchor = np.asarray(pose[:2], dtype=float)
            self.motion_since = now
        return v, w

    def step(self, pose, scan, now, dt, mode='goal'):
        """Return a bounded (v,w); recover from a stalled or degenerate field."""
        if self.done or self.failure:
            return self.pause('DONE' if self.done else self.failure)
        if not scan.healthy:
            return self.pause('SCAN_INVALID')
        if not all(math.isfinite(v) for v in (*pose, now, dt)) or dt <= 0:
            return self.pause('INVALID_INPUT')
        if dt > self.cfg.sensor_timeout:
            return self.pause('TIMING_GAP')
        x, y, yaw = pose
        distance = math.hypot(self.cfg.goal_x-x, self.cfg.goal_y-y)
        if mode == 'goal' and distance <= self.cfg.goal_tolerance:
            self.done = True
            return self.pause('DONE')
        if self.started is None:
            self.started = self.last_progress = now
        if mode == 'goal':
            if distance < self.best_distance - .03:
                self.best_distance = distance
                self.last_progress = now
            if now-self.last_progress > self.cfg.progress_timeout:
                self.failure = 'NO_PROGRESS'
            if now-self.started > self.cfg.max_runtime:
                self.failure = 'TIME_LIMIT'
            if self.failure:
                return self.pause(self.failure)
        lengths = np.linalg.norm(scan.points, axis=1)
        obstacle_distance = float(lengths.min())
        if obstacle_distance <= self.cfg.radius:
            return self.pause('TOO_CLOSE')
        if self.fallback is not None:
            return self.step_fallback(pose, scan, now, dt, mode)

        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = self.cfg.goal_x-x, self.cfg.goal_y-y
        goal = np.array([c*dx+s*dy, -s*dx+c*dy]) if mode == 'goal' else np.array([1., 0.])
        goal_norm = float(np.linalg.norm(goal))
        goal_unit = goal / max(goal_norm, 1e-9)
        normal = -scan.points[np.argmin(lengths)] / obstacle_distance
        goal_heading = math.atan2(goal[1], goal[0])
        check_length = distance if mode == 'goal' else self.cfg.lookahead
        goal_free = free_distances(scan.points, [goal_heading], self.cfg.radius+self.cfg.corridor_padding,
                                   cap=max(4., check_length+.1))[0]
        corridor_clear = goal_free >= check_length

        # rho is free distance beyond the SAME inflated footprint as the baseline.
        rho = max(.01, obstacle_distance-self.cfg.radius)
        rho0 = self.cfg.field_influence
        activation = max(0., 1-rho/rho0)
        if activation > 0 and not corridor_clear and not self.wall_direction:
            self.select_wall_direction(goal, normal, distance)
            self.last_wall_direction = self.wall_direction
        if self.wall_direction:
            progress = mode != 'goal' or distance < self.entry_distance-self.cfg.leave_progress
            may_leave = corridor_clear and progress
            self.clear_ticks = self.clear_ticks+1 if may_leave else 0
            if self.clear_ticks >= 3:
                self.wall_direction = 0
                self.clear_ticks = 0

        attraction = min(1., goal_norm) * goal_unit
        magnitude = (self.cfg.repulsion_gain*(1/rho-1/rho0)/rho**2
                     if rho < rho0 else 0.)
        magnitude = min(2.5, max(0., magnitude))
        # Fade repulsion near a reachable goal to avoid the near-obstacle goal trap.
        if mode == 'goal' and corridor_clear:
            magnitude *= min(1., distance/self.cfg.field_goal_fade)**2
        repulsion = magnitude*normal
        clockwise = np.array([normal[1], -normal[0]])
        sign = self.wall_direction or self.last_wall_direction
        tangent = clockwise if sign == -1 else -clockwise
        requested_weight = self.cfg.tangent_gain*activation if self.wall_direction else 0.
        self.tangent_weight += min(1., dt/.15)*(requested_weight-self.tangent_weight)
        field = attraction + repulsion + self.tangent_weight*tangent
        # Compare both resultant field directions using the full inflated body.
        # Distances below are available translation, not centre-to-wall ranges.
        if self.wall_direction and not corridor_clear:
            other_field = attraction + repulsion - self.tangent_weight*tangent
            headings = [math.atan2(field[1], field[0]),
                        math.atan2(other_field[1], other_field[0])]
            room = free_distances(
                scan.points, headings,
                self.cfg.radius + self.cfg.corridor_padding)
            can_switch = (np.linalg.norm(other_field) > .05
                          and room[0] < .10 and room[1] > .25)
            self.switch_ticks = self.switch_ticks+1 if can_switch else 0
            if self.switch_ticks >= 3 and now-self.last_switch_time > 2.0:
                self.wall_direction *= -1
                self.last_wall_direction = self.wall_direction
                field = other_field
                self.entry_distance = distance
                self.clear_ticks = 0
                self.switch_ticks = 0
                self.last_switch_time = now
                self.switch_count += 1
                # Give the changed direction a fresh motion-stall interval.
                # Keep the overall no-progress/time-limit watchdogs intact.
                self.motion_anchor = np.array([x, y])
                self.motion_since = now
        else:
            self.switch_ticks = 0
        # At corners, check openings when the field cannot cover the lookahead.
        # Select an open full-body heading instead of insisting on a tangent.
        # This is a local check, not a global path or a curved-sweep guarantee.
        # The unchanged guard_command below still checks the commanded arc.
        opening_selected = False
        if not corridor_clear:
            field_heading = math.atan2(field[1], field[0])
            field_room = free_distances(
                scan.points, [field_heading],
                self.cfg.radius + self.cfg.corridor_padding)[0]
            if field_room < self.cfg.lookahead:
                candidates = np.linspace(-math.pi, math.pi, 361)
                rooms = free_distances(
                    scan.points, candidates,
                    self.cfg.radius + self.cfg.corridor_padding)
                available = rooms >= .15
                if np.any(available):
                    score = np.cos(candidates-goal_heading) + .15*np.cos(candidates)
                    score[~available] = -math.inf
                    chosen = candidates[int(np.argmax(score))]
                    field = np.array([math.cos(chosen), math.sin(chosen)])
                    opening_selected = True
        # A verified full-width straight corridor to the fixed goal takes
        # priority over side-wall repulsion. Keep the final sweep guard.
        if corridor_clear:
            field = attraction


        if self.motion_anchor is None:
            self.motion_anchor = np.array([x, y])
            self.motion_since = now
        if np.linalg.norm(np.array([x, y])-self.motion_anchor) >= .05:
            self.motion_anchor = np.array([x, y])
            self.motion_since = now
        stalled = not corridor_clear and now-self.motion_since >= self.cfg.field_stall_timeout
        cycling = not corridor_clear and now-self.last_progress >= 4*self.cfg.field_stall_timeout
        low_field = float(np.linalg.norm(field)) < .05
        self.diagnostics = {
            'obstacle_distance_m': obstacle_distance,
            'wall_entry_goal_distance_m': self.entry_distance,
            'goal_normal_dot': float(np.dot(normal, goal)),
            'goal_corridor_clear': corridor_clear,
            'field_x': float(field[0]), 'field_y': float(field[1]),
            'repulsion_magnitude': magnitude, 'tangent_weight': self.tangent_weight,
            'fallback_count': self.fallback_count, 'fallback_reason': self.fallback_reason,
            'side_switch_count': self.switch_count,
            'opening_selected': opening_selected,
        }
        if low_field or ((stalled or cycling) and not opening_selected):
            self.start_fallback(goal, normal, distance, now,
                                'LOW_FIELD' if low_field else 'NO_MOTION' if stalled else 'NO_GOAL_PROGRESS')
            return self.step_fallback(pose, scan, now, dt, mode)

        heading = math.atan2(field[1], field[0])
        w = float(np.clip(self.cfg.angular_gain*heading,
                          -self.cfg.max_angular, self.cfg.max_angular))
        speed = self.cfg.max_linear
        if mode == 'goal':
            speed = min(speed, self.cfg.linear_gain*distance)
        # Continuous motion through moderate heading errors, with no 0.55-rad cutoff.
        v = speed*max(0., math.cos(heading))
        forward_free = free_distances(scan.points, [0.], self.cfg.radius+self.cfg.corridor_padding)[0]
        travel = max(0., forward_free-.02)
        a, latency = self.cfg.braking_deceleration, self.cfg.sensor_timeout
        stopping_limit = math.sqrt((a*latency)**2+2*a*travel)-a*latency
        v = min(v, stopping_limit, self.previous_v+self.cfg.acceleration*dt)
        v, w = guard_command(v, w, scan, self.cfg, self.previous_v)
        self.previous_v = v
        self.behavior = 'FIELD_BYPASS' if self.wall_direction else 'FIELD_GO_TO_GOAL'
        self.state = ('SAFETY_STOP' if v == 0 and w == 0 else
                      'TURN_IN_PLACE' if v == 0 else self.behavior)
        return v, w
