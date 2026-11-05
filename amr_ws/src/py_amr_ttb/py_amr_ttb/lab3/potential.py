"""Local full-footprint heading planner with latched brake/align/drive execution.

The historic module/class name is retained for launch compatibility. This is
NOT a pure artificial potential field and does not guarantee maze completion.
"""
import math
import numpy as np
from .core import LectureNavigator, free_distances, guard_command, wrap


class PotentialNavigator(LectureNavigator):
    def __init__(self, config):
        super().__init__(config)
        self.phase = 'BRAKE'
        self.target = None  # Persistent heading in the captured odometry frame.
        self.feedback = None
        self.previous_pose = None
        self.still_since = None
        self.scan_token = None
        self.confirmations = 0
        self.fallback_count = 0  # Existing CSV/simulation compatibility.
        self.switch_count = 0

    def set_motion_feedback(self, linear, angular, stamp):
        self.feedback = (abs(float(linear)), abs(float(angular)), float(stamp))

    def pause(self, state='WAITING'):
        self.phase = 'BRAKE'
        self.still_since = None
        self.confirmations = 0
        return super().pause(state)

    def output(self, v, w, state):
        self.previous_v = float(v)
        self.state = state
        self.behavior = state
        self.diagnostics.update(phase=self.phase, reason=state)
        return float(v), float(w)

    def step(self, pose, scan, now, dt, mode='goal'):
        if self.done or self.failure:
            return self.pause('DONE' if self.done else self.failure)
        if (not scan.healthy or not len(scan.points) or
                not np.isfinite(scan.points).all()):
            return self.pause('SCAN_INVALID')
        if not all(math.isfinite(z) for z in (*pose, now, dt)) or not 0 < dt <= self.cfg.sensor_timeout:
            return self.pause('TIMING_GAP')
        # ROS supplies measured twist. Offline callers can use pose differences.
        if self.feedback is not None:
            speed, spin, stamp = self.feedback
            if not all(math.isfinite(z) for z in self.feedback) or not 0 <= now-stamp <= self.cfg.sensor_timeout:
                return self.pause('MOTION_FEEDBACK_STALE')
        elif self.previous_pose is None:
            speed, spin = math.inf, math.inf
        else:
            speed = math.dist(pose[:2], self.previous_pose[:2])/dt
            spin = abs(wrap(pose[2]-self.previous_pose[2]))/dt
        self.previous_pose = tuple(pose)
        fresh_scan = scan is not self.scan_token
        self.scan_token = scan
        if speed < .012 and spin < .035:
            if self.still_since is None:
                self.still_since = now
        else:
            self.still_since = None
        radius = self.cfg.radius + max(self.cfg.corridor_padding, self.cfg.sweep_padding)
        nearest = float(np.linalg.norm(scan.points, axis=1).min())
        self.diagnostics = {'obstacle_distance_m': nearest, 'measured_speed_m_s': speed}
        if nearest <= radius:
            return self.pause('FOOTPRINT_BLOCKED')
        x, y, yaw = pose
        distance = math.hypot(self.cfg.goal_x-x, self.cfg.goal_y-y)
        if mode == 'goal' and distance <= self.cfg.goal_tolerance:
            self.done = True
            return self.pause('DONE')
        if self.started is None:
            self.started = self.last_progress = now
        if distance < self.best_distance-self.cfg.leave_progress:
            self.best_distance, self.last_progress = distance, now
        if mode == 'goal' and now-self.last_progress > self.cfg.progress_timeout:
            self.failure = 'NO_PROGRESS'
        if now-self.started > self.cfg.max_runtime:
            self.failure = 'TIME_LIMIT'
        if self.failure:
            return self.pause(self.failure)
        goal_heading = wrap(math.atan2(self.cfg.goal_y-y, self.cfg.goal_x-x)-yaw) if mode == 'goal' else 0.
        # One geometry model serves both selection and execution. Include exact
        # goal and remembered heading, so the grid cannot introduce a left bias.
        old = wrap(self.target-yaw) if self.target is not None else 0.
        headings = np.r_[np.linspace(-math.pi, math.pi, 181, endpoint=False), goal_heading, old, 0.]
        rooms = free_distances(scan.points, headings, radius)
        horizon = min(self.cfg.lookahead, max(.12, distance))
        usable = rooms >= min(.12, distance)
        score = (1.2*np.cos(headings-goal_heading) + .25*np.cos(headings)
                 + .35*np.minimum(rooms/horizon, 1.))
        score[~usable] = -math.inf
        best = int(np.argmax(score))
        if not np.isfinite(score[best]):
            return self.pause('NO_LOCAL_ROUTE')
        # Preserve a good direction; change only for a material improvement or
        # when the committed corridor is no longer usable. Never force left.
        old_index = len(headings)-2
        if self.target is not None and usable[old_index]:
            if self.phase == 'ALIGN' or score[best] < score[old_index]+.12:
                best = old_index
        chosen = float(headings[best])
        if self.target is not None and abs(wrap(chosen-old)) > .3:
            self.switch_count += 1
        self.target = wrap(yaw+chosen)
        self.diagnostics.update(target_heading_rad=chosen, available_travel_m=float(rooms[best]),
                                side_switch_count=self.switch_count)
        if self.phase == 'BRAKE':
            if self.still_since is not None and now-self.still_since >= .25:
                self.phase = 'ALIGN'
                self.confirmations = 0
            return self.output(0., 0., 'BRAKE_SETTLE')
        if self.phase == 'ALIGN':
            if abs(chosen) < .10:
                if fresh_scan:
                    self.confirmations += 1
                if self.confirmations >= 3 and spin < .08:
                    self.phase = 'DRIVE'
                return self.output(0., 0., 'ALIGN_CONFIRM')
            self.confirmations = 0
            w = np.clip(self.cfg.angular_gain*chosen, -self.cfg.max_angular, self.cfg.max_angular)
            return self.output(0., w, 'ALIGN_TURN')
        if abs(chosen) > .40:
            self.phase = 'BRAKE'
            self.still_since = None
            return self.output(0., 0., 'BRAKE_FOR_TURN')
        w = float(np.clip(self.cfg.angular_gain*chosen, -self.cfg.max_angular, self.cfg.max_angular))
        desired = min(self.cfg.max_linear, self.cfg.linear_gain*distance) if mode == 'goal' else self.cfg.max_linear
        # Keep tight obstacle negotiation below the straight-line speed limit.
        # A fast straight section must not dictate corner entry speed.
        if rooms[-3] < min(distance, self.cfg.lookahead) or abs(chosen) > .15:
            desired = min(desired, .10)
        # Reserve translation for receipt latency and finite braking. The
        # 2 cm reserve is additional free travel, not a smaller body radius.
        forward = float(free_distances(scan.points, [0.], radius)[0])
        travel = max(0., forward-.02)
        a = self.cfg.braking_deceleration
        latency = self.cfg.sensor_timeout + .18
        limit = math.sqrt((a*latency)**2 + 2*a*travel)-a*latency
        v = min(desired*max(0., math.cos(chosen)), limit,
                self.previous_v+self.cfg.acceleration*dt)
        # Reduce speed BEFORE clipping to a stop. Check the currently measured
        # residual speed too; a previous zero command is not proof of rest.
        for fraction in (1., .75, .5, .25):
            trial = v*fraction
            gv, gw = guard_command(trial, w, scan, self.cfg, max(speed, self.previous_v))
            if gv > .005:
                return self.output(gv, gw, 'DRIVE')
        self.phase = 'BRAKE'
        self.still_since = None
        self.target = None
        return self.output(0., 0., 'BRAKE_PATH_BLOCKED')