"""One objective owner feeds one acceleration-aware trajectory planner."""

import math

import numpy as np

from .geometry import compose, inverse_points, scan_at_pose, observed_travel, transform_points
from .types import Target, Plan
from .local_planner import TrajectoryPlanner


class Planner:
    """Keep route selection separate from the current-state motion supervisor."""

    def __init__(self, config, memory, replay_route=None):
        self.cfg, self.memory = config, memory
        self.motion = TrajectoryPlanner(config)
        self.route = replay_route
        self.route_index = 0
        self.target = None
        self.started = None
        self.best = math.inf
        self.progress_at = 0.
        self.detour = None
        self.diagnostics = {}
        self.reverse_travel = 0.
        self.forward_travel = 0.
        self.last_pose = None

    def observed_segment(self, pose, scan, point):
        distance = math.dist(pose[:2], point)
        heading = math.atan2(point[1]-pose[1], point[0]-pose[0])-pose[2]
        return observed_travel(scan, heading, self.cfg.radius, distance+.051) >= distance

    def step(self, state, scan, mapper, now, blocked=False, motion_reason=''):
        c = self.cfg
        transform = mapper.transform if mapper is not None else (0., 0., 0.)
        pose = compose(transform, state.pose)
        local_scan = scan_at_pose(scan, state.pose)
        # Changing objectives must not reset the measured reverse budget.
        if self.last_pose is not None:
            movement = math.dist(state.pose[:2], self.last_pose)
            if state.v < -.005:
                self.reverse_travel += movement
                self.forward_travel = 0.
            elif state.v > .005:
                self.forward_travel += movement
                if self.forward_travel >= c.progress_distance:
                    self.reverse_travel = 0.
        self.last_pose = state.pose[:2]
        self.memory.record(pose, now)
        if self.started is None:
            self.started, self.progress_at = now, now
        goal = (c.goal_x, c.goal_y)
        if math.dist(pose[:2], goal) <= c.goal_tolerance:
            self.memory.complete = True
            self.target = Target(goal, 'DONE')
            return Plan(self.target, transform, now, status='DONE', command=(0., 0.))
        if now-self.started > c.max_runtime:
            return Plan(None, transform, now, status='TIME_LIMIT')
        if self.detour is not None and math.dist(pose[:2], self.detour) <= .2:
            self.detour = None
        objective = goal
        source = 'GOAL'
        if self.detour is not None:
            objective, source = self.detour, 'DETOUR'
        elif self.route is not None:
            while self.route_index < len(self.route)-1 and math.dist(pose[:2], self.route[self.route_index]) < .3:
                self.route_index += 1
            for i in range(self.route_index+1, len(self.route)):
                if math.dist(pose[:2], self.route[i]) > c.target_distance:
                    break
                if not self.observed_segment(pose, local_scan, self.route[i]):
                    break
                self.route_index = i
            objective = goal if self.route_index == len(self.route)-1 else tuple(self.route[self.route_index])
            source = 'REPLAY'
        distance = math.dist(pose[:2], objective)
        if self.target is None or math.dist(self.target.point, objective) > .1:
            self.best, self.progress_at = distance, now
        elif distance < self.best-.05:
            self.best, self.progress_at = distance, now
        # Only persistent lack of progress can replace a goal with a detour.
        if now-self.progress_at > c.stall_timeout+2.:
            direction = math.atan2(objective[1]-pose[1], objective[0]-pose[0])
            self.memory.fail(pose[:2], direction, now)
            options = []
            for heading in np.linspace(-math.pi, math.pi, 25):
                travel = observed_travel(local_scan, heading, c.radius, 1.2)
                if travel < .3:
                    continue
                point = transform_points([(travel*math.cos(heading), travel*math.sin(heading))], pose)[0]
                score = travel-.4*math.dist(point, goal)-self.memory.penalty(pose[:2], pose[2]+heading, now)
                options.append((score, tuple(point)))
            if options:
                self.detour = max(options)[1]
                objective, source = self.detour, 'DETOUR'
            self.progress_at, self.best = now, math.dist(pose[:2], objective)
        self.target = Target(tuple(objective), source)
        odom_target = tuple(inverse_points([objective], transform)[0])
        command, details = self.motion.choose(state, scan, odom_target, now,
                                              allow_reverse=self.reverse_travel < c.backtrack_distance)
        self.diagnostics = dict(details, objective=source, feedback=motion_reason, blocked=blocked, reverse_travel=self.reverse_travel)
        status = details.get('rejection', 'RUNNING')
        return Plan(self.target, transform, now, status=status,
                    diagnostics=self.diagnostics, command=command)
