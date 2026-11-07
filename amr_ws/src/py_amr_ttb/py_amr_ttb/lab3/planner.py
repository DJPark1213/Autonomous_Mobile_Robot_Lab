"""Always pointing at the goal can trap the robot behind walls, while
changing direction at every scan can cause oscillation.

Balance observed travel and goal progress with turning and failure history. Hold
useful targets, recover when stalled, and follow a verified saved route when
available.
"""

import math
import numpy as np

from py_amr_ttb.odom_utils import normalize_angle
from .geometry import compose, transform_points, observed_travel, scan_at_pose
from .replay import speed_profile
from .types import Target, Plan


class Planner:
    """Exploration, wall following, replay and backtracking need one
    decision owner so they do not issue competing targets.

    Targets use map coordinates; candidate clearance uses the current robot frame.
    Motion control independently checks commands against current sensing.
    """

    def __init__(self, config, memory, replay_route=None):
        self.cfg, self.memory = config, memory
        self.target = None
        self.last_decision = -math.inf
        self.started = None
        self.anchor = None
        self.progress_at = None
        self.recent = []
        self.route = replay_route
        self.route_index = 0
        self.speeds = speed_profile(replay_route, config) if replay_route is not None else None
        self.diagnostics = {}

    def sections(self):
        """An escape from a blocked forward path may lie beside or
        behind the robot.

        Return angular intervals covering front and rear. Rear bounds can exceed pi;
        candidate angles are normalized before evaluation.
        """
        front = np.linspace(-math.pi/2, math.pi/2, self.cfg.front_sections+1)
        rear = np.linspace(math.pi/2, 3*math.pi/2, self.cfg.rear_sections+1)
        return list(zip(front[:-1], front[1:]))+list(zip(rear[:-1], rear[1:]))

    def evaluate(self, heading, scan, pose, state, grid, now):
        """A direction toward the goal is useless if the robot cannot
        fit along it or it repeats a recent failed attempt.

        For a robot-relative heading and scan, require enough observed travel, then
        rank a map target by progress, room, clearance, turning and failure history.
        Return (Target, available travel, heading), or None. The score ranks
        choices; it is not a probability.
        """
        c = self.cfg
        room = observed_travel(scan, heading, c.radius, c.scoring_horizon,
                               step=max(.05, c.grid_resolution))
        travel = min(c.target_distance, room-c.target_reserve)
        if travel < c.target_min_distance:
            return None
        goal = (c.goal_x, c.goal_y)
        distance = math.dist(pose[:2], goal)
        local = (travel*math.cos(heading), travel*math.sin(heading))
        point = transform_points([local], pose)[0]
        progress = (distance-math.dist(point, goal))/c.scoring_horizon
        hits = scan.points
        clearance = (float(np.linalg.norm(hits-np.asarray(local), axis=1).min())-c.radius
                     if len(hits) else c.scoring_horizon)
        desired_w = np.clip(c.heading_gain*heading, -c.max_angular, c.max_angular)
        score = (c.weight_goal*progress+c.weight_travel*room/c.scoring_horizon
                 +c.weight_clearance*min(1., max(0., clearance)/.35)
                 -c.weight_heading*abs(normalize_angle(heading))/math.pi
                 -c.weight_rotation*abs(desired_w-state.w)/(2*c.max_angular)
                 -c.weight_history*self.memory.penalty(pose[:2], pose[2]+heading, now))
        return Target(tuple(point), score=float(score)), room, heading

    def replay_target(self, pose, scan, grid):
        """A distant saved waypoint can tempt the robot to cut across a
        wall or an unseen corner.

        Advance the route index monotonically through verified map segments,
        returning a map Target or None. Attach a bend-speed cap for control, whose
        current scan checks still take precedence.
        """
        if self.route is None:
            return None
        while self.route_index < len(self.route)-1 and math.dist(pose[:2], self.route[self.route_index]) < self.cfg.target_reached:
            self.route_index += 1
        selected = None
        for i in range(self.route_index, len(self.route)):
            if math.dist(pose[:2], self.route[i]) > self.cfg.target_distance and selected is not None:
                break
            if grid.segment_clear(pose[:2], self.route[i], margin=self.cfg.route_margin):
                selected = i
            else:
                break
        if selected is None:
            return None
        self.route_index = max(self.route_index, selected)
        point = tuple(self.route[selected])
        # A saved route may now be obstructed; its speed cap cannot override
        # current LiDAR checks in the motion controller.
        self.diagnostics['speed_cap'] = float(self.speeds[max(0, selected-1)])
        return Target(point, 'FINAL' if selected == len(self.route)-1 else 'SECOND_RUN')

    def step(self, state, scan, mapper, now, blocked=False):
        """A locally attractive direction can stop making progress or
        lead back into the same failed area.

        Track progress, completion and timeout; retain useful targets and committed
        recovery. On stalls, revisits or observed caps, seek a verified return
        point. Otherwise use replay or scored section candidates, with a switching
        margin to limit oscillation. Return a Plan carrying its matching map
        transform.
        """
        c, grid = self.cfg, mapper.grid
        pose = compose(mapper.transform, state.pose)
        local_scan = scan_at_pose(scan, state.pose)
        self.memory.record(pose, now)
        if self.started is None:
            self.started = self.progress_at = now
            self.anchor = pose[:2]
        distance = math.hypot(c.goal_x-pose[0], c.goal_y-pose[1])
        if self.memory.complete or distance <= c.goal_tolerance:
            self.memory.complete = True
            self.target = Target((c.goal_x, c.goal_y), 'DONE')
            return Plan(self.target, mapper.transform, now, grid.revision, 'DONE')
        if now-self.started > c.max_runtime:
            return Plan(None, mapper.transform, now, grid.revision, 'TIME_LIMIT')
        near = self.target is None or math.dist(pose[:2], self.target.point) < c.target_reached
        if self.target and self.target.mode == 'BACKTRACK' and not near and not blocked:
            # Reconsidering too soon could send recovery straight back into the failed area.
            return Plan(self.target, mapper.transform, now, grid.revision)
        if distance <= c.target_distance:
            goal_heading = normalize_angle(math.atan2(c.goal_y-pose[1], c.goal_x-pose[0])-pose[2])
            if observed_travel(local_scan, goal_heading, c.radius, distance+.15) >= distance:
                self.target = Target((c.goal_x, c.goal_y), 'FINAL')
                return Plan(self.target, mapper.transform, now, grid.revision)
        if math.dist(pose[:2], self.anchor) >= c.progress_distance:
            # Tiny pose changes should not keep a stuck robot's progress timer alive.
            self.anchor, self.progress_at = pose[:2], now
        self.recent.append((now, *pose[:2]))
        self.recent = [entry for entry in self.recent if now-entry[0] <= c.revisit_window]
        revisited = any(now-entry[0] >= c.revisit_min_age
                        and math.dist(pose[:2], entry[1:]) < c.revisit_radius
                        for entry in self.recent)
        goal_angle = math.atan2(c.goal_y-pose[1], c.goal_x-pose[0])
        capped = grid.closed_ahead(pose[:2], goal_angle)
        stalled = now-self.progress_at >= c.stall_timeout or revisited or capped
        if stalled:
            # Remember this failed direction so the next decision does not repeat it.
            direction = (math.atan2(self.target.point[1]-pose[1], self.target.point[0]-pose[0])
                         if self.target else pose[2])
            self.memory.fail(pose[:2], direction, now)
            self.progress_at = now
            previous = self.memory.return_target(pose[:2], grid)
            if previous is not None:
                angle = normalize_angle(math.atan2(previous[1]-pose[1], previous[0]-pose[0])-pose[2])
                # A return point far behind may be reached by backing out; the
                # motion sweep must still verify rear clearance.
                self.target = Target(previous, 'BACKTRACK', reverse=abs(angle) > 2.2)
                self.memory.fail(previous, goal_angle, now)
                self.recent.clear()
                self.last_decision = now
                return Plan(self.target, mapper.transform, now, grid.revision)
        near = self.target is None or math.dist(pose[:2], self.target.point) < c.target_reached
        if not blocked and not near and not stalled and now-self.last_decision < c.decision_period:
            # A usable held target avoids steering changes at every sensor update.
            return Plan(self.target, mapper.transform, now, grid.revision, diagnostics=self.diagnostics)
        self.last_decision = now
        self.diagnostics = {}
        if self.route is not None and not blocked and not stalled:
            target = self.replay_target(pose, local_scan, grid)
            if target is not None:
                self.target = target
                return Plan(target, mapper.transform, now, grid.revision, diagnostics=self.diagnostics)
        # Coarse angular samples may miss a clear direct line to the goal.
        goal_heading = normalize_angle(math.atan2(c.goal_y-pose[1], c.goal_x-pose[0])-pose[2])
        options = []
        for lower, upper in self.sections():
            headings = list(np.linspace(lower, upper, c.heading_samples))
            if lower <= goal_heading <= upper:
                headings.append(goal_heading)
            candidates = [self.evaluate(normalize_angle(h), local_scan, pose, state, grid, now)
                          for h in headings]
            candidates = [candidate for candidate in candidates if candidate is not None]
            if candidates:
                best = max(candidates, key=lambda item: item[0].score)
                if not grid.closed_ahead(pose[:2], pose[2]+best[2]):
                    options.append(best)
        if not options:
            previous = self.memory.return_target(pose[:2], grid)
            angle = (normalize_angle(math.atan2(previous[1]-pose[1], previous[0]-pose[0])-pose[2])
                     if previous else 0.)
            self.target = Target(previous, 'BACKTRACK', reverse=abs(angle) > 2.2) if previous else None
            return Plan(self.target, mapper.transform, now, grid.revision, 'NO_LOCAL_ROUTE')
        best, room, heading = max(options, key=lambda item: item[0].score)
        if self.target and not near and not blocked and not stalled:
            # Nearly equal scores can alternate between scans and cause steering
            # oscillation; require a margin before abandoning a usable target.
            old_heading = normalize_angle(math.atan2(self.target.point[1]-pose[1], self.target.point[0]-pose[0])-pose[2])
            old = self.evaluate(old_heading, local_scan, pose, state, grid, now)
            if (old and old[1] >= math.dist(pose[:2], self.target.point)+c.target_reserve
                    and not grid.closed_ahead(pose[:2], pose[2]+old_heading)
                    and best.score < old[0].score+c.switch_margin):
                return Plan(self.target, mapper.transform, now, grid.revision)
        if distance <= c.target_distance and room >= distance+c.target_reserve and abs(normalize_angle(heading-goal_heading)) < .04:
            best = Target((c.goal_x, c.goal_y), 'FINAL', best.score)
        elif room < c.wall_trigger:
            best.mode = 'FOLLOW_WALL'
            best.wall_side = -1 if heading >= 0 else 1
        self.target = best
        self.diagnostics.update(candidate_count=len(options), score=best.score, available_travel=room)
        return Plan(best, mapper.transform, now, grid.revision, diagnostics=self.diagnostics)
