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
    Motion control independently checks commands against current sensing.p
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
        """A useful target can become blocked or lead back into a failed area.

        Keep it, replace it, or request recovery using the checks below in order.
        Each return ends this decision. The returned Plan carries the target's
        map transform; motion control checks velocity commands separately.
        """
        # 1. Targets use map coordinates; direction checks use the robot's frame now.
        c, grid = self.cfg, mapper.grid
        pose = compose(mapper.transform, state.pose)
        local_scan = scan_at_pose(scan, state.pose)
        self.memory.record(pose, now)

        if self.started is None:
            self.started = self.progress_at = now
            self.anchor = pose[:2]

        # 2. Finish once the goal is reached, or stop an overlong run.
        distance = math.hypot(c.goal_x-pose[0], c.goal_y-pose[1])
        if self.memory.complete or distance <= c.goal_tolerance:
            self.memory.complete = True
            self.target = Target((c.goal_x, c.goal_y), 'DONE')
            return Plan(self.target, mapper.transform, now, grid.revision, 'DONE')

        if now-self.started > c.max_runtime:
            return Plan(None, mapper.transform, now, grid.revision, 'TIME_LIMIT')

        # 3. Complete an unblocked retreat before choosing another exploration target.
        # Otherwise the planner could send the robot straight back into the same pocket.
        near = (
            self.target is None
            or math.dist(pose[:2], self.target.point) < c.target_reached
        )
        if self.target and self.target.mode == 'BACKTRACK' and not near and not blocked:
            return Plan(self.target, mapper.transform, now, grid.revision)

        # 4. If the goal is nearby and its whole approach is observed, aim there directly.
        if distance <= c.target_distance:
            goal_heading = normalize_angle(
                math.atan2(c.goal_y-pose[1], c.goal_x-pose[0])-pose[2]
            )
            if observed_travel(local_scan, goal_heading, c.radius, distance+.15) >= distance:
                self.target = Target((c.goal_x, c.goal_y), 'FINAL')
                return Plan(self.target, mapper.transform, now, grid.revision)

        # 5. Recognize getting stuck: little movement, a loop, or an observed dead end.
        # Progress means displacement from an anchor, not necessarily toward the goal.
        if math.dist(pose[:2], self.anchor) >= c.progress_distance:
            self.anchor, self.progress_at = pose[:2], now

        # Ignore very recent nearby samples: normal slow travel is not a revisit.
        self.recent.append((now, *pose[:2]))
        self.recent = [entry for entry in self.recent if now-entry[0] <= c.revisit_window]
        revisited = any(
            now-entry[0] >= c.revisit_min_age
            and math.dist(pose[:2], entry[1:]) < c.revisit_radius
            for entry in self.recent
        )
        goal_angle = math.atan2(c.goal_y-pose[1], c.goal_x-pose[0])
        capped = grid.closed_ahead(pose[:2], goal_angle)
        stalled = now-self.progress_at >= c.stall_timeout or revisited or capped

        # 6. Remember the failed direction and seek a verified path to an earlier point.
        if stalled:
            direction = (
                math.atan2(self.target.point[1]-pose[1], self.target.point[0]-pose[0])
                if self.target else pose[2]
            )
            self.memory.fail(pose[:2], direction, now)
            self.progress_at = now
            previous = self.memory.return_target(pose[:2], grid)

            if previous is not None:
                angle = normalize_angle(
                    math.atan2(previous[1]-pose[1], previous[0]-pose[0])-pose[2]
                )
                # Back out if the return point is far behind; control checks rear clearance.
                self.target = Target(previous, 'BACKTRACK', reverse=abs(angle) > 2.2)

                # Discourage choosing the same goalward direction upon returning there.
                self.memory.fail(previous, goal_angle, now)
                self.recent.clear()
                self.last_decision = now
                return Plan(self.target, mapper.transform, now, grid.revision)

        # 7. Keep a useful target between decision times to avoid constant steering changes.
        # Arrival, blockage or stalled progress allows an earlier decision.
        near = (
            self.target is None
            or math.dist(pose[:2], self.target.point) < c.target_reached
        )
        if (
            not blocked and not near and not stalled
            and now-self.last_decision < c.decision_period
        ):
            return Plan(
                self.target, mapper.transform, now, grid.revision,
                diagnostics=self.diagnostics,
            )

        self.last_decision = now
        self.diagnostics = {}

        # 8. During replay, prefer a verified saved-route target while progress is healthy.
        if self.route is not None and not blocked and not stalled:
            target = self.replay_target(pose, local_scan, grid)
            if target is not None:
                self.target = target
                return Plan(
                    target, mapper.transform, now, grid.revision,
                    diagnostics=self.diagnostics,
                )

        # 9. Otherwise, search all angular sections for a useful local direction.
        goal_heading = normalize_angle(
            math.atan2(c.goal_y-pose[1], c.goal_x-pose[0])-pose[2]
        )
        options = []
        for lower, upper in self.sections():
            headings = list(np.linspace(lower, upper, c.heading_samples))

            # Regular samples could miss a clear direction exactly toward the goal.
            if lower <= goal_heading <= upper:
                headings.append(goal_heading)

            # Each candidate keeps (Target with score, room, heading) together.
            # None means that heading cannot provide a useful observed approach.
            candidates = [
                self.evaluate(normalize_angle(h), local_scan, pose, state, grid, now)
                for h in headings
            ]
            candidates = [candidate for candidate in candidates if candidate is not None]
            if candidates:
                best = max(candidates, key=lambda item: item[0].score)

                # Nearby room can still lead into a mapped dead end. Keep this
                # section's winner only if the pocket test does not establish a cap.
                if not grid.closed_ahead(pose[:2], pose[2]+best[2]):
                    options.append(best)

        # With no surviving section, try retreating; no return point means no target.
        if not options:
            previous = self.memory.return_target(pose[:2], grid)
            angle = (
                normalize_angle(math.atan2(previous[1]-pose[1], previous[0]-pose[0])-pose[2])
                if previous else 0.
            )
            self.target = (
                Target(previous, 'BACKTRACK', reverse=abs(angle) > 2.2)
                if previous else None
            )
            return Plan(self.target, mapper.transform, now, grid.revision, 'NO_LOCAL_ROUTE')

        # Select the entire winning candidate, so its score, room and heading stay paired.
        best, room, heading = max(options, key=lambda item: item[0].score)

        # 10. A small score advantage should not make the robot alternate between targets.
        if self.target and not near and not blocked and not stalled:
            old_heading = normalize_angle(
                math.atan2(self.target.point[1]-pose[1], self.target.point[0]-pose[0])-pose[2]
            )
            old = self.evaluate(old_heading, local_scan, pose, state, grid, now)

            # Keep the old target if it still has enough room, is not capped,
            # and the new score fails to improve by switch_margin.
            # old[0] is its freshly evaluated Target; old[1] is available room.
            if (
                old and old[1] >= math.dist(pose[:2], self.target.point)+c.target_reserve
                and not grid.closed_ahead(pose[:2], pose[2]+old_heading)
                and best.score < old[0].score+c.switch_margin
            ):
                return Plan(self.target, mapper.transform, now, grid.revision)

        # 11. Tell control whether to brake at the goal or steer along a nearby wall.
        if (
            distance <= c.target_distance
            and room >= distance+c.target_reserve
            and abs(normalize_angle(heading-goal_heading)) < .04
        ):
            best = Target((c.goal_x, c.goal_y), 'FINAL', best.score)
        elif room < c.wall_trigger:
            best.mode = 'FOLLOW_WALL'
            best.wall_side = -1 if heading >= 0 else 1

        # The plan carries this target's map transform so later corrections cannot
        # make control interpret it using a different coordinate relationship.
        self.target = best
        self.diagnostics.update(
            candidate_count=len(options), score=best.score, available_travel=room,
        )
        return Plan(
            best, mapper.transform, now, grid.revision,
            diagnostics=self.diagnostics,
        )
