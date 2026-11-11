"""Always pointing at the goal can trap the robot behind walls, while
changing direction at every scan can cause oscillation.

Balance observed travel and goal progress with turning and failure history. Hold
useful targets, recover when stalled, and follow a verified saved route when
available.
"""

import math
import time
from dataclasses import replace
import numpy as np

from py_amr_ttb.odom_utils import normalize_angle
from .geometry import (
    compose, transform_points, observed_travel, scan_at_pose, circles_observed, free_distances,
)
from .motion import MotionController
from .replay import speed_profile
from .recovery import escape_sequence
from .types import Target, Plan


class Planner:
    """Exploration, replay and backtracking need one
    decision owner so they do not issue competing targets.

    Targets use map coordinates; candidate clearance uses the current robot frame.
    Motion control independently checks commands against current sensing.
    """

    def __init__(self, config, memory, replay_route=None):
        self.cfg, self.memory = config, memory
        self.target = None
        self.started = None
        self.anchor = None
        self.progress_at = None
        self.recent = []
        self.route = replay_route
        self.route_index = 0
        self.speeds = speed_profile(replay_route, config) if replay_route is not None else None
        self.diagnostics = {}
        self.search_section = 0
        self.empty_search_sections = 0
        self.target_committed_until = -math.inf
        self.blocked_time = 0.
        self.feedback_at = None
        self.recovery_stalled = False
        self.wall_side = 0
        self.wall_start = None
        self.wall_clear_scan = None
        self.escape_targets = []
        self.escape_retry_at = -math.inf

    def commit(self, target, state, pose, now):
        if target is not None and (self.target is None or target.key != self.target.key):
            heading = math.atan2(target.point[1]-pose[1], target.point[0]-pose[0])-pose[2]
            if target.reverse:
                heading -= math.pi
            turn_time = abs(normalize_angle(heading))/max(.1, self.cfg.max_angular/2)
            self.target_committed_until = now+max(self.cfg.stall_timeout, turn_time+1.)
            self.anchor, self.progress_at = state.pose[:2], now
            self.blocked_time = 0.
            self.recovery_stalled = False
        self.target = target

    def sections(self):
        """An escape from a blocked forward path may lie beside or
        behind the robot.

        Return angular intervals covering front and rear. Rear bounds can exceed pi;
        candidate angles are normalized before evaluation.
        """
        front = np.linspace(-math.pi/2, math.pi/2, self.cfg.front_sections+1)
        rear = np.linspace(math.pi/2, 3*math.pi/2, self.cfg.rear_sections+1)
        return list(zip(front[:-1], front[1:]))+list(zip(rear[:-1], rear[1:]))

    def evaluate(self, heading, scan, pose, state, grid, now, *, approach=False):
        """A direction toward the goal is useless if the robot cannot
        fit along it or it repeats a recent failed attempt.

        For a robot-relative heading and scan, require enough observed travel, then
        rank a map target by progress, room, clearance, turning and failure history.
        Return (Target, available travel, heading), or None. The score ranks
        choices; it is not a probability.
        """
        c = self.cfg
        if approach:
            room = observed_travel(scan, heading, c.radius, .35, step=.025)
            reserve, minimum_travel = .02, .08
        else:
            room = observed_travel(scan, heading, c.radius, c.scoring_horizon,
                                   step=max(.05, c.grid_resolution))
            reserve = c.target_reserve
            minimum_travel = max(c.target_min_distance, c.target_reached+.05)
        travel = min(c.target_distance, room-reserve)
        self.diagnostics['headings_checked'] = self.diagnostics.get('headings_checked', 0) + 1
        self.diagnostics['max_observed_travel'] = max(
            self.diagnostics.get('max_observed_travel', 0.), float(room))
        requirement = 'approach_required_travel' if approach else 'required_travel'
        self.diagnostics[requirement] = minimum_travel+reserve
        if travel < minimum_travel:
            self.diagnostics['short_headings'] = self.diagnostics.get('short_headings', 0) + 1
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
        mode = 'APPROACH' if approach else 'FIRST_RUN'
        return Target(tuple(point), mode, score=float(score)), room, heading

    def observed_segment(self, pose, scan, point):
        dx, dy = point[0]-pose[0], point[1]-pose[1]
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)-pose[2]
        return observed_travel(scan, heading, self.cfg.radius, distance+.051) >= distance

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
            if grid is None:
                clear = self.observed_segment(pose, scan, self.route[i])
            else:
                clear = grid.segment_clear(pose[:2], self.route[i], margin=self.cfg.route_margin)
            if not clear:
                break
            selected = i
        if selected is None:
            # Legacy replay can rejoin using the normal footprint.
            if grid is None or not grid.segment_clear(pose[:2], self.route[self.route_index]):
                return None
            selected = self.route_index
        self.route_index = max(self.route_index, selected)
        point = tuple(self.route[selected])
        # The speed cap anticipates bends. The controller checks live clearance.
        self.diagnostics['speed_cap'] = float(self.speeds[max(0, selected-1)])
        return Target(point, 'FINAL' if selected == len(self.route)-1 else 'SECOND_RUN')

    def opening_approach(self, state, scan, local_scan, pose, now, reference):
        """Opposing LiDAR returns can bound a narrow opening.

        Return a centered odometry target and diagnostics if the initial
        forward command clears the stopping sweep.
        """
        c = self.cfg
        points = local_scan.points
        for distance in (.45, .65, .85):
            band = points[abs(points[:, 0]-distance) <= .10, 1]
            left = band[band > 0.]
            right = band[band < 0.]
            if not len(left) or not len(right):
                continue
            left, right = left.min(), right.max()
            width = left-right
            if not 2*c.radius <= width <= 2*c.radius+.35:
                continue
            center = (left+right)/2
            heading = math.atan2(center, distance)
            if abs(heading) > math.pi/4 or abs(heading-reference) < math.radians(4):
                continue
            room = observed_travel(local_scan, heading, c.radius, distance+.15, step=.025)
            travel = min(math.hypot(distance, center), room-.02)
            if travel < .25:
                continue
            point = transform_points([(travel*math.cos(heading), travel*math.sin(heading))], pose)[0]
            target = Target(tuple(point), 'FORWARD_CORRECTION')
            command = MotionController(c).step(state, scan, target, now, c.control_period)
            if command.v > 0.:
                return target, dict(opening_width=float(width), opening_heading=heading,
                                    arc_v=command.v, arc_w=command.w)
        return None

    def forward_correction(self, state, scan, local_scan, pose, now):
        """A blocked line can still leave room for a small forward turn.

        Check headings in order of increasing turn angle. Return the first
        target with observed travel and a clear forward stopping sweep.
        """
        c = self.cfg
        controller = MotionController(c)
        for degrees in (0, 5, -5, 10, -10, 15, -15):
            heading = math.radians(degrees)
            room = observed_travel(local_scan, heading, c.radius, .40, step=.025)
            if room < .25:
                continue
            travel = min(.35, room)
            local = (travel*math.cos(heading), travel*math.sin(heading))
            target = Target(tuple(transform_points([local], pose)[0]), 'FORWARD_CORRECTION')
            command = controller.step(state, scan, target, now, c.control_period)
            if command.v > 0.:
                return target
        return None

    def follow_wall(self, scan, pose, state, source_scan, now):
        """A consistent obstacle side keeps recovery moving along the wall.

        Rank forward segments by usable travel and steering effort. Return
        the highest-scoring target with a clear initial stopping sweep.
        """
        c = self.cfg
        best = None
        for side in (self.wall_side,) if self.wall_side else (-1, 1):
            points = scan.points[(scan.points[:, 0] >= -.1)
                                 & (side*scan.points[:, 1] >= -.025)]
            if not len(points):
                continue
            hit = points[np.argmin(np.linalg.norm(points, axis=1))]
            tangent = normalize_angle(math.atan2(hit[1], hit[0])-side*math.pi/2)
            for heading in np.radians((0, 10, -10, 20, -20, 30, -30, 40, -40)):
                room = observed_travel(scan, heading, c.radius, c.target_distance+.1)
                travel = min(c.target_distance, room-.02)
                if travel < .15:
                    continue
                score = min(travel, .35)-.20*abs(normalize_angle(heading-tangent))-.45*abs(heading)
                if best is None or score > best.score:
                    local = (travel*math.cos(heading), travel*math.sin(heading))
                    point = tuple(transform_points([local], pose)[0])
                    target = Target(point, 'FOLLOW_WALL', score=float(score), wall_side=side)
                    command = MotionController(c).step(state, source_scan, target, now, c.control_period)
                    if command.v > 0.:
                        best = target
        return best

    def step(self, state, scan, mapper, now, blocked=False, motion_reason='', feedback_target=None):
        """A useful target can become blocked or lead back into a failed area.

        Keep it, replace it, or request recovery using the checks below in order.
        Each return ends this decision. The returned Plan carries the target's
        map transform; motion control checks velocity commands separately.
        """
        # Targets use map coordinates; direction checks use the robot's frame now.
        c = self.cfg
        grid = mapper.grid if mapper is not None else None
        transform = mapper.transform if mapper is not None else (0., 0., 0.)
        revision = grid.revision if grid is not None else 0
        pose = compose(transform, state.pose)
        scan = replace(scan, coverage_time=now, coverage_timeout=c.sensor_timeout,
                       history=tuple(previous for previous in scan.history
                       if 0 <= now-previous.received <= c.sensor_timeout))
        local_scan = scan_at_pose(scan, state.pose)
        def segment_clear(start, point):
            if grid is not None:
                return grid.segment_clear(start, point)
            return self.observed_segment(pose, local_scan, point)

        self.memory.record(pose, now)
        if self.started is None:
            self.started = self.progress_at = now
            self.anchor = state.pose[:2]

        # Finish once the goal is reached, or stop an overlong run.
        distance = math.hypot(c.goal_x-pose[0], c.goal_y-pose[1])
        if self.memory.complete or distance <= c.goal_tolerance:
            self.memory.complete = True
            self.target = Target((c.goal_x, c.goal_y), 'DONE')
            return Plan(self.target, transform, now, revision, 'DONE')

        if now-self.started > c.max_runtime:
            return Plan(None, transform, now, revision, 'TIME_LIMIT')

        # Complete an unblocked retreat before choosing another exploration target.
        # Otherwise the planner could send the robot straight back into the same pocket.
        near = (
            self.target is None
            or math.dist(pose[:2], self.target.point) < self.target.arrival_distance(c.target_reached)
        )
        elapsed = 0. if self.feedback_at is None else max(0., now-self.feedback_at)
        self.feedback_at = now
        progress_distance = c.progress_distance
        if self.target and self.target.reverse:
            # A reverse recovery has less travel available within the same
            # stall interval, especially while its initial motion settles.
            progress_distance = min(progress_distance, c.reverse_speed*c.stall_timeout/4)
        if math.dist(state.pose[:2], self.anchor) >= progress_distance:
            self.anchor, self.progress_at = state.pose[:2], now
            self.blocked_time = 0.
        if (motion_reason == 'SENSOR_STOP' or
                feedback_target is not None and (self.target is None or feedback_target != self.target.key)):
            blocked = False
        if self.target and blocked:
            self.blocked_time += min(elapsed, c.map_period)
        sustained_block = self.blocked_time >= max(c.decision_period, 2*c.map_period)
        committed = now < self.target_committed_until
        no_progress = not committed and now-self.progress_at >= c.stall_timeout
        # Finish each escape leg before goal seeking can undo the retreat.
        if self.escape_targets:
            if sustained_block or no_progress:
                self.escape_targets.clear()
                self.commit(None, state, pose, now)
                self.escape_retry_at = now+c.stall_timeout
                self.diagnostics = {'recovery_stage': 'BACKTRACK',
                                    'recovery_transition': 'ESCAPE_BLOCKED'}
            elif near:
                self.escape_targets.pop(0)
                if self.escape_targets:
                    self.commit(self.escape_targets[0], state, pose, now)
                else:
                    self.commit(None, state, pose, now)
                    self.wall_start = None
                    self.wall_side = 0
                    self.wall_clear_scan = None
                    self.diagnostics = {'recovery_stage': 'NORMAL',
                                        'recovery_transition': 'ESCAPE_COMPLETED'}
            if self.escape_targets:
                return Plan(self.target, transform, now, revision,
                            diagnostics={'recovery_stage': self.target.mode,
                                         'remaining_legs': len(self.escape_targets)})

        failed_backtrack = (self.target is not None and self.target.mode == 'BACKTRACK' and not near
                            and (sustained_block or no_progress))
        if self.target and self.target.mode == 'BACKTRACK' and not near and not failed_backtrack:
            return Plan(self.target, transform, now, revision)

        if grid is None and self.route is None and self.wall_start is None and self.target:
            correcting = self.target.mode == 'FORWARD_CORRECTION'
            if correcting and not near and not sustained_block and not no_progress:
                return Plan(self.target, transform, now, revision, diagnostics=self.diagnostics)
            if not correcting and not near:
                dx, dy = self.target.point[0]-pose[0], self.target.point[1]-pose[1]
                heading = normalize_angle(math.atan2(dy, dx)-pose[2])
                opening = self.opening_approach(state, scan, local_scan, pose, now, heading)
                if opening is not None:
                    target, diagnostics = opening
                    self.commit(target, state, pose, now)
                    self.diagnostics = diagnostics
                    return Plan(target, transform, now, revision, diagnostics=diagnostics)
                threatened = (abs(heading) <= math.radians(15)
                              and observed_travel(local_scan, heading, c.radius, .30) < .20)
                if threatened or sustained_block or no_progress:
                    correction = self.forward_correction(state, scan, local_scan, pose, now)
                    if correction is not None:
                        self.commit(correction, state, pose, now)
                        self.diagnostics = {'recovery': 'FORWARD_CORRECTION'}
                        return Plan(correction, transform, now, revision, diagnostics=self.diagnostics)

        if grid is None and (self.wall_start is not None or sustained_block or no_progress):
            if self.wall_start is None:
                self.wall_start = pose[:2]
            goal = (c.goal_x, c.goal_y)
            goal_clear = ((not self.wall_side or math.dist(pose[:2], self.wall_start) >= .25)
                          and self.observed_segment(pose, local_scan, goal))
            if goal_clear and self.wall_clear_scan is None:
                self.wall_clear_scan = scan.received
            if not goal_clear:
                self.wall_clear_scan = None
            if goal_clear and scan.received > self.wall_clear_scan:
                self.wall_side = 0
                self.wall_start = None
                self.wall_clear_scan = None
                self.commit(None, state, pose, now)
                sustained_block = no_progress = False
                near = True
            else:
                if (self.target is not None and self.target.mode in ('FOLLOW_WALL', 'FORWARD_CORRECTION')
                        and not near and not sustained_block and not no_progress):
                    target = self.target
                else:
                    opening = self.opening_approach(state, scan, local_scan, pose, now, 0.)
                    if opening is not None:
                        target, diagnostics = opening
                        self.commit(target, state, pose, now)
                        self.diagnostics = diagnostics
                        return Plan(target, transform, now, revision, diagnostics=diagnostics)
                    target = self.follow_wall(local_scan, pose, state, scan, now)
                if target is not None and not self.wall_side:
                    self.wall_side = target.wall_side
                self.commit(target, state, pose, now)
                if target is not None:
                    return Plan(target, transform, now, revision,
                                diagnostics={'wall_side': self.wall_side})
                # Exhausted forward candidates must reach shared recovery below.
                sustained_block = True
                near = True

        # If the goal is nearby and its whole approach is observed, aim there directly.
        if distance <= c.target_distance and not sustained_block and not no_progress:
            goal_heading = normalize_angle(
                math.atan2(c.goal_y-pose[1], c.goal_x-pose[0])-pose[2]
            )
            if ((grid is not None or self.route is not None or abs(goal_heading) <= math.pi/2)
                    and observed_travel(local_scan, goal_heading, c.radius, distance+.15) >= distance):
                self.commit(Target((c.goal_x, c.goal_y), 'FINAL'), state, pose, now)
                return Plan(self.target, transform, now, revision)

        # Keep a selected waypoint through alignment and transient feedback.
        # Motion still validates every command against current sensor coverage.
        if self.target is not None and not near and committed and not sustained_block:
            return Plan(self.target, transform, now, revision,
                        diagnostics=dict(self.diagnostics, holding_for='TARGET_COMMITMENT',
                                         commitment_remaining=self.target_committed_until-now))

        # Recognize getting stuck: little movement, a loop, or an observed dead end.
        # Progress means displacement from an anchor, not necessarily toward the goal.
        # Ignore very recent nearby samples: normal slow travel is not a revisit.
        self.recent.append((now, *pose[:2]))
        self.recent = [entry for entry in self.recent if now-entry[0] <= c.revisit_window]
        revisited = any(
            now-entry[0] >= c.revisit_min_age
            and math.dist(pose[:2], entry[1:]) < c.revisit_radius
            for entry in self.recent
        )
        goal_angle = math.atan2(c.goal_y-pose[1], c.goal_x-pose[0])
        active_heading = goal_angle if near else math.atan2(
            self.target.point[1]-pose[1], self.target.point[0]-pose[0])
        capped = grid is not None and grid.closed_ahead(pose[:2], active_heading)
        stalled = no_progress or sustained_block or (near and revisited) or capped
        if motion_reason in ('WAITING_FOR_FRESH_PLAN', 'WAITING_FOR_SENSORS',
                             'CONTROL_DEADLINE', 'COMMAND_EXPIRED', 'SEARCHING_ROUTE'):
            stalled = capped or sustained_block or failed_backtrack
        # Remember the failed direction and seek a verified path to an earlier point.
        if stalled:
            direction = (
                math.atan2(self.target.point[1]-pose[1], self.target.point[0]-pose[0])
                if self.target else pose[2]
            )
            self.memory.fail(pose[:2], direction, now)
            self.progress_at = now
            if failed_backtrack:
                self.target = None
                self.recovery_stalled = True
            previous = None if self.recovery_stalled else self.memory.return_target(pose[:2], segment_clear)

            if previous is not None:
                angle = normalize_angle(
                    math.atan2(previous[1]-pose[1], previous[0]-pose[0])-pose[2]
                )
                # Back out if the return point is far behind; control checks rear clearance.
                self.commit(Target(previous, 'BACKTRACK', reverse=abs(angle) > 2.2), state, pose, now)

                # Discourage choosing the same goalward direction upon returning there.
                self.memory.fail(previous, goal_angle, now)
                self.recent.clear()
                return Plan(self.target, transform, now, revision)

        if stalled and not self.recovery_stalled and now >= self.escape_retry_at:
            self.escape_retry_at = now+c.decision_period
            self.escape_targets = escape_sequence(state, scan, c, now, transform)
            if self.escape_targets:
                self.commit(self.escape_targets[0], state, pose, now)
                return Plan(self.target, transform, now, revision,
                            diagnostics={'recovery_stage': 'RECOVERY_REVERSE',
                                         'remaining_legs': len(self.escape_targets)})
            self.diagnostics['recovery_stage'] = 'WAIT_FOR_EVIDENCE'

        # Keep a useful target between decision times to avoid constant steering changes.
        # Arrival, blockage or stalled progress allows an earlier decision.
        near = (
            self.target is None
            or math.dist(pose[:2], self.target.point) < self.target.arrival_distance(c.target_reached)
        )
        if not near and not stalled:
            return Plan(
                self.target, transform, now, revision,
                diagnostics=self.diagnostics,
            )

        self.diagnostics = {}

        # During replay, prefer a verified saved-route target while progress is healthy.
        if self.route is not None and not sustained_block and not stalled:
            target = self.replay_target(pose, local_scan, grid)
            if target is not None:
                self.commit(target, state, pose, now)
                return Plan(
                    target, transform, now, revision,
                    diagnostics=self.diagnostics,
                )

        # Otherwise, search all angular sections for a useful local direction.
        goal_heading = normalize_angle(
            math.atan2(c.goal_y-pose[1], c.goal_x-pose[0])-pose[2]
        )
        options = []
        required_travel = max(c.target_min_distance, c.target_reached+.05)+c.target_reserve
        sections = self.sections()
        if grid is None and self.route is None:
            sections = sections[:c.front_sections]
        start_section = self.search_section
        search_started = time.perf_counter()
        searched = 0
        for offset in range(len(sections)):
            section_index = (start_section+offset) % len(sections)
            lower, upper = sections[section_index]
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
            if not candidates:
                # Screen intermediate bearings against hits first. Only the
                # best one needs a full observed-footprint evaluation.
                fine_count = math.ceil((upper-lower)/math.radians(.5))+1
                fine_headings = np.linspace(lower, upper, fine_count)
                room = free_distances(local_scan.points, fine_headings, c.radius, c.scoring_horizon)
                best_index = int(np.argmax(room))
                if room[best_index] >= required_travel:
                    candidate = self.evaluate(normalize_angle(fine_headings[best_index]),
                                              local_scan, pose, state, grid, now)
                    if candidate is not None:
                        candidates.append(candidate)
            if candidates:
                best = max(candidates, key=lambda item: item[0].score)

                # Nearby room can still lead into a mapped dead end. Keep this
                # section's winner only if the pocket test does not establish a cap.
                direction = pose[2]+best[2]
                if grid is None or not (grid.closed_ahead(pose[:2], direction)
                        or grid.closed_ahead(best[0].point, direction)):
                    options.append(best)
                else:
                    self.diagnostics['closed_sections'] = self.diagnostics.get('closed_sections', 0) + 1
            searched += 1
            self.search_section = (section_index+1) % len(sections)
            if time.perf_counter()-search_started >= c.map_period/2:
                break

        self.diagnostics['sections_searched'] = searched
        self.empty_search_sections = self.empty_search_sections+searched if not options else 0
        if not options and self.empty_search_sections < len(sections):
            return Plan(None, transform, now, revision, 'SEARCHING_ROUTE',
                        diagnostics=dict(self.diagnostics))
        if not options:
            self.empty_search_sections = 0

        # With no surviving section, try retreating; no return point means no target.
        if not options:
            start_details = {}
            circles_observed(local_scan, [[0., 0.]], c.radius,
                             occupied_radius=c.radius, diagnostics=start_details)
            probe_angles = np.linspace(-math.pi, math.pi, 72, endpoint=False)
            probe_distance = max(.05, c.grid_resolution)
            probe_centers = probe_distance * np.column_stack(
                (np.cos(probe_angles), np.sin(probe_angles)))
            step_details = {}
            clear_steps = circles_observed(local_scan, probe_centers, c.radius,
                                          occupied_radius=c.radius, diagnostics=step_details)
            self.diagnostics.update(start_footprint=start_details,
                                    first_step=step_details,
                                    first_step_distance=probe_distance,
                                    clear_first_steps=int(np.count_nonzero(clear_steps)))
            previous = None if grid is None or self.recovery_stalled else self.memory.return_target(pose[:2], segment_clear)
            if previous is None and not self.recovery_stalled and not sustained_block:
                approaches = []
                headings = np.unique(np.append(np.linspace(-math.pi/4, math.pi/4, 5),
                                               np.clip(goal_heading, -math.pi/4, math.pi/4)))
                for heading in headings:
                    candidate = self.evaluate(heading, local_scan, pose, state, grid, now, approach=True)
                    if candidate is None:
                        continue
                    target, _, _ = candidate
                    direction = pose[2]+heading
                    if grid is None or not (grid.closed_ahead(pose[:2], direction)
                            or grid.closed_ahead(target.point, direction)):
                        approaches.append(target)
                if approaches:
                    self.commit(max(approaches, key=lambda target: target.score), state, pose, now)
                    return Plan(self.target, transform, now, revision,
                                diagnostics=dict(self.diagnostics))
            if previous is None and not self.recovery_stalled and now >= self.escape_retry_at:
                self.escape_retry_at = now+c.decision_period
                self.escape_targets = escape_sequence(state, scan, c, now, transform)
                if self.escape_targets:
                    self.commit(self.escape_targets[0], state, pose, now)
                    return Plan(self.target, transform, now, revision,
                                diagnostics={'recovery_stage': 'RECOVERY_REVERSE',
                                             'remaining_legs': len(self.escape_targets)})
            angle = (
                normalize_angle(math.atan2(previous[1]-pose[1], previous[0]-pose[0])-pose[2])
                if previous else 0.
            )
            target = (
                Target(previous, 'BACKTRACK', reverse=abs(angle) > 2.2)
                if previous else None
            )
            self.commit(target, state, pose, now)
            status = 'RECOVERY_STALLED' if self.recovery_stalled else 'NO_LOCAL_ROUTE'
            return Plan(self.target, transform, now, revision, status,
                        diagnostics=dict(self.diagnostics))

        # Select the entire winning candidate, so its score, room and heading stay paired.
        best, room, heading = max(options, key=lambda item: item[0].score)

        # Identify the final goal for arrival braking.
        if (
            distance <= c.target_distance
            and room >= distance+c.target_reserve
            and abs(normalize_angle(heading-goal_heading)) < .04
        ):
            best = Target((c.goal_x, c.goal_y), 'FINAL', best.score)
        # A short route still uses the heading whose footprint was checked.
        # Replacing it with a wall tangent would invalidate that route selection.

        # The plan carries this target's map transform so later corrections cannot
        # make control interpret it using a different coordinate relationship.
        self.commit(best, state, pose, now)
        self.diagnostics.update(
            candidate_count=len(options), score=best.score, available_travel=room,
        )
        return Plan(
            best, transform, now, revision,
            diagnostics=self.diagnostics,
        )
