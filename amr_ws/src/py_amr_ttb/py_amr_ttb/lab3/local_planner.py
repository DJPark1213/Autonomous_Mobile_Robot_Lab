"""Rank executable motions, then validate their complete stopping sweeps."""

import heapq
import math
from dataclasses import replace

import numpy as np

from py_amr_ttb.odom_utils import normalize_angle
from .trajectory import rollout, evaluate_trajectory
from .geometry import inverse_points, transform_points


class TrajectoryPlanner:
    """Use a scan-local guide to rank motions, then verify stopping clearance."""
    def __init__(self, config):
        self.cfg = config
        self.previous = (0., 0.)
        self.guide_point = None
        self.guide_objective = None

    def guide(self, state, scan, target):
        """Search a bounded, disposable lattice through the current obstacle returns.

        Search eight-connected cells without diagonal corner cutting and select
        a directly clear lookahead point. Retain a clear previous guide across
        large direction changes while the objective remains nearby. Unknown cells
        can guide exploration; only observed stopping sweeps authorize movement.
        """
        spacing = self.cfg.local_spacing
        extent = self.cfg.local_extent
        cells = math.ceil(extent/spacing)
        axis = np.arange(-cells, cells+1)*spacing
        xx, yy = np.meshgrid(axis, axis)
        origin = np.round(np.asarray(state.pose[:2])/spacing)*spacing
        centers = np.column_stack((xx.ravel(), yy.ravel()))+origin
        hits = transform_points(scan.points, scan.pose)
        free = np.ones(len(centers), dtype=bool)
        n = len(axis)
        if len(hits):
            # Only nearby cells can intersect a hit's inflation disk. Visit those
            # cells instead of comparing every grid cell with every laser return.
            radius = self.cfg.radius+spacing*.15
            reach = math.ceil(radius/spacing)
            offsets = np.arange(-reach, reach+2)
            ox, oy = np.meshgrid(offsets, offsets)
            offsets = np.column_stack((ox.ravel(), oy.ravel()))
            base = np.floor((hits-centers[0])/spacing).astype(int)
            indices = base[:, None]+offsets[None]
            inside = np.all((indices >= 0) & (indices < n), axis=2)
            flat = indices[..., 1]*n+indices[..., 0]
            points = centers[np.clip(flat, 0, len(centers)-1)]
            occupied = inside & (np.sum((points-hits[:, None])**2, axis=2) < radius**2)
            free[flat[occupied]] = False
        start = (n//2)*n+n//2
        free[start] = True
        goal_dist = np.linalg.norm(centers-np.asarray(target), axis=1)
        destination = int(np.argmin(np.where(free, goal_dist, np.inf)))
        frontier = [(goal_dist[start], 0., start)]
        costs = {start: 0.}
        parent = {}
        best = start
        while frontier:
            _, cost, node = heapq.heappop(frontier)
            if cost > costs[node]+1e-9:
                continue
            if goal_dist[node] < goal_dist[best]:
                best = node
            if node == destination:
                best = node
                break
            row, col = divmod(node, n)
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)):
                nr, nc = row+dr, col+dc
                if not 0 <= nr < n or not 0 <= nc < n:
                    continue
                neighbor = nr*n+nc
                if not free[neighbor]:
                    continue
                if dr and dc and (not free[row*n+nc] or not free[nr*n+col]):
                    continue
                new = cost+spacing*math.hypot(dr, dc)
                if new < costs.get(neighbor, math.inf):
                    costs[neighbor] = new
                    parent[neighbor] = node
                    heapq.heappush(frontier, (new+goal_dist[neighbor], new, neighbor))
        path = [best]
        while path[-1] in parent:
            path.append(parent[path[-1]])
        path = centers[path[::-1]]
        if len(path) < 2:
            return target, path.tolist()
        # Use the furthest directly clear point within the local lookahead.
        selected = path[1]
        for point in path[1:]:
            distance = math.dist(state.pose[:2], point)
            if distance > self.cfg.local_lookahead:
                break
            probes = np.linspace(state.pose[:2], point, max(2, int(distance/.025)+1))
            if len(hits) and np.any(np.linalg.norm(probes[:, None]-hits[None], axis=2) < self.cfg.radius+.005):
                break
            selected = point
        if (math.dist(state.pose[:2], target) < self.cfg.local_lookahead
                and best == destination and goal_dist[best] < spacing):
            probes = np.linspace(state.pose[:2], target, 30)
            if not len(hits) or np.all(np.linalg.norm(probes[:, None]-hits[None], axis=2) >= self.cfg.radius+.005):
                selected = target
        # Retain a still-clear passing direction only across a large route flip.
        # Let the lookahead advance normally along the same side of an obstacle.
        if (self.guide_point is not None and self.guide_objective is not None
                and math.dist(target, self.guide_objective) < .1
                and math.dist(state.pose[:2], self.guide_point) > .2):
            old = np.asarray(self.guide_point)-state.pose[:2]
            new = np.asarray(selected)-state.pose[:2]
            if np.dot(old, new) < .5*np.linalg.norm(old)*np.linalg.norm(new):
                from .trajectory import capsule_clearance
                segment = np.asarray([state.pose[:2], self.guide_point])
                if not len(hits) or capsule_clearance(segment, hits).min() > self.cfg.radius+.005:
                    selected = self.guide_point
        self.guide_point, self.guide_objective = tuple(selected), tuple(target)
        return tuple(selected), path.tolist()

    def choose(self, state, scan, target, now, allow_reverse=True, allow_rotation=True):
        """Rank progress, heading improvement, speed and steering continuity.

        Check candidates in score order with the shared stopping validator.
        Exhausted rotation budget excludes stationary and small-displacement
        turns and prioritizes verified translation over holding still.
        """
        c = self.cfg
        objective_distance = math.dist(state.pose[:2], target)
        target, guide = self.guide(state, scan, target)
        prediction = replace(c, prediction_horizon=c.trajectory_horizon)
        distance = math.dist(state.pose[:2], target)
        bearing = math.atan2(target[1]-state.pose[1], target[0]-state.pose[0])
        heading = normalize_angle(bearing-state.pose[2])
        top = min(c.max_linear, math.sqrt(2*c.linear_braking*max(0., objective_distance-.08)))
        # Raising a speed cap must not remove the fine choices needed in tight turns.
        angular_steps = math.floor(c.max_angular/.35)
        steering = np.unique(np.append(np.arange(-angular_steps, angular_steps+1)*.35,
                                        [-c.max_angular, c.max_angular,
                                         np.clip(c.heading_gain*heading, -c.max_angular, c.max_angular),
                                         self.previous[1]]))
        candidates = []
        speeds = np.unique(np.append(np.arange(0., top, .05),
                                     [top, -c.reverse_speed] if allow_reverse else [top]))
        for v in speeds:
            for w in steering:
                if (not allow_rotation and abs(v) < .005 and abs(w) > .01) or abs(v*w) > c.lateral_acceleration:
                    continue
                path = rollout(state, v, w, prediction, max(0., now-scan.received))
                end = path[-1]
                if not allow_rotation and abs(w) > .01 and math.dist(end[:2], state.pose[:2]) < .05:
                    continue
                progress = distance-math.dist(end[:2], target)
                # A nearby guide point can fall behind a moving candidate.
                # Compare all candidates with the same approach direction so
                # passing that point does not reward turning back toward it.
                alignment = abs(heading)-abs(normalize_angle(bearing-end[2]))
                score = (progress+c.trajectory_alignment_weight*alignment
                         +c.trajectory_speed_weight*v
                         -c.trajectory_steering_weight*abs(w-self.previous[1]))
                if v < 0:
                    score -= .12
                candidates.append((score, float(v), float(w), path))
        attempts = 0
        rejections = {}
        last_check = {}
        # After exhausting rotation, prefer a verified translating escape to a
        # zero command whose score would otherwise beat every reverse candidate.
        ranked = sorted(candidates, key=lambda item: (
            allow_rotation or abs(item[1]) > .005, item[0]), reverse=True)
        for score, v, w, path in ranked:
            attempts += 1
            last_check = {}
            clear, path = evaluate_trajectory(state, scan, v, w, prediction, now, last_check)
            if not clear:
                reason = last_check.get('rejection', 'SLOW_STEERING')
                rejections[reason] = rejections.get(reason, 0)+1
            if clear:
                self.previous = (v, w)
                return (v, w), dict(score=score, candidates=len(candidates), checked=attempts,
                                    predicted_path=path, guide=guide,
                                    guide_distance=distance, guide_heading=heading,
                                    clearance=last_check, rejections=rejections,
                                    rotation_limited=not allow_rotation)
        self.previous = (0., 0.)
        return (0., 0.), dict(rejection='NO_SAFE_TRAJECTORY', checked=attempts,
                            rejections=rejections, clearance=last_check,
                            rotation_limited=not allow_rotation)
