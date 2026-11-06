"""Bounded occupancy evidence, geometry queries and conservative local alignment."""

from collections import deque
import math
import numpy as np

from .geometry import compose, transform_points, grid_line, between
from .state import PoseEstimator


def dilate(mask, cells):
    """Circular binary expansion without an additional runtime dependency."""
    result = mask.copy()
    height, width = mask.shape
    limit = math.ceil(cells)
    for dy in range(-limit, limit+1):
        for dx in range(-limit, limit+1):
            if dx*dx+dy*dy > cells*cells:
                continue
            y0, y1 = max(0, dy), min(height, height+dy)
            x0, x1 = max(0, dx), min(width, width+dx)
            result[y0:y1, x0:x1] |= mask[y0-dy:y1-dy, x0-dx:x1-dx]
    return result


class OccupancyMap:
    def __init__(self, config):
        self.cfg = config
        self.resolution = config.grid_resolution
        self.origin = np.array([config.grid_origin_x, config.grid_origin_y])
        shape = (math.ceil(config.grid_height/self.resolution),
                 math.ceil(config.grid_width/self.resolution))
        self.evidence = np.zeros(shape, dtype=np.float32)
        self.observed = np.zeros(shape, dtype=bool)
        self.last_seen = np.full(shape, -1., dtype=np.float64)
        self.revision = 0
        self.last_scan = None
        self._layers = None

    def cell(self, point):
        return tuple(np.floor((np.asarray(point)-self.origin)/self.resolution).astype(int))

    def point(self, cell):
        return tuple(self.origin+(np.asarray(cell)+.5)*self.resolution)

    def inside(self, cell):
        x, y = cell
        return 0 <= x < self.evidence.shape[1] and 0 <= y < self.evidence.shape[0]

    def integrate(self, scan, map_pose):
        """Update each cell once per scan, with hit evidence taking precedence."""
        token = (scan.sequence, scan.stamp)
        if token == self.last_scan:
            return False
        self.last_scan = token
        start = self.cell(transform_points([scan.origin], map_pose)[0])
        if not self.inside(start):
            raise ValueError('Robot left configured map bounds')
        endpoints = transform_points(scan.endpoints, map_pose)
        free, hits = set(), set()
        for i in np.flatnonzero(scan.known):
            end = self.cell(endpoints[i])
            for cell in grid_line(start, end):
                if not self.inside(cell):
                    break
                if cell == end and scan.hits[i]:
                    hits.add(cell)
                else:
                    free.add(cell)
        for cells, delta in ((free-hits, -self.cfg.free_increment),
                             (hits, self.cfg.occupied_increment)):
            if cells:
                xy = np.asarray(list(cells))
                index = (xy[:, 1], xy[:, 0])
                self.evidence[index] = np.clip(self.evidence[index]+delta,
                                               -self.cfg.evidence_limit, self.cfg.evidence_limit)
                self.observed[index] = True
                self.last_seen[index] = scan.stamp
        self.revision += 1
        self._layers = None
        return True

    def layers(self):
        if self._layers is None:
            occupied = self.evidence >= self.cfg.occupied_threshold
            # Include half-cell diagonal to protect against grid discretization.
            n = (self.cfg.radius+self.resolution/math.sqrt(2))/self.resolution
            blocked = dilate(occupied, n)
            free = self.observed & (self.evidence <= self.cfg.free_threshold) & ~blocked
            self._layers = blocked, free
        return self._layers

    def segment_clear(self, a, b, require_observed=True, margin=0.):
        """Require the full footprint to be observed for saved-route shortcuts."""
        blocked, free = self.layers()
        cells = list(grid_line(self.cell(a), self.cell(b)))
        r = math.ceil((self.cfg.radius+margin)/self.resolution)
        for x, y in cells:
            if not self.inside((x, y)) or blocked[y, x]:
                return False
            if require_observed:
                if x-r < 0 or y-r < 0 or x+r >= free.shape[1] or y+r >= free.shape[0]:
                    return False
                # Observation of body perimeter is necessary, not just centerline.
                for dy in range(-r, r+1):
                    span = int(math.sqrt(max(0, r*r-dy*dy)))
                    region = self.observed[y+dy, x-span:x+span+1]
                    evidence = self.evidence[y+dy, x-span:x+span+1]
                    if not np.all(region & (evidence <= self.cfg.free_threshold)):
                        return False
        return True

    def closed_ahead(self, position, heading):
        """Evidence-only capped-pocket test with an observed wall-to-wall gate.

        A bounded search, unknown boundary, or missing gate returns False (candidate).
        False does not mean open or guarantee any path onward.
        """
        blocked, free = self.layers()
        forward = np.array([math.cos(heading), math.sin(heading)])
        lateral = np.array([-forward[1], forward[0]])
        entrance = np.asarray(position)+.10*forward
        anchors = []
        for sign in (-1, 1):
            anchor = None
            for d in np.arange(0, 1.51, self.resolution):
                cell = self.cell(entrance+sign*d*lateral)
                if not self.inside(cell):
                    return False
                x, y = cell
                if blocked[y, x]:
                    anchor = cell
                    break
                if not free[y, x]:
                    return False
            if anchor is None:
                return False
            anchors.append(anchor)
        gate = set(grid_line(*anchors))
        seed = self.cell(entrance+.15*forward)
        if not self.inside(seed) or not free[seed[1], seed[0]] or seed in gate:
            return False
        queue, visited = deque([seed]), {seed}
        while queue:
            if len(visited) >= self.cfg.local_search_cells:
                return False
            x, y = queue.popleft()
            for cell in ((x+1, y), (x-1, y), (x, y+1), (x, y-1)):
                if cell in gate or cell in visited:
                    continue
                if not self.inside(cell):
                    return False
                cx, cy = cell
                if blocked[cy, cx]:
                    continue
                if not free[cy, cx]:
                    return False
                visited.add(cell)
                queue.append(cell)
        return len(visited) >= 4

    def align(self, scan, predicted, initial=False):
        """Bounded endpoint matching; reject weak or spatially ambiguous matches."""
        points = scan.points
        occupied = self.evidence >= self.cfg.occupied_threshold
        if len(points) < self.cfg.alignment_min_hits or occupied.sum() < self.cfg.alignment_min_hits:
            return predicted, {'accepted': False, 'reason': 'insufficient_geometry'}
        points = points[::max(1, len(points)//100)]
        close = dilate(occupied, 1)

        def score(pose):
            xy = np.floor((transform_points(points, pose)-self.origin)/self.resolution).astype(int)
            valid = ((xy[:, 0] >= 0) & (xy[:, 1] >= 0)
                     & (xy[:, 0] < occupied.shape[1]) & (xy[:, 1] < occupied.shape[0]))
            xy = xy[valid]
            if not len(xy):
                return 0.
            return float((.7*occupied[xy[:, 1], xy[:, 0]]+.3*close[xy[:, 1], xy[:, 0]]).sum()/len(points))

        span = self.cfg.replay_start_search if initial else self.cfg.alignment_translation
        offsets = np.linspace(-span, span, 7 if initial else 5)
        turns = np.linspace(-self.cfg.alignment_rotation, self.cfg.alignment_rotation, 5)
        candidates = [(score((predicted[0]+dx, predicted[1]+dy, predicted[2]+da)),
                       (predicted[0]+dx, predicted[1]+dy, predicted[2]+da))
                      for dx in offsets for dy in offsets for da in turns]
        best_score, best = max(candidates, key=lambda item: item[0])
        baseline = score(predicted)
        distant = [s for s, p in candidates if math.dist(p[:2], best[:2]) >= max(.09, span*.8)
                   or abs(p[2]-best[2]) >= .09]
        gap = best_score-max(distant, default=0.)
        # Startup must establish a convincing match even if no correction is needed.
        accepted = best_score >= self.cfg.alignment_min_score and gap >= self.cfg.alignment_ambiguity_margin
        if not initial:
            accepted &= best_score-baseline >= self.cfg.alignment_min_gain
        return (best if accepted else predicted), {
            'accepted': bool(accepted), 'score': best_score, 'gain': best_score-baseline,
            'ambiguity_gap': gap, 'reason': 'matched' if accepted else 'weak_or_ambiguous'}


class Mapper:
    """Maintain an online map with gated local pose corrections before insertion."""

    def __init__(self, config, grid=None):
        self.cfg = config
        self.grid = grid or OccupancyMap(config)
        self.transform = (0., 0., 0.)
        self.alignment = {}
        self.estimator = PoseEstimator(config)

    def update(self, scan, covariance=()):
        if self.grid.last_scan == (scan.sequence, scan.stamp):
            return compose(self.transform, scan.pose)
        predicted = self.estimator.predict(scan.pose, covariance)
        corrected = predicted
        if self.cfg.alignment_enabled:
            corrected, self.alignment = self.grid.align(scan, predicted)
            if self.alignment.get('accepted'):
                accepted = self.estimator.correct(corrected)
                self.alignment['accepted'] = accepted
                if not accepted:
                    self.alignment['reason'] = 'innovation_rejected'
                corrected = self.estimator.pose
        # corrected = transform * scan.pose; solve for transform.
        self.transform = compose(corrected, between(scan.pose, (0., 0., 0.)))
        self.grid.integrate(scan, corrected)
        return corrected
