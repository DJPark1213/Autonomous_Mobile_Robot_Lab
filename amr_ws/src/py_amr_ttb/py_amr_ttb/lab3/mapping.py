"""One scan cannot describe previously visited corridors or distinguish
an unexplored region from a known dead end.

Accumulate observed free/occupied evidence for route queries and use local scan
matching to improve placement before insertion.
"""

import math
import numpy as np

from .geometry import compose, transform_points, grid_line, between
from .state import PoseEstimator


def dilate(mask, cells):
    """A route centerline can miss an occupied cell while the robot's
    side still touches it.

    Expand a Boolean mask by circular integer offsets of radius cells. Return a
    copy. Alignment also uses this helper to tolerate nearby wall cells.
    """
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
    """A cell with no obstacle evidence may simply never have been
    observed.

    Store observation flags separately from bounded free/occupied evidence.
    Positions use map metres, cells are (x, y), and arrays use [y, x].
    """

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
        """Measured positions use metres while map evidence is
        addressed by integer array indices.

        Return the containing (x, y) cell for a map point. Check inside before array
        access.
        """
        return tuple(np.floor((np.asarray(point)-self.origin)/self.resolution).astype(int))

    def point(self, cell):
        """A selected grid cell needs a physical location before it can
        be used in geometric calculations.

        Return its center in map metres.
        """
        return tuple(self.origin+(np.asarray(cell)+.5)*self.resolution)

    def inside(self, cell):
        """Rays and route queries can extend beyond the allocated map,
        where array indexing would fail or wrap incorrectly.

        Return whether an (x, y) cell lies within the grid.
        """
        x, y = cell
        return 0 <= x < self.evidence.shape[1] and 0 <= y < self.evidence.shape[0]

    def integrate(self, scan, map_pose):
        """Reusing a scan or counting every crossing beam separately
        would exaggerate evidence and could erase a real hit with free-space
        updates.

        Update each touched cell once per new scan, with hits taking precedence.
        Only known beams contribute. map_pose places the scan frame in the map.
        Return False for a duplicate; otherwise invalidate cached layers and return
        True.
        """
        token = (scan.sequence, scan.stamp)
        if token == self.last_scan:
            return False
        self.last_scan = token
        start = self.cell(transform_points([scan.origin], map_pose)[0])
        if not self.inside(start):
            raise ValueError('Robot left configured map bounds')
        endpoints = transform_points(scan.endpoints, map_pose)
        free, hits = set(), set()
        # Unknown beams have placeholder lengths; tracing them would invent free space.
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
        """Repeated route queries need robot-width clearance, and
        uncertain cells must not be mistaken for free space.

        Return cached (blocked, free) masks. Inflate occupied cells by the footprint
        plus half a cell diagonal. free also requires observed, sufficiently
        negative evidence; some cells belong to neither mask.
        """
        if self._layers is None:
            occupied = self.evidence >= self.cfg.occupied_threshold
            # An obstacle can lie at a cell corner rather than its center.
            # Half a cell diagonal covers that extra distance during inflation.
            n = (self.cfg.radius+self.resolution/math.sqrt(2))/self.resolution
            blocked = dilate(occupied, n)
            free = self.observed & (self.evidence <= self.cfg.free_threshold) & ~blocked
            self._layers = blocked, free
        return self._layers

    def segment_clear(self, a, b, require_observed=True, margin=0.):
        """Two reachable endpoints do not prove that the straight path
        between them avoids walls or stays inside observed space.

        Check a and b in map metres along their raster line. Always reject centers
        in inflated occupied cells or outside the grid. With require_observed=True,
        require free evidence across the footprint plus margin. False returns reject
        the segment; skipping the coverage check means True no longer establishes
        that it was seen.
        """
        blocked, free = self.layers()
        cells = list(grid_line(self.cell(a), self.cell(b)))
        r = math.ceil((self.cfg.radius+margin)/self.resolution)
        for x, y in cells:
            if not self.inside((x, y)) or blocked[y, x]:
                return False
            if require_observed:
                if x-r < 0 or y-r < 0 or x+r >= free.shape[1] or y+r >= free.shape[0]:
                    return False
                # A known centerline can still put the robot's side in unknown cells.
                for dy in range(-r, r+1):
                    span = int(math.sqrt(max(0, r*r-dy*dy)))
                    region = self.observed[y+dy, x-span:x+span+1]
                    evidence = self.evidence[y+dy, x-span:x+span+1]
                    if not np.all(region & (evidence <= self.cfg.free_threshold)):
                        return False
        return True

    def align(self, scan, predicted, initial=False):
        """Odometry drift can shift new scan hits away from walls
        already in the map.

        Search near predicted for better wall agreement, rejecting weak or ambiguous
        matches. initial permits a wider replay-start search; later matches must
        also improve on predicted. Return (pose, diagnostics), keeping predicted
        when rejected. This cannot recover arbitrary drift.
        """
        points = scan.points
        occupied = self.evidence >= self.cfg.occupied_threshold
        if len(points) < self.cfg.alignment_min_hits or occupied.sum() < self.cfg.alignment_min_hits:
            return predicted, {'accepted': False, 'reason': 'insufficient_geometry'}
        points = points[::max(1, len(points)//100)]
        close = dilate(occupied, 1)

        def score(pose):
            """A candidate pose must not look better merely because it
            moves mismatched hits outside the map.

            Divide wall-agreement evidence by the full sampled hit count, including
            out-of-bounds hits in that denominator.
            """
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
        # Repeated-looking walls can fit several poses; reject ambiguous matches.
        # Startup needs evidence of a match even if its best pose equals predicted.
        accepted = best_score >= self.cfg.alignment_min_score and gap >= self.cfg.alignment_ambiguity_margin
        if not initial:
            accepted &= best_score-baseline >= self.cfg.alignment_min_gain
        return (best if accepted else predicted), {
            'accepted': bool(accepted), 'score': best_score, 'gain': best_score-baseline,
            'ambiguity_gap': gap, 'reason': 'matched' if accepted else 'weak_or_ambiguous'}


class Mapper:
    """Inserting a scan before correcting its pose can duplicate or
    smear mapped walls.

    Predict, gate local correction, then insert evidence. transform locates starting
    odometry in the map and accompanies planner targets.
    """

    def __init__(self, config, grid=None):
        self.cfg = config
        self.grid = grid or OccupancyMap(config)
        self.transform = (0., 0., 0.)
        self.alignment = {}
        self.estimator = PoseEstimator(config)

    def update(self, scan, covariance=()):
        """The new scan and the live robot pose need a consistent map
        relationship after any accepted correction.

        Predict from odometry, optionally correct from scan matching, update
        transform, and integrate once. Return the scan's map pose. Duplicate scans
        reuse the current transform without adding evidence.
        """
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
        # Live odometry and targets still need to agree after this scan is
        # corrected: solve corrected = transform * scan.pose for transform.
        self.transform = compose(corrected, between(scan.pose, (0., 0., 0.)))
        self.grid.integrate(scan, corrected)
        return corrected
