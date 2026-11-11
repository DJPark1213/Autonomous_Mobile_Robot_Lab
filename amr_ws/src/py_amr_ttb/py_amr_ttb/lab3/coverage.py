"""NumPy-only prepared angular coverage and conservative area queries."""
from dataclasses import dataclass
import math
import numpy as np
from .geometry import transform_points, MAX_OBSERVED_GAP, FULL_TURN

MAX_NEAR_FIELD_GAP = math.radians(4.)
MAX_NEAR_FIELD_RANGE = .50


@dataclass
class AngularCoverage:
    origin: np.ndarray
    angles: np.ndarray
    invalid_prefix: np.ndarray
    minimums: tuple
    hits: np.ndarray

    def covers(self, centers, radii):
        delta = centers-self.origin
        distance = np.linalg.norm(delta, axis=1)
        inside = distance <= radii
        half = np.arcsin(np.minimum(1., radii/np.maximum(distance, 1e-12)))
        bearing = np.arctan2(delta[:, 1], delta[:, 0])
        low = (bearing-half-self.angles[0]) % FULL_TURN+self.angles[0]
        high = low+2*half
        start = np.searchsorted(self.angles, low, side='right')-1
        end = np.searchsorted(self.angles, high, side='right')-1
        count = len(self.minimums[0])//2
        start = np.where(inside, 0, start)
        end = np.where(inside, count-1, end)
        lengths = end-start+1
        levels = np.floor(np.log2(lengths)).astype(int)
        bound = np.empty(len(centers))
        for level in np.unique(levels):
            chosen = levels == level
            width = 1 << int(level)
            bound[chosen] = np.minimum(self.minimums[level][start[chosen]],
                                      self.minimums[level][end[chosen]-width+1])
        known = self.invalid_prefix[end+1] == self.invalid_prefix[start]
        return known & (distance+radii <= bound+1e-6)


@dataclass
class PreparedCoverage:
    sources: tuple
    receipts: tuple

    def select(self, now, timeout):
        eligible = tuple(source for source, receipt in zip(self.sources, self.receipts)
                         if 0 <= now-receipt <= timeout)
        if not eligible or eligible[0] is not self.sources[0]:
            return None
        return eligible


def prepare_coverage(scans, *, bridge_small_gaps=False):
    sources, receipts = [], []
    for scan in sorted(scans, key=lambda item: item.received, reverse=True):
        angles = (scan.angles+scan.pose[2]+math.pi) % FULL_TURN-math.pi
        order = np.argsort(angles)
        angles = angles[order]
        ranges, known = scan.ranges[order], scan.known[order]
        widths = (np.roll(angles, -1)-angles) % FULL_TURN
        valid = known & np.roll(known, -1) & (widths <= MAX_OBSERVED_GAP)
        bound = np.minimum(ranges, np.roll(ranges, -1))*np.cos(widths/2)
        if bridge_small_gaps:
            # Infer only short, bracketed near-field intervals. Limit each
            # interval by the shorter valid beam and the chord across the gap.
            indices = np.flatnonzero(known)
            if len(indices) >= 2:
                next_indices = np.roll(indices, -1)
                steps = (next_indices-indices) % len(angles)
                spans = (angles[next_indices]-angles[indices]) % FULL_TURN
                gaps = (steps > 1) & (spans > 0.) & (spans <= MAX_NEAR_FIELD_GAP)
                for left, right, count, span in zip(indices[gaps], next_indices[gaps],
                                                     steps[gaps], spans[gaps]):
                    sectors = (left+np.arange(count)) % len(angles)
                    valid[sectors] = True
                    limit = min(MAX_NEAR_FIELD_RANGE, ranges[left], ranges[right])
                    bound[sectors] = limit*math.cos(span/2)
        values = np.tile(bound, 2)
        tables = [values]
        width = 1
        while 2*width <= len(values):
            tables.append(np.minimum(tables[-1][:-width], tables[-1][width:]))
            width *= 2
        prefix = np.concatenate(([0], np.cumsum(np.tile(~valid, 2))))
        sources.append(AngularCoverage(transform_points([scan.origin], scan.pose)[0],
                                       np.concatenate((angles, angles+FULL_TURN)),
                                       prefix, tuple(tables), transform_points(scan.points, scan.pose)))
        receipts.append(scan.received)
    return PreparedCoverage(tuple(sources), tuple(receipts))


def _covered(target_centers, target_radii, sources, occupied, occupied_radius):
    """Certify complete footprint areas with batched conservative subdivisions."""
    bearings = np.linspace(-math.pi, math.pi, 64, endpoint=False)
    directions = np.vstack(([0., 0.], np.column_stack((np.cos(bearings), np.sin(bearings)))))
    probes = (target_centers[:, None, :]+target_radii[:, None, None]*directions).reshape(-1, 2)
    known = np.linalg.norm(probes-occupied, axis=1) <= occupied_radius
    for source in sources:
        known |= source.covers(probes, np.zeros(len(probes)))
    # Probes can reject a hole; acceptance still requires the complete area proof.
    result = known.reshape(len(target_centers), -1).all(axis=1)
    owners = np.flatnonzero(result)
    centers = target_centers[owners].copy()
    halves = target_radii[owners].copy()
    offsets = np.array([[-1., -1.], [-1., 1.], [1., -1.], [1., 1.]])
    while len(centers):
        disk_radii = halves*math.sqrt(2.)
        covered = np.linalg.norm(centers-occupied, axis=1)+disk_radii <= occupied_radius
        for source in sources:
            remaining = np.flatnonzero(~covered)
            if not len(remaining):
                break
            covered[remaining] = source.covers(centers[remaining], disk_radii[remaining])
        failed = ~covered & (halves <= .001)
        result[owners[failed]] = False
        remaining = ~covered & result[owners]
        centers, halves, owners = centers[remaining], halves[remaining]/2, owners[remaining]
        children = (centers[:, None, :]+halves[:, None, None]*offsets).reshape(-1, 2)
        halves = np.repeat(halves, 4)
        owners = np.repeat(owners, 4)
        nearest = np.maximum(np.abs(children-target_centers[owners])-halves[:, None], 0.)
        intersects = np.sum(nearest**2, axis=1) <= target_radii[owners]**2
        centers, halves, owners = children[intersects], halves[intersects], owners[intersects]
    return result

def prepared_circles(scan, centers, radii, occupied_radius, occupied_center, diagnostics, selected):
    centers = transform_points(centers, scan.pose)
    occupied = transform_points([occupied_center], scan.pose)[0]
    hits = np.concatenate([transform_points(scan.points, scan.pose)]+[s.hits for s in selected], axis=0)
    clear = np.ones(len(centers), dtype=bool)
    if len(hits):
        delta = centers[:,None,:]-hits[None,:,:]
        clear &= np.all(np.sum(delta**2, axis=2) > radii[:,None]**2, axis=1)
    allowance = occupied_radius+.001 if occupied_radius > 0 else 0.
    covered = np.linalg.norm(centers-occupied, axis=1)+radii <= allowance
    for source in selected:
        remaining = np.flatnonzero(~covered)
        if not len(remaining):
            break
        covered[remaining] = source.covers(centers[remaining], radii[remaining])
    remaining = np.flatnonzero(clear & ~covered)
    if len(remaining):
        covered[remaining] = _covered(centers[remaining], radii[remaining], selected, occupied, allowance)
    if diagnostics is not None:
        diagnostics.update(footprints=len(centers), hit_rejections=int(sum(~clear)),
                           coverage_rejections=int(sum(~covered)),
                           unknown_beams=int(sum(~scan.known)), prepared_coverage=True)
    return clear & covered
