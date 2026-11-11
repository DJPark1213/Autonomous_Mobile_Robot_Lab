"""Prepare conservative scan coverage outside the command callback."""

from dataclasses import dataclass
import math
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
from shapely import wkb

from .geometry import transform_points, MAX_OBSERVED_GAP, FULL_TURN


@dataclass
class PreparedCoverage:
    # Newest-only coverage first, followed by successively older observations.
    regions: tuple
    receipts: tuple
    hit_sets: tuple

    def select(self, now, timeout):
        eligible = [i for i, receipt in enumerate(self.receipts)
                    if 0 <= now-receipt <= timeout]
        if not eligible or eligible[0] != 0:
            return None
        index = max(eligible)
        if getattr(self, '_selected', None) != index:
            self._selected = index
            self._region = wkb.loads(self.regions[index], hex=True)
            self._hits = np.concatenate(self.hit_sets[:index+1], axis=0)
        return self._region, self._hits


def prepare_coverage(scans):
    """Build nested unions so expiry only selects an already-built result."""
    regions, receipts, hit_sets = [], [], []
    accumulated = None
    for scan in sorted(scans, key=lambda item: item.received, reverse=True):
        angles = scan.angles
        widths = (np.roll(angles, -1)-angles) % FULL_TURN
        valid = scan.known & np.roll(scan.known, -1) & (widths <= MAX_OBSERVED_GAP)
        bounds = np.minimum(scan.ranges, np.roll(scan.ranges, -1))
        origin = transform_points([scan.origin], scan.pose)[0]
        polygons = []
        directions = np.column_stack((np.cos(angles), np.sin(angles)))
        if valid.all():
            radii = np.minimum(bounds, np.roll(bounds, 1))
            polygons.append(Polygon(transform_points(
                scan.origin+radii[:, None]*directions, scan.pose)))
        else:
            for start in np.flatnonzero(valid & ~np.roll(valid, 1)):
                sectors = [int(start)]
                while valid[(sectors[-1]+1) % len(angles)]:
                    sectors.append((sectors[-1]+1) % len(angles))
                indices = sectors+[(sectors[-1]+1) % len(angles)]
                radii = [bounds[sectors[0]]]+[
                    min(bounds[a], bounds[b]) for a, b in zip(sectors[:-1], sectors[1:])
                ]+[bounds[sectors[-1]]]
                ends = transform_points(scan.origin+np.asarray(radii)[:, None]*directions[indices], scan.pose)
                polygons.append(Polygon(np.vstack((origin, ends))))
        region = unary_union(polygons)
        accumulated = region if accumulated is None else accumulated.union(region)
        regions.append(accumulated.wkb_hex)
        receipts.append(scan.received)
        hit_sets.append(transform_points(scan.points, scan.pose))
    return PreparedCoverage(tuple(regions), tuple(receipts), tuple(hit_sets))


def prepared_circles(scan, centers, radii, occupied_radius, occupied_center, diagnostics, selected):
    """Query outward polygon bounds for footprints against prepared coverage."""
    region, historic_hits = selected
    global_centers = transform_points(centers, scan.pose)
    occupied = transform_points([occupied_center], scan.pose)[0]
    if occupied_radius > 0:
        region = region.union(Point(occupied).buffer(occupied_radius+.001, 32))
    current_hits = transform_points(scan.points, scan.pose)
    hits = np.concatenate((historic_hits, current_hits), axis=0)
    clear = np.ones(len(centers), dtype=bool)
    if len(hits):
        differences = global_centers[:, None, :]-hits[None, :, :]
        clear &= np.all(np.sum(differences**2, axis=2) > radii[:, None]**2, axis=1)
    covered = np.array([region.covers(Point(center).buffer(
        float(radius)/math.cos(math.pi/128), 32))
        for center, radius in zip(global_centers, radii)])
    if diagnostics is not None:
        diagnostics.update(footprints=len(centers), hit_rejections=int(sum(~clear)),
                           coverage_rejections=int(sum(~covered)),
                           unknown_beams=int(sum(~scan.known)), prepared_coverage=True)
    return clear & covered
