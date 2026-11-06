"""Reusable scan and footprint geometry, independent of ROS.

Distances are metres and angles are radians, positive counterclockwise. Scan
points use the base frame identified by scan.pose: x forward, y left. Initially
this is the acquisition frame; scan_at_pose can re-express it at another pose.
The laser may be offset from the base origin; scan.origin carries that offset.
"""

from dataclasses import replace
import math

import numpy as np

from py_amr_ttb.odom_utils import normalize_angle

from .types import Scan


FULL_TURN = 2.0 * math.pi
MIN_SCAN_BEAMS = 72
MAX_BEAM_STEP = math.radians(5)
SCAN_COVERAGE_TOLERANCE = math.radians(12)
MAX_OBSERVED_GAP = math.radians(6)
RANGE_TOLERANCE = 1e-6


def transform_points(points, pose):
    """Convert Nx2 local points into the parent frame of an (x, y, yaw) pose."""
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    # Points are row vectors, so multiplication uses the rotation's transpose.
    rotation = np.array([[cosine, sine], [-sine, cosine]])
    return np.asarray(points) @ rotation + np.asarray(pose[:2])


def inverse_points(points, pose):
    """Convert Nx2 parent-frame points into the local frame described by pose."""
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    inverse_rotation = np.array([[cosine, -sine], [sine, cosine]])
    translated_points = np.asarray(points) - np.asarray(pose[:2])
    return translated_points @ inverse_rotation


def compose(a, b):
    """Chain poses: a is parent-from-local, b is local-from-child."""
    child_position = transform_points([b[:2]], a)[0]
    child_heading = normalize_angle(a[2] + b[2])
    return float(child_position[0]), float(child_position[1]), child_heading


def between(parent, child):
    """Express child in parent's local frame; both inputs share a fixed frame."""
    relative_position = inverse_points([child[:2]], parent)[0]
    relative_heading = normalize_angle(child[2] - parent[2])
    return float(relative_position[0]), float(relative_position[1]), relative_heading


def _validate_scan_geometry(
    ranges, angle_min, angle_increment, range_min, range_max, transform, angle_sign,
):
    """Require finite metadata and sufficiently dense, full-circle coverage."""
    metadata = (angle_min, angle_increment, range_min, range_max, *transform)
    if not all(math.isfinite(value) for value in metadata) or angle_sign not in (-1, 1):
        raise ValueError('Invalid laser metadata or transform')

    beam_step = abs(angle_increment)
    if (
        ranges.ndim != 1
        or len(ranges) < MIN_SCAN_BEAMS
        or range_min < 0
        or range_max <= range_min
        or not 0 < beam_step <= MAX_BEAM_STEP
    ):
        raise ValueError('Invalid laser geometry')

    angular_coverage = beam_step * len(ranges)
    if abs(angular_coverage - FULL_TURN) > SCAN_COVERAGE_TOLERANCE:
        raise ValueError('Full 360 degree scan required')


def process_scan(
    ranges, angle_min, angle_increment, range_min, range_max,
    config, pose=(0.0, 0.0, 0.0), transform=(0.0, 0.0, 0.0),
    angle_sign=1.0, stamp=0.0, received=0.0, sequence=0,
):
    """Validate raw laser data and return a Scan sorted by base-frame bearing.

    pose locates the robot in starting odometry at acquisition. transform is the
    laser's (x, y, yaw) in the base frame; angle_sign=-1 handles an inverted laser.
    stamp uses the sensor clock; received is the monotonic freshness timestamp.

    Valid finite readings before the usable range limit are obstacle hits.
    Longer valid readings and positive infinity are known up to that limit.
    NaN, negative infinity, and out-of-range readings remain unknown.

    Stored ranges are finite for downstream geometry. An unknown beam's
    placeholder length is never clearance: callers must also check known.
    """
    raw_ranges = np.asarray(ranges, dtype=float)
    _validate_scan_geometry(
        raw_ranges, angle_min, angle_increment, range_min, range_max,
        transform, angle_sign,
    )

    # Classify measurements before replacing missing values with finite lengths.
    valid_return = (
        np.isfinite(raw_ranges)
        & (raw_ranges >= range_min)
        & (raw_ranges <= range_max)
    )
    no_return = np.isposinf(raw_ranges)
    known = valid_return | no_return
    if known.mean() < config.scan_known_fraction:
        raise ValueError('Too many unknown laser beams')

    usable_range = min(config.scan_range, range_max)
    hits = valid_return & (raw_ranges < usable_range)
    effective_ranges = np.where(
        valid_return, np.minimum(raw_ranges, usable_range), usable_range,
    )

    # Convert bearings to the base frame, then apply one ordering to all arrays.
    laser_angles = angle_min + np.arange(len(raw_ranges)) * angle_increment
    base_angles = angle_sign * laser_angles + transform[2]
    base_angles = (base_angles + math.pi) % FULL_TURN - math.pi
    bearing_order = np.argsort(base_angles)

    return Scan(
        angles=base_angles[bearing_order],
        ranges=effective_ranges[bearing_order],
        known=known[bearing_order],
        hits=hits[bearing_order],
        origin=np.asarray(transform[:2]),
        stamp=stamp,
        received=received,
        pose=tuple(pose),
        sequence=sequence,
    )


def scan_at_pose(scan, pose):
    """Express a recorded scan in another robot pose without inventing new data.

    Both poses use starting odometry. Ranges and acquisition timestamps stay the
    same; the laser origin and bearings move into the requested base frame.
    """
    acquisition_in_current_frame = between(pose, scan.pose)
    current_angles = scan.angles + acquisition_in_current_frame[2]
    current_angles = (current_angles + math.pi) % FULL_TURN - math.pi
    bearing_order = np.argsort(current_angles)
    current_origin = transform_points([scan.origin], acquisition_in_current_frame)[0]

    return replace(
        scan,
        origin=current_origin,
        angles=current_angles[bearing_order],
        ranges=scan.ranges[bearing_order],
        known=scan.known[bearing_order],
        hits=scan.hits[bearing_order],
        pose=pose,
    )


def free_distances(points, headings, radius, cap):
    """Return travel before a circular robot touches a hit, for each heading.

    Nx2 hit points and headings share the robot frame; travel starts at (0, 0).
    Results are capped in metres. This tests obstacles only, so use
    observed_travel when unknown space must also limit travel.
    """
    headings = np.atleast_1d(headings)
    if not len(points):
        return np.full(len(headings), cap)

    # Each matrix entry pairs one obstacle with one proposed travel direction.
    cosine, sine = np.cos(headings), np.sin(headings)
    forward_distance = points[:, 0, None] * cosine + points[:, 1, None] * sine
    lateral_distance = -points[:, 0, None] * sine + points[:, 1, None] * cosine

    # Contact occurs before the obstacle's centerline projection by this offset.
    contact_offset = np.sqrt(np.maximum(0.0, radius**2 - lateral_distance**2))
    can_contact = (
        (np.abs(lateral_distance) <= radius)
        & (forward_distance + contact_offset >= 0)
    )
    distance_to_contact = np.maximum(0.0, forward_distance - contact_offset)
    candidate_distances = np.where(can_contact, distance_to_contact, cap)
    return np.minimum(cap, candidate_distances.min(axis=0))


def observed(scan, points):
    """Return whether each Nx2 point lies inside known scan coverage.

    Points use the scan's base frame. Both rays bracketing a point must be known,
    and the shorter range bounds the entire interval conservatively.
    """
    from_laser = np.atleast_2d(points) - scan.origin
    bearings = np.arctan2(from_laser[:, 1], from_laser[:, 0])
    upper_indices = np.searchsorted(scan.angles, bearings) % len(scan.angles)
    lower_indices = (upper_indices - 1) % len(scan.angles)

    angular_gaps = (scan.angles[upper_indices] - scan.angles[lower_indices]) % FULL_TURN
    known_intervals = (
        scan.known[upper_indices]
        & scan.known[lower_indices]
        & (angular_gaps <= MAX_OBSERVED_GAP)
    )
    shorter_ranges = np.minimum(scan.ranges[upper_indices], scan.ranges[lower_indices])
    range_bounds = shorter_ranges * np.cos(angular_gaps / 2)
    point_distances = np.linalg.norm(from_laser, axis=1)
    return known_intervals & (point_distances <= range_bounds + RANGE_TOLERANCE)


def _circle_sector_extents(scan, centers, radii):
    """Find each circle's furthest radial reach in every adjacent-ray sector.

    The extent matrix has shape (circles, beams); negative infinity means no
    overlap. Sector widths include the last-to-first bearing interval.
    """
    from_laser = centers - scan.origin
    cosine, sine = np.cos(scan.angles), np.sin(scan.angles)
    along_ray = from_laser[:, 0, None] * cosine + from_laser[:, 1, None] * sine
    across_ray = -from_laser[:, 0, None] * sine + from_laser[:, 1, None] * cosine

    # Intersect each ray with each circle and keep the farther intersection.
    half_chord = np.sqrt(np.maximum(0.0, radii[:, None]**2 - across_ray**2))
    ray_exit = along_ray + half_chord
    ray_intersects = (np.abs(across_ray) <= radii[:, None]) & (ray_exit >= 0)
    boundary_extents = np.where(ray_intersects, ray_exit, -np.inf)
    sector_extents = np.maximum(boundary_extents, np.roll(boundary_extents, -1, axis=1))

    # The maximum is on a sector boundary, unless the sector contains the
    # direction to the circle's center. That direction reaches center + radius.
    sector_widths = (np.roll(scan.angles, -1) - scan.angles) % FULL_TURN
    center_bearings = np.arctan2(from_laser[:, 1], from_laser[:, 0])
    center_offsets = (center_bearings[:, None] - scan.angles) % FULL_TURN
    contains_center_bearing = center_offsets <= sector_widths
    furthest_from_laser = np.linalg.norm(from_laser, axis=1)[:, None] + radii[:, None]
    sector_extents = np.where(contains_center_bearing, furthest_from_laser, sector_extents)
    return sector_extents, sector_widths


def circles_observed(scan, centers, radius):
    """Return a clearance mask for circular footprints in the scan's base frame.

    centers is Nx2. radius is either one shared radius or N radii, as used for
    padded motion-sweep segments. A circle must miss every hit and fit fully
    inside observed space; even one unknown beam can invalidate its coverage.
    """
    centers = np.atleast_2d(centers)
    radii = np.broadcast_to(np.asarray(radius), (len(centers),))
    misses_hits = np.ones(len(centers), dtype=bool)
    hits = scan.points
    if len(hits):
        to_hits = centers[:, None, :] - hits[None, :, :]
        squared_distances = np.sum(to_hits * to_hits, axis=2)
        misses_hits &= np.all(squared_distances > radii[:, None]**2, axis=1)

    # Every adjacent-ray sector is checked; sparse perimeter samples could
    # overlook a missing beam between two samples.
    sector_extents, sector_widths = _circle_sector_extents(scan, centers, radii)
    known_sectors = (
        scan.known
        & np.roll(scan.known, -1)
        & (sector_widths <= MAX_OBSERVED_GAP)
    )
    shorter_ranges = np.minimum(scan.ranges, np.roll(scan.ranges, -1))
    sector_bounds = shorter_ranges * np.cos(sector_widths / 2)
    outside_sector = sector_extents < 0
    fits_observed_sector = known_sectors & (sector_extents <= sector_bounds + RANGE_TOLERANCE)
    fully_observed = (outside_sector | fits_observed_sector).all(axis=1)
    return misses_hits & fully_observed


def circles_clear(scan, centers, radius):
    """Return True only when every circular footprint passes the clearance test."""
    return bool(circles_observed(scan, centers, radius).all())


def observed_travel(scan, heading, radius, cap, step=0.05):
    """Return safe observed travel along a heading, starting at the base origin.

    First bound travel by obstacle hits, then check footprints at step metre
    intervals. Keep a one-step reserve before the first failed footprint or the
    distance limit. This is a sampled geometric check, not a speed command.
    """
    contact_limit = float(free_distances(scan.points, [heading], radius, cap)[0])
    sample_distances = np.arange(0.0, contact_limit + 1e-9, step)
    direction = np.array([math.cos(heading), math.sin(heading)])
    centers = sample_distances[:, None] * direction
    clear_footprints = circles_observed(scan, centers, radius)
    blocked_indices = np.flatnonzero(~clear_footprints)

    if len(blocked_indices):
        first_blocked_distance = sample_distances[blocked_indices[0]]
        return max(0.0, float(first_blocked_distance - step))
    return max(0.0, contact_limit - step)


def grid_line(start, end):
    """Yield Bresenham grid cells (x, y), including both start and end."""
    x, y = start
    end_x, end_y = end
    delta_x = abs(end_x - x)
    delta_y = -abs(end_y - y)
    step_x = 1 if x < end_x else -1
    step_y = 1 if y < end_y else -1
    error = delta_x + delta_y
    while True:
        yield x, y
        if x == end_x and y == end_y:
            break
        doubled_error = 2 * error
        if doubled_error >= delta_y:
            error += delta_y
            x += step_x
        if doubled_error <= delta_x:
            error += delta_x
            y += step_y
