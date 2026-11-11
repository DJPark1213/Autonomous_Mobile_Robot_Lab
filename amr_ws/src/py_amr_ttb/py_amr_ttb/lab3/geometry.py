"""A location with no obstacle hit may still be unseen, and the robot's
body can collide even when its center is clear.

These helpers place scans in matching frames and check both obstacle contact and
observed coverage. Distances are metres; angles are radians, positive
counterclockwise. An Nx2 array has N [x, y] rows. Scan coordinates use the robot
frame located by scan.pose in starting odometry: x forward, y left. scan.origin
locates the laser in that frame. The geometry uses supplied pose estimates and
sampled beams; it cannot correct drift or reveal missed obstacles.
"""

from dataclasses import replace
import math

import numpy as np

from py_amr_ttb.odom_utils import normalize_angle

from .types import Scan


FULL_TURN = 2.0 * math.pi
# Widely spaced beams leave unseen gaps. These chosen acceptance limits
# require at least 72 beams (360 / 5), rather than describing sensor hardware.
MIN_SCAN_BEAMS = 72
MAX_BEAM_STEP = math.radians(5)
# Approximate full-circle metadata can still hide a large last-to-first gap;
# the coverage checks also limit actual gaps between neighboring beams.
SCAN_COVERAGE_TOLERANCE = math.radians(12)
MAX_OBSERVED_GAP = math.radians(6)
# Roundoff should not reject a point exactly on a range boundary. This tiny
# allowance does not compensate for sensor noise or provide body clearance.
RANGE_TOLERANCE = 1e-6
# Allow sub-millimetre pose noise at the already-occupied boundary. This only
# affects coverage; obstacle contact uses the original footprint without relief.
OCCUPIED_COVERAGE_TOLERANCE = 0.001


def transform_points(points, pose):
    """Measurements taken from different robot positions cannot be
    compared until they share a coordinate frame.

    Return Nx2 local points in the parent frame. pose is the local frame's (x, y,
    yaw) in that parent. For example, transform_points(scan.points, scan.pose)
    places obstacle points in starting odometry.
    """
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    # Using a column-vector rotation directly would turn these row-vector
    # points the wrong way; transpose the usual rotation matrix.
    rotation = np.array([[cosine, sine], [-sine, cosine]])
    return np.asarray(points) @ rotation + np.asarray(pose[:2])


def inverse_points(points, pose):
    """Steering needs a target's position relative to the robot, even
    when that target is stored in map or odometry coordinates.

    Return Nx2 parent-frame points in the local frame located by pose=(x, y, yaw).
    This reverses transform_points.
    """
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    inverse_rotation = np.array([[cosine, -sine], [sine, cosine]])
    translated_points = np.asarray(points) - np.asarray(pose[:2])
    return translated_points @ inverse_rotation


def compose(a, b):
    """The robot may be located in odometry while the goal is located
    in the map; navigation needs the relationship across both frames.

    a locates an intermediate frame in the parent; b locates the child in the
    intermediate frame. Return the child's (x, y, yaw) in the parent. For example,
    combine odometry's map pose with the robot's odometry pose.
    """
    child_position = transform_points([b[:2]], a)[0]
    child_heading = normalize_angle(a[2] + b[2])
    return float(child_position[0]), float(child_position[1]), child_heading


def between(parent, child):
    """Two absolute poses do not directly tell us where one viewpoint
    lies relative to the other.

    Both inputs are (x, y, yaw) in the same reference frame. Return child as seen
    from parent. scan_at_pose uses this relative transform to place a saved scan in
    another robot frame.
    """
    relative_position = inverse_points([child[:2]], parent)[0]
    relative_heading = normalize_angle(child[2] - parent[2])
    return float(relative_position[0]), float(relative_position[1]), relative_heading


def _validate_scan_geometry(
    ranges, angle_min, angle_increment, range_min, range_max, transform, angle_sign,
):
    """Sparse or partial scans can leave large unseen wedges where
    later geometry expects nearby beams around a full circle.

    Raise ValueError for invalid metadata, too few beams, excessive spacing or
    unsuitable angular coverage. process_scan separately classifies individual
    readings.
    """
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
    """A missing laser reading and an unobstructed beam must not become
    the same kind of free-space evidence.

    Return a Scan with matching angles, ranges, known and hits arrays sorted by
    robot-relative bearing. Reject unsuitable geometry or too many unknown readings.
    The usable distance is min(config.scan_range, range_max): valid finite returns
    strictly before it are hits; valid returns at or beyond it and positive infinity
    are known up to the limit. Invalid readings remain unknown despite their finite
    range placeholders.

    pose is the robot's acquisition pose in starting odometry; transform is the
    laser's (x, y, yaw) in the base frame. angle_sign=-1 handles an inverted laser.
    stamp uses sensor time; received uses monotonic time for freshness.
    """
    raw_ranges = np.asarray(ranges, dtype=float)
    _validate_scan_geometry(
        raw_ranges, angle_min, angle_increment, range_min, range_max,
        transform, angle_sign,
    )

    # Replacing invalid readings first would erase the distinction between
    # unknown space and a valid beam with no nearby return.
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

    # The laser may face a different direction from the robot. Correct mounting
    # before sorting, then keep ranges and flags attached to their beam angles.
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
    """The robot can move after taking a scan, so using its old robot-
    relative coordinates would misplace obstacles during planning.

    Return a copy in the robot frame at pose; both poses use starting odometry.
    Moving 0.5 m toward a saved hit 2 m ahead places it 1.5 m ahead. Beam lengths
    stay unchanged while the saved laser origin and directions move. Acquisition
    timestamps remain unchanged. Accuracy depends on the relative pose estimate and
    assumes saved obstacles have not moved.
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
    """The robot's front or side can touch an obstacle before its
    center reaches that obstacle's position.

    Return one straight-line travel limit per supplied heading, using N obstacle [x,
    y] rows and a circular radius including any caller-added margin. Points and
    headings use the robot frame; travel starts at (0, 0), with heading 0 forward
    and pi/2 left.

    Distances are capped in metres. A result equal to cap cannot distinguish contact
    exactly there, contact farther away or no contact. Only supplied hits constrain
    travel; observed_travel also checks unknown space.
    """
    headings = np.atleast_1d(headings)
    if not len(points):
        return np.full(len(headings), cap)

    # A fixed obstacle may block forward travel but miss a leftward path.
    # Project onto each path's forward/sideways axes to check that difference.
    # Broadcasting creates one entry per obstacle and candidate heading.
    cosine, sine = np.cos(headings), np.sin(headings)
    forward_distance = points[:, 0, None] * cosine + points[:, 1, None] * sine
    lateral_distance = -points[:, 0, None] * sine + points[:, 1, None] * cosine

    # The front edge contacts a point before the center reaches it. Pythagoras
    # gives that lead distance: offset^2 + lateral^2 = radius^2.
    contact_offset = np.sqrt(np.maximum(0.0, radius**2 - lateral_distance**2))
    # Points outside the body's path cannot block travel, but a point already
    # inside the starting footprint must still produce zero travel.
    can_contact = (
        (np.abs(lateral_distance) <= radius)
        & (forward_distance + contact_offset >= 0)
    )
    distance_to_contact = np.maximum(0.0, forward_distance - contact_offset)
    # Only the earliest possible collision constrains a heading; other points
    # must not reduce its travel limit.
    candidate_distances = np.where(can_contact, distance_to_contact, cap)
    return np.minimum(cap, candidate_distances.min(axis=0))


def observed(scan, points):
    """No detected obstacle at a point does not mean the laser saw that
    point: it may be behind a wall or in a missing-beam direction.

    Return one coverage Boolean per [x, y] point in the scan's base frame. Both
    surrounding beams must be known, close enough in angle and long enough to cover
    the point under the conservative range bound. This checks points;
    circles_observed checks the robot's body. Neither can reveal obstacles missed
    between beams.
    """
    # Coverage radiates from the laser, which may be offset from the robot.
    from_laser = np.atleast_2d(points) - scan.origin
    bearings = np.arctan2(from_laser[:, 1], from_laser[:, 0])
    # A point between beams needs support from both neighbors. Angle wrapping
    # must not create an artificial blind seam between the last and first beams.
    upper_indices = np.searchsorted(scan.angles, bearings) % len(scan.angles)
    lower_indices = (upper_indices - 1) % len(scan.angles)

    angular_gaps = (scan.angles[upper_indices] - scan.angles[lower_indices]) % FULL_TURN
    known_intervals = (
        scan.known[upper_indices]
        & scan.known[lower_indices]
        & (angular_gaps <= MAX_OBSERVED_GAP)
    )
    shorter_ranges = np.minimum(scan.ranges[upper_indices], scan.ranges[lower_indices])
    # The longer beam cannot establish coverage behind the shorter return.
    # Shrink that shorter range to the chord's midpoint distance across the gap.
    range_bounds = shorter_ranges * np.cos(angular_gaps / 2)
    point_distances = np.linalg.norm(from_laser, axis=1)
    return known_intervals & (point_distances <= range_bounds + RANGE_TOLERANCE)


def _circle_sector_extents(scan, centers, radii):
    """A robot's center can be observed while part of its body extends
    into unknown or obstructed space.

    Find how far coverage must reach in each sector to include the whole part of
    each circle inside it. A sector is the wedge between neighboring beams.
    circles_observed compares these required distances with the scan's coverage.

    centers contains [x, y] rows in the scan frame; radii has one radius per circle.
    Return (sector_extents, sector_widths): a (circles, beams) distance matrix and
    angular widths in radians. An extent of -inf means no overlap. Include the last-
    to-first beam sector.
    """
    from_laser = centers - scan.origin
    cosine, sine = np.cos(scan.angles), np.sin(scan.angles)
    along_ray = from_laser[:, 0, None] * cosine + from_laser[:, 1, None] * sine
    across_ray = -from_laser[:, 0, None] * sine + from_laser[:, 1, None] * cosine

    # Stopping coverage at the near circle intersection would leave the far
    # side of the body unseen; use the beam's exit from the circle.
    half_chord = np.sqrt(np.maximum(0.0, radii[:, None]**2 - across_ray**2))
    ray_exit = along_ray + half_chord
    ray_intersects = (np.abs(across_ray) <= radii[:, None]) & (ray_exit >= 0)
    boundary_extents = np.where(ray_intersects, ray_exit, -np.inf)
    sector_extents = np.maximum(boundary_extents, np.roll(boundary_extents, -1, axis=1))

    # Boundary beams can miss the circle's farthest bulge inside the sector.
    # If it contains the center direction, also require center distance + radius.
    sector_widths = (np.roll(scan.angles, -1) - scan.angles) % FULL_TURN
    center_bearings = np.arctan2(from_laser[:, 1], from_laser[:, 0])
    center_offsets = (center_bearings[:, None] - scan.angles) % FULL_TURN
    contains_center_bearing = center_offsets <= sector_widths
    furthest_from_laser = np.linalg.norm(from_laser, axis=1)[:, None] + radii[:, None]
    sector_extents = np.where(contains_center_bearing, furthest_from_laser, sector_extents)
    return sector_extents, sector_widths


def circles_observed(scan, centers, radius, occupied_radius=0., occupied_center=(0., 0.), diagnostics=None,
                     prefix_only=False):
    """A clear center point does not prevent the robot's edge from
    hitting an obstacle or entering space the laser never observed.

    Return one Boolean per circle, requiring no hit on or inside it and enough
    observed coverage across every overlapping sector. centers contains [x, y] rows
    in the scan frame; radius can be shared or supplied per circle. Callers may
    enlarge circles to cover movement between checked positions. Results depend on
    the scan model and cannot account for missed obstacles.
    """
    centers = np.atleast_2d(centers)
    radii = np.broadcast_to(np.asarray(radius), (len(centers),))
    misses_hits = np.ones(len(centers), dtype=bool)
    hits = scan.points
    if len(hits):
        to_hits = centers[:, None, :] - hits[None, :, :]
        squared_distances = np.sum(to_hits * to_hits, axis=2)
        misses_hits &= np.all(squared_distances > radii[:, None]**2, axis=1)

    # A few perimeter samples could straddle an unknown wedge and falsely pass
    # the footprint. Check every sector the circle overlaps.
    coverage_scan = scan
    from_laser = centers-scan.origin
    distances = np.linalg.norm(from_laser, axis=1)
    if np.all(distances > radii) and not scan.history:
        bearings = np.arctan2(from_laser[:, 1], from_laser[:, 0])
        half_angles = np.arcsin(radii/distances) + 2*MAX_OBSERVED_GAP
        offsets = (scan.angles[None, :]-bearings[:, None]+math.pi) % FULL_TURN-math.pi
        relevant = np.any(np.abs(offsets) <= half_angles[:, None], axis=0)
        if np.count_nonzero(relevant) >= 2:
            coverage_scan = replace(scan, angles=scan.angles[relevant],
                                    ranges=scan.ranges[relevant], known=scan.known[relevant],
                                    hits=scan.hits[relevant])
    sector_extents, sector_widths = _circle_sector_extents(coverage_scan, centers, radii)
    known_sectors = (
        coverage_scan.known
        & np.roll(coverage_scan.known, -1)
        & (sector_widths <= MAX_OBSERVED_GAP)
    )
    shorter_ranges = np.minimum(coverage_scan.ranges, np.roll(coverage_scan.ranges, -1))
    sector_bounds = shorter_ranges * np.cos(sector_widths / 2)
    outside_sector = sector_extents < 0
    fits_observed_sector = known_sectors & (sector_extents <= sector_bounds + RANGE_TOLERANCE)
    occupied_from_laser = np.asarray(occupied_center) - scan.origin
    coverage_radius = occupied_radius + OCCUPIED_COVERAGE_TOLERANCE
    if occupied_radius > 0 and np.linalg.norm(occupied_from_laser) < occupied_radius:
        # The current footprint supplies coverage only inside its boundary. Use
        # the shortest ray exit over each sector, so unseen space beyond the
        # footprint still rejects a proposed movement.
        center = occupied_from_laser
        distance = np.linalg.norm(center)
        along = center[0]*np.cos(coverage_scan.angles) + center[1]*np.sin(coverage_scan.angles)
        exits = along + np.sqrt(np.maximum(0., coverage_radius**2-distance**2+along**2))
        occupied_bounds = np.minimum(exits, np.roll(exits, -1))
        opposite = math.atan2(center[1], center[0]) + math.pi
        contains_minimum = (opposite-coverage_scan.angles) % FULL_TURN <= sector_widths
        occupied_bounds = np.where(contains_minimum, coverage_radius-distance, occupied_bounds)
        fits_observed_sector |= sector_extents <= occupied_bounds + RANGE_TOLERANCE
    fully_observed = (outside_sector | fits_observed_sector).all(axis=1)
    if occupied_radius > 0:
        fully_observed |= (np.linalg.norm(centers-np.asarray(occupied_center), axis=1)
                           + radii <= coverage_radius + RANGE_TOLERANCE)
    history_recovered = 0
    if scan.history:
        initially_observed = fully_observed.copy()
        cache_key = (float(occupied_radius), tuple(occupied_center))
        if getattr(scan, '_coverage_cache_key', None) != cache_key:
            scan._coverage_cache_key = cache_key
            scan._coverage_cache = {}
        sources = [replace(scan, history=())] + [
            replace(scan_at_pose(previous, scan.pose), history=()) for previous in scan.history]
        # Retain recent obstacle evidence too. Old free space cannot erase a hit.
        for previous in sources[1:]:
            fully_observed |= circles_observed(previous, centers, radii,
                                               occupied_radius, occupied_center)
            if len(previous.points):
                distances = centers[:, None, :] - previous.points[None, :, :]
                misses_hits &= np.all(np.sum(distances**2, axis=2) > radii[:, None]**2, axis=1)
        for index in np.flatnonzero(misses_hits & ~fully_observed):
            if prefix_only and not np.all(misses_hits[:index] & fully_observed[:index]):
                break
            if _covered_by_scans(sources, centers[index], radii[index],
                                 occupied_radius, occupied_center, scan._coverage_cache):
                fully_observed[index] = True
            elif prefix_only:
                break
        history_recovered = int(np.count_nonzero(fully_observed & ~initially_observed & misses_hits))
    if diagnostics is not None:
        diagnostics.update(
            footprints=len(centers),
            history_scans=len(scan.history),
            history_recovered=history_recovered,
            hit_rejections=int(np.count_nonzero(~misses_hits)),
            coverage_rejections=int(np.count_nonzero(~fully_observed)),
            unknown_beams=int(np.count_nonzero(~scan.known)),
            wide_sectors=int(np.count_nonzero(sector_widths > MAX_OBSERVED_GAP)),
            laser_origin=np.asarray(scan.origin).tolist(),
            occupied_radius=float(occupied_radius),
            occupied_coverage_active=bool(occupied_radius > 0 and
                                         np.linalg.norm(occupied_from_laser) < occupied_radius),
            nearest_hits=hits[np.argsort(np.linalg.norm(
                hits-np.asarray(occupied_center), axis=1))[:5]].tolist() if len(hits) else [],
            nearest_hit=float(np.linalg.norm(hits-np.asarray(occupied_center), axis=1).min())
            if len(hits) else None,
        )
    return misses_hits & fully_observed


def _covered_by_scans(sources, center, radius, occupied_radius, occupied_center, cache):
    """Certify a disk by covering intersecting squares with observed disks.

    Each accepted square is wholly observed by one source or the occupied body.
    Subdivide ambiguous squares down to 1 cm. Unresolved leaves fail closed.
    """
    for source in sources[1:]:
        if circles_observed(source, [center], radius, occupied_radius, occupied_center)[0]:
            return True
    # Reject obvious holes cheaply before attempting the full area certificate.
    # Passing these probes alone never authorizes a footprint.
    bearings = np.linspace(-math.pi, math.pi, 128, endpoint=False)
    probes = np.vstack((center, center + radius*np.column_stack(
        (np.cos(bearings), np.sin(bearings)))))
    probe_coverage = np.zeros(len(probes), dtype=bool)
    if occupied_radius > 0:
        probe_coverage |= np.linalg.norm(probes-np.asarray(occupied_center), axis=1) <= (
            occupied_radius + OCCUPIED_COVERAGE_TOLERANCE)
    for source in sources:
        probe_coverage |= observed(source, probes)
    if not probe_coverage.all():
        return False
    cell_size = .1
    lower = np.floor((center-radius)/cell_size).astype(int)
    upper = np.floor((center+radius)/cell_size).astype(int)
    cells = []
    for x in range(lower[0], upper[0]+1):
        for y in range(lower[1], upper[1]+1):
            cell_center = (np.array([x, y])+.5)*cell_size
            nearest = np.maximum(np.abs(cell_center-center)-cell_size/2, 0.)
            if np.dot(nearest, nearest) <= radius**2:
                cells.append((cell_center, cell_size/2))
    while cells:
        cell_centers = np.array([cell[0] for cell in cells])
        half_sizes = np.array([cell[1] for cell in cells])
        keys = [tuple(np.round([*point, half], 9)) for point, half in cells]
        covered = np.array([cache.get(key, False) for key in keys])
        uncached = np.array([key not in cache for key in keys])
        for source in sources:
            remaining = np.flatnonzero(~covered & uncached)
            if not len(remaining):
                break
            covered[remaining] = circles_observed(
                source, cell_centers[remaining], half_sizes[remaining]*math.sqrt(2),
                occupied_radius, occupied_center)
        for index in np.flatnonzero(uncached):
            cache[keys[index]] = bool(covered[index])
        unresolved = np.flatnonzero(~covered)
        if np.any(half_sizes[unresolved] <= .005):
            return False
        children = []
        for index in unresolved:
            half = half_sizes[index]/2
            for offset in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
                child = cell_centers[index] + np.asarray(offset)*half
                nearest = np.maximum(np.abs(child-center)-half, 0.)
                if np.dot(nearest, nearest) <= radius**2:
                    children.append((child, half))
        cells = children
    return True


def circles_clear(scan, centers, radius, occupied_radius=0., occupied_center=(0., 0.), diagnostics=None):
    """A motion segment must be rejected if even one of its covering
    footprints fails the clearance check.

    Combine circles_observed's results into one Boolean. The caller must provide
    circles that cover movement between positions, not just endpoints.
    """
    return bool(circles_observed(scan, centers, radius, occupied_radius, occupied_center,
                                 diagnostics, prefix_only=True).all())


def observed_travel(scan, heading, radius, cap, step=0.05):
    """Obstacle-only checks can allow travel into unknown space simply
    because that space produced no valid hits.

    Return a straight-line distance limited by both hits and observed circular
    footprints. heading is robot-relative, radius is the footprint size, and cap and
    step are metres. Starting at the base origin, sample every step up to the
    contact limit or cap, retaining one step before the first failure or limit. Zero
    can mean the starting footprint fails. This sampled check does not prove
    coverage between every pair of samples.
    """
    contact_limit = float(free_distances(scan.points, [heading], radius, cap)[0])
    sample_distances = np.arange(0.0, contact_limit + 1e-9, step)
    direction = np.array([math.cos(heading), math.sin(heading)])
    centers = sample_distances[:, None] * direction
    clear_footprints = circles_observed(scan, centers, radius, occupied_radius=radius,
                                      prefix_only=True)
    blocked_indices = np.flatnonzero(~clear_footprints)

    if len(blocked_indices):
        first_blocked_distance = sample_distances[blocked_indices[0]]
        return max(0.0, float(first_blocked_distance - step))
    return max(0.0, contact_limit - step)


def grid_line(start, end):
    """Map updates and route checks need the cells along a beam or
    segment, not only its two endpoints.

    Yield a Bresenham line including start and end, given integer (x, y) cell
    indices. The caller decides which cells to mark or inspect. The line has no
    robot-width padding.
    """
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
