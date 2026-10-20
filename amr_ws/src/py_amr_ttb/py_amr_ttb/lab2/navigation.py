"""Pure waypoint math shared by Lab 2 nodes and offline plotting."""

from bisect import bisect_right
import math

from py_amr_ttb.odom_utils import normalize_angle


def waypoint_command(x, y, yaw, target_x, target_y, config):
    """Return linear m/s, angular rad/s, reached for a stop/turn/go target."""
    dx, dy = target_x - x, target_y - y
    distance = math.hypot(dx, dy)
    if distance <= config.NAV_POSITION_TOLERANCE:
        return 0.0, 0.0, True
    error = normalize_angle(math.atan2(dy, dx) - yaw)
    angular = max(
        -config.NAV_MAX_ANGULAR_SPEED,
        min(config.NAV_MAX_ANGULAR_SPEED, config.NAV_ANGULAR_GAIN * error),
    )
    linear = 0.0
    if abs(error) <= config.NAV_YAW_TOLERANCE:
        linear = min(config.NAV_MAX_LINEAR_SPEED, config.NAV_LINEAR_GAIN * distance)
    return linear, angular, False


def square_waypoints(side):
    """Return four square corners in the starting frame."""
    return [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]


def star_waypoints(radius):
    """Return one closed five-point star, bottom tip at the starting origin."""
    vertices = [
        (radius * math.cos(-math.pi / 2 + i * 2 * math.pi / 5),
         radius * math.sin(-math.pi / 2 + i * 2 * math.pi / 5) + radius)
        for i in range(5)
    ]
    return [vertices[i] for i in (2, 4, 1, 3)] + [(0.0, 0.0)]


def _lengths(points):
    """Return cumulative polyline lengths in meters."""
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1] + math.dist(a, b))
    return lengths


def _point_at(points, lengths, distance):
    """Interpolate a point by distance along a polyline."""
    distance = max(0.0, min(lengths[-1], distance))
    index = min(len(points) - 2, bisect_right(lengths, distance) - 1)
    fraction = (distance - lengths[index]) / (lengths[index + 1] - lengths[index])
    a, b = points[index:index + 2]
    return a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1])


def heart_points(width, samples_per_lobe=200):
    """Sample a closed heart by arc length; x forward, y left, bottom at zero.

    The first half traces the left lobe to the notch; the second half returns
    around the right lobe. The notch is at index samples_per_lobe. Width is
    the full lateral extent in meters (up to numerical sampling precision).
    """
    if not math.isfinite(width) or width <= 0:
        raise ValueError('heart width must be finite and positive')
    if isinstance(samples_per_lobe, bool) or not isinstance(samples_per_lobe, int):
        raise ValueError('samples_per_lobe must be an integer >= 10')
    if samples_per_lobe < 10:
        raise ValueError('samples_per_lobe must be an integer >= 10')
    lobes = []
    for section in range(2):
        dense = []
        for i in range(2001):
            t = math.pi * (1 + section + i / 2000)
            raw_x = 16 * math.sin(t) ** 3
            raw_y = (13 * math.cos(t) - 5 * math.cos(2 * t)
                     - 2 * math.cos(3 * t) - math.cos(4 * t))
            dense.append(((raw_y + 17) * width / 32, -raw_x * width / 32))
        lengths = _lengths(dense)
        lobes.append([_point_at(dense, lengths, lengths[-1] * i / samples_per_lobe)
                      for i in range(samples_per_lobe + 1)])
    points = lobes[0] + lobes[1][1:]
    points[0] = points[-1] = (0.0, 0.0)
    points[samples_per_lobe] = (22 * width / 32, 0.0)
    return points


class HeartPath:
    """Track two ordered heart lobes, with explicit advancement at the notch.

    Progress is projected only in a local forward window of the active lobe.
    Lookahead never crosses a sharp point. The caller must stop at the notch
    before calling advance_section, and stop permanently after the second lobe.
    """

    def __init__(self, width):
        self.points = heart_points(width)
        midpoint = (len(self.points) - 1) // 2
        self.sections = [self.points[:midpoint + 1], self.points[midpoint:]]
        self.lengths = [_lengths(points) for points in self.sections]
        self.section = 0
        self.distance = 0.0

    @property
    def remaining(self):
        """Return meters remaining in the current lobe."""
        return self.lengths[self.section][-1] - self.distance

    @property
    def progress(self):
        """Return fraction of the full route traversed in order."""
        completed = sum(lengths[-1] for lengths in self.lengths[:self.section])
        total = sum(lengths[-1] for lengths in self.lengths)
        return (completed + self.distance) / total

    @property
    def endpoint(self):
        """Return the active sharp point (notch or final bottom tip)."""
        return self.sections[self.section][-1]

    def point_at(self, distance):
        """Interpolate within the current lobe only."""
        return _point_at(self.sections[self.section], self.lengths[self.section], distance)

    def update(self, x, y, window):
        """Project measured pose locally; return distance from the tracked path."""
        points, lengths = self.sections[self.section], self.lengths[self.section]
        lower = max(0.0, self.distance - window / 4)
        upper = min(lengths[-1], self.distance + window)
        best_error, best_distance = float('inf'), self.distance
        for i, (a, b) in enumerate(zip(points, points[1:])):
            if lengths[i + 1] < lower or lengths[i] > upper:
                continue
            dx, dy = b[0] - a[0], b[1] - a[1]
            segment = lengths[i + 1] - lengths[i]
            fraction = ((x - a[0]) * dx + (y - a[1]) * dy) / (segment * segment)
            fraction = max(0.0, min(1.0, fraction))
            distance = max(lower, min(upper, lengths[i] + fraction * segment))
            fraction = (distance - lengths[i]) / segment
            error = math.hypot(x - a[0] - fraction * dx, y - a[1] - fraction * dy)
            if error < best_error:
                best_error, best_distance = error, distance
        self.distance = max(self.distance, best_distance)
        return best_error

    def target(self, lookahead):
        """Return a forward arc-length target, clamped at the sharp point."""
        return self.point_at(self.distance + lookahead)

    def section_reached(self, x, y, tolerance):
        """Require ordered progress and proximity, not proximity alone."""
        return self.remaining <= 2 * tolerance and math.dist((x, y), self.endpoint) <= tolerance

    def advance_section(self):
        """Begin the right lobe after the caller has stopped at the notch."""
        if self.section != 0:
            raise ValueError('heart already on final lobe')
        self.section = 1
        self.distance = 0.0
