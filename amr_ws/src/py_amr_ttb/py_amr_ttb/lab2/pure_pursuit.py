"""Circle geometry and differential-drive pure pursuit, without ROS imports."""

import math

from py_amr_ttb.lab2.circle_odom import normalize_angle


class CirclePath:
    """Start at (0, 0), tangent to +x; direction +1 is counterclockwise."""

    def __init__(self, radius, direction=1):
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('radius must be finite and positive')
        if direction not in (-1, 1):
            raise ValueError('direction must be +1 (CCW) or -1 (CW)')
        self.radius = radius
        self.direction = direction

    def point(self, phase):
        """Return a route point at directed phase in radians."""
        return (self.radius * math.sin(phase),
                self.direction * self.radius * (1 - math.cos(phase)))

    def phase(self, x, y):
        """Return directed polar phase in [-pi, pi], zero at the start."""
        return math.atan2(x, self.radius - self.direction * y)

    def radial_error(self, x, y):
        """Return signed distance from the circumference, positive outside."""
        return math.hypot(x, y - self.direction * self.radius) - self.radius

    def target(self, x, y, lookahead):
        """Choose the forward circle/lookahead-circle intersection.

        If the circles do not intersect, advance from the nearest path point
        by the nominal chord angle. The steering denominator always uses the
        actual target distance, including in that recovery case.
        """
        if not math.isfinite(lookahead) or not 0 < lookahead < 2 * self.radius:
            raise ValueError('lookahead must be positive and less than 2 * radius')
        rho = math.hypot(x, y - self.direction * self.radius)
        advance = 2 * math.asin(lookahead / (2 * self.radius))
        if rho > 1e-12:
            cosine = (rho * rho + self.radius ** 2 - lookahead ** 2) / (
                2 * rho * self.radius)
            if -1 <= cosine <= 1:
                advance = math.acos(cosine)
        return self.point(self.phase(x, y) + advance)


class CircleProgress:
    """Unwrap measured directed phase; reversing subtracts from progress."""

    def __init__(self, path):
        self.path = path
        self.previous_phase = None
        self.radians = 0.0

    def update(self, x, y):
        """Accumulate signed motion around the center from odometry positions."""
        phase = self.path.phase(x, y)
        if self.previous_phase is not None:
            self.radians += normalize_angle(phase - self.previous_phase)
        self.previous_phase = phase
        return self.radians


def pursuit_command(x, y, yaw, target_x, target_y, speed, max_linear, max_angular):
    """Return (v, omega), with x forward/y left in the current robot frame.

    Curvature kappa = 2*y_body / (x_body**2 + y_body**2), omega = v*kappa.
    Reduce v if needed to respect the angular limit while retaining curvature.
    Turn in place when the target is behind, then resume forward tracking.
    """
    dx, dy = target_x - x, target_y - y
    c, s = math.cos(yaw), math.sin(yaw)
    forward = c * dx + s * dy
    left = -s * dx + c * dy
    distance_squared = dx * dx + dy * dy
    if distance_squared < 1e-12:
        return 0.0, 0.0
    if forward <= 0:
        angle = math.atan2(left, forward)
        return 0.0, max(-max_angular, min(max_angular, 2.0 * angle))
    curvature = 2.0 * left / distance_squared
    linear = min(speed, max_linear)
    if abs(curvature) > 1e-12:
        linear = min(linear, max_angular / abs(curvature))
    return linear, linear * curvature
