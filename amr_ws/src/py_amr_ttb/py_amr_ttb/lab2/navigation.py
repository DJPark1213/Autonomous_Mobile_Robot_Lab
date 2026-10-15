"""Pure waypoint math shared by Lab 2 nodes and offline plotting."""

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


def shape_waypoints(radius):
    """Return one closed five-point star, bottom tip at the starting origin."""
    vertices = [
        (radius * math.cos(-math.pi / 2 + i * 2 * math.pi / 5),
         radius * math.sin(-math.pi / 2 + i * 2 * math.pi / 5) + radius)
        for i in range(5)
    ]
    return [vertices[i] for i in (2, 4, 1, 3)] + [(0.0, 0.0)]
