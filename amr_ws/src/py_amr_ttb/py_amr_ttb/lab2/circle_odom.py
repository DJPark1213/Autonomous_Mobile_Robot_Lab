"""Pose conversion into a fixed frame aligned with the robot at task start."""

import math


def normalize_angle(angle):
    """Wrap a heading in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    """Return planar heading in radians from a unit ROS quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def relative_pose(x, y, yaw, initial_x, initial_y, initial_yaw):
    """Return (x forward, y left, yaw CCW) in the fixed starting frame.

    Positions are in meters and angles in radians. Capture the initial pose
    after any external odometry reset and keep it fixed for the entire run.
    """
    dx = x - initial_x
    dy = y - initial_y
    c = math.cos(initial_yaw)
    s = math.sin(initial_yaw)
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
        normalize_angle(yaw - initial_yaw),
    )


def read_planar_pose(message):
    """Read finite x/y/yaw, normalizing the quaternion; reject invalid poses."""
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    values = (p.x, p.y, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite odometry pose')
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError('Invalid odometry quaternion')
    x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return p.x, p.y, yaw


def odometry_is_fresh(now, last_received, timeout):
    """Check receipt age using monotonic seconds, including missing data."""
    return last_received is not None and 0 <= now - last_received <= timeout
