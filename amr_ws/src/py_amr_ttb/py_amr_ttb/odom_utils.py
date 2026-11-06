"""Pose conversion into a fixed frame aligned with the robot at task start."""

import math


def normalize_angle(angle):
    """Wrap a heading in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    """Return planar heading in radians from a unit ROS quaternion."""
    yaw_sine = 2.0 * (q.w * q.z + q.x * q.y)
    yaw_cosine = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(yaw_sine, yaw_cosine)


def relative_pose(x, y, yaw, initial_x, initial_y, initial_yaw):
    """Return (x forward, y left, yaw CCW) in the fixed starting frame.

    Positions are in meters and angles in radians. Capture the initial pose
    after any external odometry reset and keep it fixed for the entire run.
    """
    offset_x = x - initial_x
    offset_y = y - initial_y
    cosine = math.cos(initial_yaw)
    sine = math.sin(initial_yaw)
    # Rotate the world-frame displacement back into the initial robot axes.
    return (
        cosine * offset_x + sine * offset_y,
        -sine * offset_x + cosine * offset_y,
        normalize_angle(yaw - initial_yaw),
    )


def read_planar_pose(message):
    """Read finite x/y/yaw, normalizing the quaternion; reject invalid poses."""
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    pose_values = (position.x, position.y, *quaternion)
    if not all(math.isfinite(value) for value in pose_values):
        raise ValueError('Non-finite odometry pose')

    norm = math.hypot(*quaternion)
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError('Invalid odometry quaternion')

    # The normalized quaternion is local data; never mutate the ROS message.
    x, y, z, w = (value / norm for value in quaternion)
    yaw_sine = 2 * (w * z + x * y)
    yaw_cosine = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(yaw_sine, yaw_cosine)
    return position.x, position.y, yaw


def odometry_is_fresh(now, last_received, timeout):
    """Check receipt age using monotonic seconds, including missing data."""
    return last_received is not None and 0 <= now - last_received <= timeout
