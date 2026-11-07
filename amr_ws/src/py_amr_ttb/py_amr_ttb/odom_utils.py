"""Make odometry usable as a consistent local reference across robot tasks.

Pose conversion anchors coordinates at task start; angle wrapping and input
checks prevent representation errors from distorting navigation. These helpers
change coordinate representation, not the sensor's accumulated drift.
"""

import math


def normalize_angle(angle):
    """Keep angular differences on the short turn across the +/-pi boundary.

    Return an equivalent angle in radians in [-pi, pi], so a small crossing of
    the wrap boundary does not look like an almost-full-circle turn.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    """Extract the ground-plane heading needed by planar navigation.

    q must be a unit ROS quaternion. Return radians; use read_planar_pose when
    incoming odometry also needs normalization and validity checks.
    """
    yaw_sine = 2.0 * (q.w * q.z + q.x * q.y)
    yaw_cosine = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(yaw_sine, yaw_cosine)


def relative_pose(x, y, yaw, initial_x, initial_y, initial_yaw):
    """Anchor a task to its starting location regardless of absolute odometry coordinates.

    Return (x forward, y left, yaw counterclockwise) in the fixed initial robot
    frame. Positions use metres and angles radians. Capture the initial pose after
    any external reset and keep it fixed for the run; later odometry drift remains.
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
    """Reject unusable odometry before it enters pose tracking and control.

    Extract finite x/y and normalize the orientation quaternion before deriving
    yaw. Return (x, y, yaw), or raise ValueError for invalid data. The incoming
    ROS message is preserved.
    """
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
    """Gate motion when odometry is missing, old or has an invalid time relationship.

    now and last_received use monotonic seconds. Return True only for an age from
    zero through timeout; callers may adjust last_received for acquisition delay.
    """
    return last_received is not None and 0 <= now - last_received <= timeout
