"""Circle-owned settings, independent of shared Lab 1/Lab 2 configuration."""


class CircleConfig:
    """Keep circle defaults here without changing any teammate's configuration."""

    ROBOT_NAMESPACE = '/TTB10'
    ODOM_TOPIC = ROBOT_NAMESPACE + '/odom'
    CMD_VEL_TOPIC = ROBOT_NAMESPACE + '/cmd_vel'

    CONTROL_PERIOD = 0.1
    SENSOR_TIMEOUT = 0.5
    NAV_POSITION_TOLERANCE = 0.05
    NAV_MAX_LINEAR_SPEED = 0.20  # Fixed physical/safety bound, not a trial speed.
    NAV_MAX_ANGULAR_SPEED = 0.40  # Fixed angular bound (rad/s).

    CIRCLE_RADIUS = 0.75
    CIRCLE_TARGET_SPEED = 0.12  # Independent virtual target speed (m/s).
    CIRCLE_ROBOT_SPEED = 0.12  # Independent follower speed limit (m/s).
    CIRCLE_DURATION = 0.0  # >0: fixed seconds; 0: finish after target laps.
    CIRCLE_LOOKAHEAD = 0.30
    CIRCLE_LAPS = 1
    CIRCLE_DIRECTION = 1
    CIRCLE_FINISH_YAW_TOLERANCE = 0.25
    CIRCLE_MAX_RADIAL_ERROR = 0.35
