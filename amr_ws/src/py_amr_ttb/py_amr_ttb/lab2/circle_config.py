"""Circle-owned settings, independent of shared Lab 1/Lab 2 configuration."""


class CircleConfig:
    """Keep circle defaults here without changing any teammate's configuration."""

    ROBOT_NAMESPACE = '/TTB10'
    ODOM_TOPIC = ROBOT_NAMESPACE + '/odom'
    CMD_VEL_TOPIC = ROBOT_NAMESPACE + '/cmd_vel'

    CONTROL_PERIOD = 0.1
    SENSOR_TIMEOUT = 0.5
    NAV_POSITION_TOLERANCE = 0.05
    NAV_MAX_LINEAR_SPEED = 0.20
    NAV_MAX_ANGULAR_SPEED = 0.40

    CIRCLE_RADIUS = 0.75
    CIRCLE_SPEED = 0.12
    CIRCLE_LOOKAHEAD = 0.30
    CIRCLE_LAPS = 1
    CIRCLE_DIRECTION = 1
    CIRCLE_FINISH_YAW_TOLERANCE = 0.25
    CIRCLE_MAX_RADIAL_ERROR = 0.35

    # Initial PID gains require validation on the physical robot.
    CONTROL_DEFAULTS = {
        'speed_kp': 0.1, 'speed_ki': 0.01, 'speed_kd': 0.005,
        'acceleration': 0.15, 'align_gain': 2.0,
        'align_enter': 1.2, 'align_exit': 0.35,
    }
