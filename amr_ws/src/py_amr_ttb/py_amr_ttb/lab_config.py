"""Shared settings for Tasks 1, 2, 3, and 4."""


class LabConfig:
    """Robot topics, joystick mappings, and controller settings."""

    ROBOT_NAMESPACE = '/TTB10'

    JOY_TOPIC = ROBOT_NAMESPACE + '/joy'
    ODOM_TOPIC = ROBOT_NAMESPACE + '/odom'
    IR_TOPIC = ROBOT_NAMESPACE + '/ir_intensity'
    CMD_VEL_TOPIC = ROBOT_NAMESPACE + '/cmd_vel'

    # Joystick axes.
    LEFT_STICK_HORIZONTAL_AXIS = 0
    LEFT_STICK_VERTICAL_AXIS = 1
    RIGHT_STICK_HORIZONTAL_AXIS = 3
    RIGHT_STICK_VERTICAL_AXIS = 4

    FORWARD_AXIS = LEFT_STICK_VERTICAL_AXIS
    TURN_AXIS = RIGHT_STICK_HORIZONTAL_AXIS

    # Joystick buttons.
    BUTTON_CROSS = 0
    BUTTON_CIRCLE = 1
    BUTTON_TRIANGLE = 2
    BUTTON_SQUARE = 3
    BUTTON_L1 = 4
    BUTTON_R1 = 5
    BUTTON_L2 = 6
    BUTTON_R2 = 7

    # Task 1: teleoperation.
    TELEOP_LINEAR_SCALE = 0.5
    TELEOP_ANGULAR_SCALE = 0.2
    JOYSTICK_TIMEOUT = 0.5

    # Controller timing.
    CONTROL_PERIOD = 0.1
    SENSOR_TIMEOUT = 0.5

    # Task 2: autonomous wandering.
    WANDER_FORWARD_SPEED = 0.3
    WANDER_TURN_SPEED = 0.4
    WANDER_TURN_MIN_TIME = 0.8
    WANDER_TURN_MAX_TIME = 1.8
    WANDER_CLEAR_SAMPLES = 3

    # Lab 2: odometry-relative goal.
    GOAL_X_OFFSET = 1.5
    GOAL_Y_OFFSET = 1.5

    # Sensor order observed in this robot's IR messages:
    # 0: ir_intensity_side_left
    # 1: ir_intensity_left
    # 2: ir_intensity_front_left
    # 3: ir_intensity_front_center_left
    # 4: ir_intensity_front_center_right
    # 5: ir_intensity_front_right
    # 6: ir_intensity_right
    IR_FRONT_SENSOR_INDICES = (2, 3, 4, 5)

    # Trial intensity thresholds, not distances in meters.
    IR_OBSTACLE_THRESHOLD = 15
    IR_CLEAR_THRESHOLD = 10
    IR_OBSTACLE_WHEN_ABOVE_THRESHOLD = True

    # Task 3: PID gains.
    KP = 0.1
    KI = 0.001
    KD = 0.03

    PID_TARGET_SPEED = 0.3
    PID_INTEGRAL_LIMIT = 1.0
    PID_MIN_OUTPUT = 0.0
    PID_MAX_OUTPUT = 0.4

    # Shared cruise-control limit.
    CRUISE_MAX_COMMAND = 0.5
