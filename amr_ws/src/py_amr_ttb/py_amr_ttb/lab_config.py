"""Shared robot settings for teleoperation, wandering, and cruise."""


class LabConfig:
    """Hold robot mappings, calibration, timing, and controller limits."""

    # Task 4: known robot namespace and joystick mapping; verify axis signs.
    ROBOT_NAMESPACE = '/TTB10'
    JOY_TOPIC = ROBOT_NAMESPACE + '/joy'
    ODOM_TOPIC = ROBOT_NAMESPACE + '/odom'
    IR_TOPIC = ROBOT_NAMESPACE + '/ir_intensity'
    CMD_VEL_TOPIC = ROBOT_NAMESPACE + '/cmd_vel'

    LEFT_STICK_HORIZONTAL_AXIS = 0
    LEFT_STICK_VERTICAL_AXIS = 1
    RIGHT_STICK_HORIZONTAL_AXIS = 3
    RIGHT_STICK_VERTICAL_AXIS = 4
    FORWARD_AXIS = LEFT_STICK_VERTICAL_AXIS
    TURN_AXIS = RIGHT_STICK_HORIZONTAL_AXIS
    BUTTON_CROSS = 0
    BUTTON_CIRCLE = 1
    BUTTON_TRIANGLE = 2
    BUTTON_SQUARE = 3
    BUTTON_L1 = 4
    BUTTON_R1 = 5
    BUTTON_L2 = 6
    BUTTON_R2 = 7
    TELEOP_LINEAR_SCALE = 0.5
    TELEOP_ANGULAR_SCALE = 0.2

    WANDER_FORWARD_SPEED = 0.3
    WANDER_TURN_SPEED = 0.4
    WANDER_TURN_MIN_TIME = 0.8
    WANDER_TURN_MAX_TIME = 1.8
    SENSOR_TIMEOUT = 0.5
    CONTROL_PERIOD = 0.1
    JOYSTICK_TIMEOUT = 0.5

    # Confirmed TurtleBot 4 IrIntensityVector data and sensor order.
    # The overall message frame is `base_link`:
    #   0: ir_intensity_left
    #   1: ir_intensity_front_left
    #   2: ir_intensity_front_center_left
    #   3: ir_intensity_front_center_right
    #   4: ir_intensity_front_right
    #   5: ir_intensity_right
    # Only the four forward-facing readings control obstacle detection.
    # - Clear path: every sensor was below 7.
    # - Object about 0.1 m ahead: the four front readings were
    #   [176, 155, 220, 66].
    IR_OBSTACLE_THRESHOLD = 35
    IR_OBSTACLE_WHEN_ABOVE_THRESHOLD = True
    IR_FRONT_SENSOR_INDICES = (1, 2, 3, 4)

    # Required gains and supplied implementation limits; verify on the robot.
    KP = 0.1
    KI = 0.001
    KD = 0.03

    PID_INTEGRAL_LIMIT = 1.0
    CRUISE_MAX_COMMAND = 0.5
