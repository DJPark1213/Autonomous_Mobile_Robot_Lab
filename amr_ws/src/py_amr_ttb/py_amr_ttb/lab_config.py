"""Shared robot settings for teleoperation, wandering, and cruise."""


class LabConfig:
    """Hold robot mappings, calibration, timing, and controller limits."""


    ROBOT_NAMESPACE = '/TTB10'

    JOY_TOPIC = ROBOT_NAMESPACE + '/joy'
    ODOM_TOPIC = ROBOT_NAMESPACE + '/odom'
    IR_TOPIC = ROBOT_NAMESPACE + '/ir_intensity'
    CMD_VEL_TOPIC = ROBOT_NAMESPACE + '/cmd_vel'

    # Joystick axis mappings
    LEFT_STICK_HORIZONTAL_AXIS = 0
    LEFT_STICK_VERTICAL_AXIS = 1
    RIGHT_STICK_HORIZONTAL_AXIS = 3
    RIGHT_STICK_VERTICAL_AXIS = 4

    FORWARD_AXIS = LEFT_STICK_VERTICAL_AXIS
    TURN_AXIS = RIGHT_STICK_HORIZONTAL_AXIS

    # Joystick button mappings
    BUTTON_CROSS = 0
    BUTTON_CIRCLE = 1
    BUTTON_TRIANGLE = 2
    BUTTON_SQUARE = 3
    BUTTON_L1 = 4
    BUTTON_R1 = 5
    BUTTON_L2 = 6
    BUTTON_R2 = 7

    # Teleoperation settings
    TELEOP_LINEAR_SCALE = 0.5
    TELEOP_ANGULAR_SCALE = 0.2

    # Task 2 autonomous wandering settings
    WANDER_FORWARD_SPEED = 0.3
    WANDER_TURN_SPEED = 0.4
    WANDER_TURN_MIN_TIME = 0.8
    WANDER_TURN_MAX_TIME = 1.8

    # Node timing settings
    SENSOR_TIMEOUT = 0.5
    CONTROL_PERIOD = 0.1
    JOYSTICK_TIMEOUT = 0.5

    # TurtleBot 4 IR sensor order:
    # 0: left
    # 1: front left
    # 2: front center left
    # 3: front center right
    # 4: front right
    # 5: right
    #
    # Clear readings were below 7.
    # Readings near an obstacle at approximately 0.1 m were much higher.
    IR_FRONT_SENSOR_INDICES = (1, 2, 3, 4)
    IR_OBSTACLE_THRESHOLD = 35
    IR_OBSTACLE_WHEN_ABOVE_THRESHOLD = True

    # Required gains and supplied implementation limits; verify on the robot.
    KP = 0.1
    KI = 0.001
    KD = 0.03

    PID_INTEGRAL_LIMIT = 1.0
    CRUISE_MAX_COMMAND = 0.5
    # Task 3 target speed and output limits
    PID_TARGET_SPEED = 0.3
    PID_MIN_OUTPUT = 0.0
    PID_MAX_OUTPUT = 0.4
