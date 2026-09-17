"""Known teleoperation settings and shared configuration placeholders."""


class LabConfig:
    """Keep verified mappings; fill pending sections with the task owners."""

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

    # Task 2 TODO: wandering settings and calibrated IR threshold/indices.
    # The plan proposes 120; validate against the 0.1 m requirement first.
    IR_OBSTACLE_THRESHOLD = None

    # Task 3: required gains; TODO: implement PID and choose limits.
    KP = 0.1
    KI = 0.001
    KD = 0.03

    # Task 4 TODO: control timing, shared timeouts, and mode integration.
