"""Shared robot settings and independently tunable laboratory configurations."""

from dataclasses import dataclass, fields
import math


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
    NAV_POSITION_TOLERANCE = 0.05
    NAV_YAW_TOLERANCE = 0.10
    NAV_MAX_LINEAR_SPEED = 0.40
    NAV_MAX_ANGULAR_SPEED = 0.40
    NAV_LINEAR_GAIN = 1.0
    NAV_ANGULAR_GAIN = 2.0
    SQUARE_SIDE_LENGTH = 1.0
    # Star circumradius (m); each traveled edge is 2*sin(2*pi/5)*radius.
    STAR_RADIUS = 0.45

    # Heart: full lateral width, continuous tracking with a stop at the notch.
    HEART_WIDTH = 1.0
    HEART_SPEED = 0.40
    HEART_LOOKAHEAD = 0.12
    HEART_POSITION_TOLERANCE = 0.03
    HEART_CUSP_STOP_TIME = 0.30
    HEART_MAX_PATH_ERROR = 0.20

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


@dataclass
class Lab3Config:
    """Competition settings; motion response defaults require robot calibration."""

    robot_namespace: str = LabConfig.ROBOT_NAMESPACE
    base_frame: str = 'base_link'
    run_mode: str = 'learn'
    run_file: str = '~/amr_lab3_runs/course.npz'
    log_dir: str = '~/amr_lab3_runs'
    goal_x: float = 10.0
    goal_y: float = 0.0
    goal_tolerance: float = 0.12
    max_linear: float = 0.30
    max_angular: float = 1.4
    reverse_speed: float = 0.08
    linear_acceleration: float = 0.4
    linear_braking: float = 0.5
    angular_acceleration: float = 1.8
    angular_braking: float = 2.2
    command_latency: float = 0.10
    lateral_acceleration: float = 0.35
    robot_radius: float = 0.18
    safety_margin: float = 0.04
    control_period: float = 0.05
    map_period: float = 0.20
    decision_period: float = 0.50
    sensor_timeout: float = 0.40
    plan_timeout: float = 1.50
    prediction_step: float = 0.05
    prediction_horizon: float = 0.50
    scan_range: float = 6.0
    scan_known_fraction: float = 0.75
    pose_sync_tolerance: float = 0.12
    front_sections: int = 4
    rear_sections: int = 4
    heading_samples: int = 9
    scoring_horizon: float = 1.5
    target_distance: float = 1.0
    target_min_distance: float = 0.15
    target_reserve: float = 0.06
    target_reached: float = 0.18
    switch_margin: float = 0.10
    weight_goal: float = 0.40
    weight_travel: float = 0.30
    weight_clearance: float = 0.10
    weight_heading: float = 0.10
    weight_rotation: float = 0.10
    weight_history: float = 0.35
    heading_gain: float = 2.0
    rotate_threshold: float = 1.0
    wall_trigger: float = 0.40
    wall_distance: float = 0.35
    wall_gain: float = 3.0
    stall_timeout: float = 3.0
    progress_distance: float = 0.15
    backtrack_distance: float = 0.70
    attempt_radius: float = 0.45
    history_decay: float = 90.0
    revisit_window: float = 12.0
    revisit_min_age: float = 8.0
    revisit_radius: float = 0.20
    route_spacing: float = 0.10
    route_margin: float = 0.10
    route_heading_spacing: float = 0.20
    max_runtime: float = 300.0
    grid_resolution: float = 0.05
    grid_width: float = 24.0
    grid_height: float = 24.0
    grid_origin_x: float = -6.0
    grid_origin_y: float = -12.0
    occupied_increment: float = 0.90
    free_increment: float = 0.70
    evidence_limit: float = 4.0
    occupied_threshold: float = 0.60
    free_threshold: float = -0.40
    alignment_enabled: bool = True
    alignment_translation: float = 0.10
    alignment_rotation: float = 0.10
    alignment_min_hits: int = 25
    alignment_min_score: float = 0.65
    alignment_min_gain: float = 0.08
    alignment_ambiguity_margin: float = 0.04
    odom_translation_noise: float = 0.03
    odom_rotation_noise: float = 0.04
    alignment_position_noise: float = 0.04
    alignment_heading_noise: float = 0.04
    innovation_limit: float = 11.34
    replay_start_search: float = 0.30
    local_search_cells: int = 2500

    def __post_init__(self):
        """Validate units, budgets, limits and independent task settings."""
        signed = {'goal_x', 'goal_y', 'grid_origin_x', 'grid_origin_y', 'free_threshold'}
        nonnegative = {'switch_margin', 'route_margin', 'command_latency', 'alignment_min_gain',
                       'alignment_ambiguity_margin'}
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                zero_allowed = field.name.startswith('weight_') or field.name in nonnegative
                invalid = value < 0 if zero_allowed else value <= 0
                if not math.isfinite(value) or (field.name not in signed and invalid):
                    raise ValueError(f'{field.name} must be finite and positive')
        for name in ('front_sections', 'rear_sections', 'heading_samples',
                     'alignment_min_hits', 'local_search_cells'):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f'{name} must be an integer')
        if self.run_mode not in ('learn', 'replay'):
            raise ValueError('run_mode must be learn or replay')
        if not self.robot_namespace.startswith('/') or self.robot_namespace.endswith('/'):
            raise ValueError('robot_namespace must be absolute without trailing slash')
        if self.max_linear > .31 or self.max_angular > 1.9:
            raise ValueError('Keep commands within TurtleBot 4 safety-enabled limits')
        if self.reverse_speed > self.max_linear or self.radius < .20:
            raise ValueError('Invalid reverse speed or footprint')
        if not 0 < self.scan_known_fraction <= 1 or self.free_threshold >= 0:
            raise ValueError('Invalid observation thresholds')
        if self.control_period * 2 > self.sensor_timeout:
            raise ValueError('sensor_timeout must span at least two control cycles')
        if self.target_min_distance >= self.target_distance or self.wall_distance <= self.radius:
            raise ValueError('Invalid target or wall clearance')
        if self.revisit_min_age >= self.revisit_window:
            raise ValueError('revisit_min_age must be less than revisit_window')
        if self.prediction_step > self.control_period:
            raise ValueError('prediction_step must not exceed control_period')
        if not (self.grid_origin_x <= self.goal_x < self.grid_origin_x+self.grid_width
                and self.grid_origin_y <= self.goal_y < self.grid_origin_y+self.grid_height):
            raise ValueError('Goal must be inside configured map bounds')
        if self.front_sections+self.rear_sections > 32 or not 2 <= self.heading_samples <= 33:
            raise ValueError('Section sampling exceeds the bounded planning budget')
        if self.grid_resolution > .10 or self.grid_width*self.grid_height/self.grid_resolution**2 > 1000000:
            raise ValueError('Grid must resolve gaps and fit the one-million-cell budget')

    @property
    def radius(self):
        """Inflated robot footprint in metres."""
        return self.robot_radius + self.safety_margin

    def topic(self, suffix):
        """Build every topic from the effective namespace."""
        return self.robot_namespace + '/' + suffix.lstrip('/')
