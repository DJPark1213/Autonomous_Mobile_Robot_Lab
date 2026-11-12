"""Keep robot wiring and tuning choices consistent across each lab's components.

Legacy exercises use LabConfig constants. Lab 3 shares a validated Lab3Config
instance across sensing, planning, motion and persistence so geometry and limits
agree throughout a run.
"""

from dataclasses import dataclass, fields
import math


class LabConfig:
    """Centralize topics and legacy exercise tuning so controllers use the same robot setup."""

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
    """Keep navigation assumptions aligned from scan acceptance through stopping checks.

    The ROS adapter exposes these fields as parameters; simulation uses the same
    settings directly. Distances are metres, angles radians and durations seconds;
    linear/angular speeds and accelerations use the corresponding per-second units.
    Goal coordinates refer to the learned map's starting frame. Response limits and
    noise values are tuning assumptions that require calibration, not sensor facts.
    """

    # Bind sensing and commands to one robot and choose learning versus replay.
    robot_namespace: str = LabConfig.ROBOT_NAMESPACE
    base_frame: str = 'base_link'
    run_mode: str = 'learn'
    run_file: str = '~/amr_lab3_runs/course.npz'
    log_dir: str = '~/amr_lab3_runs'
    # Completion requires this map location; open space alone is not an exit.
    goal_x: float = 10.0
    goal_y: float = 0.0
    goal_tolerance: float = 0.12
    # Command limits and calibrated response govern the predicted stopping path.

    # Forward speed cap (m/s); raise gradually after braking checks.
    max_linear: float = 0.46

    max_angular: float = 2.46
    reverse_speed: float = 0.08
    linear_acceleration: float = 0.4
    linear_braking: float = 0.5
    angular_acceleration: float = 1.8
    angular_braking: float = 2.2
    command_latency: float = 0.10
    lateral_acceleration: float = 0.35
    # Clearance uses the physical body plus allowance for imperfect geometry.
    robot_radius: float = 0.18
    safety_margin: float = 0.01
    # Keep command checks frequent while bounding slower map/decision work.
    control_period: float = 0.05
    map_period: float = 0.20
    # Stale observations or decisions must expire rather than authorize motion.
    sensor_timeout: float = 0.40
    plan_timeout: float = 1.50
    # Resolve predicted movement before extending it through braking to rest.
    prediction_step: float = 0.05
    prediction_horizon: float = 0.50
    # Limit usable sensor distance and reject scans with too much missing data.
    scan_range: float = 6.0
    scan_known_fraction: float = 0.75
    # Bound how far pose_at may extrapolate to place a laser measurement in time.
    pose_sync_tolerance: float = 0.12
    # Bound scan-local route search and acceleration-aware trajectory sampling.
    local_extent: float = 2.4
    local_spacing: float = 0.05
    local_lookahead: float = 0.65
    trajectory_horizon: float = 0.70
    angular_response_fraction: float = 0.75
    heading_gain: float = 2.0
    # Bound saved-route lookahead during replay.
    target_distance: float = 1.0
    # Detect unproductive motion and bound measured reverse travel.
    stall_timeout: float = 3.0
    progress_distance: float = 0.15
    backtrack_distance: float = 0.70
    # Remember failures locally, with decay so a direction can become useful again.
    attempt_radius: float = 0.45
    history_decay: float = 90.0
    # Retain route detail for replay and reserve extra room for shortcut checks.
    route_spacing: float = 0.10
    route_margin: float = 0.10
    route_heading_spacing: float = 0.20
    # Bound runs that cannot reach the goal, including repeated waiting/recovery.
    max_runtime: float = 300.0
    # Allocate a finite map in metres; resolution controls gap representation.
    grid_resolution: float = 0.05
    grid_width: float = 24.0
    grid_height: float = 24.0
    grid_origin_x: float = -6.0
    grid_origin_y: float = -12.0
    # Repeated observations build bounded evidence; unknown starts at zero.
    # Separate positive/negative thresholds leave uncertain cells unclassified.
    occupied_increment: float = 0.90
    free_increment: float = 0.70
    evidence_limit: float = 4.0
    occupied_threshold: float = 0.60
    free_threshold: float = -0.40
    # Permit local drift correction only when wall agreement is convincing.
    alignment_enabled: bool = True
    alignment_translation: float = 0.10
    alignment_rotation: float = 0.10
    alignment_min_hits: int = 25
    alignment_min_score: float = 0.65
    alignment_min_gain: float = 0.08
    alignment_ambiguity_margin: float = 0.04
    # Weight odometry prediction against scan matches and reject unlikely jumps.
    odom_translation_noise: float = 0.03
    odom_rotation_noise: float = 0.04
    alignment_position_noise: float = 0.04
    alignment_heading_noise: float = 0.04
    innovation_limit: float = 11.34
    # Replay needs a wider initial match.
    replay_start_search: float = 0.30

    def __post_init__(self):
        """Reject incompatible settings before mapping or motion starts using them.

        Check finite values, frame naming, footprint/clearance relationships and bounded
        sampling/grid costs. These consistency checks do not calibrate the robot or
        prove that the configured response model matches hardware.
        """
        signed = {'goal_x', 'goal_y', 'grid_origin_x', 'grid_origin_y', 'free_threshold'}
        nonnegative = {'route_margin', 'command_latency', 'alignment_min_gain',
                       'alignment_ambiguity_margin'}
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                zero_allowed = field.name in nonnegative
                invalid = value < 0 if zero_allowed else value <= 0
                if not math.isfinite(value) or (field.name not in signed and invalid):
                    raise ValueError(f'{field.name} must be finite and positive')
        for name in ('alignment_min_hits',):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f'{name} must be an integer')
        if self.run_mode not in ('learn', 'replay'):
            raise ValueError('run_mode must be learn or replay')
        if not self.robot_namespace.startswith('/') or self.robot_namespace.endswith('/'):
            raise ValueError('robot_namespace must be absolute without trailing slash')
        if self.max_linear > .46 or self.max_angular > 2.46:
            raise ValueError('Commands exceed configured platform limits (0.46 m/s, 2.46 rad/s)')
        if self.reverse_speed > self.max_linear or self.robot_radius < .18 or self.safety_margin < .01:
            raise ValueError('Invalid reverse speed or footprint')
        if not 0 < self.scan_known_fraction <= 1 or self.free_threshold >= 0:
            raise ValueError('Invalid observation thresholds')
        if self.control_period * 2 > self.sensor_timeout:
            raise ValueError('sensor_timeout must span at least two control cycles')
        if not .04 <= self.local_spacing <= .1 or (2*math.ceil(self.local_extent/self.local_spacing)+1)**2 > 12000:
            raise ValueError('Local search exceeds its sampling budget')
        if self.local_lookahead >= self.local_extent or not 0 < self.angular_response_fraction <= 1:
            raise ValueError('Invalid local lookahead or response fraction')
        if self.prediction_step > self.control_period:
            raise ValueError('prediction_step must not exceed control_period')
        if not (self.grid_origin_x <= self.goal_x < self.grid_origin_x+self.grid_width
                and self.grid_origin_y <= self.goal_y < self.grid_origin_y+self.grid_height):
            raise ValueError('Goal must be inside configured map bounds')
        if self.grid_resolution > .10 or self.grid_width*self.grid_height/self.grid_resolution**2 > 1000000:
            raise ValueError('Grid must resolve gaps and fit the one-million-cell budget')

    @property
    def radius(self):
        """Give all clearance checks the same body radius plus a safety allowance."""
        return self.robot_radius + self.safety_margin

    def topic(self, suffix):
        """Keep sensing and command topics on the selected robot namespace."""
        return self.robot_namespace + '/' + suffix.lstrip('/')
