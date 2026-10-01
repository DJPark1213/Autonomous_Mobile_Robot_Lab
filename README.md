# AMR Lab One

ROS 2 Humble + TurtleBot 4 implementation of:

- **Task 1:** Joystick teleoperation
- **Task 2:** IR-based autonomous wandering
- **Task 3:** PID speed control
- **Task 4:** Integrated WANDER / CRUISE / TELEOP controller

---

## 1. Setup

After modifying the code:

```bash
cd ~/amr_ws
colcon build --packages-select py_amr_ttb
source install/setup.bash
```

For every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/amr_ws/install/setup.bash
```

---

## 2. How to Run

### Task 1 — Joystick Teleoperation

Run:

```bash
ros2 launch py_amr_ttb py_lab_one_joystick.launch.py
```

Controls:

- Left stick up/down (`Axis 1`) → forward/backward
- Right stick left/right (`Axis 3`) → turn left/right

Stop with `Ctrl+C`.

Main source:

```text
lab_one_joystick.py
└── joy_callback()
```

Key code:

```python
self.subscriber_ = self.create_subscription(
    Joy,
    '/TTB10/joy',
    self.joy_callback,
    qos_profile
)

self.publisher_ = self.create_publisher(
    Twist,
    '/TTB10/cmd_vel',
    10
)

def joy_callback(self, msg):
    self.axes = msg.axes
    self.linear_x = self.axes[1]
    self.angular_z = self.axes[3]

    new_msg = Twist()
    new_msg.linear.x = 0.7 * self.linear_x
    new_msg.angular.z = 0.7 * self.angular_z
    self.publisher_.publish(new_msg)
```

Data flow:

```text
/TTB10/joy
     ↓
joy_callback()
     ↓
Twist
     ↓
/TTB10/cmd_vel
     ↓
Robot
```

---

### Task 2 — Autonomous Wandering

#### Run controller only

```bash
ros2 launch py_amr_ttb task2_wander.launch.py
```

The robot repeatedly:

```text
Drive forward
     ↓
Detect obstacle with IR
     ↓
Stop
     ↓
Turn left/right
     ↓
Check again
     ↓
Continue driving
```

#### Run with plot and rosbag

```bash
bash ~/amr_ws/run_lab.sh 2
```

This starts the Task 2 controller, the plot helper, `rqt_plot`, and rosbag recording.

Stop the experiment from the same terminal with `Ctrl+C`.

#### Task 2 plot

The plot shows:

```text
ir_front_left
ir_front_center_left
ir_front_center_right
ir_front_right
threshold
```

Current IR configuration:

```python
IR_FRONT_SENSOR_INDICES = (1, 2, 3, 4)
IR_OBSTACLE_THRESHOLD = 35
IR_OBSTACLE_WHEN_ABOVE_THRESHOLD = True
```

Sensor order:

```text
0: left
1: front left
2: front center left
3: front center right
4: front right
5: right
```

An obstacle is detected if any selected front IR value is greater than or equal to the threshold.

Full plot topics:

```text
/TTB10/lab_plot/ir_front_left/data
/TTB10/lab_plot/ir_front_center_left/data
/TTB10/lab_plot/ir_front_center_right/data
/TTB10/lab_plot/ir_front_right/data
/TTB10/lab_plot/threshold/data
```

Main source files:

```text
ir_sensor.py
wander_behavior.py
lab_config.py
```

Obstacle detection:

```python
def obstacle_detected(self):
    values = self.values
    indices = self.config.IR_FRONT_SENSOR_INDICES

    if indices is not None:
        values = [self.values[index] for index in indices]

    threshold = float(self.config.IR_OBSTACLE_THRESHOLD)

    if self.config.IR_OBSTACLE_WHEN_ABOVE_THRESHOLD:
        return any(value >= threshold for value in values)

    return any(value <= threshold for value in values)
```

Wander state machine:

```python
class WanderState(Enum):
    DRIVE = 'drive'
    TURN = 'turn'
```

Core behavior:

```python
def command(self, now, ir_state):
    obstacle = ir_state.obstacle_detected()

    if self.state is WanderState.DRIVE:
        if obstacle:
            self._begin_random_turn(now)
            return Twist(), None

        output = Twist()
        output.linear.x = self.config.WANDER_FORWARD_SPEED
        return output, None

    if now >= self.turn_end_time:
        if obstacle:
            self._begin_random_turn(now)
            return Twist(), None

        self.state = WanderState.DRIVE
        return Twist(), None

    output = Twist()
    output.angular.z = (
        self.turn_direction * self.config.WANDER_TURN_SPEED
    )
    return output, None
```

Current wandering parameters:

```python
WANDER_FORWARD_SPEED = 0.3
WANDER_TURN_SPEED = 0.4
WANDER_TURN_MIN_TIME = 0.8
WANDER_TURN_MAX_TIME = 1.8
```

---

### Task 3 — PID Speed Control

#### Run controller only

```bash
ros2 launch py_amr_ttb task3_pid.launch.py
```

Default target speed:

```text
0.3 m/s
```

#### Run with plot and rosbag

```bash
bash ~/amr_ws/run_lab.sh 3
```

This starts the Task 3 PID controller, the plot helper, `rqt_plot`, and rosbag recording.

#### Task 3 plot

The plot shows:

```text
target_speed
measured_speed
command_speed
```

| Curve | Meaning |
|---|---|
| `target_speed` | Desired speed |
| `measured_speed` | Actual speed from `/TTB10/odom` |
| `command_speed` | PID output sent through `/TTB10/cmd_vel` |

Full plot topics:

```text
/TTB10/lab_plot/measured_speed/data
/TTB10/lab_plot/target_speed/data
/TTB10/lab_plot/command_speed/data
```

Main source files:

```text
lab_one_pid.py
pid_speed_controller.py
lab_config.py
```

ROS interface in `lab_one_pid.py`:

```python
self.publisher = self.create_publisher(
    Twist,
    self.config.CMD_VEL_TOPIC,
    10
)

self.odom_subscription = self.create_subscription(
    Odometry,
    self.config.ODOM_TOPIC,
    self.odom_callback,
    qos_profile_sensor_data
)
```

Measured speed:

```python
def odom_callback(self, message):
    self.measured_speed = message.twist.twist.linear.x
    self.last_odom_time = time.monotonic()
```

PID update:

```python
def update(self, setpoint, measurement, dt):
    error = setpoint - measurement

    self.integral += error * dt

    derivative = (
        error - self.previous_error
    ) / dt

    self.output += (
        self.config.KP * error
        + self.config.KI * self.integral
        + self.config.KD * derivative
    )

    self.output = max(
        self.min_output,
        min(self.max_output, self.output)
    )

    self.previous_error = error
    return self.output
```

Current PID parameters:

```python
KP = 0.1
KI = 0.001
KD = 0.03

PID_TARGET_SPEED = 0.3
PID_MIN_OUTPUT = 0.0
PID_MAX_OUTPUT = 0.4
```

Control loop:

```text
target_speed
      │
      ▼
    Error ◄──── measured_speed
      │
      ▼
     PID
      │
      ▼
command_speed
      │
      ▼
/TTB10/cmd_vel
```

Use the plot to inspect rise time, overshoot, oscillation, and steady-state error.

---

### Task 4 — Integrated Controller

Run:

```bash
ros2 launch py_amr_ttb py_lab_one_controller.launch.py
```

The controller starts in `STOP`.

Controls:

| Button | Function |
|---|---|
| `L1` | Select WANDER |
| `L2` | Select CRUISE |
| Hold `R1` | Temporary TELEOP override |

CRUISE target speeds:

| Button | Target speed |
|---|---:|
| Cross / X | 0.0 m/s |
| Square | 0.1 m/s |
| Triangle | 0.2 m/s |
| Circle | 0.4 m/s |

While `R1` is held, joystick control temporarily overrides the selected mode. Releasing `R1` returns to the previous WANDER or CRUISE mode.

Main source:

```text
lab_one_controller.py
```

Persistent modes:

```python
class Mode(Enum):
    STOP = 'stop'
    WANDER = 'wander'
    CRUISE = 'cruise'
```

Task 4 creates one final velocity publisher and three subscribers:

```python
self.publisher = self.create_publisher(
    Twist,
    self.config.CMD_VEL_TOPIC,
    10,
)

self.ir_subscription = self.create_subscription(
    IrIntensityVector,
    self.config.IR_TOPIC,
    self.ir_callback,
    qos_profile_sensor_data,
)

self.joy_subscription = self.create_subscription(
    Joy,
    self.config.JOY_TOPIC,
    self.joy_callback,
    qos_profile_sensor_data,
)

self.odom_subscription = self.create_subscription(
    Odometry,
    self.config.ODOM_TOPIC,
    self.odom_callback,
    qos_profile_sensor_data,
)
```

Mode and target-speed selection:

```python
def joy_callback(self, message):
    self.previous_buttons = self.buttons
    self.buttons = list(message.buttons)
    self.axes = list(message.axes)

    if self.pressed(self.config.BUTTON_R1):
        self.reset_pid()
        return

    if self.newly_pressed(self.config.BUTTON_L1):
        self.selected_mode = Mode.WANDER

    elif self.newly_pressed(self.config.BUTTON_L2):
        self.selected_mode = Mode.CRUISE

    if self.selected_mode is Mode.CRUISE:
        if self.newly_pressed(self.config.BUTTON_CROSS):
            new_target = 0.0
        elif self.newly_pressed(self.config.BUTTON_SQUARE):
            new_target = 0.1
        elif self.newly_pressed(self.config.BUTTON_TRIANGLE):
            new_target = 0.2
        elif self.newly_pressed(self.config.BUTTON_CIRCLE):
            new_target = 0.4
```

Final control arbitration:

```python
def controller_callback(self):
    now = self.now_seconds()

    if self.pressed(self.config.BUTTON_R1):
        command, warning = self.teleop_command(now)

    elif self.selected_mode is Mode.WANDER:
        command, warning = self.wander_command(now)

    elif self.selected_mode is Mode.CRUISE:
        command, warning = self.cruise_command(now, dt)

    else:
        command, warning = Twist(), None

    self.publisher.publish(command)
```

This is the key Task 4 design rule:

> Only `LabOneController` publishes the final `/TTB10/cmd_vel` command.

This prevents multiple behaviors from sending conflicting commands at the same time.

---

## 3. `run_lab.sh`

`run_lab.sh` is an experiment helper for Task 2 and Task 3.

Use:

```bash
bash ~/amr_ws/run_lab.sh 2
```

or:

```bash
bash ~/amr_ws/run_lab.sh 3
```

The script selects the correct launch file and plot curves:

```bash
case "$task" in
    2)
        launch_file="task2_wander.launch.py"
        curves=(ir_front_left ir_front_center_left
                ir_front_center_right ir_front_right threshold)
        ;;
    3)
        launch_file="task3_pid.launch.py"
        curves=(measured_speed target_speed command_speed)
        ;;
esac
```

It also creates simple `Float64` topics for plotting.

Task 2 plot helper:

```python
node.create_subscription(
    IrIntensityVector,
    C.IR_TOPIC,
    on_ir,
    qos_profile_sensor_data,
)
```

Task 3 plot helper:

```python
node.create_subscription(
    Odometry,
    C.ODOM_TOPIC,
    lambda msg: emit(
        "measured_speed",
        msg.twist.twist.linear.x
    ),
    qos_profile_sensor_data,
)

node.create_subscription(
    Twist,
    C.CMD_VEL_TOPIC,
    lambda msg: emit(
        "command_speed",
        msg.linear.x
    ),
    qos_profile_sensor_data,
)
```

The plot helper is only for visualization. It does not control robot motion.

---

## 4. ROS Topics

| Topic | Message Type | Purpose |
|---|---|---|
| `/TTB10/joy` | `sensor_msgs/Joy` | Joystick input |
| `/TTB10/ir_intensity` | `irobot_create_msgs/IrIntensityVector` | IR proximity data |
| `/TTB10/odom` | `nav_msgs/Odometry` | Odometry and measured speed |
| `/TTB10/cmd_vel` | `geometry_msgs/Twist` | Robot velocity command |

---

## 5. Publisher / Subscriber Summary

| Task | Subscribers | Publisher |
|---|---|---|
| Task 1 | `/TTB10/joy` | `/TTB10/cmd_vel` |
| Task 2 | `/TTB10/ir_intensity` | `/TTB10/cmd_vel` |
| Task 3 | `/TTB10/odom` | `/TTB10/cmd_vel` |
| Task 4 | `/TTB10/joy`, `/TTB10/ir_intensity`, `/TTB10/odom` | `/TTB10/cmd_vel` |

External ROS components:

```text
Joystick driver
    └── publishes /TTB10/joy

Robot IR sensors
    └── publish /TTB10/ir_intensity

Robot odometry
    └── publishes /TTB10/odom

Robot base
    └── subscribes to /TTB10/cmd_vel
```

---

## 6. Overall Architecture

```text
Task 1:
Joystick → Teleoperation → Robot

Task 2:
IR → Obstacle Detection → WanderBehavior → Robot

Task 3:
Odometry → PID → Robot

Task 4:

             Joystick
                │
IR ────────> LabOneController <──── Odometry
                │
                ▼
         /TTB10/cmd_vel
                │
                ▼
              Robot
```

---

## 7. Useful Debug Commands

Show running ROS nodes:

```bash
ros2 node list
```

Show available topics:

```bash
ros2 topic list
```

Check IR:

```bash
ros2 topic echo /TTB10/ir_intensity
```

Check odometry:

```bash
ros2 topic echo /TTB10/odom
```

Check velocity commands:

```bash
ros2 topic echo /TTB10/cmd_vel
```

Check which node is publishing robot commands:

```bash
ros2 topic info /TTB10/cmd_vel --verbose
```

Pay attention to:

```text
Publisher count
Node name
```

There should not be multiple unintended controllers publishing `/TTB10/cmd_vel` at the same time.
