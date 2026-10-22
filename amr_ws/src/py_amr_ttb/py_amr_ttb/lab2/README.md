# Lab 2 developer and demonstration guide

Heart tracking is now implemented as a separate task with pathing in
`navigation.py`. See [HEART_README.md](HEART_README.md) for configuration,
`ros2 run py_amr_ttb lab2_heart`, recording/plotting and robot checks.
Heart robot validation is pending; existing tasks retain their behavior.

Goal, square, DJ's five-point star, and Jianyu's circle pursuit are implemented.
The team reports star movement; saved star trajectory/acceptance evidence remains
pending. Circle CSV recordings are under package `lab_runs/circle/`.
The original Word plan predates this implementation; see
[the current handoff](../../../../../docs/LAB2_HANDOFF.md).

## Starting frame and reset

The first valid odometry pose defines (0 m, 0 m, 0 rad). Positive x points
along the initial heading, positive y points left, and positive yaw is
counterclockwise. These axes stay fixed. Goal (1.5, 1.5) means forward 1.5 m
and left 1.5 m from the start. Square uses (L, 0), (L, L), (0, L), (0, 0).
The star starts at its bottom tip, visits alternating outer tips, and returns
to (0, 0); current radius is 0.45 m. It uses stop/turn/go, not circle pursuit.

An odometry reset is optional: nonzero raw odometry works too. Stop all motion
controllers, keep the robot still, optionally reset with its supported bash
service call, wait for fresh odometry, then launch the task. Stop/relaunch to
capture a new origin. Never reset odometry while a task runs.

On the robot, inspect available services and their actual type before calling:

```bash
ros2 service list -t
# For the discovered reset service:
ros2 service type /ACTUAL_SERVICE_NAME
ros2 interface show ACTUAL_PACKAGE/srv/ACTUAL_TYPE
```

No reset service endpoint is assumed or called by our nodes. The
[Create 3 documentation](https://iroboteducation.github.io/create3_docs/api/ros2/)
describes service discovery. Firmware supports different interfaces; verify the
installed robot. Plain bash `reset` only resets the terminal.

## What moved and how teammates use it

| File/helper | Interface and purpose |
| --- | --- |
| `py_amr_ttb/odom_utils.py` | Pure pose math; no ROS runtime import |
| `quaternion_to_yaw(q)` | Unit quaternion -> yaw radians; general math helper |
| `read_planar_pose(message)` | Odometry -> finite `(x, y, yaw)`; normalizes quaternion, raises ValueError on invalid pose |
| `relative_pose(x, y, yaw, initial_x, initial_y, initial_yaw)` | World pose -> fixed starting-frame `(x, y, yaw)`; meters/radians |
| `normalize_angle(angle)` | Wrap radians into [-pi, pi] |
| `odometry_is_fresh(now, last_received, timeout)` | Monotonic receipt seconds -> boolean; None means no valid pose |
| `lab2/navigation.py: waypoint_command(...)` | `(x, y, yaw, target_x, target_y, config)` -> `(linear, angular, reached)`; caller publishes Twist |
| `square_waypoints(side)` / `star_waypoints(radius)` | Starting-frame coordinates for control and intended-route plotting |
| `lab_config.py` | Topics, timing, NAV_* settings, GOAL_* offsets, SQUARE_SIDE_LENGTH, STAR_RADIUS |

Each node owns its publisher/subscriber/timer and runtime state explicitly in
`__init__`. Initial/current pose, last receipt time, waypoint index, and completion
state remain on the node. There is no shared ROS base class. Shared math has
meaningful offline tests and can be reused by circle without taking its control
logic away from Jianyu.

The odometry callback validates pose, captures the origin once, transforms the
pose, updates receipt time, and publishes x/y/yaw/distance diagnostics. The timer
publishes zero before a valid pose, after completion, on invalid data, or after
SENSOR_TIMEOUT (0.5 s) without valid receipt. Valid data resumes an unfinished
route. Age is measured at local callback receipt, not the sensor timestamp;
this does not detect a publisher repeatedly sending a frozen pose. Shutdown
sends a zero command when the ROS context is still available.

Go-to-goal and square use the same distance/heading controller. It turns in
place outside the yaw tolerance; inside the tolerance it drives with heading
correction. Speed decreases with target distance. Reaching a corner publishes
zero and advances only one target. Arrival does not require a final heading.
Circle uses its own pursuit algorithm, path progress and lap detection; see
[CIRCLE_README.md](CIRCLE_README.md).

## Ubuntu build and launch

### Go-to-goal error and speed experiments

Go-to-goal uses proportional distance/heading control, without integral or
derivative terms. Its goal remains (1.5, 1.5) in the fixed starting frame.
The following startup parameters apply only to this node; square/star settings
are unchanged. Restart the node to apply a different experiment configuration.

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `linear_gain` | 1.0 | Forward speed per meter of distance error |
| `angular_gain` | 2.0 | Turning speed per radian of heading error |
| `max_linear_speed` | 0.4 | Forward command limit, m/s |
| `max_angular_speed` | 0.4 | Turning command limit, rad/s |
| `position_tolerance` | 0.05 | Arrival radius, m |
| `yaw_tolerance` | 0.10 | Forward motion allowed within this heading error, rad |

Start recording from the repository root while the robot is stationary, before
launching the controller in another sourced terminal:

```bash
python3 amr_ws/scripts/plot_lab2.py --record output/lab2/goal_slow01.csv
```

```bash
ros2 launch py_amr_ttb lab2_go_to_goal.launch.py max_linear_speed:=0.15
```

Keep recording briefly after arrival to capture residual movement, then stop
the recorder with Ctrl+C. Plot from the repository root:

```bash
python3 amr_ws/scripts/plot_lab2.py --csv output/lab2/goal_slow01.csv --route goal --output output/lab2/goal_slow01.png
```

The goal plot includes trajectory, distance and signed along-route error,
heading error, and commanded/measured forward speed. Negative signed remaining
distance means crossing the goal line perpendicular to the start-to-goal route;
it does not by itself establish accurate arrival. The JSON includes maximum
goal-line overshoot and minimum/final distance. Old pose-only CSVs still work;
missing heading/velocity panels are labeled. Velocity commands are the most
recent received values at each odometry sample, with NaN gaps before a command
or when its receipt age exceeds 0.5 s. They are not synchronized sensor samples.

Repeat from a stationary start using unique filenames and speed limits such as
0.15, 0.25 and 0.40 m/s, holding gains fixed. Then vary `linear_gain` with speed
limits fixed. Proportional slowing near the goal can suppress overshoot, so a
higher speed limit does not guarantee wobble. Arrival remains latched: the node
stays stopped after entering tolerance, even if momentum carries it outside.
Heading error near/after arrival may jump as the goal bearing changes; interpret
it with the trajectory and distance plot, rather than as turning motion alone.

Live Float64 diagnostics under `/TTB10/lab2_go_to_goal/` include `error_x`,
`error_y`, `heading_error`, `remaining_along_route`, `command_linear`,
`command_angular`, `measured_linear`, and `measured_angular`, in addition to
`x`, `y`, `yaw`, and `distance`. Append `/data` when plotting in rqt_plot.

### Build and run

Run from the ROS workspace on the robot laptop; adjust checkout paths as needed.

```bash
source /opt/ros/humble/setup.bash
cd ~/amr_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select py_amr_ttb
source install/setup.bash
# Select ONE controller:
ros2 launch py_amr_ttb lab2_go_to_goal.launch.py
# ros2 launch py_amr_ttb lab2_square.launch.py
# ros2 launch py_amr_ttb lab2_star.launch.py
```

Default topics are `/TTB10/odom` and `/TTB10/cmd_vel`. Run one controller at
a time. Check `ros2 topic info /TTB10/cmd_vel --verbose` and stop unintended
publishers. Stop a task with Ctrl+C. Launches for existing tasks are installed by
setup.py; circle launches its module through `lab2_circle.launch.py`. All launch files
are package-local under `amr_ws/src/py_amr_ttb/launch/`, matching setup.py.

In another sourced terminal, inspect time-series diagnostics:

```bash
ros2 run rqt_plot rqt_plot /TTB10/lab2_go_to_goal/x/data /TTB10/lab2_go_to_goal/y/data /TTB10/lab2_go_to_goal/distance/data
```

Square/star diagnostics use `lab2_square` / `lab2_star` in the same topic path.
The diagnostics only publish upon valid odometry; velocity stops still run on
the timer if odometry disappears. Save a screenshot of actual rqt_plot results.

## Record and plot measured trajectories

The recorder/plotter is a checkout script, not a ROS executable. Commands below
run from the repository root; keep `amr_ws/scripts` when copying the workspace.
Matplotlib is declared as the package's `python3-matplotlib` dependency. Standalone
plotting outside ROS needs matplotlib installed in that Python environment.

Start recording while stationary, wait for valid odometry, then launch the
controller in another terminal. Both capture the same stationary starting pose.
Use a unique run name to preserve earlier data. Ctrl+C ends CSV recording.

```bash
source /opt/ros/humble/setup.bash
source ~/amr_ws/install/setup.bash
python3 amr_ws/scripts/plot_lab2.py --record output/lab2/goal_run01.csv
# Optional: --duration 120 (seconds), --topic /TTB10/odom
```

Then plot the actual recording:

```bash
python3 amr_ws/scripts/plot_lab2.py --csv output/lab2/goal_run01.csv --route goal --output output/lab2/goal_run01.png
# For other runs: --route square or --route star
```

CSV preserves raw odometry and starting-frame x/y/yaw. Plotting saves an equal-axis
PNG plus a JSON summary of measured duration, path length, and final distance to
the last waypoint. Returning to the final point alone does not prove every
corner was visited. The overlay uses current LabConfig: preserve the settings
used for each run alongside its measurements if you tune between runs.

For raw ROS transport evidence, additionally record in another terminal:

```bash
mkdir -p output/lab2
ros2 bag record -o output/lab2/goal_run01_bag /TTB10/odom /TTB10/cmd_vel
```

Stop recording after the task stops. CSV plots use measured data, never generated
commands. Circle recordings are present in package `lab_runs/circle/`; star recordings
and measured plots still need saving. The renamed command is `lab2_star`;
its plot selector is `--route star`, and its size setting is `STAR_RADIUS`.
See [the migration review](../../../../../docs/PULL_REVIEW_2026_10_20.md).

## Offline checks

From the repository root:

```bash
python3 -m unittest discover -s amr_ws/src/py_amr_ttb/test -p 'test_controller_logic.py' -v
python3 -m unittest discover -s amr_ws/src/py_amr_ttb/test -p 'test_odom_utils.py' -v
python3 -m unittest discover -s amr_ws/src/py_amr_ttb/test -p 'test_lab2_*.py' -v
```

Tests use ROS doubles, pure geometry and ideal kinematics. Ubuntu colcon checks,
message types/QoS, real stopping, wheel slip and accuracy require the robot.
