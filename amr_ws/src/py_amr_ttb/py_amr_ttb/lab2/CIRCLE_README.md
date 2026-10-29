# Lab 2: follow an independently moving circular goal

Only the virtual goal is constrained to a circle. The robot pursues its current
position and may cut inside or leave the circle. Its controller receives only
robot pose, goal position, robot_speed and hardware bounds; it has no circle
radius, target phase, target speed, radial-error or phase-gap feedback.

The fixed starting frame has x forward and y left. The goal position is
`(R*sin(phase), direction*R*(1-cos(phase)))`. Its phase advances by
`target_speed*dt/R` on a monotonic clock, independent of robot motion.
`lookahead` sets the initial goal lead (a chord distance), not a robot path.

## Build and run

```bash
cd ~/amr_ws
colcon build --symlink-install --packages-select py_amr_ttb
source install/setup.bash
ros2 launch py_amr_ttb lab2_circle.launch.py target_speed:=0.10 robot_speed:=0.15
```

| Parameter | Default | Meaning |
| --- | --- | --- |
| target_speed | 0.12 | Goal speed along its circle, m/s; nonnegative |
| robot_speed | 0.12 | Independent follower forward speed limit, 0 to 0.20 m/s |
| radius | 0.75 | Goal circle radius, m |
| lookahead | 0.30 | Initial goal chord lead, m; between 0 and 2*radius |
| direction | 1 | +1 counterclockwise, -1 clockwise |
| laps | 1 | Goal revolutions, excluding initial lead |
| duration | 0.0 | Positive: stop goal after this many seconds; zero: use laps |
| robot_namespace | /TTB10 | Robot topic prefix |

Change the two speeds independently while running:

```bash
ros2 param set /lab2_circle target_speed 0.08
ros2 param set /lab2_circle robot_speed 0.16
```

Set robot_speed to zero to stop robot commands while the goal continues. Set
target_speed to zero to freeze the goal while the robot continues approaching.
Geometry and finish-rule parameters require restarting. The old `speed`
parameter is replaced by `robot_speed`.

## Robot control and completion

The follower uses wrapped bearing error to the current goal:

```text
w = clip(2*heading_error, -0.40, 0.40)
v = min(robot_speed, 0.20, distance_to_goal) * max(0, cos(heading_error))
```

It moves and turns together for a goal ahead, slows near the goal, and turns
in place for a goal behind. robot_speed is a requested maximum, not a forced
constant measured speed. The robot can lag or cut corners; it is not required
to complete the goal's number of revolutions.

After the goal finishes its laps or duration, its position freezes. The robot
continues pursuing this final position, then latches zero commands within
0.05 m. A full lap ends at the initial lead point, not at the robot origin.
A duration limit applies to goal motion, not to total robot catch-up time.

Missing, invalid, or stale odometry (>0.5 s) stops robot commands. After the
first valid odometry starts the target clock, that clock continues during
odometry outages. Normal Ctrl+C shutdown sends zero velocity.

## Diagnostics

Scalar topics and paired JSON samples are under `/TTB10/lab2_circle/`.
`target_speed` is actual goal speed (zero once it finishes), `robot_speed_limit`
is the configured robot_speed, and `linear_command` / `measured_linear` show
commanded / measured robot motion. `done` means the robot reached the final
stopped goal. Radial error, robot laps and phase gap remain diagnostic only;
none influences robot commands.

```bash
python3 -m py_amr_ttb.lab2.circle_plot --record lab_runs/circle/run01.csv --duration 90
python3 -m py_amr_ttb.lab2.circle_plot --csv lab_runs/circle/run01.csv --output lab_runs/circle/run01.png
```

Tests use ROS doubles and ideal kinematics, not physical robot measurements.
