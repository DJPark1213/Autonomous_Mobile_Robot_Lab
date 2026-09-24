# ECE 4060 AMR workspace

This source-only ROS 2 workspace reflects the team plan: standalone joystick
teleoperation is done; Task 2 IR processing and wandering are implemented for
offline testing with controller hooks prepared for future integration. Tasks
3-4 remain incomplete. Task 2 still needs physical tuning and verification on
TurtleBot 10. The provided timed-turn example and existing package support
files are retained.

The Python package is `src/py_amr_ttb`. Joystick input methods now live
directly in `LabOneController`; no separate joystick helper module is needed.
The Part 2 modules validate IR messages, reject stale data, and implement the
drive-stop-random-turn-drive sequence. The TurtleBot 4 IR message fields and
six-sensor order are confirmed; physical behavior still needs validation.
The PID still returns `0.0`, and the integrated joystick methods still return
zero `Twist` commands. The combined node constructs the Part 2 objects and
exposes `ir_callback()` and `wander_command()` hooks, but its subscription,
timer, and velocity publisher are intentionally commented out until the full
integration is ready.

`lab_config.py` retains the known joystick mappings and scales needed by the
completed standalone node. Part 2 speed, timeout, turn timing, sensor indices,
threshold direction, and threshold are grouped there for robot-side tuning.
Required PID gains are constants only.

## Build on Ubuntu with ROS 2

```bash
cd ~/amr_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select py_amr_ttb
source install/setup.bash
ros2 pkg executables py_amr_ttb
```

Launch the inert integration scaffold:

```bash
ros2 launch py_amr_ttb py_lab_one_controller.launch.py
```

This node does not subscribe, publish, or move the robot. ROS dependencies
must still be installed; no local substitute drivers are included.

