# ECE 4060 AMR workspace

This source-only ROS 2 workspace reflects the team plan: standalone joystick teleoperation is done; Tasks 2–4 are minimal TODO templates. The provided timed-turn example and existing package support files are retained.

The Python package is `src/py_amr_ttb`. Joystick input methods now live directly in `LabOneController`; no separate joystick helper module is needed. Pending classes retain basic method signatures with inert returns. Wandering and integrated joystick methods return zero `Twist` commands; PID returns `0.0`. IR state reports unavailable data and defaults to blocked. The combined node only logs its template status and spins; it has no subscriptions or velocity publisher.

`lab_config.py` retains the known joystick mappings and scales needed by the completed standalone node. Pending calibration/timing sections are TODOs; required PID gains are constants only.

## Build on Ubuntu with ROS 2

```bash
cd ~/amr_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select py_amr_ttb
source install/setup.bash
ros2 pkg executables py_amr_ttb
```

Launch the idle integration template:

```bash
ros2 launch py_amr_ttb py_lab_one_controller.launch.py
```

This does not drive the robot or demonstrate Tasks 2–4. ROS dependencies must be installed; no local substitute drivers are included.

