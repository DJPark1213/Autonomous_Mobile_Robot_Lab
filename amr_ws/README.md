# ECE 4060 AMR workspace

The combined controller now connects the existing IR/wandering code to the
supplied joystick mode selection and cruise PID logic. It starts in STOP and
publishes zero velocity until a mode is selected. Physical validation, bags,
and plots remain pending.

## Controls

- L1 selects WANDER and resets its drive/turn state.
- L2 selects CRUISE, initially at 0.3 m/s. Subsequent entries retain the last target.
- In CRUISE: Square = 0.1, Triangle = 0.2, Circle = 0.4, X = 0.0 m/s.
- Holding R1 temporarily selects teleoperation; releasing it restores the
  selected mode. Mode and speed button selections are ignored while R1 is held.

Joystick logic lives directly in `lab_one_controller.py`. The existing
`IrSensorState` and `WanderBehavior` handle obstacle detection and wandering.
`PidSpeedController` owns cruise calculation/history; it implements the pasted
equation `target + Kp*error + Ki*integral + Kd*derivative`, including integral
limits and conditional anti-windup. This is not the handout's incremental
output equation; confirm the required form before final acceptance.

The timer is the sole normal command publisher; shutdown also sends a zero
command. Missing/stale IR stops wandering, missing/stale odometry stops cruise,
and missing/stale or non-finite joystick axes stop an R1 override. X in cruise
clears PID memory and commands zero even without odometry. Clock jumps or long
timer gaps reset PID timing. Autonomous modes use their sensor timeouts and
do not require a continuous joystick stream; a stale held R1 remains stopped
until a fresh release message arrives.

`lab_config.py` holds topics, axes, scales, timing, PID limits, and the existing
IR calibration. Threshold 35 and front sensor indices 1–4 are retained.

## Build and run on Ubuntu

```bash
cd ~/amr_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select py_amr_ttb
source install/setup.bash
ros2 launch py_amr_ttb py_lab_one_controller.launch.py
```

This launch is active, not an idle template. Start with wheels raised and run
only one motion controller: do not run `lab_one_joystick` or `ttb_turn` alongside
the combined node. Verify ROS topic types/QoS and stop/override behavior before
floor trials. ROS 2 and the dependencies in `package.xml` must be installed.

## Offline checks

```bash
python3 -m unittest discover -s src/py_amr_ttb/test -p test_controller_logic.py -v
```

These tests use ROS interface doubles and the real behavior modules. They do
not validate discovery, QoS, message transport, or physical robot behavior.
Run the ROS package checks and robot trials separately on Ubuntu.
