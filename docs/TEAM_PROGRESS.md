# Lab 1 team progress and ownership

Status transcribed from the supplied cleaned team work plan on September 17, 2026. Owners are unassigned in that plan. Tasks 2–4 now contain basic class/method templates only; their behavior is not implemented. Completed teleoperation is retained.

| Task | Team status | Owner | Remaining deliverables |
|---|---|---|---|
| 1 Joystick teleoperation | Done; standalone file needs cleanup/integration | Not named | Coordinate remaining cleanup with integration lead |
| 2 Wandering and IR plots | Pending | Unassigned | Calibrated obstacle detection, behavior demonstration, ROS bag, labeled IR plots, calibration notes |
| 3 Cruise control and speed plots | Pending | Unassigned | Assigned PID equation, 0.3 m/s demonstration, speed plots and setpoint-change tests |
| 4 State machine and integration | Pending | Unassigned | Combined launch, all transitions, cruise buttons, R1 override/return, zero-speed stop |
| 5 IR and odometry mapping | Optional; decide later | Unassigned | Separate mapping implementation, map, and assumptions after required work |

## File ownership

Paths below are relative to `amr_ws/src/py_amr_ttb/`.

| Task | Files | Shared configuration sections |
|---|---|---|
| 1 | `py_amr_ttb/lab_one_joystick.py` | Coordinate with Task 4 |
| 2 | `py_amr_ttb/ir_sensor.py`, `py_amr_ttb/wander_behavior.py` | WANDER settings and IR calibration in `lab_config.py` |
| 3 | `py_amr_ttb/pid_speed_controller.py` | PID gains and limits in `lab_config.py` |
| 4 | `py_amr_ttb/lab_one_controller.py`, `launch/py_lab_one_controller.launch.py`, `setup.py`, `package.xml` | Topics, joystick mappings, control timing, shared timeouts in `lab_config.py` |
| 5 | A future separate mapping/plotting module; none supplied | Coordinate dependencies and executable registration with Task 4 |

`ttb_turn.py`, its launch file, the joystick launch file, `__init__.py`, `setup.cfg`, the resource marker, and package tests are provided supporting files. `setup.py` and `package.xml` already contain starter integration support but remain subject to team review. Assign owners before overlapping edits to shared files.

## Template behavior and pending decisions

Joystick input, button detection, and R1 override belong directly in `lab_one_controller.py`. The separate joystick helper module has been removed; these methods remain TODO templates. The supplied Word plan retains the original proposed file split; this checklist reflects the updated structure.

- `LabOneController` starts as an idle ROS node; it has no subscriptions, mode logic, or command publisher.
- `LabOneController.teleop_command()` and `WanderBehavior.command()` return zero commands with a pending-task message. `PidSpeedController.update()` returns `0.0`.
- `IrSensorState` reports data as unavailable and defaults to blocked until implemented.
- Known joystick mappings/scales are retained for completed standalone teleoperation. Required PID gains remain constants; calibration and control timing are TODO sections.
- The IR threshold is unset (`None`). The work plan proposes 120; Task 2 must calibrate it against the 0.1 m requirement.
- The previous PID implementation was removed. Task 3 must implement the assignment equation and coordinate the 0.3 m/s demonstration with Task 4.
- Launch and packaging support are retained so the template can start in a ROS 2 environment. Robot verification and experimental plots remain pending.

## Remaining acceptance checklist

- [x] Joystick teleoperation completed, as reported by the team plan.
- [ ] Assign owners for Tasks 2, 3, and 4.
- [ ] Clean up the standalone joystick implementation and integrate it.
- [ ] Verify topic contracts and joystick axis signs on TurtleBot 10.
- [ ] Wander at 0.3 m/s; detect obstacles within the required 0.1 m threshold, explicitly stop, turn randomly, and resume.
- [ ] Record ROS bag data and produce labeled IR plots and calibration notes.
- [ ] Implement and validate the required PID equation with Kp 0.1, Ki 0.001, Kd 0.03.
- [ ] Demonstrate cruise at 0.3 m/s and plot measured speed through setpoint changes.
- [ ] Verify L1 wander and L2 cruise selection.
- [ ] Verify Square 0.1, Triangle 0.2, Circle 0.4, and X 0.0 m/s.
- [ ] Verify held R1 teleoperation and return to the previous autonomous mode on release.
- [ ] Verify one final command publisher, startup stop, and timeout behavior.
- [ ] Deliver one working combined launch command.
- [ ] Decide whether to begin optional IR/odometry mapping after required deliverables.

