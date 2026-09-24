# Lab 1 team progress and ownership

Status transcribed from the supplied cleaned team work plan on September 17,
2026. Owners are unassigned in that plan. Task 2 now contains offline-tested
IR processing and wandering behavior. Tasks 3-4 still contain basic
class/method templates only. Completed teleoperation is retained.

| Task | Team status | Owner | Remaining deliverables |
|---|---|---|---|
| 1 Joystick teleoperation | Done; standalone file needs cleanup/integration | Not named | Coordinate remaining cleanup with integration lead |
| 2 Wandering and IR plots | Logic implemented; controller hooks prepared; robot verification pending | Unassigned | Enable integration later, tune turns, demonstrate, record bag, produce plots and notes |
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

- `LabOneController` starts as an inert scaffold. It constructs the Part 2 objects and exposes `ir_callback()` and `wander_command()`, while the future subscription, timer, and publisher remain commented out for Task 4.
- `LabOneController.teleop_command()` returns a zero command with a pending-task message. `PidSpeedController.update()` returns `0.0`.
- `IrSensorState` validates readings and applies configurable sensor selection and threshold direction. `WanderBehavior` implements drive, explicit stop, random timed turn, and resume.
- The confirmed TurtleBot 4 interface is `IrIntensityVector.readings`, with `IrIntensity.header` and `IrIntensity.value`; malformed or stale data produces a zero command.
- Known joystick mappings/scales are retained for completed standalone teleoperation. Required PID gains remain constants; calibration and control timing are TODO sections.
- The configured IR threshold is 35 because clear readings were below 7 and the lowest recorded 0.1 m front reading was 66; physical behavior still requires validation.
- Recorded IR layout: message frame `base_link`; reading order is left, front-left, front-center-left, front-center-right, front-right, right. Part 2 checks indices 1-4.
- Recorded samples: all clear-path values were below 7; at approximately 0.1 m, the four front readings were 176, 155, 220, and 66. The front-right value means right-side approaches still need verification with threshold 100.
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

