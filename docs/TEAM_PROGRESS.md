# Lab 1 team progress and ownership

Updated September 24, 2026 after integrating the supplied controller logic.
Existing IR/wandering code is retained. Cruise and joystick mode integration
are implemented and checked offline; physical verification and required
experimental deliverables remain pending. Owner assignments are unchanged.

| Task | Team status | Owner | Remaining deliverables |
|---|---|---|---|
| 1 Joystick teleoperation | Done; standalone file needs cleanup/integration | Not named | Coordinate remaining cleanup with integration lead |
| 2 Wandering and IR plots | Logic connected; robot verification pending | Unassigned | Validate integration, tune turns, demonstrate, record bag, produce plots and notes |
| 3 Cruise control and speed plots | Supplied PID integrated; verification pending | Unassigned | Assigned PID equation, 0.3 m/s demonstration, speed plots and setpoint-change tests |
| 4 State machine and integration | Logic integrated; robot verification pending | Unassigned | Combined launch, all transitions, cruise buttons, R1 override/return, zero-speed stop |
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

## Integrated behavior and pending decisions

- Joystick axes, button edges, mode selection, and R1 override live in `lab_one_controller.py`. No separate joystick or odometry module is needed.
- The controller subscribes to Joy, Odometry, and IrIntensityVector using sensor-data QoS. The timer publishes one selected velocity command per tick; startup is STOP and shutdown sends zero.
- L1 selects WANDER; L2 selects CRUISE, initially at 0.3 m/s. Cruise buttons select 0.0, 0.1, 0.2, and 0.4 m/s. R1 temporarily overrides and ignores mode/speed changes while held.
- `PidSpeedController` now contains the supplied target-speed-plus-PID formula, integral clamp, conditional anti-windup, and reset logic. Confirm the assignment's expected equation before marking Task 3 complete.
- The existing IR and wandering modules are unchanged. Threshold 35, sensor indices 1–4, recorded samples, and turn settings remain in `lab_config.py`.
- Stale/invalid sensor data stops the corresponding behavior. Stale joystick data stops an R1 override; autonomous modes continue according to their own sensor freshness checks.
- The supplied Word plan remains planning material. This checklist and the workspace README describe the current implementation.
- ROS build/launch, physical trials, recordings, plots, and the 0.3 m/s demonstration still require Ubuntu/TurtleBot validation. Offline tests are not robot test results.

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

