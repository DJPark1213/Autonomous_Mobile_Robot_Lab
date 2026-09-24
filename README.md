# Autonomous Mobile Robot Lab

ECE 4060 Lab 1 team workspace for TurtleBot 10. Due September 24, 2026, during lab.

The team has completed standalone joystick teleoperation. The supplied
combined controller is now integrated with the existing IR/wandering modules
and a reusable cruise PID. It starts in STOP; L1/L2 select autonomous modes and
R1 temporarily overrides with teleoperation. Robot validation, calibration,
data recording, and plots remain pending. Optional mapping is deferred.

## Start here

- [Team progress and file ownership](docs/TEAM_PROGRESS.md)
- [Original team work plan](docs/Lab_1_Team_Work_Plan.docx)
- [ROS workspace overview](amr_ws/README.md)


## Layout

```text
amr_ws/
  src/py_amr_ttb/          ROS 2 Python package, launch files, and supplied tests
docs/
  TEAM_PROGRESS.md         Current status, ownership, and remaining deliverables
  Lab_1_Team_Work_Plan.docx
  references/             Combined Lab 1/2 handout and its page images
```

On the Ubuntu lab computer, place `amr_ws` at `~/amr_ws`, then follow the build instructions in `amr_ws/README.md`. ROS builds and hardware tests have not been performed as part of this file transfer.

## Transfer scope

Imported from ECE4060 on September 17, 2026. On September 24, the supplied
controller logic was integrated while preserving the team's IR/wandering
implementation and configuration. Joystick handling stays in the combined
controller; cruise PID calculation lives in `pid_speed_controller.py`.

General lecture material, study guides, temporary document renders, earlier work-plan drafts, editor metadata, and generated ROS build/install/log files were excluded. The combined Lab 1/2 handout is retained intact as a reference; it does not change the current Lab 1 scope.

Physical validation and confirmation of the supplied PID equation remain TODOs. The Part 2
sensor interface, sensor order, and recorded 0.1 m samples are documented in
`lab_config.py`. The ECE4060 originals remain unchanged.
