# Autonomous Mobile Robot Lab

ECE 4060 Lab 1 team workspace for TurtleBot 10. Due September 24, 2026, during lab.

The team has completed joystick teleoperation. Part 2 IR processing and the
wandering state machine are implemented for offline testing, but still require
robot-side calibration, tuning, data recording, and physical validation.
Part 2 exposes controller hooks for later integration, but its ROS subscription,
timer, and command publisher are intentionally disabled. Cruise control and
the full joystick state machine remain pending. Optional mapping is deferred.

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

Imported on September 17, 2026 from the local ECE4060 workspace and the
supplied work plan. Completed teleoperation and provided examples are
preserved. Task 2 now has a configuration-driven implementation with a
confirmed TurtleBot 4 IR interface, offline tests, and commented controller
integration points. Tasks 3-4 remain incomplete; launch files and package
metadata retain the scaffolding needed for the full combined controller.

General lecture material, study guides, temporary document renders, earlier work-plan drafts, editor metadata, and generated ROS build/install/log files were excluded. The combined Lab 1/2 handout is retained intact as a reference; it does not change the current Lab 1 scope.

Physical validation and the assignment PID equation remain TODOs. The Part 2
sensor interface, sensor order, and recorded 0.1 m samples are documented in
`lab_config.py`. The ECE4060 originals remain unchanged.
