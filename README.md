# Autonomous Mobile Robot Lab

ECE 4060 Lab 1 team workspace for TurtleBot 10. Due September 24, 2026, during lab.

The team has completed joystick teleoperation. Wandering, cruise control, and combined state-machine integration remain pending; their files contain minimal TODO templates with inert return values. Optional mapping is deferred.

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

Imported on September 17, 2026 from the local ECE4060 workspace and the supplied work plan. Completed teleoperation and provided examples are preserved. Pending Tasks 2–4 have been reduced to class/method templates; launch files and package metadata retain the scaffolding needed to start the idle integration node.

General lecture material, study guides, temporary document renders, earlier work-plan drafts, editor metadata, and generated ROS build/install/log files were excluded. The combined Lab 1/2 handout is retained intact as a reference; it does not change the current Lab 1 scope.

IR calibration and the assignment PID equation remain TODOs. The old autonomous and integration implementations have been removed from this project; the ECE4060 originals remain unchanged.
