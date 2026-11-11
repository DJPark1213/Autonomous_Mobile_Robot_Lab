# Autonomous Mobile Robot Lab

Built for the TurtleBot 4 with ROS 2 Humble.

[Architecture](#lab-3-module-diagram) · [First run](#build-and-first-run) ·
[Second run](#second-run) · [Tests](#offline-checks-and-current-status) ·
[Detailed Lab 3 guide](amr_ws/src/py_amr_ttb/py_amr_ttb/lab3/README.md)

## Lab 3 maze escape

Lab 3 navigates an obstacle course with live LiDAR and odometry. Go-to-goal
holds a forward waypoint, steers toward visible openings, and follows a wall
when the direct path stays blocked. The controller checks the full footprint
through each commanded turn and stop.

### Lab 3 module diagram

```mermaid
flowchart LR
  S["LiDAR and odometry"] --> N["node.py<br/>Fresh sensor snapshot"]
  N --> W["worker.py<br/>Latest snapshot and motion feedback"]
  W --> P["planner.py<br/>Opening approach and wall following"]
  P <--> R["memory.py<br/>Odometry route"]
  R --> F["Completed course.npz"]
  F --> P
  P --> C["motion.py<br/>Steering and stopping sweep"]
  N --> C
  C --> V["node.py<br/>Publish cmd_vel"]
  C --> T["telemetry.py<br/>CSV and failure captures"]
```

Go-to-goal records a completed route. Replay starts at the same physical
position and heading and checks saved waypoint shortcuts against live LiDAR.

- [Build, launch, and replay](amr_ws/README.md#lab-3-learn-and-replay)
- [Decision loop, configuration, and diagnostics](amr_ws/src/py_amr_ttb/py_amr_ttb/lab3/README.md)

## Earlier labs

| Lab | Reference |
| --- | --- |
| Lab 2: goals and shape tracking | [Lab 2 guide](amr_ws/src/py_amr_ttb/py_amr_ttb/lab2/README.md) · [Handoff checklist](docs/LAB2_HANDOFF.md) |
| Lab 1: teleoperation, wandering, and PID | [Demo guide](amr_ws/src/py_amr_ttb/py_amr_ttb/demo/README.md) |

### Lab 2 work plan

Previous team assignments, checklists, and detailed Lab 1 examples are preserved
in the [historical reference](docs/LABS1_2_REFERENCE.md#lab-2-work-plan).
