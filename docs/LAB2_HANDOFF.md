# Lab 2 handoff and acceptance checklist

DJ owns shared refactoring, go-to-goal completion, launch/plot support, and the
interesting-shape extra. Hammouh owns square completion/tuning; Jianyu owns
circle pursuit and lap detection. No teammates have been messaged automatically.

## Implemented by DJ

- [x] Fixed starting frame: initial forward x, initial left y, initial yaw zero.
- [x] Shared quaternion/relative-pose/angle conversion and finite-pose validation.
- [x] Shared monotonic receipt-age check; stop if no valid message for 0.5 s.
- [x] Common tolerances, gains, limits, goal offsets, square size and shape size in LabConfig.
- [x] Refactor goal/square; keep explicit node initialization and task state.
- [x] Complete go-to-goal; remove unused speed PID and SciPy dependency.
- [x] Five-point star extra, one closed route with five target vertices.
- [x] Register and launch goal, square, and star; diagnostics for rqt_plot.
- [x] Measured odometry CSV recorder and offline PNG/JSON plotting tool.
- [x] Offline rotated-start navigation simulation and invalid/stale/startup/arrival stop checks.
- [x] Developer explanation and Ubuntu demonstration instructions.

Local verification: 25 offline tests passed (16 existing controller tests,
4 pose-math tests, 2 Lab 2 callback/simulation tests, and 3 plotting tests).
Plot rendering was visually checked using a synthetic fixture kept under
ignored `build/`; it is not a measured robot result. Python syntax and Git
whitespace checks passed. ROS/ament build and lint checks remain for Ubuntu.

## Teammate changes

Hammouh: square corners are now simple starting-frame coordinates. Shared
`waypoint_command` preserves the previous corner tolerance, speed limits,
heading correction and one-tick stop at each corner. Continue tuning and
demonstrating square; avoid applying the starting-heading rotation twice.

Jianyu: use `read_planar_pose`, `relative_pose`, and `odometry_is_fresh` in the
circle node, and preserve a single velocity publisher. Circle needs its own
pursuit/lookahead and progress/lap logic; `waypoint_command` is for stop/turn/go
routes, not the course's circle pursuit implementation. When the file and
`main` exist, register `lab2_circle = py_amr_ttb.lab2.lab2_circle:main` in
`setup.py` and copy an existing launch with executable/name `lab2_circle`.
DJ can integrate that change. No nonfunctional circle launch is provided.

## Remaining acceptance work

- [ ] Confirm instructor dimensions, circle radius, pursuit method and lap count.
- [ ] Confirm whether the chosen star satisfies the interesting-shape extra.
- [ ] On Ubuntu, install dependencies and successfully build/launch the package.
- [ ] Verify actual robot topic types, QoS, axes and optional reset service.
- [ ] Verify each task stops at startup, completion, stale odometry and Ctrl+C.
- [ ] Verify exactly one intended `/TTB10/cmd_vel` publisher per run.
- [ ] Demonstrate goal arrival; record measured error and the time-series plot.
- [ ] Hammouh: demonstrate all square corners and return to start.
- [ ] Jianyu: implement/integrate/demonstrate circle pursuit and lap completion.
- [ ] DJ: demonstrate star and save measured x-y PNG, CSV and JSON summary.
- [ ] Rehearse demos and preserve actual recordings under `output/lab2/`.

No ROS 2 runtime or TurtleBot is connected in the Windows coding environment.
Offline simulations establish ideal control behavior only; they do not validate
wheel slip, transport, QoS, acceleration, obstacle handling, or physical accuracy.
The nodes do not detect a mid-run odometry reset: stop and relaunch after reset.
