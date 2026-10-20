# Lab 2 handoff and acceptance checklist

Heart extension: implemented in navigation.py + lab2_heart.py with launch,
configuration, plot overlay and tests. All 44 offline tests pass, including
heart curve tracking at three widths, notch dwell and recovery from a small
disturbance. The heart has not been run on the robot by the coding agent.
See [the heart guide](../amr_ws/src/py_amr_ttb/py_amr_ttb/lab2/HEART_README.md).

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

Current verification and migration details are in
[PULL_REVIEW_2026_10_20.md](PULL_REVIEW_2026_10_20.md). Star is now `lab2_star`,
with `STAR_RADIUS = 0.45`, `star_waypoints`, and plot selector `--route star`.
Circle code and CSV recordings arrived in the pulled changes. Ubuntu build
and physical checks of this renamed version remain required.

## Teammate changes

Hammouh: square corners are now simple starting-frame coordinates. Shared
`waypoint_command` preserves the previous corner tolerance, speed limits,
heading correction and one-tick stop at each corner. Continue tuning and
demonstrating square; avoid applying the starting-heading rotation twice.

Jianyu's circle node, pure pursuit, lap tracking, dedicated config/odometry helpers,
plotter and launch are integrated. Circle retains independent ownership of its
controller/math. Its launch executes the module and needs no console-script
registration. Circle tests now live in package `test/` alongside the other tests.

## Remaining acceptance work

- [ ] Confirm instructor dimensions, circle radius, pursuit method and lap count.
- [ ] Confirm whether the chosen star satisfies the interesting-shape extra.
- [ ] On Ubuntu, install dependencies and successfully build/launch the package.
- [ ] Verify actual robot topic types, QoS, axes and optional reset service.
- [ ] Verify each task stops at startup, completion, stale odometry and Ctrl+C.
- [ ] Verify exactly one intended `/TTB10/cmd_vel` publisher per run.
- [ ] Demonstrate goal arrival; record measured error and the time-series plot.
- [ ] Hammouh: demonstrate all square corners and return to start.
- [x] Jianyu: circle pursuit/lap implementation and recordings are present.
- [ ] Confirm circle demonstration acceptance against instructor requirements.
- [ ] DJ: demonstrate star and save measured x-y PNG, CSV and JSON summary.
- [ ] Rehearse demos and preserve actual recordings under `output/lab2/`.

No ROS 2 runtime or TurtleBot is connected in the Windows coding environment.
Offline simulations establish ideal control behavior only; they do not validate
wheel slip, transport, QoS, acceleration, obstacle handling, or physical accuracy.
The nodes do not detect a mid-run odometry reset: stop and relaunch after reset.
