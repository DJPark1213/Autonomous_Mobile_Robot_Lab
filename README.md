# Autonomous Mobile Robot Lab

Built for the TurtleBot 4 with ROS 2 Humble.

[Architecture](#lab-3-module-diagram) · [First run](#build-and-first-run) ·
[Second run](#second-run) · [Tests](#offline-checks-and-current-status) ·
[Detailed Lab 3 guide](amr_ws/src/py_amr_ttb/py_amr_ttb/lab3/README.md)

## Lab 3 maze escape

The robot starts without a map. It uses 360-degree LiDAR and odometry to choose
reachable targets, favor useful forward travel, reduce turning, and recover from
dead ends. The first run saves map evidence and its trajectory. The second run
aligns near the original start and follows a shortened route through verified
free space, while continuing to check live LiDAR.

The current goal assumption is **10 metres ahead of the initial pose**, at
`(10, 0)` in the starting frame. Completion uses that coordinate; open space
alone does not identify the finish. The default cruise limit is **0.30 m/s**.
Scoring, timing, clearance, and motion response are configurable.

### Lab 3 module diagram

```mermaid
flowchart TB
  ODOM["ROS odometry messages"]
  LASER["ROS laser messages + laser-to-base TF"]

  subgraph NODE["Process 1: node.py / Lab3Node — ROS I/O and motion control"]
    STATE["on_odom<br/>Validate time and frame<br/>state.py / StateTracker.update"]
    SCAN["on_scan<br/>StateTracker.pose_at acquisition time<br/>geometry.py / process_scan"]
    TICK["tick — every control_period<br/>Read newest Plan; keep latest state and scan<br/>Check freshness, faults, pause and worker health"]
    CONTROL["motion.py / MotionController.step_plan<br/>Check plan expiry; transform target<br/>Steer and check predicted stopping sweep"]
    PUBLISH["publish<br/>Send cmd_vel from MotionResult<br/>Default result is zero velocity when gated"]
    LOG["telemetry.py / Telemetry.record<br/>Queue measurements, command and diagnostics<br/>Background thread writes CSV"]
    LIFECYCLE["stop_run / save_run / reset_run / close<br/>Pause, request save, restart or stop worker"]
  end

  subgraph IPC["Created by Lab3Node.start_worker — communication between processes"]
    INCOMING["self.incoming — Queue with capacity 1<br/>Work: state, scan, now, blocked"]
    OUTGOING["self.outgoing — Queue with capacity 1<br/>Plan: target, transform, created, map_revision, status, diagnostics"]
    EVENTS["save_requested and stopping<br/>Multiprocessing events"]
  end

  subgraph WORKER["Process 2: worker.py — started with multiprocessing spawn"]
    LOOP["worker_main<br/>Own a NavigationEngine<br/>Handle events and consume newest available Work"]
    ENGINE["NavigationEngine.step<br/>1. Require initial alignment in replay<br/>2. Mapper.update when map update is due<br/>3. Planner.step to keep or choose a target"]
    SAVE["NavigationEngine.save<br/>RunMemory.save writes current evidence and experience"]
  end

  ROBOT["Robot receives velocity command"]
  SAVED[("Generated run file<br/>Path from config.run_file<br/>Not included in the repository")]

  ODOM --> STATE
  LASER --> SCAN
  STATE -->|scan-time pose history| SCAN
  STATE -->|latest RobotState| TICK
  SCAN -->|latest valid Scan, or None| TICK
  TICK -->|enqueue snapshot when due or previously blocked| INCOMING
  INCOMING -->|worker receives Work without ROS callbacks| LOOP
  LOOP -->|engine.step with selected Work| ENGINE
  ENGINE -->|Plan with timing and alignment diagnostics| LOOP
  LOOP -->|put_latest| OUTGOING
  OUTGOING -->|take_latest at start of tick| TICK
  TICK -->|fresh sensors and active worker| CONTROL
  CONTROL -->|MotionResult; blocked saved for next Work| TICK
  TICK -->|publish result every tick| PUBLISH
  PUBLISH --> ROBOT
  TICK -->|state, result and plan diagnostics| LOG
  LIFECYCLE -->|set save_requested or stopping as appropriate| EVENTS
  EVENTS --> LOOP
  LOOP -->|save request or final save on exit| SAVE
  ENGINE -->|save once on DONE or TIME_LIMIT| SAVE
  SAVE --> SAVED
```

The outer boxes are **process boundaries**, not additional source files.
`node.py` creates the worker process, queues and events in `start_worker()` and
passes the validated `Lab3Config` to it. The file is `worker.py`; it defines both
the `Work` record and the worker entry point. `Plan` is defined in `types.py`.
Queues transfer records between processes; the worker's live map and planner
objects remain owned by its `NavigationEngine`.

**`node.py` owns sensing and velocity publication.** `on_odom()` maintains pose
history; `on_scan()` uses it to attach an acquisition-time pose to the scan.
`tick()` submits snapshots without waiting for a planning result. If no newer
plan has arrived, it can reuse its held plan until that plan expires. With valid
prerequisites, it calls `MotionController.step_plan()` using current sensor data;
otherwise it publishes the default zero-velocity result. The worker never
publishes `cmd_vel`.

**`worker.py` owns the slower navigation state.** `worker_main()` receives a
snapshot and calls `NavigationEngine.step()`; it does not run on a fixed control
timer. Within that engine, the existing modules interact as follows:

| Call or owner | Interaction |
| --- | --- |
| `NavigationEngine.__init__()` | Creates `RunMemory` (`memory.py`), `Mapper` (`mapping.py`) and `Planner` (`planner.py`). In replay, loads a completed generated run file and calls `replay.optimize_route()` before constructing the planner. |
| Replay initialization | `optimize_route()` queries the loaded grid's `segment_clear()` for shortcuts and rounded corners. `Planner.__init__()` calls `replay.speed_profile()` for bend-speed caps. |
| `NavigationEngine.step()` | First requires an accepted initial scan alignment in replay. Calls `Mapper.update(scan, covariance)` if a map update is due. After that check, calls `Planner.step(state, scan, mapper, now, blocked)` for each aligned snapshot, including those without a map update. |
| `Mapper.update()` | Uses `state.PoseEstimator` and local scan matching to update the odometry-to-map transform, then integrates the scan into `OccupancyMap`. Duplicate scans are not inserted again. |
| `Planner.step()` | Queries map evidence, uses `geometry.py` to check candidate travel, and calls `RunMemory` to record poses, penalize failed directions and find return points. Replay uses the optimized route when appropriate. |
| Return to `worker_main()` | The engine adds diagnostics to the `Plan`. The worker puts it in `outgoing`; a later `node.py` tick reads it. A target and its frame transform travel together. |
| `NavigationEngine.save()` | Passes memory, map layers and transform to `RunMemory.save()`. Saving creates a runtime file at the configured path; replay writes a separate result file. |

The `blocked` flag in a submitted `Work` comes from the **previous control
result**. It asks planning to reconsider; the next plan does not bypass current
clearance checks. Each queue holds at most one pending item, and the helpers
attempt to replace old pending data rather than build a backlog. Work already
being processed is not interrupted.

With default settings, control runs every **0.05 s**, snapshots are normally
submitted every **0.20 s**, and the worker updates the map no more frequently
than every **0.20 s**. Planning is called for processed snapshots but normally
holds a target between **0.50 s** decisions. Blockage or reaching a target can
trigger earlier reconsideration. These are configured intervals, not guarantees
that worker computation finishes within them.

`stop_run()` pauses control, publishes zero and sets `save_requested` without
ending the worker. `save_run()` only requests saving. `reset_run()` requires a
paused, nearly stationary robot and restarts the worker and pose history;
`close()` requests zero velocity and shuts down the worker and logging.

### How the two runs work

| Stage | What the robot does | What it produces |
| --- | --- | --- |
| Observe | Combine 360-degree LiDAR with scan-time odometry; distinguish free, occupied, and unknown space | Usable travel distances, local pose, and map evidence |
| Choose | Score headings across four front and four rear sections; favor goal progress and travel while penalizing turns and failed directions | A reachable point held between decisions |
| Drive | Steer continuously using measured linear/angular velocity; check the predicted path through a complete stop | A velocity command that accounts for motion response |
| Recover | Follow nearby walls when appropriate, or backtrack from a stall or confirmed capped pocket | A new approach and a penalty for the failed direction |
| Learn | Record the route, map, and directional experience during the first run | A saved course for replay |
| Replay | Align near the original start, trim detours through observed free space, and follow the improved route | A second run that still reacts to current LiDAR |

Unknown continuation remains a possible way forward. A region is rejected as
closed only when the observed geometry supports that conclusion; unknown space
does not count as verified clearance for driving or route shortcuts.

### Build and first run

From the repository root on the ROS machine:

```bash
cd amr_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select py_amr_ttb
source install/setup.bash
ros2 launch py_amr_ttb lab3_go_to_goal.launch.py robot_namespace:=/TTB10
```

The launch begins moving after valid sensors and a plan arrive. It uses
`/TTB10/scan`, `/TTB10/odom`, and `/TTB10/cmd_vel`; run one velocity controller
at a time. The robot's odometry child frame must match the `base_frame` launch
argument, which defaults to `base_link`.

### Second run

After completing the first run, stop that process and place the robot near its
original position and heading. In a terminal with the ROS workspace sourced:

```bash
ros2 launch py_amr_ttb lab3_replay.launch.py robot_namespace:=/TTB10
```

Saving a learning run creates a file at `~/amr_lab3_runs/course.npz` by default;
that file is not supplied with the repository. Both launches accept
`run_file:=/absolute/path/course.npz`. Replay requires a completed first run and
a successful initial scan alignment. Its output is saved separately as
`course.replay.npz`.

To stop and queue a save from another sourced terminal:

```bash
ros2 service call /lab3_controller/stop std_srvs/srv/Trigger '{}'
```

### Code and configuration

The new implementation lives in
[`lab3/`](amr_ws/src/py_amr_ttb/py_amr_ttb/lab3/). It reuses
[`odom_utils.py`](amr_ws/src/py_amr_ttb/py_amr_ttb/odom_utils.py) and the shared
[`lab_config.py`](amr_ws/src/py_amr_ttb/py_amr_ttb/lab_config.py).
`Lab3Config` contains the tunable defaults; every field is also a ROS startup
parameter. Restart the controller to apply changes.

| Setting group | Where to start |
| --- | --- |
| Goal and speed | `goal_x`, `goal_y`, `max_linear`, `max_angular` |
| Direction scoring | `weight_goal`, `weight_travel`, `weight_heading`, `weight_rotation`, `weight_history` |
| Lookahead and commitment | `scoring_horizon`, `target_distance`, `decision_period`, `switch_margin` |
| Clearance and motion response | `robot_radius`, `safety_margin`, acceleration/braking limits, `command_latency` |
| Recovery and replay | `stall_timeout`, `backtrack_distance`, `route_margin`, `replay_start_search` |

The [detailed Lab 3 guide](amr_ws/src/py_amr_ttb/py_amr_ttb/lab3/README.md)
explains each module, the parameter defaults, pose filtering, and save/reset/logging
instructions. The [workspace quick start](amr_ws/README.md) gives commands from
inside `amr_ws`.

### Offline checks and current status

From the repository root, with Python 3.10+ and NumPy:

```bash
python3 -m unittest discover -s amr_ws/src/py_amr_ttb/test -p test_lab3.py -v
PYTHONPATH=amr_ws/src/py_amr_ttb python3 -m py_amr_ttb.lab3.simulate \
  --scenario pocket --two-runs --output output/lab3-pocket
```

The simulator supports `straight`, `obstacle`, `pocket`, and `corner`. It uses
the navigation implementation with a separate robot response model and reports
completion, clearance, speed, and elapsed simulated time.

All 28 Lab 3 tests passed in local validation, including recovery, pose filtering,
sensor/plan expiry, worker execution, and two-run reuse. **ROS integration and
physical TurtleBot validation remain pending.** Acceleration, braking, and noise
settings need robot measurements; simulation results do not establish hardware
performance. See the detailed guide for localization and scan-timing limits.

## Earlier labs

| Lab | Reference |
| --- | --- |
| Lab 2: goals and shape tracking | [Lab 2 guide](amr_ws/src/py_amr_ttb/py_amr_ttb/lab2/README.md) · [Handoff checklist](docs/LAB2_HANDOFF.md) |
| Lab 1: teleoperation, wandering, and PID | [Demo guide](amr_ws/src/py_amr_ttb/py_amr_ttb/demo/README.md) |

### Lab 2 work plan

Previous team assignments, checklists, and detailed Lab 1 examples are preserved
in the [historical reference](docs/LABS1_2_REFERENCE.md#lab-2-work-plan).
