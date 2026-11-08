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

In the diagrams, `node.py`, `worker.py`, and the other algorithm files are in
[`amr_ws/src/py_amr_ttb/py_amr_ttb/lab3/`](amr_ws/src/py_amr_ttb/py_amr_ttb/lab3/).
Each code box names its source file and function or class. The outer boxes show
which process executes that code. This runtime diagram applies to **both runs**;
their different startup paths are shown under [How the two runs work](#how-the-two-runs-work).

```mermaid
flowchart TB
  ODOM["ROS odometry messages"]
  LASER["ROS laser messages + laser-to-base TF"]

  subgraph NODE["Process 1: node.py / Lab3Node — ROS I/O and motion control"]
    STATE["node.py / Lab3Node.on_odom<br/>Validate time and frame<br/>Calls state.py / StateTracker.update"]
    SCAN["node.py / Lab3Node.on_scan<br/>Calls state.py / StateTracker.pose_at<br/>Calls geometry.py / process_scan"]
    TICK["node.py / Lab3Node.tick<br/>Runs every control_period<br/>Reads Plan, state and scan; checks readiness"]
    CONTROL["motion.py / MotionController.step_plan<br/>Check plan expiry; transform target<br/>Steer and check predicted stopping sweep"]
    PUBLISH["node.py / Lab3Node.publish<br/>Converts types.py / MotionResult velocities to Twist<br/>Publishes cmd_vel; zero when motion is disallowed"]
    LOG["telemetry.py / Telemetry.record<br/>Queue measurements, command and diagnostics<br/>Background thread writes CSV"]
    LIFECYCLE["node.py / Lab3Node methods<br/>stop_run, save_run, reset_run, close<br/>Pause, request save, restart or stop worker"]
  end

  subgraph IPC["node.py / Lab3Node.start_worker creates these runtime objects"]
    INCOMING["node.py / self.incoming — Queue, capacity 1<br/>Carries worker.py / Work<br/>state, scan, now, blocked"]
    OUTGOING["node.py / self.outgoing — Queue, capacity 1<br/>Carries types.py / Plan<br/>target, transform, created, map_revision, status, diagnostics"]
    EVENTS["node.py / self.save_requested, self.stopping<br/>Multiprocessing events passed to worker.py"]
  end

  subgraph WORKER["Process 2: worker.py — started with multiprocessing spawn"]
    LOOP["worker.py / worker_main<br/>Owns worker.py / NavigationEngine<br/>Handles events and consumes newest available Work"]
    ENGINE["worker.py / NavigationEngine.step<br/>Replay alignment via mapping.py / OccupancyMap.align<br/>Calls mapping.py / Mapper.update when due<br/>Calls planner.py / Planner.step"]
    SAVE["worker.py / NavigationEngine.save<br/>Calls memory.py / RunMemory.save<br/>Writes current evidence and experience"]
  end

  ROBOT["Robot receives velocity command"]
  SAVED[("Generated run file<br/>Path configured by lab_config.py / Lab3Config.run_file<br/>Not included in the repository")]

  ODOM --> STATE
  LASER --> SCAN
  STATE -->|scan-time pose history| SCAN
  STATE -->|latest RobotState| TICK
  SCAN -->|latest valid Scan, or None| TICK
  TICK -->|enqueue snapshot when due or previously blocked| INCOMING
  INCOMING -->|worker receives Work without ROS callbacks| LOOP
  LOOP -->|engine.step with selected Work| ENGINE
  ENGINE -->|Plan with timing and alignment diagnostics| LOOP
  LOOP -->|worker.py / put_latest| OUTGOING
  OUTGOING -->|worker.py / take_latest called by tick| TICK
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

Both launch files start the **same executable**, `lab3_controller`. Its entry in
[`setup.py`](amr_ws/src/py_amr_ttb/setup.py) calls `lab3/node.py:main`, which
constructs `Lab3Node`. The launch files select `run_mode`; they do not start two
different controller implementations or automatically launch one run after the other.

```mermaid
flowchart TB
  FIRST["First run: launch/lab3_go_to_goal.launch.py<br/>generate_launch_description sets run_mode = learn"]
  SECOND["Second run: launch/lab3_replay.launch.py<br/>generate_launch_description sets run_mode = replay"]
  ENTRY["setup.py / console_scripts<br/>lab3_controller calls lab3/node.py:main"]
  NODEINIT["node.py / main and Lab3Node.__init__<br/>Load Lab3Config from lab_config.py<br/>Create sensor subscriptions, motion controller and timer"]
  SPAWN["node.py / Lab3Node.start_worker<br/>Create queues and events; spawn worker.py / worker_main"]
  INIT["worker.py / NavigationEngine.__init__<br/>Branch on config.run_mode"]

  FIRST --> ENTRY
  SECOND --> ENTRY
  ENTRY --> NODEINIT --> SPAWN --> INIT

  subgraph LEARN["First run: learn"]
    NEW["worker.py / NavigationEngine.__init__<br/>New memory.py / RunMemory<br/>New mapping.py / Mapper with empty OccupancyMap"]
    EXPLORE["planner.py / Planner constructed without a replay route<br/>No initial saved-map alignment required"]
  end

  subgraph REPLAY["Second run: replay"]
    LOAD["memory.py / RunMemory.load<br/>Read generated file at config.run_file<br/>worker.py requires a completed route with at least two points"]
    OPTIMIZE["replay.py / optimize_route<br/>Verify shortcuts and corner rounding against the loaded map"]
    PREPARE["worker.py / NavigationEngine.__init__<br/>Keep loaded grid in mapping.py / Mapper<br/>Construct planner.py / Planner with optimized route<br/>Planner calls replay.py / speed_profile"]
    ALIGN["worker.py / NavigationEngine.step on incoming scans<br/>Calls mapping.py / OccupancyMap.align with initial = True<br/>Return ALIGNMENT_REQUIRED without a target until accepted"]
  end

  INIT -->|learn| NEW --> EXPLORE
  INIT -->|replay| LOAD --> OPTIMIZE --> PREPARE --> ALIGN
  RUN["Shared runtime shown above<br/>node.py / tick checks commands using live sensors<br/>worker.py / NavigationEngine.step updates map and calls planner.py / Planner.step"]
  EXPLORE --> RUN
  ALIGN -->|accepted alignment initializes state.py / PoseEstimator| RUN
```

**During the first run**, `Planner.step()` has no saved route to follow. It uses
`evaluate()` to score observed directions and map/memory queries to hold targets,
follow walls or backtrack. `Mapper.update()` builds map evidence as new scans
arrive, and `RunMemory.record()` stores the path. At completion, timeout, an
explicit save request or shutdown, the worker attempts to save the run. Only a
completed saved run can be used for replay.

**During the second run**, startup loads the saved grid and recorded route,
optimizes the route, and then waits for an accepted initial alignment. Once
aligned, the same `Planner.step()` can use `replay_target()` and the route's speed
caps. It still checks completion, stalls and recovery; if replay cannot supply a
usable target, local direction selection can take over. `Mapper.update()` still
runs, so the map is not frozen. `MotionController.step_plan()` still checks live
scan clearance in `node.py`'s process. The robot does not simply play back saved
velocity commands. Replay saves its new results to a separate `.replay.npz` path.

| Execution | First run (`learn`) | Second run (`replay`) |
| --- | --- | --- |
| ROS startup and motion checks | `node.py` + `state.py` + `geometry.py` + `motion.py` | Same code and live sensor checks |
| Worker entry | `worker.py / worker_main` | Same worker entry |
| Initial map | New `mapping.py / OccupancyMap` | Saved grid restored by `memory.py / RunMemory.load` |
| Route preparation | No replay route | `replay.py / optimize_route`, then `speed_profile` |
| Initial scan alignment | Not required against a saved map | Required before ordinary mapping/planning proceeds |
| Target selection | `planner.py / step` uses local scoring and recovery | Same method adds saved-route targeting, with local fallback and recovery |
| Run output | Generated file at `Lab3Config.run_file` | Separate generated `.replay.npz` file |

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
