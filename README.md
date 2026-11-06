# Autonomous Mobile Robot Lab

**Current focus: Lab 3 — escape an unknown maze quickly, then improve the second
run.** Built for the TurtleBot 4 with ROS 2 Humble.

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
  LASER["360-degree LiDAR + laser TF"]
  ODOM["Odometry"]
  CONFIG["lab_config.py / Lab3Config<br/>Shared tunable settings"]

  subgraph ROS["node.py — sensor callbacks and 20 Hz control"]
    GEOMETRY["geometry.py<br/>Hits, unknown beams, footprint clearance"]
    STATE["state.py / StateTracker<br/>Reuses odom_utils.py<br/>Pose, measured velocity, scan-time interpolation"]
    MOTION["motion.py<br/>Steering, acceleration, turning and stopping sweep"]
    COMMAND["node.py publishes cmd_vel<br/>Robot moves toward the target"]
  end

  subgraph WORKER["worker.py — separate planning process"]
    MAP["mapping.py + state.PoseEstimator<br/>Occupancy evidence and pose correction"]
    PLAN["planner.py<br/>Score directions, hold a target, recover from dead ends"]
    MEMORY["memory.py<br/>Trajectory and failed directions"]
    REPLAY["replay.py<br/>Verified shortcuts, corner smoothing, speed profile"]
  end

  SAVED["course.npz<br/>Saved map and first-run experience"]
  LOG["telemetry.py<br/>CSV commands, motion and decision reasons"]

  LASER --> GEOMETRY
  ODOM --> STATE
  GEOMETRY -->|latest scan| MAP
  STATE -->|timestamped pose| MAP
  MAP -->|map and corrected pose| PLAN
  PLAN <--> MEMORY
  MAP -->|map evidence| SAVED
  MEMORY -->|route and history| SAVED
  SAVED -->|second run| REPLAY
  REPLAY -->|improved route| PLAN
  PLAN -->|held target and frame transform| MOTION
  GEOMETRY -->|fresh scan| MOTION
  STATE -->|current pose and measured velocities| MOTION
  MOTION --> COMMAND
  MOTION --> LOG
  CONFIG -.-> MAP
  CONFIG -.-> PLAN
  CONFIG -.-> MOTION
```

The control loop checks live scan clearance every **0.05 s**. The worker updates
the map every **0.20 s** and normally chooses a direction every **0.50 s**.
A blocked or nearby target can trigger an earlier decision. Only the latest
pending sensor snapshot is retained, so planning work does not accumulate.

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

The default learned course is `~/amr_lab3_runs/course.npz`. Both launches accept
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
