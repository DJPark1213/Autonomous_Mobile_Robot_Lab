# Jianyu's Lab 2 Task 3: circle pursuit

Circle pursuit is now integrated into the repository. Circle retains its own
configuration, odometry helpers, path tracking and plotting. All launch files
are under the ROS package's `launch/` directory. Circle tests live under its
`test/` directory, matching the commands below.

## Added files (relative to amr_ws/src/py_amr_ttb)

| File | Purpose |
| --- | --- |
| `py_amr_ttb/lab2/circle_config.py` | Circle-owned defaults, including robot settings |
| `py_amr_ttb/lab2/circle_odom.py` | Circle-owned pose conversion and freshness helpers |
| `py_amr_ttb/lab2/pure_pursuit.py` | Circle geometry, lookahead target, curvature and signed progress |
| `py_amr_ttb/lab2/lab2_circle.py` | Feedback node, diagnostics and stopping |
| `py_amr_ttb/lab2/circle_plot.py` | Independent odometry recorder and plotter |
| `launch/lab2_circle.launch.py` | Launches the Python module with ROS parameters |
| `test/test_lab2_circle.py` | Geometry and actual-node callback tests using ROS doubles |
| `test/test_lab2_circle_plot.py` | Plot CLI tests using synthetic fixtures |
| `py_amr_ttb/lab2/CIRCLE_README.md` | This guide |

The existing `setup.py` already discovers packages with `find_packages()` and
installs `launch/*.launch.py`. Rebuilding therefore installs the added Python
modules and launch file. The new launch uses `python -m` through `ExecuteProcess`;
it does not need a shared console-script entry added to `setup.py`.

## Build and run

Copy only the added `amr_ws` files into the corresponding existing workspace,
or check out the contribution branch on the Ubuntu robot computer. From the
actual workspace (here `~/amr_ws`):

```bash
source /opt/ros/humble/setup.bash
cd ~/amr_ws
colcon build --symlink-install --packages-select py_amr_ttb
source install/setup.bash
ros2 launch py_amr_ttb lab2_circle.launch.py
```

Defaults: radius 0.75 m, speed 0.12 m/s, lookahead 0.30 m, one CCW lap, namespace
`/TTB10`. These dimensions are choices; the supplied assignment screenshot did
not specify a radius. Override them without modifying source:

```bash
ros2 launch py_amr_ttb lab2_circle.launch.py radius:=0.50 speed:=0.10 lookahead:=0.25 laps:=1 direction:=-1
```

`direction=1` means CCW/left; `-1` means CW/right. `laps` is an integer >=1,
`0 < lookahead < 2*radius`, `0 < speed <= 0.20` m/s. For another robot use
`robot_namespace:=/TTBXX` and match the recorder's `--topic` and plotting topics.
Parameters are read at startup. Circle defaults live in `circle_config.py`.

The robot starts at the circumference, facing the tangent. The first valid pose
defines the fixed starting frame: x forward, y left. The center is
`(0, direction*radius)`. Default CCW coverage is x -0.75..+0.75 m, y 0..1.5 m,
plus space for the robot footprint and errors. No external odometry reset is
needed. Keep the robot still during origin capture; do not reset during a run.

## Record a complete demonstration

Run only one motion controller. Keep the robot stationary until the recorder
reports readiness. All new terminals need ROS/workspace setup sourcing.

Terminal 1, start CSV recording first:

```bash
source /opt/ros/humble/setup.bash
source ~/amr_ws/install/setup.bash
cd ~/amr_ws
python3 -m py_amr_ttb.lab2.circle_plot --record lab_runs/circle/run01.csv
```

Wait for `First valid odometry received. Ready to start controller.`

Terminal 2, start motion:

```bash
source /opt/ros/humble/setup.bash
source ~/amr_ws/install/setup.bash
ros2 launch py_amr_ttb lab2_circle.launch.py
```

Terminal 3, open time-series plots after the controller advertises its topics:

```bash
source /opt/ros/humble/setup.bash
source ~/amr_ws/install/setup.bash
ros2 run rqt_plot rqt_plot /TTB10/lab2_circle/x/data /TTB10/lab2_circle/y/data /TTB10/lab2_circle/radial_error/data /TTB10/lab2_circle/laps/data
```

After `Circle completed ... Stopped`, verify the robot has stopped, press Ctrl+C
in Terminal 2 and then Terminal 1. Closing rqt_plot alone does not stop motion.
Use Ctrl+C in the controller terminal to stop a run early.

In Terminal 1, plot the recorded data using the actual radius and direction:

```bash
python3 -m py_amr_ttb.lab2.circle_plot --csv lab_runs/circle/run01.csv --radius 0.75 --direction 1 --output lab_runs/circle/run01.png
```

The PNG overlays measured odometry and the ideal circle; the JSON reports
radial RMSE, maximum radial error, directed laps and closure error. These errors
are relative to odometry, not external ground truth. Use `run02` etc. for new
runs; recording and plotting refuse to overwrite earlier results.

## Algorithm and stopping

Choose the forward circle/lookahead-circle intersection, transform the target
into the current robot frame and calculate `kappa = 2*y_body/(x_body^2+y_body^2)`.
Command `omega=v*kappa`; reduce speed if needed to retain curvature within the
angular limit. A target behind the robot triggers alignment before driving.
The standard method is described in
[Coulter, CMU-RI-TR-92-01](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm).
The instructor's detailed pure-pursuit lecture pages were not in the upload.

Unwrap measured directed polar position about the circle center. Reversing
subtracts progress; rotating in place does not count as a lap. Finish requires
the requested laps, closure <=0.05 m, and heading error <=0.25 rad. It then
remains stopped until shutdown. Phase unwrapping assumes consecutive samples
advance less than pi radians about the center.

Missing/invalid/stale odometry commands zero (receipt timeout 0.5 s); valid data
resumes an unfinished run. Radial deviation >0.35 m latches a stop and requires
inspection/relaunch. The controller does not implement obstacle avoidance.

## Offline checks and remaining validation

From the repository root:

```bash
python3 -m unittest discover -s amr_ws/src/py_amr_ttb/test -p 'test_lab2_*.py' -v
python3 -m unittest discover -s amr_ws/src/py_amr_ttb/test -p 'test_odom_utils.py' -v
python3 -m unittest discover -s amr_ws/src/py_amr_ttb/test -p 'test_controller_logic.py' -v
```

Tests use ideal kinematics, ROS doubles and synthetic plotting fixtures. ROS
build/launch, message transport, physical accuracy and stopping still require
the actual Ubuntu/TurtleBot environment. No physical test results are claimed.
