# ECE 4060 AMR workspace

This directory is the ROS workspace. The Python package is in
`src/py_amr_ttb`, and its launch files are in `src/py_amr_ttb/launch`.
Commands below assume this directory is your working directory.

## Lab 3: learn and replay

Lab 3 uses LiDAR and odometry to navigate an obstacle course and save its
driven route. The default goal is (10, 0) metres in starting odometry and the
default forward speed limit is 0.46 m/s. See the
[repository architecture and decision flow](../README.md#lab-3-module-diagram)
and [detailed Lab 3 guide](src/py_amr_ttb/py_amr_ttb/lab3/README.md).

Build and start the first run:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select py_amr_ttb
source install/setup.bash
ros2 launch py_amr_ttb lab3_go_to_goal.launch.py robot_namespace:=/TTB10
```

This starts moving once valid sensors and a plan arrive. Run one velocity
controller at a time. Both Lab 3 launches accept `robot_namespace`, `base_frame`
(default `base_link`), and `run_file` (default `~/amr_lab3_runs/course.npz`).

After a completed first run, stop its process, return the robot near the original
position and heading, and start replay from a sourced terminal:

```bash
ros2 launch py_amr_ttb lab3_replay.launch.py robot_namespace:=/TTB10
```

Replay checks saved waypoints against the current scan. Map-backed recordings
use scan alignment at startup. Replay saves results to
`course.replay.npz`, preserving `course.npz`.

Stop and queue a save from another sourced terminal:

```bash
ros2 service call /lab3_controller/stop std_srvs/srv/Trigger '{}'
```

Settings live in `Lab3Config` in
[`lab_config.py`](src/py_amr_ttb/py_amr_ttb/lab_config.py). They can be overridden
with ROS startup parameters when running `lab3_controller` directly; the detailed
guide has an example. Pose conversion reuses
[`odom_utils.py`](src/py_amr_ttb/py_amr_ttb/odom_utils.py).

Offline checks from this workspace directory, with Python 3.10+ and NumPy:

```bash
python3 -m unittest discover -s src/py_amr_ttb/test -p test_lab3.py -v
PYTHONPATH=src/py_amr_ttb python3 -m py_amr_ttb.lab3.simulate \
  --scenario pocket --two-runs --output ../output/lab3-pocket
```

## Earlier labs

- [Lab 2 goals and shape tracking](src/py_amr_ttb/py_amr_ttb/lab2/README.md)
- [Lab 1 task demos](src/py_amr_ttb/py_amr_ttb/demo/README.md)
- [Historical work plans and controller notes](../docs/LABS1_2_REFERENCE.md)
