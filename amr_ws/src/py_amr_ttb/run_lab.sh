#!/usr/bin/env bash
set -u
set -o pipefail

task="${1:-}"

case "$task" in
  2)
    launch="task2_wander.launch.py"
    curves=(
      ir_front_left
      ir_front_center_left
      ir_front_center_right
      ir_front_right
      threshold
    )
    ;;
  3)
    launch="task3_pid.launch.py"
    curves=(
      measured_speed
      target_speed
      command_speed
    )
    ;;
  *)
    echo "Usage: bash ~/amr_ws/run_lab.sh 2|3"
    exit 1
    ;;
esac

source /opt/ros/humble/setup.bash
source "$HOME/amr_ws/install/setup.bash"

run_dir="$HOME/amr_ws/lab_runs/task${task}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"

pids=()

cleanup() {
  trap - INT TERM EXIT

  echo
  echo "Stopping everything..."

  for pid in "${pids[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
  done

  sleep 1

  for pid in "${pids[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done

  wait 2>/dev/null || true

  echo "Stopped."
  echo "Saved to: $run_dir"
}

trap cleanup INT TERM EXIT


# ==================================================
# Create simple topics for plotting
# ==================================================

setsid python3 - "$task" <<'PY' &
import sys
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector
from py_amr_ttb.lab_config import LabConfig as C

task = sys.argv[1]

rclpy.init()
node = Node("lab_plot_values")

prefix = "/TTB10/lab_plot/"

if task == "2":
    names = [
        "ir_front_left",
        "ir_front_center_left",
        "ir_front_center_right",
        "ir_front_right",
        "threshold",
    ]
else:
    names = [
        "measured_speed",
        "target_speed",
        "command_speed",
    ]

pubs = {
    name: node.create_publisher(Float64, prefix + name, 10)
    for name in names
}

def emit(name, value):
    pubs[name].publish(Float64(data=float(value)))


if task == "2":

    def ir_cb(msg):
        for r in msg.readings:
            frame = r.header.frame_id.rsplit("/", 1)[-1]

            if frame.startswith("ir_intensity_"):
                name = "ir_" + frame[len("ir_intensity_"):]

                if name in pubs:
                    emit(name, r.value)

    node.create_subscription(
        IrIntensityVector,
        C.IR_TOPIC,
        ir_cb,
        qos_profile_sensor_data
    )

    node.create_timer(
        0.1,
        lambda: emit("threshold", C.IR_OBSTACLE_THRESHOLD)
    )

else:

    node.create_subscription(
        Odometry,
        C.ODOM_TOPIC,
        lambda m: emit(
            "measured_speed",
            m.twist.twist.linear.x
        ),
        qos_profile_sensor_data
    )

    node.create_subscription(
        Twist,
        C.CMD_VEL_TOPIC,
        lambda m: emit(
            "command_speed",
            m.linear.x
        ),
        qos_profile_sensor_data
    )

    node.create_timer(
        0.1,
        lambda: emit(
            "target_speed",
            C.PID_TARGET_SPEED
        )
    )


try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()
PY

pids+=("$!")


# ==================================================
# Plot arguments
# ==================================================

plot_args=()

for curve in "${curves[@]}"; do
  plot_args+=("/TTB10/lab_plot/$curve/data")
done


# ==================================================
# Wait for real data
# ==================================================

echo "Waiting for sensor data..."

ready="/TTB10/lab_plot/${curves[0]}"

if ! timeout 20s ros2 topic echo \
  "$ready" std_msgs/msg/Float64 --once >/dev/null
then
  echo "No sensor data."
  exit 1
fi

echo "Sensor data OK."


# ==================================================
# Record bag
# ==================================================

if [[ "$task" == "2" ]]; then

  setsid ros2 bag record \
    -a \
    -o "$run_dir/bag" \
    >"$run_dir/bag.log" 2>&1 &

else

  setsid ros2 bag record \
    /TTB10/odom \
    /TTB10/cmd_vel \
    /TTB10/lab_plot/measured_speed \
    /TTB10/lab_plot/target_speed \
    /TTB10/lab_plot/command_speed \
    -o "$run_dir/bag" \
    >"$run_dir/bag.log" 2>&1 &

fi

bag_pid=$!
pids+=("$bag_pid")


# ==================================================
# Start rqt_plot
# Retry curves until ROS discovers them
# ==================================================

setsid python3 - "${plot_args[@]}" \
  >"$run_dir/plot.log" 2>&1 <<'PY_PLOT' &

import sys

from python_qt_binding.QtCore import QTimer
from rqt_plot.plot_widget import PlotWidget
from rqt_plot.main import main

original_add_topic = PlotWidget.add_topic


def add_topic_with_retry(self, topic_name):

    if topic_name in self._rosdata:
        return

    original_add_topic(self, topic_name)

    if topic_name in self._rosdata:
        print("Added:", topic_name, flush=True)
        return

    if not hasattr(self, "_retry"):
        self._retry = {}

    n = self._retry.get(topic_name, 0) + 1
    self._retry[topic_name] = n

    if n < 60:
        QTimer.singleShot(
            500,
            lambda: add_topic_with_retry(
                self,
                topic_name
            )
        )
    else:
        print("Failed:", topic_name, flush=True)


PlotWidget.add_topic = add_topic_with_retry

sys.argv = [
    "rqt_plot",
    "-e",
    *sys.argv[1:]
]

main()

PY_PLOT

plot_pid=$!
pids+=("$plot_pid")


# ==================================================
# Start robot
# ==================================================

sleep 1

echo
echo "Starting Task $task"
echo "Close the plot OR press Ctrl+C to stop everything."
echo

setsid ros2 launch py_amr_ttb "$launch" \
  >"$run_dir/task.log" 2>&1 &

control_pid=$!
pids+=("$control_pid")


# ==================================================
# IMPORTANT:
# plot closes -> script ends -> cleanup -> robot stops
# controller ends -> script ends too
# ==================================================

wait -n "$plot_pid" "$control_pid"

echo "Plot/task closed."
