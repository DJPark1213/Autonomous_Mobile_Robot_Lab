#!/usr/bin/env bash
set -eo pipefail
set -m

task="${1:-}"
case "$task" in
    2)
        launch_file="task2_wander.launch.py"
        curves=(ir_front_left ir_front_center_left
                ir_front_center_right ir_front_right threshold)
        ;;
    3)
        launch_file="task3_pid.launch.py"
        curves=(measured_speed target_speed command_speed)
        ;;
    *)
        echo "Usage: bash ~/amr_ws/run_lab.sh 2|3"
        exit 1
        ;;
esac

source /opt/ros/humble/setup.bash
source "$HOME/amr_ws/install/setup.bash"
ros2 pkg prefix rqt_plot >/dev/null

# Prevent two copies of this script from running together.
exec 9>"$HOME/amr_ws/.run_lab.lock"
flock -n 9 || {
    echo "Another run_lab.sh is running."
    exit 1
}

run_dir="$HOME/amr_ws/lab_runs/task${task}_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$run_dir"

workers=()
control_pid=""

stop_group() {
    local pid="$1" sig attempt

    for sig in INT TERM; do
        kill -"$sig" -- "-$pid" 2>/dev/null || true

        for attempt in {1..40}; do
            if ! kill -0 -- "-$pid" 2>/dev/null; then
                return 0
            fi
            sleep 0.1
        done
    done

    echo "Forced shutdown of process group $pid"
    kill -KILL -- "-$pid" 2>/dev/null || true
}

cleanup() {
    trap '' INT TERM
    trap - EXIT

    # Stop the controller before closing the recorder.
    if [[ -n "$control_pid" ]]; then
        stop_group "$control_pid"
        wait "$control_pid" 2>/dev/null || true
    fi

    for pid in "${workers[@]}"; do
        stop_group "$pid"
    done

    for pid in "${workers[@]}"; do
        wait "$pid" 2>/dev/null || true
    done

    echo "Stopped. Data and logs: $run_dir"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Publish display-only values using local receipt time for plotting.
python3 - "$task" <<'PY' &
import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector
from py_amr_ttb.lab_config import LabConfig as C

task = sys.argv[1]
rclpy.init(args=[])
node = Node("lab_plot_values")
prefix = "/TTB10/lab_plot/"

names = (
    ["ir_front_left", "ir_front_center_left",
     "ir_front_center_right", "ir_front_right", "threshold"]
    if task == "2"
    else ["measured_speed", "target_speed", "command_speed"]
)

pubs = {
    name: node.create_publisher(Float64, prefix + name, 10)
    for name in names
}


def emit(name, value):
    value = float(value)
    if math.isfinite(value):
        pubs[name].publish(Float64(data=value))


if task == "2":
    def on_ir(message):
        for reading in message.readings:
            frame = reading.header.frame_id.rsplit("/", 1)[-1]
            if frame.startswith("ir_intensity_"):
                name = "ir_" + frame[len("ir_intensity_"):]
                if name in pubs:
                    emit(name, reading.value)

    node.create_subscription(
        IrIntensityVector,
        C.IR_TOPIC,
        on_ir,
        qos_profile_sensor_data,
    )

    node.create_timer(
        0.1,
        lambda: emit("threshold", C.IR_OBSTACLE_THRESHOLD),
    )

else:
    node.create_subscription(
        Odometry,
        C.ODOM_TOPIC,
        lambda msg: emit("measured_speed", msg.twist.twist.linear.x),
        qos_profile_sensor_data,
    )

    node.create_subscription(
        Twist,
        C.CMD_VEL_TOPIC,
        lambda msg: emit("command_speed", msg.linear.x),
        qos_profile_sensor_data,
    )

    node.create_timer(
        0.1,
        lambda: emit("target_speed", C.PID_TARGET_SPEED),
    )

try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY

workers+=("$!")

plot_args=()
for curve in "${curves[@]}"; do
    plot_args+=("/TTB10/lab_plot/$curve/data")
done

# Require real sensor data before starting motion.
echo "Waiting for sensor data; the task has not started."

ready_topic="/TTB10/lab_plot/${curves[0]}"

if ! timeout 20s ros2 topic echo "$ready_topic" \
    std_msgs/msg/Float64 --once >/dev/null; then
    echo "No sensor data received. Check IR/odom and try again."
    exit 1
fi

# Task 2 records all topics; Task 3 records speed-related topics.
if [[ "$task" == "2" ]]; then
    record_args=(-a)
else
    record_args=(
        /TTB10/odom
        /TTB10/cmd_vel
        /TTB10/lab_plot/measured_speed
        /TTB10/lab_plot/target_speed
        /TTB10/lab_plot/command_speed
    )
fi

ros2 bag record -o "$run_dir/bag" "${record_args[@]}" \
    </dev/null >"$run_dir/record.log" 2>&1 &
workers+=("$!")

ros2 run rqt_plot rqt_plot -e "${plot_args[@]}" \
    >"$run_dir/plot.log" 2>&1 &
workers+=("$!")

sleep 2

for pid in "${workers[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "A helper exited. Check logs in $run_dir"
        exit 1
    fi
done

echo "Starting Task $task. Press Ctrl+C in this terminal to stop."
echo "Recording to: $run_dir/bag"

ros2 launch py_amr_ttb "$launch_file" &
control_pid=$!

# Closing the plot or losing a helper also ends the run.
wait -n "$control_pid" "${workers[@]}"
