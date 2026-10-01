#!/usr/bin/env bash
set -e

task="${1:-}"

case "$task" in
  2)
    launch="task2_wander.launch.py"
    curves=(ir_front_left ir_front_center_left ir_front_center_right ir_front_right threshold)
    ;;
  3)
    launch="task3_pid.launch.py"
    curves=(measured_speed target_speed command_speed)
    ;;
  *)
    echo "Usage: bash ~/amr_ws/run_lab.sh 2|3"
    exit 1
    ;;
esac

source /opt/ros/humble/setup.bash
source ~/amr_ws/install/setup.bash

run_dir="$HOME/amr_ws/lab_runs/task${task}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"

pids=()

cleanup() {
  trap - INT TERM EXIT
  echo
  echo "Stopping..."

  for pid in "${pids[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done

  sleep 1

  for pid in "${pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done

  wait 2>/dev/null || true
  echo "Saved to: $run_dir"
}

trap cleanup INT TERM EXIT


# --------------------------------------------------
# Create simple topics for plotting
# --------------------------------------------------

python3 - "$task" <<'PY' &
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


def make_pub(name):
    return node.create_publisher(Float64, prefix + name, 10)


if task == "2":

    names = [
        "ir_front_left",
        "ir_front_center_left",
        "ir_front_center_right",
        "ir_front_right",
        "threshold",
    ]

    pubs = {name: make_pub(name) for name in names}

    def publish(name, value):
        pubs[name].publish(Float64(data=float(value)))

    def ir_callback(msg):
        for r in msg.readings:
            frame = r.header.frame_id.rsplit("/", 1)[-1]

            if frame.startswith("ir_intensity_"):
                name = "ir_" + frame[len("ir_intensity_"):]

                if name in pubs:
                    publish(name, r.value)

    node.create_subscription(
        IrIntensityVector,
        C.IR_TOPIC,
        ir_callback,
        qos_profile_sensor_data,
    )

    node.create_timer(
        0.1,
        lambda: publish("threshold", C.IR_OBSTACLE_THRESHOLD)
    )


else:

    names = [
        "measured_speed",
        "target_speed",
        "command_speed",
    ]

    pubs = {name: make_pub(name) for name in names}

    def publish(name, value):
        pubs[name].publish(Float64(data=float(value)))

    node.create_subscription(
        Odometry,
        C.ODOM_TOPIC,
        lambda m: publish(
            "measured_speed",
            m.twist.twist.linear.x
        ),
        qos_profile_sensor_data,
    )

    node.create_subscription(
        Twist,
        C.CMD_VEL_TOPIC,
        lambda m: publish(
            "command_speed",
            m.linear.x
        ),
        qos_profile_sensor_data,
    )

    node.create_timer(
        0.1,
        lambda: publish(
            "target_speed",
            C.PID_TARGET_SPEED
        )
    )


try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

pids+=("$!")


# --------------------------------------------------
# Plot topics
# --------------------------------------------------

plot_args=()

for curve in "${curves[@]}"; do
  plot_args+=("/TTB10/lab_plot/$curve/data")
done


echo "Waiting for sensor data..."

first_topic="/TTB10/lab_plot/${curves[0]}"

if ! timeout 20s ros2 topic echo \
    "$first_topic" std_msgs/msg/Float64 --once >/dev/null; then

  echo "No sensor data."
  exit 1
fi


# --------------------------------------------------
# Record bag
# --------------------------------------------------

if [[ "$task" == "2" ]]; then

  ros2 bag record -a \
    -o "$run_dir/bag" \
    >"$run_dir/bag.log" 2>&1 &

else

  ros2 bag record \
    /TTB10/odom \
    /TTB10/cmd_vel \
    /TTB10/lab_plot/measured_speed \
    /TTB10/lab_plot/target_speed \
    /TTB10/lab_plot/command_speed \
    -o "$run_dir/bag" \
    >"$run_dir/bag.log" 2>&1 &

fi

pids+=("$!")


# Give ROS discovery a moment
sleep 1


# --------------------------------------------------
# Open plot
# --------------------------------------------------

ros2 run rqt_plot rqt_plot \
  -e "${plot_args[@]}" \
  >"$run_dir/plot.log" 2>&1 &

pids+=("$!")


sleep 1


# --------------------------------------------------
# Start task
# --------------------------------------------------

echo
echo "Starting Task $task"
echo "Press Ctrl+C to stop."
echo "Data: $run_dir"
echo

ros2 launch py_amr_ttb "$launch" &

pids+=("$!")


# End when one of the main programs exits
wait -n "${pids[@]}"
