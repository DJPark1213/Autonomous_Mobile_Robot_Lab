#!/usr/bin/env bash
set -euo pipefail

robot_host=ubuntu@192.168.0.210
socket_dir=$(mktemp -d)
socket="$socket_dir/ssh"

cleanup() {
    ssh -S "$socket" -O exit "$robot_host" >/dev/null 2>&1 || true
    rmdir "$socket_dir"
}
trap cleanup EXIT

echo "Connecting to TTB10 ($robot_host). Enter the SSH password if prompted."
ssh -o ConnectTimeout=5 -M -S "$socket" -fnN "$robot_host"
echo "Measuring the TTB10 clock offset."
before=$(date +%s.%N)
robot_time=$(timeout 5s ssh -S "$socket" -o BatchMode=yes -o ConnectTimeout=5 \
    "$robot_host" 'date +%s.%N') || {
    echo "Could not read the clock from TTB10." >&2
    exit 1
}
after=$(date +%s.%N)

if ! offset=$(LC_ALL=C awk -v before="$before" -v robot="$robot_time" -v after="$after" '
    BEGIN {
        if (robot !~ /^[0-9]+\.[0-9]+$/ || after < before || after - before > 0.08)
            exit 1
        printf "%.6f", (before + after) / 2 - robot
    }'); then
    echo "Could not measure the robot clock within 80 ms. Run this script again." >&2
    exit 1
fi

cleanup
trap - EXIT
echo "Launching Lab 3 with a clock offset of $offset seconds."
exec ros2 launch py_amr_ttb lab3_go_to_goal.launch.py sensor_clock_offset:="$offset"
