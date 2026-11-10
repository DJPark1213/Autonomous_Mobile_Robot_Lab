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
coproc ROBOT_CLOCK {
    ssh -T -S "$socket" -o BatchMode=yes -o ConnectTimeout=5 "$robot_host" \
        'while IFS= read -r request; do date +%s.%N; done'
}

samples=''
for ((sample = 0; sample < 10; sample++)); do
    before=$(date +%s.%N)
    printf '\n' >&"${ROBOT_CLOCK[1]}" || break
    IFS= read -r -t 5 robot_time <&"${ROBOT_CLOCK[0]}" || break
    after=$(date +%s.%N)
    samples+="$before $robot_time $after"$'\n'
done

if ! offset=$(LC_ALL=C awk '
    $2 ~ /^[0-9]+\.[0-9]+$/ && $3 >= $1 {
        elapsed = $3 - $1
        if (!found || elapsed < fastest) {
            found = 1
            fastest = elapsed
            offset = ($1 + $3) / 2 - $2
        }
    }
    END {
        if (!found || fastest > 0.08)
            exit 1
        printf "%.6f", offset
    }' <<< "$samples"); then
    echo "Could not measure the robot clock within 80 ms. Run this script again." >&2
    exit 1
fi

cleanup
trap - EXIT
echo "Launching Lab 3 with a clock offset of $offset seconds."
exec ros2 launch py_amr_ttb lab3_go_to_goal.launch.py sensor_clock_offset:="$offset"
