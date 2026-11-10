#!/usr/bin/env bash
set -euo pipefail

echo "Launching Lab 3 using local sensor receipt times."
exec ros2 launch py_amr_ttb lab3_go_to_goal.launch.py "$@"
