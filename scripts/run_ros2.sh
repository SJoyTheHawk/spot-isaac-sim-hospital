#!/usr/bin/env bash
set -euo pipefail
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/jazzy/setup.bash

# Source the built workspace if available
if [[ -f "$REPO_DIR/ros2_ws/install/setup.bash" ]]; then
    source "$REPO_DIR/ros2_ws/install/setup.bash"
else
    echo "[run_ros2.sh] Workspace not built yet. Run:"
    echo "  cd $REPO_DIR/ros2_ws && colcon build --packages-select spot_hospital_bringup"
    exit 1
fi

ros2 launch spot_hospital_bringup robot_state_publisher.launch.py
