#!/usr/bin/env bash
# Launch Spot's ROS 2 bridge plus Isaac people simulation in one Isaac Sim app.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

ENV_FILE="$REPO_DIR/env/spot_isaac.env"
if [[ -f "$ENV_FILE" ]]; then
    set -a; source "$ENV_FILE"; set +a
fi

: "${ISAAC_SIM_PATH:=$HOME/isaac-sim}"
: "${ROS_DISTRO:=jazzy}"
: "${SPOT_PEOPLE_USD:=$REPO_DIR/assets/isaac_hospital_scene_spot_w_characters.usd}"
: "${PEOPLE_COMMAND_FILE:=/tmp/spot_isaac_people_runtime_commands.txt}"
: "${PEOPLE_INITIAL_COMMANDS:=$REPO_DIR/assets/people_initial_commands.yaml}"
export ISAAC_SIM_PATH ROS_DISTRO SPOT_PEOPLE_USD PEOPLE_COMMAND_FILE PEOPLE_INITIAL_COMMANDS

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_VERSION ROS_PYTHON_VERSION
unset ROS_LOCALHOST_ONLY ROS_DOMAIN_ID

_strip_ros() {
    echo "$1" | tr ':' '\n' \
        | grep -v '/opt/ros/' \
        | grep -v '/ros2_ws' \
        | paste -sd ':' -
}
export PATH=$(_strip_ros "${PATH:-}")
export LD_LIBRARY_PATH=$(_strip_ros "${LD_LIBRARY_PATH:-}")
export PYTHONPATH=$(_strip_ros "${PYTHONPATH:-}")

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAAC_SIM_PATH}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"

cd "$ISAAC_SIM_PATH"
exec ./python.sh "$REPO_DIR/isaac_sim/spot_bridge_with_people.py" "$@"
