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
: "${SPOT_PEOPLE_USD:=$REPO_DIR/assets/isaac_hospital_scene_spot_w_characters_6.usd}"
: "${PEOPLE_COMMAND_FILE:=/tmp/spot_isaac_people_runtime_commands.txt}"
: "${PEOPLE_INITIAL_COMMANDS:=$REPO_DIR/assets/people_initial_commands.yaml}"
: "${SPOT_ISAAC_HEADLESS:=0}"
: "${SPOT_ISAAC_RENDERER:=RaytracedLighting}"
: "${SPOT_ISAAC_ANTI_ALIASING:=1}"
: "${SPOT_ISAAC_MULTI_GPU:=0}"
: "${SPOT_ISAAC_CREATE_NEW_STAGE:=0}"
: "${SPOT_ISAAC_WIDTH:=1280}"
: "${SPOT_ISAAC_HEIGHT:=720}"
: "${SPOT_ISAAC_WINDOW_WIDTH:=1440}"
: "${SPOT_ISAAC_WINDOW_HEIGHT:=900}"
export ISAAC_SIM_PATH ROS_DISTRO SPOT_PEOPLE_USD PEOPLE_COMMAND_FILE PEOPLE_INITIAL_COMMANDS
export SPOT_ISAAC_HEADLESS SPOT_ISAAC_RENDERER SPOT_ISAAC_ANTI_ALIASING
export SPOT_ISAAC_MULTI_GPU SPOT_ISAAC_CREATE_NEW_STAGE
export SPOT_ISAAC_WIDTH SPOT_ISAAC_HEIGHT SPOT_ISAAC_WINDOW_WIDTH SPOT_ISAAC_WINDOW_HEIGHT

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

_append_env_path() {
    local var_name="$1"
    local path="$2"
    [[ -d "$path" ]] || return 0
    if [[ -n "${!var_name:-}" ]]; then
        export "$var_name=${!var_name}:$path"
    else
        export "$var_name=$path"
    fi
}

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_CORE_ROOT="${ISAAC_SIM_PATH}/exts/isaacsim.ros2.core/${ROS_DISTRO}"
_append_env_path LD_LIBRARY_PATH "${ROS_CORE_ROOT}/lib"
_append_env_path PYTHONPATH "${ROS_CORE_ROOT}/rclpy"

cd "$ISAAC_SIM_PATH"
exec ./python.sh "$REPO_DIR/isaac_sim/spot_bridge_with_people.py" "$@"
