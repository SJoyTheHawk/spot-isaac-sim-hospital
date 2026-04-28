#!/usr/bin/env bash
# Launch the Spot hospital simulation in Isaac Sim.
#
# Strips ROS sourcing from the environment, then runs
# isaac_sim/spot_standalone.py via Isaac Sim's bundled python.sh.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# ── 1. Load local env overrides if present ─────────────────────────────────
ENV_FILE="$REPO_DIR/env/spot_isaac.env"
if [[ -f "$ENV_FILE" ]]; then
    set -a; source "$ENV_FILE"; set +a
fi

# ── 2. Defaults ─────────────────────────────────────────────────────────────
: "${ISAAC_SIM_PATH:=$HOME/isaac-sim}"
: "${ROS_DISTRO:=jazzy}"
: "${HOSPITAL_USD:=$REPO_DIR/assets/isaac_hospital_scene_spot.usd}"
export ISAAC_SIM_PATH ROS_DISTRO HOSPITAL_USD

# ── 3. Strip ROS-specific variables from the environment ───────────────────
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_VERSION ROS_PYTHON_VERSION
unset ROS_LOCALHOST_ONLY ROS_DOMAIN_ID

# ── 4. Scrub ROS paths from PATH, LD_LIBRARY_PATH, PYTHONPATH ──────────────
_strip_ros() {
    echo "$1" | tr ':' '\n' \
        | grep -v '/opt/ros/' \
        | grep -v '/ros2_ws' \
        | paste -sd ':' -
}
export PATH=$(_strip_ros "${PATH:-}")
export LD_LIBRARY_PATH=$(_strip_ros "${LD_LIBRARY_PATH:-}")
export PYTHONPATH=$(_strip_ros "${PYTHONPATH:-}")

# ── 5. Re-export Isaac Sim / ROS bridge variables ──────────────────────────
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAAC_SIM_PATH}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"

# ── 6. Run the standalone script ───────────────────────────────────────────
SCRIPT="${1:-$REPO_DIR/isaac_sim/spot_standalone.py}"
shift || true

cd "$ISAAC_SIM_PATH"
exec ./python.sh "$SCRIPT" "$@"
