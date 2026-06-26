#!/usr/bin/env bash
# Launch a standalone people-control test scene without touching the Spot runtime.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"

ENV_FILE="$REPO_DIR/env/spot_isaac.env"
if [[ -f "$ENV_FILE" ]]; then
    set -a; source "$ENV_FILE"; set +a
fi

: "${ISAAC_SIM_PATH:=$HOME/isaac-sim}"
: "${PEOPLE_TEST_USD:=$REPO_DIR/assets/isaac_hospital_scene_spot_w_characters_6.usd}"

export PEOPLE_TEST_USD

cd "$ISAAC_SIM_PATH"
exec ./python.sh "$REPO_DIR/isaac_sim/people_control_sim.py" "$@"
