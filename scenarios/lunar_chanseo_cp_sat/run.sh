#!/usr/bin/env bash
set -eo pipefail
LUNAR_INTEGRATION_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LUNAR_INTEGRATION_REPO="$(cd "$LUNAR_INTEGRATION_DIR/../.." && pwd)"
if [[ -z "${LUNAR_LEGACY_REPO:-}" ]]; then
  if [[ -f "$LUNAR_INTEGRATION_REPO/install_lunar/setup.bash" ]]; then
    export LUNAR_LEGACY_REPO="$LUNAR_INTEGRATION_REPO"
  elif [[ -f "$(dirname "$LUNAR_INTEGRATION_REPO")/py_bt_ros/install_lunar/setup.bash" ]]; then
    export LUNAR_LEGACY_REPO="$(dirname "$LUNAR_INTEGRATION_REPO")/py_bt_ros"
  else
    export LUNAR_LEGACY_REPO="$LUNAR_INTEGRATION_REPO"
  fi
fi
if [[ ! -f "$LUNAR_LEGACY_REPO/install_lunar/setup.bash" || ! -x "$LUNAR_LEGACY_REPO/lunar_cp_env/bin/python" ]]; then
  echo "Run scenarios/lunar_chanseo_cp_sat/prepare.sh once before starting this checkout." >&2
  exit 1
fi
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$LUNAR_LEGACY_REPO/install_lunar/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-64}"
mkdir -p "$LUNAR_INTEGRATION_REPO/artifacts/lunar_chanseo_cp_sat"
exec 9>"$LUNAR_INTEGRATION_REPO/artifacts/lunar_chanseo_cp_sat/runtime.lock"
if ! flock -n 9; then
  echo "Another lunar demonstration is running. Stop its terminal before starting a new method." >&2
  exit 1
fi
exec ros2 launch "$LUNAR_INTEGRATION_DIR/lunar.launch.py" explore:=false "$@"
