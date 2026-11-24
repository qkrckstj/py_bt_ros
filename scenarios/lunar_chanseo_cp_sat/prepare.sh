#!/usr/bin/env bash
# One-time preparation for a fresh checkout. Requires ROS Humble + Webots.
set -eo pipefail
LUNAR_SETUP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LUNAR_SETUP_REPO="$(cd "$LUNAR_SETUP_DIR/../.." && pwd)"
cd "$LUNAR_SETUP_REPO"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
python3 -m venv --system-site-packages "$LUNAR_SETUP_REPO/lunar_cp_env"
"$LUNAR_SETUP_REPO/lunar_cp_env/bin/python" -m pip install -r "$LUNAR_SETUP_REPO/scenarios/lunar_exploration/requirements.txt" 'scipy>=1.15,<1.16'
colcon build --base-paths ros_packages/multirobot_sim ros_packages/moon_mapping_demo \
  --build-base build_lunar --install-base install_lunar --symlink-install
printf 'Prepared. Run: python3 scenarios/lunar_chanseo_cp_sat/manage.py start\n'
