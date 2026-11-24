#!/usr/bin/env bash
set -e
LUNAR_REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$LUNAR_REPO_DIR"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ ! -x lunar_cp_env/bin/python ]; then
  /usr/bin/python3 -m venv lunar_cp_env
fi
if ! lunar_cp_env/bin/python -c 'import ortools, yaml, cv2' >/dev/null 2>&1; then
  lunar_cp_env/bin/python -m pip install -r scenarios/lunar_exploration/requirements.txt
fi
colcon --log-base artifacts/lunar_build_logs build \
  --base-paths ros_packages/moon_mapping_demo \
  --build-base build_lunar --install-base install_lunar --symlink-install
source install_lunar/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-64}"
exec ros2 launch scenarios/lunar_exploration/four_robot_nav.launch.py \
  explore:=true rviz:=false "$@"
