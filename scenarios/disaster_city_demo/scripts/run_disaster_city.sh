#!/usr/bin/env bash
set -eo pipefail
CITY_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
# Optional workspace supplying webots_ros2_driver on a source installation.
CITY_ROS_OVERLAY="${CITY_ROS_OVERLAY:-$HOME/limo_multi_allocator_ws/install/setup.bash}"
if [ -f "$CITY_ROS_OVERLAY" ]; then source "$CITY_ROS_OVERLAY"; fi
export PYTHONPATH="$CITY_REPO:${PYTHONPATH:-}"
export PYTHONFAULTHANDLER=1
export CITY_SCENARIO_DIR="${CITY_SCENARIO_DIR:-$CITY_REPO/disaster_city/profiles/junyong_10_damage}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-168}" ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$CITY_REPO/resource/exploration_fastdds.xml"
if [ -z "${WEBOTS_HOME:-}" ] && [ -d "$HOME/.ros/webotsR2025a/webots" ]; then
  export WEBOTS_HOME="$HOME/.ros/webotsR2025a/webots"
fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/limo_city_ros_logs}"
exec 9>/tmp/limo_disaster_runtime.lock
flock -n 9 || { echo 'City simulation is already running'; exit 1; }
mkdir -p "$CITY_REPO/disaster_city/runtime"
if [ -f "$CITY_REPO/disaster_city/runtime/events.jsonl" ]; then
  mv "$CITY_REPO/disaster_city/runtime/events.jsonl" "$CITY_REPO/disaster_city/runtime/events_$(date +%Y%m%d_%H%M%S).jsonl"
fi
chmod +x "$CITY_REPO/disaster_city/supervisor.py"
exec ros2 launch "$CITY_REPO/disaster_city/city.launch.py"
