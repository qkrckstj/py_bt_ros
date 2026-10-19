#!/usr/bin/env bash
# Install only the reviewed launch file. Does not start ROS nodes or Webots.
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo 'Run: sudo bash scripts/install_webots_launch.sh' >&2
    exit 1
fi

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
target=/opt/ros/jazzy/share/webots_ros2_turtlebot/launch/robot_launch.py
backup="${target}.before_py_bt_ros"

if [[ ! -f "$target" ]]; then
    echo "ROS 2 Jazzy Webots launch file not found: $target" >&2
    exit 1
fi
if [[ ! -e "$backup" ]]; then
    cp -p -- "$target" "$backup"
fi
install -m 0644 -- "$repo_root/launch/robot_launch.py" "$target"
echo "Updated: $target"
echo "Original backup: $backup"
echo 'Ready: ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true'
