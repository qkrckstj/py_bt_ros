#!/usr/bin/env bash
# Install only the reviewed launch file. Does not start ROS nodes or Webots.
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo 'Run: sudo bash scripts/install_webots_launch.sh humble  (or jazzy)' >&2
    exit 1
fi

if [[ $# -gt 1 ]]; then
    echo 'Usage: sudo bash scripts/install_webots_launch.sh [humble|jazzy]' >&2
    exit 1
fi
ros_distro="${1:-${ROS_DISTRO:-}}"
# sudo commonly removes ROS_DISTRO. Auto-detect only an unambiguous install.
if [[ -z "$ros_distro" ]]; then
    candidates=()
    for candidate in humble jazzy; do
        if [[ -f "/opt/ros/$candidate/share/webots_ros2_turtlebot/launch/robot_launch.py" ]]; then
            candidates+=("$candidate")
        fi
    done
    if [[ ${#candidates[@]} -ne 1 ]]; then
        echo 'Specify the distribution: sudo bash scripts/install_webots_launch.sh humble  (or jazzy)' >&2
        exit 1
    fi
    ros_distro="${candidates[0]}"
fi
case "$ros_distro" in
    humble|jazzy) ;;
    *) echo "Unsupported ROS distribution: $ros_distro (use humble or jazzy)" >&2; exit 1 ;;
esac

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
target="/opt/ros/$ros_distro/share/webots_ros2_turtlebot/launch/robot_launch.py"
backup="${target}.before_py_bt_ros"

if [[ ! -f "$target" ]]; then
    echo "ROS 2 $ros_distro Webots launch file not found: $target" >&2
    exit 1
fi
if [[ ! -e "$backup" ]]; then
    cp -p -- "$target" "$backup"
fi
install -m 0644 -- "$repo_root/launch/robot_launch.py" "$target"
echo "Updated: $target"
echo "Original backup: $backup"
echo 'Ready: ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true'
