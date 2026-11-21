#!/usr/bin/env bash
# Run on the real LIMO in the SAME ROS environment/container as its drivers.
set -euo pipefail
twin_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$twin_root${PYTHONPATH:+:$PYTHONPATH}"

case "${1:-help}" in
  check)
    printf 'User: %s\nROS: %s\nRMW: %s\nDOMAIN: %s\n' "$(id -un)" "${ROS_DISTRO:-unset}" "${RMW_IMPLEMENTATION:-default}" "${ROS_DOMAIN_ID:-0}"
    ros2 topic list -t
    ros2 topic info /scan --verbose
    ros2 topic info /cmd_vel --verbose
    ;;
  map)
    exec ros2 launch slam_toolbox online_async_launch.py \
      use_sim_time:=false autostart:=true use_lifecycle_manager:=false \
      "slam_params_file:=$twin_root/resource/real_slam.yaml"
    ;;
  save)
    twin_prefix="${2:?Usage: twin_robot.sh save /absolute/path/map_name}"
    mkdir -p -- "$(dirname -- "$twin_prefix")"
    if [[ -e "$twin_prefix.yaml" || -e "$twin_prefix.pgm" ]]; then
      printf 'Map already exists; choose a new name: %s\n' "$twin_prefix" >&2
      exit 1
    fi
    exec ros2 run nav2_map_server map_saver_cli -f "$twin_prefix" \
      --ros-args -p save_map_timeout:=20.0
    ;;
  feed)
    shift
    exec python3 -m webots_ros2_limo.twin_bridge source "$@"
    ;;
  *)
    printf '%s\n' 'Source your existing real ROS environment first.' \
      'check: inspect current topics/environment' \
      'map: start only SLAM (existing lidar/odom drivers required)' \
      'save /absolute/path/map_name: save active /map, keeping SLAM running' \
      'feed --target PC_IP [--cmd-topic /cmd_vel --cmd-type twist]: read-only relay'
    ;;
esac
