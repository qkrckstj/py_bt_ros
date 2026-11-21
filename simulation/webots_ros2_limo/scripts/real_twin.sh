#!/usr/bin/env bash
# Laptop-side entry points. Each long-running command belongs in its own terminal.
set -euo pipefail
twin_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
twin_ssh=(ssh -i "${TWIN_SSH_KEY:-$HOME/.ssh/limo_twin_ed25519}" -o BatchMode=yes -o ConnectTimeout=5)
twin_robot_ip="${TWIN_ROBOT_IP:-172.20.10.5}"
twin_map_dir="${TWIN_MAP_DIR:-$twin_root/resource/real_maps}"
twin_map_pgm="$twin_map_dir/limo_real.pgm"
twin_map_yaml="$twin_map_dir/limo_real_sim.yaml"
[[ "$twin_robot_ip" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo 'Invalid robot IPv4 address'; exit 1; }
twin_target="wego@$twin_robot_ip"
twin_env='source /opt/ros/humble/setup.bash; source /home/wego/wego_ws/install/setup.bash; export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=0;
twin_running() { local twin_pid; read -r twin_pid 2>/dev/null < "$1" || return 1; [[ "$twin_pid" =~ ^[0-9]+$ ]] || return 1; [[ "$(ps -p "$twin_pid" -o args=)" == *"$2"* ]]; };'

case "${1:-help}" in
  teleop)
    exec "${twin_ssh[@]}" -tt "$twin_target" "$twin_env exec /usr/bin/python3 /home/wego/limo_twin_robot/scripts/twin_keyboard.py"
    ;;
  robot)
    # A new map can have a different origin. Do not localize with the old map
    # while displaying the new one, including after an interrupted transfer.
    for twin_map_file in "$twin_map_pgm" "$twin_map_yaml"; do
      [[ -f "$twin_map_file" ]] || { echo "Private map file missing: $twin_map_file" >&2; exit 1; }
    done
    twin_expected_map="$(sha256sum "$twin_map_pgm" "$twin_map_yaml" | awk '{print $1}')"
    twin_actual_map="$("${twin_ssh[@]}" "$twin_target" 'sha256sum /home/wego/limo_twin_robot/resource/real_maps/limo_real.pgm /home/wego/limo_twin_robot/resource/real_maps/limo_real_sim.yaml' | awk '{print $1}')"
    if [[ "$twin_expected_map" != "$twin_actual_map" ]]; then
      echo 'Real localization map differs from the current simulation. Sync resource/real_maps/limo_real.pgm and limo_real_sim.yaml to the robot first.' >&2
      exit 1
    fi
    exec "${twin_ssh[@]}" -tt "$twin_target" "$twin_env if twin_running /home/wego/limo_twin_robot/bringup.pid 'ros2 launch /home/wego/limo_twin_robot/launch/real_twin_robot.launch.py'; then echo 'Robot sensors/localization already running.'; exit 0; fi; if pgrep -f '^/home/wego/wego_ws/install/limo_base/lib/limo_base/limo_base' >/dev/null; then echo 'Existing chassis driver detected; keep it and start localization separately. Do not run another bringup.'; exit 1; fi; echo \$\$ > /home/wego/limo_twin_robot/bringup.pid; exec ros2 launch /home/wego/limo_twin_robot/launch/real_twin_robot.launch.py map:=/home/wego/limo_twin_robot/resource/real_maps/limo_real_sim.yaml"
    ;;
  feed|peers)
    twin_pc_ip="$(ip -4 route get "$twin_robot_ip" | awk '{for(i=1;i<=NF;i++)if($i=="src"){print $(i+1);exit}}')"
    [[ "$twin_pc_ip" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo 'Cannot determine laptop IP'; exit 1; }
    if [[ "$1" == peers ]]; then
      exec "${twin_ssh[@]}" -tt "$twin_target" "$twin_env if twin_running /home/wego/limo_twin_robot/peers.pid 'python3 -m webots_ros2_limo.peer_relay receive'; then echo 'Peer receiver already running.'; exit 0; fi; export PYTHONPATH=/home/wego/limo_twin_robot:\$PYTHONPATH; echo \$\$ > /home/wego/limo_twin_robot/peers.pid; exec /usr/bin/python3 -m webots_ros2_limo.peer_relay receive --peer-ip $twin_pc_ip"
    fi
    exec "${twin_ssh[@]}" -tt "$twin_target" "$twin_env if twin_running /home/wego/limo_twin_robot/feed.pid 'python3 -m webots_ros2_limo.twin_bridge source'; then echo 'Robot feed already running.'; exit 0; fi; export PYTHONPATH=/home/wego/limo_twin_robot:\$PYTHONPATH; echo \$\$ > /home/wego/limo_twin_robot/feed.pid; exec /usr/bin/python3 -m webots_ros2_limo.twin_bridge source --target $twin_pc_ip --velocity-source odometry --odom-topic /odom --rate 50"
    ;;
  sim|status)
    # ROS setup scripts do not support nounset.
    set +u
    source /opt/ros/jazzy/setup.bash
    source "${TWIN_INSTALL_SETUP:-$HOME/ros2_ws/install/local_setup.bash}"
    set -u
    export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    if [[ "$1" == status ]]; then
      exec ros2 topic echo /digital_twin/status std_msgs/msg/String --once --full-length
    fi
    if pgrep -f '[r]os2 launch webots_ros2_limo real_(twin|fleet)_launch.py' >/dev/null; then
      echo 'Webots twin already running.'; exit 0
    fi
    [[ -f "$twin_map_yaml" && -f "$twin_map_pgm" ]] || { echo "Private map files missing in $twin_map_dir" >&2; exit 1; }
    exec ros2 launch webots_ros2_limo real_fleet_launch.py \
      "map:=$twin_map_yaml" real_domain:=0 "robot_ip:=$twin_robot_ip" wall_height:=0.5
    ;;
  *)
    printf '%s\n' 'Usage: bash scripts/real_twin.sh robot|feed|peers|sim|status|teleop' \
      'robot: real sensors + localization, requires initial pose after restart' \
      'feed: read-only real command/pose relay' \
      'sim: real pose replica + two simulated peers' \
      'peers: receive virtual robot data on the real robot under /sim/*' \
      'status: one connection-status message' \
      'teleop: OPERATOR keyboard driving the real robot (hold keys; k stops)'
    ;;
esac
