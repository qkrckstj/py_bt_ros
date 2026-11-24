#!/usr/bin/env bash
# Laptop-side helper for one real LIMO. The robot never moves until `start`.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IP="${LIMO_IP:-172.20.10.5}"
KEY="${LIMO_SSH_KEY:-$HOME/.ssh/limo_twin_ed25519}"
REMOTE="wego@$IP"
TARGET=/home/wego/limo_single_exploration

if [[ ! "$IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "LIMO_IP must be an IPv4 address" >&2
  exit 2
fi

ssh_robot() {
  ssh -o BatchMode=yes -o ConnectTimeout=8 -i "$KEY" "$REMOTE" "$@"
}

case "${1:-}" in
  deploy)
    files=(
      launch/real_single_exploration_launch.py
      resource/real_slam.yaml resource/real_single_nav2.yaml
      scripts/real_single_bt.py
      webots_ros2_limo/__init__.py
      webots_ros2_limo/exploration_bt.py
      webots_ros2_limo/exploration_policy.py
      webots_ros2_limo/exploration_radio.py
      webots_ros2_limo/exploration_ros.py
      webots_ros2_limo/exploration_runtime.py
      webots_ros2_limo/exploration_sweep.py
      webots_ros2_limo/slam_scan_adapter.py
    )
    tar -C "$ROOT" -cf - "${files[@]}" | ssh_robot "mkdir -p '$TARGET' && tar -C '$TARGET' -xf -"
    echo "Deployed to $REMOTE:$TARGET"
    ;;
  check)
    ssh_robot "bash --noprofile --norc -c 'source /opt/ros/humble/setup.bash; source /home/wego/wego_ws/install/setup.bash; export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0; python3 -c \"from scipy import ndimage; from nav2_msgs.action import NavigateToPose; print(\\\"Python dependencies OK\\\")\"; ros2 pkg prefix slam_toolbox; ros2 pkg prefix nav2_bringup; ros2 node list'"
    ;;
  run|run-existing)
    mode=false
    [[ "$1" == run ]] && mode=true
    if ssh_robot "bash --noprofile --norc -c 'source /opt/ros/humble/setup.bash; source /home/wego/wego_ws/install/setup.bash; export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0; ros2 node list 2>/dev/null | grep -q /exploration_team'"; then
      echo "Exploration is already running on $REMOTE; use status or start" >&2
      exit 1
    fi
    ssh -tt -o BatchMode=yes -o ConnectTimeout=8 -i "$KEY" "$REMOTE" \
      "bash --noprofile --norc -c 'source /opt/ros/humble/setup.bash; source /home/wego/wego_ws/install/setup.bash; export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0; ros2 launch $TARGET/launch/real_single_exploration_launch.py hardware:=$mode'"
    ;;
  start|stop)
    ssh_robot "bash --noprofile --norc -c 'source /opt/ros/humble/setup.bash; source /home/wego/wego_ws/install/setup.bash; export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0; ros2 topic pub --once /exploration/$1 std_msgs/msg/Empty \"{}\"'"
    ;;
  status)
    ssh_robot "bash --noprofile --norc -c 'source /opt/ros/humble/setup.bash; source /home/wego/wego_ws/install/setup.bash; export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0; timeout 8 ros2 topic echo -f --once /exploration/team_status std_msgs/msg/String'"
    ;;
  *)
    echo "Usage: $0 {deploy|check|run|run-existing|start|stop|status}" >&2
    exit 2
    ;;
esac
