#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "${LIMO_ROS_SETUP:-$HOME/wego_ws/install/setup.bash}"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
: "${HYBRID_LAPTOP_IP:?노트북의 현재 IPv4 주소를 export 하세요}"

exec 9>.dashboard_mapping.lock
flock -n 9 || { echo '대시보드 매핑이 이미 실행 중입니다.' >&2; exit 1; }
nodes=$(timeout 10s ros2 node list --no-daemon)
if printf '%s\n' "$nodes" | grep -Eq '/(slam_toolbox|slam_scan_adapter|hybrid_readonly_observer|hybrid_physical_executor)$'; then
    echo '기존 매핑·관찰기·실제 주행 실행기를 먼저 종료하세요.' >&2
    exit 1
fi
if ! timeout 8s ros2 topic echo --once --field header --qos-reliability best_effort /scan sensor_msgs/msg/LaserScan; then
    echo '대시보드에서 센서를 활성화한 뒤 /scan을 확인하세요.' >&2
    exit 1
fi
if [ -f real_reference.json ]; then
    backup=$(mktemp ./real_reference.backup.XXXXXX)
    mv -- real_reference.json "$backup"
fi
exec ros2 launch "$PWD/dashboard_mapping.launch.py"
