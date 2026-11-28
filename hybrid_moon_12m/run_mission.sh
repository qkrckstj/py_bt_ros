#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source "${HYBRID_ROS_SETUP:-../install/local_setup.bash}"
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
/usr/bin/python3 - <<'CHECK'
import json
import socket
from pathlib import Path

port = json.loads(Path('config/environment.json').read_text())['real_map_udp_port']
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
    try:
        probe.bind(('0.0.0.0', port))
    except OSError as exc:
        raise SystemExit(f'지도 수신 포트 UDP {port} 사용 중: 기존 Webots 또는 별도로 실행한 mapping.py를 종료한 뒤 다시 실행하세요. ({exc})')
CHECK
exec ros2 launch "$PWD/mission.launch.py"
