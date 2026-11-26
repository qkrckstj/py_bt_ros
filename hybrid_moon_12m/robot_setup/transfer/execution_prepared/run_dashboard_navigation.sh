#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "${LIMO_ROS_SETUP:-$HOME/wego_ws/install/setup.bash}"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
: "${HYBRID_LAPTOP_IP:?노트북의 현재 IPv4 주소를 export 하세요}"
: "${1:?차체 ROS 노드 이름을 입력하세요}" "${2:?차체 cmd_vel 토픽을 입력하세요}"
if [[ ! -s pairing.key ]]; then
    echo '주행 실행 중단: pairing.key 파일이 없거나 비어 있습니다.' >&2
    exit 1
fi
if [[ ! -s real_reference.json ]]; then
    echo '매핑이 실제 리모의 기준 위치를 만들 때까지 최대 30초 기다립니다.'
    deadline=$((SECONDS + 30))
    until [[ -s real_reference.json ]] || (( SECONDS >= deadline )); do
        sleep 0.25
    done
    if [[ ! -s real_reference.json ]]; then
        echo '기준 위치 생성 시간 초과: 대시보드 센서와 run_dashboard_mapping.sh를 확인하세요.' >&2
        exit 1
    fi
fi
exec ros2 launch "$PWD/real_navigation.launch.py" key_file:="$PWD/pairing.key" \
    dashboard_base:="$1" dashboard_output:="$2" allow_motion:="${3:-false}"
