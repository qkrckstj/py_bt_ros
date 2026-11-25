#!/usr/bin/env bash
# 현재 체크아웃을 Jazzy로 빌드·검사하고 Webots와 RViz를 실행한다.
set -euo pipefail

usage() {
  cat <<'EOF'
사용법: bash docker/lunar/run_demo.sh [bt-fixed|explore|explore-wide|single|preview|multi|gpu-check|status|start|stop|diagnose]
  bt-fixed 네 로봇의 기존 목표를 TeamBT·RobotBT와 시간 경로로 실행 (기본)
  explore  400 m² 고정 영역에서 센서 기반 반복 탐사 (180 시뮬레이션 초)
  explore-wide 3364 m² 영역의 장시간 평가 (단기 검증 후 사용)
  single   robot1의 기존 단일 목표 회귀 주행
  preview  기본 4대 경로 시각화, 정지 유지
  multi    robot1~4의 기존 실행기 다중 목표 주행
  gpu-check GPU와 OpenGL 렌더러만 확인, 시뮬레이터는 시작하지 않음
  status   현재 실행 중인 임무 상태 확인
  start    열린 시뮬레이터에서 현재 위치·시각으로 임무 다시 시작
  stop     현재 임무 정지, 시뮬레이터 창 유지
  diagnose 12초 동안 clock·4대 odom 수신 간격 측정
화면을 사용할 수 있는 호스트 터미널에서 실행합니다. 종료는 Ctrl+C입니다.
기본은 드라이버에 맞는 GPU 자동 선택입니다. LUNAR_GPU=mesa|nvidia|software로 지정할 수 있습니다.
BT 출발 정책: LUNAR_DEPARTURE_POLICY=simultaneous|staggered_pairs (기본 simultaneous)
위험 대응 비교: LUNAR_RISK_STAGE=baseline|early|goals|candidates|heading (미지정 시 YAML 설정)
EOF
}

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  usage
  exit 0
fi
if (( $# > 1 )); then
  usage >&2
  exit 2
fi
LUNAR_DEMO_MODE=${1:-bt-fixed}
LUNAR_DEPARTURE_POLICY=${LUNAR_DEPARTURE_POLICY:-simultaneous}
LUNAR_RISK_STAGE=${LUNAR_RISK_STAGE:-}
case "$LUNAR_RISK_STAGE" in
  ''|baseline|early|goals|candidates|heading) ;;
  *) printf '%s\n' 'LUNAR_RISK_STAGE의 단계 이름을 확인하세요.' >&2; exit 2 ;;
esac
case "$LUNAR_DEPARTURE_POLICY" in
  simultaneous|staggered_pairs) ;;
  *) printf '%s\n' 'LUNAR_DEPARTURE_POLICY는 simultaneous 또는 staggered_pairs여야 합니다.' >&2; exit 2 ;;
esac
case "$LUNAR_DEMO_MODE" in
  bt-fixed|explore|explore-wide|single|preview|multi|gpu-check|status|start|stop|diagnose) ;;
  *) usage >&2; exit 2 ;;
esac

LUNAR_SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
export LUNAR_REPO
LUNAR_REPO=$(cd -- "$LUNAR_SCRIPT_DIR/../.." && pwd -P)
export LUNAR_UID=${LUNAR_UID:-$(id -u)}
export LUNAR_GID=${LUNAR_GID:-$(id -g)}
export LUNAR_ROS_DOMAIN_ID=${LUNAR_ROS_DOMAIN_ID:-63}
cd -- "$LUNAR_REPO"

if ! command -v docker >/dev/null; then
  printf '%s\n' 'Docker 실행 파일을 찾지 못했습니다.' >&2
  exit 1
fi
if ! docker info --format '{{.ServerVersion}}' >/dev/null; then
  printf '%s\n' '이 터미널에서 Docker 데몬에 접근할 수 없습니다. Docker를 사용할 수 있는 호스트 터미널에서 실행하세요.' >&2
  exit 1
fi
docker compose version >/dev/null
case "$LUNAR_DEMO_MODE" in
  status|start|stop|diagnose)
    mapfile -t LUNAR_DEMO_CONTAINERS < <(docker ps \
      --filter label=com.docker.compose.project=lunar-shadow-hungarian \
      --format '{{.ID}} {{.Label "com.docker.compose.service"}}' \
      | while read -r LUNAR_CONTAINER_ID LUNAR_CONTAINER_SERVICE; do
          case "$LUNAR_CONTAINER_SERVICE" in
            lunar-gui|lunar-gpu|lunar-dri) printf '%s\n' "$LUNAR_CONTAINER_ID" ;;
          esac
        done)
    if (( ${#LUNAR_DEMO_CONTAINERS[@]} != 1 )); then
      printf '이 프로젝트의 실행 중인 GUI 컨테이너가 정확히 1개여야 합니다. 현재: %s개\n' \
        "${#LUNAR_DEMO_CONTAINERS[@]}" >&2
      exit 1
    fi
    exec docker exec "${LUNAR_DEMO_CONTAINERS[0]}" bash -lc '
      set -eo pipefail
      source /opt/ros/jazzy/setup.bash
      source /workspace/py_bt_ros/install/jazzy/setup.bash
      cd /workspace/py_bt_ros
      if [[ $1 == diagnose ]]; then
        exec python3 ros_packages/moon_mapping_demo/tests/input_probe.py
      fi
      if [[ $1 == status ]]; then
        exec timeout 12s ros2 topic echo /shadow_mission/status \
          --qos-durability transient_local --once
      fi
      exec timeout 12s ros2 service call "/shadow_mission/$1" std_srvs/srv/Trigger "{}"
    ' lunar-demo-control "$LUNAR_DEMO_MODE"
    ;;
esac
if ! docker image inspect lunar-shadow-hungarian:jazzy-r2025a >/dev/null 2>&1; then
  printf '%s\n' '실행 이미지가 없습니다. 저장소 루트에서 먼저 다음 명령을 실행하세요:' \
    'docker compose -f docker/lunar/compose.yaml build lunar' >&2
  exit 1
fi
if [[ -z ${DISPLAY:-} ]]; then
  printf '%s\n' 'DISPLAY가 없습니다. Ubuntu 데스크톱의 터미널에서 실행하세요.' >&2
  exit 1
fi
export LUNAR_XAUTHORITY=${LUNAR_XAUTHORITY:-${XAUTHORITY:-}}
if [[ -z $LUNAR_XAUTHORITY || ! -r $LUNAR_XAUTHORITY ]]; then
  printf '%s\n' '현재 데스크톱의 읽을 수 있는 XAUTHORITY 또는 LUNAR_XAUTHORITY가 필요합니다.' >&2
  exit 1
fi
if command -v xdpyinfo >/dev/null && ! XAUTHORITY="$LUNAR_XAUTHORITY" xdpyinfo >/dev/null 2>&1; then
  printf '%s\n' '현재 DISPLAY와 XAUTHORITY로 화면에 연결할 수 없습니다.' >&2
  exit 1
fi

LUNAR_GPU=${LUNAR_GPU:-auto}
if [[ $LUNAR_GPU == auto ]]; then
  if [[ -d /sys/module/nvidia ]]; then
    LUNAR_GPU=nvidia
  else
    LUNAR_GPU=mesa
  fi
fi
case "$LUNAR_GPU" in
  nvidia) LUNAR_GUI_SERVICE=lunar-gpu ;;
  mesa)
    LUNAR_GUI_SERVICE=lunar-dri
    if [[ ! -d /dev/dri ]]; then
      printf '%s\n' 'GPU 장치 /dev/dri가 없습니다. 호스트 데스크톱 터미널에서 실행하세요.' >&2
      exit 1
    fi
    LUNAR_RENDER_DEVICE=
    for LUNAR_VENDOR_FILE in /sys/class/drm/renderD*/device/vendor; do
      [[ -r $LUNAR_VENDOR_FILE ]] || continue
      LUNAR_DEVICE_DIR=${LUNAR_VENDOR_FILE%/vendor}
      LUNAR_RENDER_DIR=${LUNAR_DEVICE_DIR%/device}
      LUNAR_RENDER_CANDIDATE="/dev/dri/${LUNAR_RENDER_DIR##*/}"
      [[ -c $LUNAR_RENDER_CANDIDATE ]] || continue
      [[ -n $LUNAR_RENDER_DEVICE ]] || LUNAR_RENDER_DEVICE=$LUNAR_RENDER_CANDIDATE
      read -r LUNAR_DEVICE_VENDOR < "$LUNAR_VENDOR_FILE"
      if [[ $LUNAR_DEVICE_VENDOR == 0x10de ]]; then
        LUNAR_RENDER_DEVICE=$LUNAR_RENDER_CANDIDATE
        LUNAR_PCI_PATH=$(readlink -f "$LUNAR_DEVICE_DIR")
        LUNAR_PCI_ADDRESS=${LUNAR_PCI_PATH##*/}
        LUNAR_PCI_ADDRESS=${LUNAR_PCI_ADDRESS//:/_}
        LUNAR_PCI_ADDRESS=${LUNAR_PCI_ADDRESS//./_}
        export LUNAR_DRI_PRIME=${LUNAR_DRI_PRIME:-pci-$LUNAR_PCI_ADDRESS}
        break
      fi
    done
    if [[ -z $LUNAR_RENDER_DEVICE ]]; then
      printf '%s\n' '사용할 수 있는 GPU renderD 장치를 찾지 못했습니다.' >&2
      exit 1
    fi
    export LUNAR_RENDER_GID=${LUNAR_RENDER_GID:-$(stat -c %g "$LUNAR_RENDER_DEVICE")}
    for LUNAR_CARD_DEVICE in /dev/dri/card*; do
      if [[ -c $LUNAR_CARD_DEVICE ]]; then
        export LUNAR_VIDEO_GID=${LUNAR_VIDEO_GID:-$(stat -c %g "$LUNAR_CARD_DEVICE")}
        break
      fi
    done
    ;;
  software) LUNAR_GUI_SERVICE=lunar-gui ;;
  *) printf '%s\n' 'LUNAR_GPU는 auto, nvidia, mesa 또는 software여야 합니다.' >&2; exit 2 ;;
esac
LUNAR_DEMO_OUTPUT="results/gui_${LUNAR_DEMO_MODE}_$(date +%Y%m%d_%H%M%S)_$$"
printf '체크아웃: %s\n모드: %s\n렌더링: %s\n결과: %s/%s\n' \
  "$LUNAR_REPO" "$LUNAR_DEMO_MODE" "$LUNAR_GPU" "$LUNAR_REPO" "$LUNAR_DEMO_OUTPUT"
mkdir -p -- "$LUNAR_DEMO_OUTPUT"
LUNAR_GRAPHICS_STACK=$(docker image inspect lunar-shadow-hungarian:jazzy-r2025a \
    --format '{{ index .Config.Labels "io.py_bt_ros.lunar.graphics" }}')
LUNAR_EXPLORATION_STACK=$(docker image inspect lunar-shadow-hungarian:jazzy-r2025a \
    --format '{{ index .Config.Labels "io.py_bt_ros.lunar.exploration" }}')
if [[ ( $LUNAR_GPU == mesa && $LUNAR_GRAPHICS_STACK != mesa-vulkan-v1 ) ||
      ( $LUNAR_DEMO_MODE != gpu-check && $LUNAR_EXPLORATION_STACK != team-bt-v1 ) ]]; then
    printf '%s\n' 'Mesa/BT 탐사 의존성을 포함하는 이미지로 증분 빌드합니다.'
    docker compose -f docker/lunar/compose.yaml build lunar 2>&1 \
      | tee "$LUNAR_DEMO_OUTPUT/image_build.log"
fi
if [[ $LUNAR_DEMO_MODE == gpu-check ]]; then
  set +e
  docker compose -f docker/lunar/compose.yaml run --rm --no-deps -T \
    "$LUNAR_GUI_SERVICE" bash docker/lunar/check_graphics.sh "$LUNAR_GPU" "$LUNAR_DEMO_OUTPUT" \
    > "$LUNAR_DEMO_OUTPUT/container.log" 2>&1
  LUNAR_CHECK_STATUS=$?
  set -e
  cat "$LUNAR_DEMO_OUTPUT/container.log"
  exit "$LUNAR_CHECK_STATUS"
fi
exec docker compose -f docker/lunar/compose.yaml run --rm --no-deps \
  --env "LUNAR_DEMO_MODE=$LUNAR_DEMO_MODE" \
  --env "LUNAR_GPU_MODE=$LUNAR_GPU" \
  --env "LUNAR_DEPARTURE_POLICY=$LUNAR_DEPARTURE_POLICY" \
  --env "LUNAR_RISK_STAGE=$LUNAR_RISK_STAGE" \
  --env "LUNAR_DEMO_OUTPUT=$LUNAR_DEMO_OUTPUT" \
  "$LUNAR_GUI_SERVICE" bash -lc '
    set -eo pipefail
    bash docker/lunar/check_graphics.sh "$LUNAR_GPU_MODE" "$LUNAR_DEMO_OUTPUT"
    source /opt/ros/jazzy/setup.bash
    colcon --log-base log/jazzy build \
      --base-paths ros_packages/multirobot_sim ros_packages/moon_mapping_demo \
      --build-base build/jazzy --install-base install/jazzy --symlink-install
    source install/jazzy/setup.bash
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider \
      ros_packages/moon_mapping_demo/tests
    config="$PWD/ros_packages/moon_mapping_demo/config/shadow_experiment.yaml"
    execute=true
    case "$LUNAR_DEMO_MODE" in
      single) config="$PWD/ros_packages/moon_mapping_demo/config/shadow_single.yaml" ;;
      preview) execute=false ;;
    esac
    export ROS_LOG_DIR="$PWD/$LUNAR_DEMO_OUTPUT/ros_logs"
    mkdir -p "$ROS_LOG_DIR"
    case "$LUNAR_DEMO_MODE" in
      bt-fixed|explore|explore-wide)
        config="$PWD/ros_packages/moon_mapping_demo/config/exploration_short.yaml"
        mode=explore
        [[ $LUNAR_DEMO_MODE != bt-fixed ]] || mode=fixed
        [[ $LUNAR_DEMO_MODE != explore-wide ]] || config="$PWD/ros_packages/moon_mapping_demo/config/exploration_wide.yaml"
        exec ros2 launch moon_mapping_demo moon_exploration.launch.py \
          rviz:=true "mode:=$mode" "config:=$config" "output:=$LUNAR_DEMO_OUTPUT" \
          "departure_policy:=$LUNAR_DEPARTURE_POLICY" "risk_stage:=$LUNAR_RISK_STAGE"
        ;;
    esac
    exec ros2 launch moon_mapping_demo moon_shadow.launch.py \
      rviz:=true "execute:=$execute" "config:=$config" "output:=$LUNAR_DEMO_OUTPUT"
  '
