# LIMO 실시간 매핑

팀의 Webots LIMO 3대 시뮬레이터에서 `robot1`을 주행시키며 사전 지도 없이 실시간 지도를 만듭니다. `robot2`와 `robot3`는 시작 위치에서 대기합니다.

## 매핑 방식

1. `live_mapping`이 `/robot1/scan`의 각도 간격을 라이다 빔 개수에 맞춰 보정한 뒤 `/robot1/scan_slam`으로 전달합니다.
2. SLAM Toolbox가 라이다 스캔과 `robot1`의 오도메트리 TF를 이용해 `/robot1/map`을 생성합니다.
3. `live_mapping`이 `config/environment.yaml`의 시작 위치와 방향을 사용해 지도를 공통 `map` 좌표계에 배치하고, 해상도 0.05 m의 `/map`으로 발행합니다. Nav2와 RViz가 이 지도를 구독합니다.

## 실행 방법

ROS 2 Jazzy, Webots R2025a, `webots_ros2_driver`, Nav2, RViz 2, SLAM Toolbox가 설치된 환경에서 실행합니다. 저장소 루트에서 패키지를 빌드합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros_packages --symlink-install
source install/setup.bash

ros2 launch multirobot_sim multirobot_sim.launch.py \
  robots:=3 slam:=true mapping_robot:=robot1 yolo:=false pygame:=false
```

1. `Shared live map is ready from robot1`과 Nav2의 `Managed nodes are active` 메시지를 확인합니다.
2. RViz에서 **2D Goal Pose**로 로봇 위치와 연결된 흰색 통로에 가까운 목표를 지정합니다. 화살표 방향으로 도착 자세를 지정할 수 있습니다.
3. 로봇이 이동하며 지도가 갱신되면 다음 관측 구간에 목표를 지정합니다.

## 지도 저장

시뮬레이터를 켜 둔 상태에서 같은 ROS 통신 설정으로 별도 터미널을 엽니다. Docker에서 호스트로 지도를 꺼내려면 `LIMO_MAP_DIR=/work/maps`처럼 호스트에 연결된 경로를 지정합니다.

```bash
source /opt/ros/jazzy/setup.bash
limo_maps="${LIMO_MAP_DIR:-$HOME/limo_maps}"
mkdir -p "$limo_maps"
limo_map="$limo_maps/limo_factory_live_$(date +%Y%m%d_%H%M%S)"

ros2 run nav2_map_server map_saver_cli \
  -t /map -f "$limo_map" --fmt pgm \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true &&
ls -lh "${limo_map}.pgm" "${limo_map}.yaml"
```

저장할 때마다 위 블록을 실행해 현재 시간으로 파일명을 정합니다. `.pgm`은 지도 이미지이고 `.yaml`에는 지도 해상도와 원점 등 메타데이터가 들어 있습니다.

## 시연 결과

Webots에서 `robot1`의 Nav2 주행과 실시간 지도 갱신, PGM·YAML 저장을 확인했습니다. 출입구 부근에서 제자리 회전을 반복할 때 지도에 이미 그려진 벽이 기울거나 겹치는 현상이 관찰됐습니다.
