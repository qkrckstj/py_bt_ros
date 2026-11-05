# 공장 사람 탐색 BT

트리 구조:
```text
ReactiveFallback
├─ Sequence: IsPerson → RecordPerson
└─ Sequence: MoveToGoal(1) → MoveToGoal(2) → MoveToGoal(3) → AlwaysFailure
```
매 틱 사람을 확인하면서 1→2→3번으로 이동한다. 사람 발견 시 주행 목표를 취소하고
사람 좌표 기록까지 성공하면 전체 SUCCESS. 모든 방을 돌고도 없으면 FAILURE.
방별 대기는 없다. 좌표가 늦게 오면 정지 상태로 최대 5초 기다린다.
감지 데이터가 5초 이상 끊기거나 좌표 기록/주행이 실패하면 오류로 종료한다.

기본 실행은 `ros_packages/human_yolo/models/best.pt`로 RGB-D 인간 더미를 감지한다.
`IsPerson`은 YOLO가 발행하는 `/perception/is_person`을 읽는다. 유효한 깊이 좌표까지
있고 신뢰도 0.85 이상인 동일 후보가 5프레임·0.6초 이상 연속 확인되면 SUCCESS다.
그 전이나 미검출/깊이 실패일 때는 FAILURE다. 기존 1m 거리 조건은
기본 실행에 사용하지 않는다. 3번 이동 목표 `(4.85, 0.65)`는 유지한다.

- `/perception/is_person`: `std_msgs/Bool`.
- `/perception/person_pose`: `geometry_msgs/PoseStamped`, RGB 촬영 시각과 카메라 광학 좌표계의 위치.
- `RecordPerson`이 촬영 시각의 카메라→map TF로 변환한 뒤 기록한다.
- 모델은 초록 상의·파란 바지 더미로 학습됐다. 현재 공장 더미의 복장/자세에서의 인식률은 별도 확인이 필요하다.
- 기존 테스트용 거리 감지를 원할 때만 `mock_detector:=true`로 실행한다. 이때 YOLO는 실행하지 않는다.

최초 1회 YOLO 환경 준비:
```bash
python3 -m venv --system-site-packages ~/.venvs/human-yolo
~/.venvs/human-yolo/bin/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
~/.venvs/human-yolo/bin/python -m pip install -r ~/ros2_ws/src/py_bt_ros/ros_packages/human_yolo/requirements.txt
```
다른 가상환경을 쓰면 `yolo_python:=/절대경로/bin/python`을 launch 인자로 전달한다.

기록은 `~/.ros/py_bt_ros/person_detections.jsonl`에 JSON 한 줄씩 누적한다.
필드: UTC, 감지 시뮬레이션 시각, mission_id, frame_id, source_frame, position(x,y,z).
`/bt/search/found_person`에도 map 기준 PoseStamped를 발행한다.
YOLO 기록 좌표는 깊이로 얻은 사람 표면의 대표점이며 바닥 중심이 아니다.
모의 감지를 명시적으로 선택했을 때만 고정 사람 좌표 `(5.55, 0.65, 0.0)`를 사용한다.

각 터미널 공통:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
```

터미널 1 (초기 위치에서 새로 시작):
```bash
ros2 launch webots_ros2_limo robot_launch.py world:=limo_factory_hall.wbt
```
터미널 2 (저장된 지도 + AMCL + Nav2 + RViz + BT + YOLO):
```bash
ros2 launch webots_ros2_limo factory_search_launch.py
```
RViz 지도/로봇 위치가 표시되고 Nav2가 active, YOLO 로그가 `Perception ready`가 된 뒤 터미널 3:
```bash
ros2 topic pub --once /bt/search/start std_msgs/msg/Empty '{}'
```
중단:
```bash
ros2 topic pub --once /bt/search/cancel std_msgs/msg/Empty '{}'
```
BT 창을 닫으면 진행 중인 목표도 취소한다. 다시 시작하려면 BT 실행 프로세스가
살아 있어야 한다. 전체 종료는 터미널 2, 터미널 1 순서로 Ctrl+C.

## 장애물과 도착 조건

추가했던 장애물 박스 두 개는 월드에서 제거했다. 추후 저장된 지도에 없는 박스를 추가해도 `/scan` → 실시간 local/global costmap으로 반영한다.
시뮬레이터의 laser_link 높이(-0.034m)를 허용하며, 사각형 로봇 footprint와
여유 공간을 사용한다. inflation_radius는 0.10m다. 이는 장애물 주변 비용 확장 반경이며, 로봇 외곽과 장애물 사이의 보장 거리와는 다르다. 2D 라이다 높이보다 낮은 물체는 이 구성만으로 감지할 수 없다.
도착은 Nav2 PositionGoalChecker 기준 XY 거리 0.30m 이내이며 목표 방향은 무시한다.
DWB의 RotateToGoal/GoalAlign도 사용하지 않는다. 허용 거리 변경:
```bash
ros2 launch webots_ros2_limo factory_search_launch.py goal_tolerance:=0.40
```

## 새 지도를 만드는 SLAM 옵션

터미널 2 명령을 다음으로 바꾼다. 기존 Nav2/AMCL 실행은 먼저 종료한다.
```bash
ros2 launch webots_ros2_limo factory_search_launch.py slam:=true
```
이 모드는 저장된 지도와 AMCL 대신 SLAM Toolbox를 사용한다. `/scan_slam` 변환 노드가
라이다 샘플 개수와 각도 간격을 일치시켜 SLAM으로 전달하며 원래 `/scan`은 유지한다. 기존 지도와 합치는
기능이 아니며, 출발 위치를 원점으로 새 지도를 만든다. BT 구역 좌표도 그 원점에
맞춰 변환한다. 시뮬레이터를 초기 상태로 재시작한 다음 실행해야 한다.
아직 관측하지 않은 먼 방은 지도 범위 밖이라 Nav2가 경로를 만들지 못할 수 있다.
전체 공장 1→2→3 탐색에는 기본 저장 지도 모드를 사용하고, SLAM 모드에서는 먼저
수동 주행 또는 RViz에서 가까운 목표를 지정하여 지도를 넓힌다.
지도 저장(별도 터미널):
```bash
ros2 run nav2_map_server map_saver_cli -f "$HOME/ros2_ws/factory_slam" --ros-args -p use_sim_time:=true
```
저장된 기본 공장 지도는 Webots 라이다 측량으로 생성한 시뮬레이션 참조 지도다.
실제 로봇에서 만든 SLAM 지도는 아니다. 공장 벽/고정 설비 변경 시 다시 생성해야 한다.
