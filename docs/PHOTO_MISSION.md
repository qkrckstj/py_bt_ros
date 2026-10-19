# TurtleBot3 사진 촬영 과제: 작동 원리

설치와 실행 순서는 [README](../README.md)를 참고한다.
이 문서는 현재 Agent·blackboard 기반 구현과 통신 흐름을 설명한다.

## 실행 구조

```text
py_bt_ros/
├── main.py                    # BT 프로세스 시작
├── config.yaml                # 토픽, 주기, 시간 제한, 도착 허용 오차
├── modules/                   # 수업의 BT 실행 기반
│   ├── bt_runner.py           # Agent 생성, 약 20 Hz로 BT tick
│   ├── agent.py               # blackboard와 트리, 임무 관리
│   ├── bt_constructor.py      # XML을 Python 노드 객체로 구성
│   ├── base_bt_nodes.py       # Status, Sequence 등 기본 노드
│   ├── base_bt_nodes_ros.py   # ROS Action/Service 클라이언트 기반
│   ├── ros_bridge.py          # BT가 공유하는 ROS 노드와 콜백 처리
│   └── utils.py               # 설정·XML 로딩
├── launch/robot_launch.py     # 기존 Webots launch의 설치용 수정본
├── scripts/install_webots_launch.sh
├── scenarios/turtlebot3_photo/
│   ├── default_bt.xml         # 과제 수행 순서
│   ├── bt_nodes.py            # 과제 Action/Condition 구현
│   ├── capture_image_server.py # 별도 ROS 촬영 서버
│   ├── launch_support.py      # 실행에 필요한 임시 설정 생성
│   ├── control_description.py # Webots 로봇 설명 전달
│   ├── front_camera.wbt       # 카메라 장치 정의
│   ├── nav2_overrides.yaml    # Nav2 설정
│   └── photo_mission.rviz     # RViz 설정
└── output/captures/           # 촬영 결과 PNG
```

기존 launch 명령을 실행하면 Webots와 로봇 드라이버가 시작되고,
로봇 연결 후 Nav2, RViz, BT 프로세스, 촬영 서버가 시작된다.
`launch_support.py`는 원본 Webots 월드 및 Nav2 설정에 필요한 변경을 합쳐
임시 디렉터리에 생성한다. 임시 디렉터리는 launch 종료 시 정리한다.

**BT 노드와 ROS 노드는 다르다.** `MoveToGoal`, `IsAtGoal`, `CaptureImage`,
`Return` 등은 한 BT 프로세스 안의 Python 객체다. 이들은 `ROSBridge`의
`photo_mission_bt` ROS 노드를 공유한다. `capture_image_server`는 별도
프로세스이자 별도 ROS 노드이며, Nav2와도 역할이 분리되어 있다.

목표가 없을 때 BT 프로세스는 ROS 콜백을 처리하며 대기한다. 목표를 받으면
`Agent.receive_goal()`이 TF에서 현재 위치·방향을 읽고, 목표와 출발점을 Agent의
`blackboard`(노드 간 공유 딕셔너리)에 저장한다. 출발점은 고정된 `(0, 0)`이
아니라 **매 임무의 목표를 수신한 순간의 위치와 방향**이다.

## Agent와 blackboard

별도 Mission 객체 없이 `Agent`가 목표 구독과 임무 시작·종료를 담당한다.

1. `/bt/goal_pose` 콜백에서 목표 좌표와 quaternion을 검사한다.
2. TF의 `map → base_link`에서 현재 위치·방향을 읽는다.
3. `goal_pose`, `home_pose`를 blackboard에 저장하고 `active=True`로 설정한다.
4. `run_tree()`가 ROS 콜백을 처리하고, 활성 임무의 BT를 반복 실행한다.
5. 트리가 성공·실패하면 `active=False`로 바꾸고 최종 상태를 발행한다.

| blackboard 키 | 내용 |
| --- | --- |
| `goal_pose` | 수신한 목적지 PoseStamped |
| `home_pose` | 목표 수신 시 저장한 출발점 PoseStamped |
| `active` | 임무 진행 여부 |
| `image_path` | 촬영 서비스가 응답한 PNG 경로 |
| `error` | 실패 원인 |
| `arrival_checks` | 도착·복귀 Condition의 거리 및 방향 오차와 통과 여부 |
| `mission_id` | 수락한 임무의 순번 |

`/bt/mission_status`는 이 정보를 외부에 알리는 출력 토픽이다.
Condition이 이 토픽을 구독해서 성공을 판단하는 구조는 아니다.
`HasGoal`은 blackboard의 `active`를 확인하며 Agent의 실행 전 검사와 중복된다.
BT XML에 목표 수신 조건을 드러내기 위해 유지한다.

## BT 구성

```text
Sequence
├── HasGoal      [Condition] 유효한 목표를 받았는가?
├── MoveToGoal   [Action]    Nav2에 목표를 전달하고 이동 결과를 기다린다.
├── IsAtGoal     [Condition] 현재 위치·헤딩이 목적지에 도달했는가?
├── CaptureImage [Action]    별도 ROS 서비스에 사진 촬영을 요청한다.
├── Return       [Action]    Nav2를 이용해 저장한 출발점으로 돌아간다.
└── IsAtHome     [Condition] 현재 위치·헤딩이 출발점에 도달했는가?
```

`MoveToGoal`, `Return`은 수업의 `ActionWithROSAction`을 상속한 Nav2 액션 클라이언트다.
Nav2 이동 중에는 `RUNNING`, 정상 종료 시에는 `SUCCESS`, 거절·실패 결과는 `FAILURE`를 반환한다.
`MoveToGoal` 안에서 목표 전송·수락·결과 확인을 처리한다. `Return`은 이를 상속하고
blackboard에서 읽을 키만 `goal_pose` 대신 `home_pose`로 바꾼다.
도착 검사는 `IsAtGoal`에 구현하고 `IsAtHome`도 같은 방식으로 출발점을 비교한다.

**도착 판단은 별도로 구현한 `IsAtGoal`, `IsAtHome` Condition에서 수행한다.**
각 Condition은 현재 `map → base_link` TF를 읽고 목표의 위치와 헤딩을 직접 비교한다.
거리와 헤딩 오차 모두 허용 범위 이내여야 `SUCCESS`이고, 그 외에는 `FAILURE`다.
Nav2의 성공 응답만으로 Condition을 통과시키지 않는다.

- `IsAtGoal` 실패: 사진 촬영과 복귀를 실행하지 않고 `FAILED`로 종료한다.
- `IsAtHome` 실패: 임무 완료로 처리하지 않고 `FAILED`로 종료한다.
- `config.yaml`의 Condition 허용 오차: 거리 0.20 m, 헤딩 0.20 rad.
- 제공된 Nav2 설정의 이동 종료 허용 오차: 거리 0.15 m, 헤딩 0.15 rad.
- 헤딩은 quaternion에서 yaw를 구하고 ±π 경계를 고려하여 비교한다.
- TF가 없거나 odometry가 오래됐으면 Condition을 실패 처리한다.

## 촬영 서비스와 결과 처리

`CaptureImage`는 Condition이 아니라 Action이다. `/bt/capture_image`에
빈 `std_srvs/srv/Trigger` 요청을 보내고, 응답을 기다리는 동안 `RUNNING`을 반환한다.
별도 촬영 서버는 외부 이미지 구독자를 확인하고 임시 구독을 생성한다.
요청 시각 이후의 프레임을 최대 3초 기다려 `cv_bridge`로 변환한 뒤 PNG로 저장한다.
처리가 끝나면 촬영용 구독을 해제한다.

서비스 응답의 `success=True`이면 `message`에 사진 경로가 담기고,
BT가 이를 blackboard에 저장해 `SUCCESS`를 반환한다.
`success=False`이면 오류를 기록하고 `FAILURE`를 반환한다.
사진 저장 성공 후에만 `Return`이 시작된다.

## 파일별 역할

| 파일 | 역할 |
| --- | --- |
| `config.yaml` | 시나리오, 토픽, 시간 제한, 도착 Condition 허용 오차 |
| `scenarios/turtlebot3_photo/default_bt.xml` | 위 순서의 BT 정의 |
| `scenarios/turtlebot3_photo/bt_nodes.py` | Action 및 Condition 노드 |
| `modules/agent.py` | 목표 구독, blackboard에 목표·출발점 저장, BT 실행 및 종료 상태 발행 |
| `scenarios/turtlebot3_photo/capture_image_server.py` | 카메라 구독 및 별도 촬영 서비스 |
| `launch/robot_launch.py` | 기존 Webots launch에 과제 자동 실행을 추가한 수정본 |
| `scripts/install_webots_launch.sh` | 기존 launch 원본 백업 및 수정본 적용 (실행은 하지 않음) |
| `scenarios/turtlebot3_photo/launch_support.py` | 카메라·Nav2·BT의 실행용 설정 준비 |
| `scenarios/turtlebot3_photo/photo_mission.rviz` | `/bt/goal_pose`로 보내는 RViz 설정 |
| `scenarios/turtlebot3_photo/nav2_overrides.yaml` | TurtleBot3에 맞춘 Nav2 설정 |
| `scenarios/turtlebot3_photo/front_camera.wbt` | 기본 Webots 로봇에 추가하는 전방 카메라 |
| `scenarios/turtlebot3_photo/control_description.py` | Webots와 설치된 Jazzy ros2_control의 로봇 설명 전달 호환 처리 |

## 최초 한 번: 기존 launch에 연결

`/opt/ros/jazzy/share/webots_ros2_turtlebot/launch/robot_launch.py`는 관리자 소유다.
다음 명령으로 원본을 백업하고 수정본을 적용한다. 이 명령은 시뮬레이터를 실행하지 않는다.

```bash
cd ~/ros2_ws/src/py_bt_ros
sudo bash scripts/install_webots_launch.sh
```

원본은 같은 디렉터리의 `robot_launch.py.before_py_bt_ros`에 보관한다.
Webots 패키지 업데이트로 launch 파일이 교체되면 설치 명령을 다시 실행한다.

## 실행: 기존 명령 그대로

기존 Webots/Nav2/과제 노드를 종료한 뒤 실행한다.

```bash
cd ~/ros2_ws/src/py_bt_ros
source /opt/ros/jazzy/setup.bash
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
```

`nav:=true`이면 Webots 연결 후 Nav2·과제 BT·촬영 서버·과제용 RViz를 함께 시작한다.
Python 소스에서 실행하므로 colcon 빌드는 필요 없다.
Webots 재생과 Nav2 준비를 기다린 후 RViz의 `2D Goal Pose`로 목적지와 방향을 지정한다.
Fixed Frame은 `map`, 도구의 Topic은 `/bt/goal_pose`다.
지도와 현재 로봇 위치가 맞지 않으면 `2D Pose Estimate`로 위치와 방향을 맞춘다.
RViz는 설치된 Nav2의 기본 설정에 `/bt/goal_pose`로 발행하는 `2D Goal Pose`만 추가했다.
기존 `Nav2 Goal`, 패널, 디스플레이 및 기본 활성화 상태는 유지한다.
과제의 이동·촬영·복귀는 `2D Goal Pose`로 시작한다. `Nav2 Goal`은 Nav2에 직접
이동을 요청하므로 과제 BT를 시작하지 않는다. 기본 Realsense 그룹은 비활성 상태다.
종료는 `Ctrl+C`다.

`nav:=false`이면 과제 BT·촬영 서버를 자동 실행하지 않는다.
기본 소스 경로는 `~/ros2_ws/src/py_bt_ros`다. 다른 경로로 옮겼다면
`PY_BT_ROS_ROOT` 환경 변수에 그 절대 경로를 지정한다.

## 토픽, 서비스, 저장 위치

- 입력: `/bt/goal_pose`, `geometry_msgs/msg/PoseStamped`.
- 이동과 복귀: `/navigate_to_pose`, `nav2_msgs/action/NavigateToPose`.
- 촬영 요청: `/bt/capture_image`, `std_srvs/srv/Trigger`.
- 카메라: `/TurtleBot3Burger/front_camera/image_color`, `sensor_msgs/msg/Image`.
- 출발점: 목표 수신 시 이동 명령 전에 저장한 `map` 좌표의 위치와 방향.
- 촬영: 서비스 요청 후 새로 받은 프레임을 PNG로 저장하고 성공 응답에 파일 경로를 담는다.
- 사진: `~/ros2_ws/src/py_bt_ros/output/captures/`.

카메라는 `alwaysOn=false`로 설정한다. 촬영 서버는 대기 중에는 이미지 토픽을
구독하지 않는다. 사용자가 먼저 아래 명령으로 이미지 토픽을 구독하고,
이미지 메시지가 들어오는 것을 확인한 뒤 목표를 지정한다. 촬영이 끝날 때까지
구독을 유지한다. `Ctrl+C`로 구독을 해제한다.

촬영 서비스는 자신 이외의 이미지 구독자가 있는지 먼저 검사한다.
없으면 구독을 생성하지 않고 촬영 실패를 반환하므로 카메라를 자동으로 켜지 않는다.
외부 구독자가 있을 때만 촬영용 구독을 잠시 생성하고, 처리 후 해제한다.
프레임 대기 중 외부 구독이 사라진 것이 발견되어도 촬영을 실패 처리하고 해제한다.
ROS 그래프의 구독자 발견·해제 정보에는 전파 지연이 있을 수 있다.
촬영 실패 시 BT는 `FAILED`로 종료하며 자동 복귀하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /TurtleBot3Burger/front_camera/image_color sensor_msgs/msg/Image
```

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /bt/goal_pose
```

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /bt/mission_status --qos-durability transient_local
```

정상 상태 순서는 다음과 같다.

```text
STARTING → MoveToGoal → IsAtGoal → CaptureImage → Return → IsAtHome → COMPLETE
```

`arrival_checks`에는 각 Condition의 통과 여부, 거리, 헤딩 오차가 기록된다.
`error`는 실패 원인이고, `image_path`는 저장한 사진의 절대 경로다.

```bash
xdg-open ~/ros2_ws/src/py_bt_ros/output/captures
```

## 실패 처리

진행 중 새 목표는 무시하여 출발점이 덮어써지지 않게 한다.
액션 거절·실패, 서버 준비 시간 초과, 도착 Condition 실패, 촬영 실패가 발생하면 임무를 종료한다.
촬영 성공 응답 전에는 복귀를 시작하지 않는다.
서버 준비는 최대 15초 기다린다. 요청을 전송한 뒤에는 수락·결과를 기다리며,
별도의 클라이언트 응답 시간 제한은 두지 않는다. 서버가 응답하지 않으면
`RUNNING`에 머물 수 있으므로 launch를 종료하고 재시작해야 한다.
촬영 서버의 새 프레임 대기 제한은 3초다. 종료 시 이동 취소를 요청하지만
별도 취소 완료 추적은 하지 않는다.

## 환경 의존성과 확인 범위

`launch_support.py`는 설치된 Nav2 기본 YAML에 overrides를 합친다.
현재 개발 환경의 기본 제어기는 MPPI이며 overrides는 이를 전제로 속도를 설정한다.
Nav2 패키지 버전이 달라지면 기본 설정·플러그인도 달라질 수 있다.
`control_description.py`와 카메라 월드 설정은 현재 launch에서 사용하므로
파일만 삭제하면 실행할 수 없다.

정적 구문·참조 검사를 수행했으며 최종 단순화 이후의 시뮬레이터 실행 검증은
수행하지 않았다. 제출 환경에서 목표 이동, 별도 도착 검사, PNG 저장,
출발 위치·방향 복귀 및 `COMPLETE` 상태를 확인한다.

## 기존 Webots launch 원본 복원

```bash
sudo cp /opt/ros/jazzy/share/webots_ros2_turtlebot/launch/robot_launch.py.before_py_bt_ros \
  /opt/ros/jazzy/share/webots_ros2_turtlebot/launch/robot_launch.py
```
