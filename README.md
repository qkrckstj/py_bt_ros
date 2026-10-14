# TurtleBot3 이동 · 촬영 · 복귀 과제

ROS 2 Jazzy와 Webots의 TurtleBot3를 사용한다. RViz에서 목표를 지정하면
Python Behaviour Tree가 **이동 → 도착 검사 → 사진 저장 → 출발점 복귀 → 복귀 검사**를 수행한다.

## 실행 환경

- ROS 2 Jazzy, Webots 및 `webots_ros2_turtlebot`
- Nav2와 RViz2
- Python 의존성: ROS의 `rclpy`, `tf2_ros`, `cv_bridge` 및 메시지 패키지, OpenCV, PyYAML

수업의 Webots·TurtleBot3·Nav2 기본 환경이 설치되어 있어야 한다.
이 저장소의 Python 코드는 소스에서 직접 실행하므로 별도 `colcon build`는 필요 없다.
`requirements.txt`만 설치해서 ROS 환경 전체가 준비되는 것은 아니다.

## 최초 설정

저장소를 `~/ros2_ws/src/py_bt_ros`에 둔다. 이미 클론했다면 다시 클론할 필요 없다.

```bash
git clone https://github.com/qkrckstj/py_bt_ros.git ~/ros2_ws/src/py_bt_ros
cd ~/ros2_ws/src/py_bt_ros
sudo bash scripts/install_webots_launch.sh
```

설치 스크립트는 기존 Webots `robot_launch.py`를 백업하고 과제 자동 실행용 파일로 교체한다.
다른 경로에 두었다면 실행 터미널에서 `PY_BT_ROS_ROOT`를 해당 저장소의 절대 경로로 설정한다.

## 실행 순서

첫 번째 터미널에서 실행하고 유지한다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
```

두 번째 터미널에서 카메라를 구독한다. 촬영이 끝날 때까지 유지한다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /TurtleBot3Burger/front_camera/image_color sensor_msgs/msg/Image
```

토픽이 이미 발견된 상태라면 메시지 타입은 생략할 수 있다.
픽셀 배열 출력을 줄이려면 마지막에 `--no-arr`를 붙인다.

Webots 재생과 Nav2 준비, 이미지 수신을 확인한 뒤 RViz의 **2D Goal Pose**로
위치와 방향을 지정한다. Fixed Frame은 `map`, 도구의 Topic은 `/bt/goal_pose`다.
기존 **Nav2 Goal**은 Nav2에 직접 이동을 요청하므로 과제 BT를 시작하지 않는다.
위치 추정이 지도와 맞지 않으면 **2D Pose Estimate**로 맞춘다.

사진은 `output/captures/`에 저장된다. 종료는 첫 번째 터미널에서 `Ctrl+C`다.

## BT와 코드 구조

```text
Sequence
├── HasGoal       목표 수신 여부 확인
├── MoveToGoal    Nav2 이동 요청
├── IsAtGoal      실제 위치·방향 확인
├── CaptureImage  별도 촬영 서비스 요청
├── Return        저장된 출발점으로 이동
└── IsAtHome      실제 복귀 위치·방향 확인
```

`main.py → BTRunner → Agent` 순서로 실행된다.
`Agent`가 `/bt/goal_pose`를 받아 목표와 현재 출발점을 `blackboard`에 저장한다.
BT 노드들은 같은 딕셔너리를 읽는다. 별도 Mission 클래스는 없다.
`Return`은 `MoveToGoal`을 재사용하되 목적지 대신 출발점을 읽는다.

| 파일 | 역할 |
| --- | --- |
| `modules/agent.py` | 목표 수신, 출발점 저장, BT 실행·종료 |
| `scenarios/turtlebot3_photo/default_bt.xml` | BT 순서 |
| `scenarios/turtlebot3_photo/bt_nodes.py` | Action·Condition 구현 |
| `scenarios/turtlebot3_photo/capture_image_server.py` | 별도 촬영 ROS 서비스 |
| `launch/robot_launch.py` | 기존 명령으로 전체 실행 연결 |
| `scenarios/turtlebot3_photo/launch_support.py` | 카메라·Nav2·BT 실행용 설정 생성 |
| `scenarios/turtlebot3_photo/control_description.py` | 로봇 설명을 바퀴 제어기와 TF 발행기에 전달 |
| `scenarios/turtlebot3_photo/nav2_overrides.yaml` | 로봇 크기·속도·도착 오차 및 Jazzy 호환 설정 |
| `config.yaml` | BT 주기, 토픽, Condition 허용 오차 |

## 통신과 상태 확인

| 종류 | 이름 | 방향 |
| --- | --- | --- |
| Topic | `/bt/goal_pose` | RViz → Agent |
| Action | `/navigate_to_pose` | MoveToGoal·Return → Nav2 |
| Service | `/bt/capture_image` | CaptureImage → 촬영 서버 |
| Topic | `/TurtleBot3Burger/front_camera/image_color` | Webots 카메라 → 외부 구독자·촬영 서버 |
| Topic | `/bt/home_pose` | Agent → 출발점 확인용 구독자 |
| Topic | `/bt/mission_status` | Agent → 진행 상태 확인용 구독자 |

```bash
ros2 topic echo /bt/mission_status --qos-durability transient_local
```

완료 상태는 `COMPLETE`, 실패 상태는 `FAILED`다.
촬영 서버는 외부 이미지 구독자가 있을 때만 촬영용 구독을 생성한다.
외부 구독자가 없거나 촬영에 실패하면 자동 복귀하지 않는다.
진행 중 추가 목표는 무시한다. 요청 전송 이후 서버 응답이 끊기면
`RUNNING`에 머물 수 있으며, 종료 시 취소 완료까지 보장하지는 않는다.

[상세 작동 원리와 설정](docs/PHOTO_MISSION.md)

## 출처와 라이선스

수업의 [py_bt_ros](https://github.com/inmo-jang/py_bt_ros) BT 기본 모듈을 기반으로 한다.
기본 모듈의 원 출처는 [space-simulator](https://github.com/inmo-jang/space-simulator)다.
라이선스는 [LICENSE](LICENSE)를 참고한다. Webots launch 파일의 원 저작권·라이선스 표기는 해당 파일에 유지한다.
