# Humble 호환 수정 및 실행 검증 (2026-10-19)

## 이동 실패의 원인과 해결

설치된 `ros-humble-tf2`는 0.25.23이었다. 이 버전의 BufferCore와 MessageFilter 사이 잠금 순서 역전으로 Nav2 costmap의 TF 수신이 멈출 수 있다. 이번 실행도 `/tf`와 `/odom`은 계속 갱신되는데 controller_server 내부 TF만 시작 직후 시각에 고정됐고, 종료 시 planner/controller가 SIGKILL을 요구했다. 이 상태에서 Humble Nav2의 도착 판단은 변환 실패 후 기본 좌표를 사용해 이동 없이 성공을 반환하기도 했다.

UDP 설정만 적용한 전체 실행에서도 같은 증상을 재현했다. 앞서 공유 메모리 통신 문제로 추정했던 판단은 정정한다. UDP 기본 설정은 제거했다.

공식 tf2 0.25.24에는 해당 교착 수정이 포함돼 있다. `scripts/prepare_humble_tf2.sh`는 apt의 공식 패키지를 내려받아 `output/compat/tf2`에 추출한다. 시스템 패키지를 설치하거나 교체하지 않으며 sudo가 필요 없다. 과제 launch는 설치 버전이 0.25.21~0.25.23일 때만 이 라이브러리를 자식 프로세스의 `LD_LIBRARY_PATH`에 추가한다. 수정본이 없으면 준비 명령을 안내하고 중단한다. 시스템 tf2가 0.25.24 이상으로 업데이트되면 로컬 라이브러리를 사용하지 않는다.

실행 중 `/proc/<controller_server PID>/maps`에서 프로젝트의 수정된 `libtf2.so`가 로드된 것을 확인했다.

근거: [공식 수정 0.25.24 변경 내역](https://github.com/ros2/geometry2/blob/0.25.24/tf2/CHANGELOG.rst), [교착 보고 #992](https://github.com/ros2/geometry2/issues/992), [회귀 보고 #995](https://github.com/ros2/geometry2/issues/995).

## 함께 수정한 호환 문제

- 시스템에 설치된 launch는 helper 인자 6개를 전달했지만 저장소 함수는 5개만 받았다. 인자와 생성 파일을 일치시켰다.
- Humble의 DWB와 Jazzy의 MPPI에 각각 맞는 속도 파라미터를 적용한다. Humble은 Twist, Jazzy는 TwistStamped 경로를 사용한다.
- 진행 중인 `/clock`을 확인한 뒤 두 제어기를 그룹 활성화하고, 성공한 경우에만 Nav2를 시작한다. 이를 적용한 실행에서 두 제어기의 active 상태를 확인했다.
- RViz 설정은 설치된 Nav2 기본 설정으로 생성해 Humble에 없는 Selector/Docking 패널을 피한다.
- 실제 저장소 경로를 인식하고 설치 진입점은 소스를 매번 읽도록 바꿨다.
- Navfn의 목표 대체 허용 오차를 0.5m에서 0.05m로 줄였다. 장애물 안의 목표 (1, 0)에 대해 기존 planner가 (0.55, 0)으로 끝나는 경로를 반환하는 것도 확인했다. 정확한 목적지 검사를 통과하지 못하는 대체 경로의 성공을 줄이기 위한 변경이다.
- 기존 `.bak`, `.before_humble*` 파일은 보존했다.

## 실제 검증 결과

별도 제어기가 아닌 **저장소 전체 launch**를 실행하고 다음을 확인했다. 화면 없이 같은 Webots/ROS 노드를 실행하는 `gui:=false`로 검사했다.

1. `/navigate_to_pose`에 (0.5, 0) 목표 전송: 바퀴 속도 명령이 지속 발행됐고 odometry가 (0, 0)에서 약 (0.356, 0.0003)m로 변한 뒤 0.15m 허용 오차 안에서 성공했다. RViz Nav2 Goal과 같은 액션 경로다.
2. `/bt/goal_pose`에 지도상 빈 공간 (0, 0.5) 목표 전송: MoveToGoal → IsAtGoal → CaptureImage → Return → IsAtHome → COMPLETE. RViz 2D Goal Pose와 같은 토픽 경로다.
3. 도착 검사 오차: 거리 0.118856m, 헤딩 0.146461rad. 복귀 검사 오차: 거리 0.089996m, 헤딩 0.154204rad. 두 조건 모두 통과했다.
4. 외부 카메라 구독을 유지한 상태에서 320×240 이미지와 유효한 PNG 저장을 확인했다.

검증 사진: `output/captures/20261019_163109_380052_420b390f.png`.
검증 ROS 로그: `/tmp/photo_fixed_logs/`.

최종 설정으로 화면이 있는 Webots/RViz 실행도 재검사했다. 직접 Nav2 목표 (0.5, 0)는 odometry 약 (0.354, 0.001)m에서 성공했다. 이어 `/bt/goal_pose` 목표 (0, -0.5)는 촬영·복귀 후 COMPLETE였다. 도착 오차는 거리 0.126920m/헤딩 0.139292rad, 복귀 오차는 거리 0.113483m/헤딩 0.138892rad였다. 사진은 `output/captures/20261019_163537_805191_a5f8c7d3.png`, 로그는 `/tmp/photo_ready_logs/`에 있다.

Snap VS Code에서 실행할 때는 Webots와 외부 제어기의 임시 소켓 경로도 일치시킨다. 같은 환경에서 RViz가 Snap의 GNOME schema 때문에 시작 전에 중단되는 문제는 호스트 schema 경로를 RViz에만 전달해 처리했다.

카메라 구독이 없으면 원래 과제 조건에 따라 촬영은 실패한다. 장애물 안의 목표에 대한 도착은 보장하지 않는다. 실제 Jazzy 실행은 검사하지 않았다. BallJoint URDF 내보내기 경고가 있는 캐스터 구성을 그대로 두고 위 검증을 통과했다.

## 실행

현재 컴퓨터에는 로컬 수정 라이브러리를 이미 준비했다. 새로 받은 저장소에서 영향을 받는 Humble 버전을 쓰면 준비 명령을 한 번 실행한다.

```bash
cd /home/seoyeong/new_bt_ws/src/py_bt_ros
source /opt/ros/humble/setup.bash
bash scripts/prepare_humble_tf2.sh
ros2 launch ./launch/robot_launch.py nav:=true
```

기존 패키지 명령으로 실행하려면 소스 진입점을 한 번 설치한다.

```bash
sudo bash scripts/install_webots_launch.sh humble
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
```

촬영을 포함한 과제는 별도 터미널에서 카메라 구독을 유지하고 RViz의 **2D Goal Pose**를 사용한다. **Nav2 Goal**은 이동만 요청한다.

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /TurtleBot3Burger/front_camera/image_color sensor_msgs/msg/Image --qos-reliability best_effort --no-arr
```
