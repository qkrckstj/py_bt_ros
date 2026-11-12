cd ~/py_bt_ros

cat > scenarios/limo_factory_search_mrta/README.md <<'EOF'
# LIMO 3대 MRTA 업무 할당

## 1. 프로젝트 개요

Webots에서 LIMO 3대의 실제 위치를 받아
Greedy 알고리즘으로 순찰 구역을 자동 배정합니다.

배정 결과는 ROS2 토픽을 통해 multirobot_bt에 전달하며,
BT와 Nav2가 각 로봇을 목적지까지 이동시킵니다.

### 주요 기능
- TF를 통해 map 기준 실제 로봇 위치 수신
- Greedy 알고리즘으로 로봇별 순찰 구역 배정
- ROS2 토픽 /multi_bt/patrol_tasks 발행
- 기존 multirobot_bt와 연동

## 2. 실행 환경

테스트 환경:
- ROS2 Jazzy
- Webots R2025a
- Docker: ros_jazzy_gpu
- LIMO 3대
- Nav2 / multirobot_bt

아래 명령어는 위 Docker 환경 기준입니다.
다른 환경에서는 컨테이너 이름과 경로를 변경해야 합니다.

## 3. 전체 실행 순서

Webots + Nav2 → MRTA → BT → 이동 시작

각 프로그램은 별도 터미널에서 실행합니다.
기존 터미널은 종료하지 않습니다.

### STEP 1. Webots + Nav2 실행

우분투 터미널에서 Docker 접속:

sudo docker exec -it ros_jazzy_gpu bash

Docker 내부에서:

cd /root/py_bt_ros
source /opt/ros/jazzy/setup.bash
source install_bt/setup.bash

export USER=root
export USERNAME=root
export WEBOTS_HOME=/usr/local/webots
export LD_LIBRARY_PATH="$WEBOTS_HOME/lib/webots:$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH"

ros2 launch multirobot_sim multirobot_sim.launch.py \
  repo_path:=/root/py_bt_ros \
  rviz:=false pygame:=false yolo:=false

Webots에서 LIMO 3대와 공장 환경이 표시되고
시뮬레이션 시간이 진행되는지 확인합니다.

### STEP 2. MRTA 실행

새 우분투 터미널에서 Docker 접속:

sudo docker exec -it ros_jazzy_gpu bash

Docker 내부에서:

source /opt/ros/jazzy/setup.bash
cd /root/py_bt_ros/scenarios/limo_factory_search_mrta

python3 patrol_publisher.py --ros-args -p use_sim_time:=true

정상 실행 시:
- Robot positions: ...
- Patrol tasks: ...

로그가 출력됩니다.

MRTA는 실제 로봇 위치를 받은 후
Greedy 방식으로 작업을 한 번 배정합니다.

### STEP 3. BT 실행

다른 우분투 터미널에서 Docker 접속:

sudo docker exec -it ros_jazzy_gpu bash

Docker 내부에서:

cd /root/py_bt_ros
source /opt/ros/jazzy/setup.bash
source install_bt/setup.bash
export PYTHONPATH=/root/py_bt_ros:$PYTHONPATH

ros2 run multirobot_bt multi_bt \
  --ros-args -p use_sim_time:=true

정상 실행 시:

BT ready: waiting for external patrol tasks and /multi_bt/start

메시지가 출력됩니다.

### STEP 4. 로봇 이동 시작

새 터미널에서 Docker 접속:

sudo docker exec -it ros_jazzy_gpu bash

ROS2 환경 설정:

source /opt/ros/jazzy/setup.bash
source /root/py_bt_ros/install_bt/setup.bash

BT 시작 서비스 호출:

ros2 service call /multi_bt/start std_srvs/srv/Trigger "{}"

정상적으로 시작 요청을 받으면:

success=True
message='Assigned patrol started'

가 출력됩니다.

Webots에서 LIMO 3대의 이동을 확인합니다.

## 4. 순찰 목표 좌표

robot1, robot2, robot3의 실제 위치에 따라
Greedy 알고리즘이 아래 세 구역을 배정합니다.

구역 1: (1.0, 8.6, -1.570796327)
구역 2: (1.2, -8.6, 1.570796327)
구역 3: (4.85, 0.65, -1.570796327)

목표 배정은 로봇 현재 위치에 따라 달라질 수 있습니다.

## 5. 업무 할당 결과 확인

ros2 topic echo /multi_bt/patrol_tasks \
  --qos-durability transient_local --once

JSON 형식으로 각 로봇의 배정 목표가 출력됩니다.

중복 발행자 확인:

ros2 topic info /multi_bt/patrol_tasks

정상적인 단일 MRTA 실행 환경에서는
Publisher count: 1
Subscription count: 1

이 예상됩니다.

## 6. 목적지 도착 확인

robot1 예시:

ros2 topic echo /robot1/navigate_to_pose/_action/status \
  --qos-durability transient_local --once

robot2, robot3도 같은 방식으로 확인합니다.

Nav2 상태 코드:
- 2: 실행 중
- 4: 성공(SUCCEEDED)
- 5: 취소(CANCELED)

## 7. 임무 중단 및 종료

BT 임무 중단:

ros2 service call /multi_bt/stop std_srvs/srv/Trigger "{}"

로봇 3대가 멈춘 것을 확인한 후
각 실행 터미널에서 Ctrl+C로 종료합니다.

종료 순서:
1. MRTA
2. BT
3. Webots + Nav2
