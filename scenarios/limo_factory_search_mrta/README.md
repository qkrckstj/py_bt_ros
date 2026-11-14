# LIMO 3대 MRTA 업무 할당

## 1. 구현 내용

Webots에서 LIMO 3대의 현재 위치를 받아 각 로봇에 순찰 구역을 배정하는 기능을 구현했습니다.

- ROS2 TF를 이용해 로봇 3대의 위치 확인
- Greedy 알고리즘으로 가까운 로봇과 순찰 구역부터 배정
- 배정된 목표 좌표를 `/multi_bt/patrol_tasks` 토픽으로 전달
- BT 및 Nav2와 연동해 각 로봇 이동

## 2. 실행 환경

- ROS2 Jazzy
- Webots R2025a
- Docker (`ros_jazzy_gpu`)
- LIMO 3대
- Nav2 / multirobot_bt

아래 명령어는 테스트에 사용한 Docker 환경 기준입니다.

## 3. 실행 방법

실행 순서: Webots → MRTA → 도착 확인 → 로봇 출발

각 프로그램은 별도 터미널에서 실행합니다.

### STEP 1. Webots 실행

우분투 터미널에서 Docker 접속:

```bash
sudo docker exec -it ros_jazzy_gpu bash
```

Docker 내부에서 실행:

```bash
cd /root/py_bt_ros
source /opt/ros/jazzy/setup.bash
source install_bt/setup.bash

export USER=root USERNAME=root WEBOTS_HOME=/usr/local/webots
export LD_LIBRARY_PATH="$WEBOTS_HOME/lib/webots:$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH"

ros2 launch multirobot_sim multirobot_sim.launch.py \
  repo_path:=/root/py_bt_ros rviz:=false pygame:=false yolo:=false
```

Webots에서 LIMO 3대가 나타나는지 확인합니다.
현재 실행 환경에서는 BT도 함께 실행되므로 별도로 실행하지 않습니다.

### STEP 2. MRTA 실행

새 우분투 터미널에서 Docker 접속:

```bash
sudo docker exec -it ros_jazzy_gpu bash
```

Docker 내부에서 실행:

```bash
source /opt/ros/jazzy/setup.bash
cd /root/py_bt_ros/scenarios/limo_factory_search_mrta

python3 patrol_publisher.py --ros-args -p use_sim_time:=true
```

로봇 3대의 위치를 받아 순찰 구역을 배정합니다.

출력 예시:

```text
===== MRTA 업무 할당 결과 =====
robot1 → 1구역
robot2 → 3구역
robot3 → 2구역
============================
```

로봇 위치에 따라 배정 결과는 달라질 수 있습니다.

### STEP 3. 로봇 도착 확인

새 우분투 터미널에서 Docker 접속:

```bash
sudo docker exec -it ros_jazzy_gpu bash
```

Docker 내부에서 실행:

```bash
source /opt/ros/jazzy/setup.bash

python3 /root/py_bt_ros/scenarios/limo_factory_search_mrta/limo_arrival_monitor.py --ros-args -p use_sim_time:=true
```

각 로봇의 이동 상태와 도착 여부를 터미널에서 확인합니다.

### STEP 4. 로봇 이동 시작

새 우분투 터미널에서 Docker 접속:

```bash
sudo docker exec -it ros_jazzy_gpu bash
```

Docker 내부에서 실행:

```bash
source /opt/ros/jazzy/setup.bash

ros2 service call /multi_bt/start std_srvs/srv/Trigger "{}"
```

정상 실행 시:

```text
success=True
message='Assigned patrol started'
```

Webots에서 로봇 3대의 이동을 확인합니다.

## 4. 순찰 구역

| 구역 | 목표 좌표 (x, y) |
|---|---|
| 1구역 | (1.0, 8.6) |
| 2구역 | (1.2, -8.6) |
| 3구역 | (4.85, 0.65) |

각 로봇의 현재 위치를 기준으로 가까운 조합부터 배정합니다.

## 5. 테스트 결과

- LIMO 3대 위치 수신 확인
- Greedy 기반 업무 할당 확인
- BT 연동 및 로봇 3대 이동 확인
- Nav2 이동 성공 상태 확인
- 로봇별 도착 완료 메시지 출력 확인
