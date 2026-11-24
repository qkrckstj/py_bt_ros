# 실제 LIMO 연동

이 디렉터리는 **실제 LIMO 1대의 지도 없는 탐색**과 **실제 위치를 Webots에 복제하는 실험**을 담는다. 실측 지도, SSH 키, 실행 로그는 저장소에 포함하지 않는다. 로봇은 ROS 2 Humble, 노트북 시뮬레이션은 ROS 2 Jazzy를 사용한다.

## 지도 없이 실제 LIMO 1대 탐색

실제 `/scan`과 `/odom`을 이용해 `slam_toolbox`가 `/map`과 `map→odom`을 만들고, 팀 BT가 미탐색 경계 목표를 할당한다. 개별 BT가 `/navigate_to_pose`로 목표에 이동한다. 이 구성에서는 통신 제약과 다중 로봇을 끄며, 저장 지도·AMCL·Webots를 사용하지 않는다. SLAM용 `/scan_slam`에서는 신뢰할 수 있는 구간의 무반사값만 빈 공간으로 처리한다. 원본 `/scan`은 유지된다.

노트북의 이 패키지 디렉터리에서 실행한다. `LIMO_IP`와 `LIMO_SSH_KEY`로 접속 주소와 SSH 키를 바꿀 수 있다.

```bash
bash scripts/real_single_exploration.sh check
bash scripts/real_single_exploration.sh deploy
bash scripts/real_single_exploration.sh run
# 차체·라이다 드라이버가 이미 실행 중이면 run 대신 run-existing
```

부분 지도가 생기고 `map→base_link`가 준비되면 다른 터미널에서 시작한다.

```bash
bash scripts/real_single_exploration.sh status
bash scripts/real_single_exploration.sh start
# 정지: bash scripts/real_single_exploration.sh stop
```

노트북 RViz에서는 `rviz2 -d rviz/real_single_exploration.rviz`를 실행한다. 다른 SLAM, AMCL 또는 Nav2 인스턴스와 동시에 실행하지 않는다. 탐색할 경계가 10초 동안 없으면 탐색을 종료하며, 자동 복귀는 설정하지 않았다.

## 실제 위치를 시뮬레이션에 복제

실제 `map→base_link` 위치 추정값을 약 50 Hz로 전송하여 Webots의 `robot1`에 적용한다. `robot1`은 표시 전용이며 가상 센서·모터·충돌 형상이 없다. `robot2`와 `robot3`는 물리 시뮬레이션 로봇이다. 이 두 로봇의 정보는 실제 LIMO에 `/sim/...` 토픽으로 전달된다. 실제 주행 명령은 전달하지 않는다.

`resource/real_maps/` 또는 `TWIN_MAP_DIR`로 지정한 **로컬 비공개 폴더**에 같은 지도에서 나온 `limo_real.pgm`과 `limo_real_sim.yaml`을 둔다. 두 파일을 로봇의 `/home/wego/limo_twin_robot/resource/real_maps/`에도 배포한다. 원격 패키지를 만들 때는 다음 명령을 쓸 수 있다.

```bash
python3 scripts/package_twin_robot.py --map-dir /path/to/private/maps --output /tmp/limo_twin_robot.tar.gz
# 보안이 필요한 지도이므로 생성한 압축 파일을 공개 저장소에 올리지 않는다.
```

가상 `robot2`, `robot3`의 드라이버는 별도의 `multirobot_sim` 패키지가 필요하다. `real_fleet_launch.py`의 `multirobot_source` 인자로 해당 패키지 디렉터리를 지정할 수 있다. 실제 지도와 가상 로봇 시작 위치는 지도 좌표계에 맞게 설정해야 한다.

실기 위치 추정과 노트북 시뮬레이션이 준비된 뒤, 아래 명령을 각각 별도 터미널에서 실행한다. 로봇에서 기존 차체 드라이버가 돌고 있다면 `robot`을 중복 실행하지 않는다. 새 AMCL 시작 후에는 RViz의 **2D Pose Estimate**로 실제 현재 위치·방향을 설정한다.

```bash
TWIN_ROBOT_IP=<robot-ip> bash scripts/real_twin.sh robot
TWIN_ROBOT_IP=<robot-ip> bash scripts/real_twin.sh feed
TWIN_ROBOT_IP=<robot-ip> bash scripts/real_twin.sh peers
TWIN_ROBOT_IP=<robot-ip> bash scripts/real_twin.sh sim
```

`TWIN_SSH_KEY`, `TWIN_MAP_DIR`, `TWIN_INSTALL_SETUP`으로 SSH 키, 비공개 지도 폴더, 노트북 설치 환경 경로를 지정할 수 있다. 실기와 시뮬은 ROS 도메인을 분리하며, 위치 전송은 UDP 17440, 동료 정보 전달은 TCP 17441을 사용한다. 실제 조종기나 ROS `/cmd_vel`로 실기를 움직여도 **실기 위치 추정 결과**가 복제본에 반영된다. 실기 TF가 없거나 오래되면 복제본은 마지막 위치에서 멈춘다. 위치 추정 오차와 전송 지연까지 없애지는 못한다.
