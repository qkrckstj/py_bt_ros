# Hybrid Moon 12m

찬서 BT로 실제 LIMO 1대와 Webots LIMO 3대를 탐사시킨다. 실제 탐사 구역은 4×4m다. [터미널별 실행 명령](dashboard_sim_run.txt)을 0번부터 따르면 된다.

- **노트북**은 Ubuntu 24.04, ROS 2 Jazzy, Webots, `webots_ros2_driver`, Nav2, 시스템 Python의 NumPy·PyYAML이 필요하다. 이 폴더를 `ros_packages/multirobot_sim`이 있는 저장소 최상위에 놓는다. 저장소를 빌드한 뒤 기본 `install/local_setup.bash`를 사용하거나 `export HYBRID_ROS_SETUP=/절대경로/install/local_setup.bash`로 빌드 환경을 지정한다.
- **리모**에는 Ubuntu 22.04, ROS 2 Humble, LIMO 대시보드·드라이버, Nav2와 SLAM Toolbox가 필요하다. 기본 ROS 오버레이는 `~/wego_ws/install/setup.bash`이고 `LIMO_ROS_SETUP`으로 지정할 수 있다. 기존 `~/limo_single_exploration`에는 스캔 어댑터와 `resource/real_slam.yaml`, `resource/real_single_nav2.yaml`이 있어야 한다. 다른 경로면 리모 실행 터미널에 `LIMO_EXPLORATION_DIR`을 지정한다.
- **네트워크**는 양쪽 기기 간 통신이 가능한 같은 LAN 또는 핫스팟을 사용한다. SSH 없이 NoMachine에서 리모 명령을 입력할 수 있다. 설치용 HTTP는 노트북 TCP 17443, 지도 전송은 노트북 UDP 17442, 주행 연결은 리모 TCP 17444를 사용한다. 같은 핫스팟이어도 기기 간 통신을 차단하는 설정이 있으면 연결할 수 없다.
- **페어링 키**는 처음 한 번만 노트북의 이 폴더에서 `(umask 077; python3 -c 'import secrets; print(secrets.token_hex(32))' > pairing.key)`로 만든다. 기존 키가 있다면 그대로 사용한다. 같은 파일을 SSH/SCP 또는 USB처럼 비공개 경로로 리모의 설치 폴더에 복사한다. GitHub나 HTTP 다운로드 폴더에 키를 올리지 않는다.

라이다 빔 수가 불규칙하면 다운로드 서버의 `fix_dashboard_lidar.sh`를 리모에서 사용한다. 센서를 종료한 상태에서 기존 라이다 설정을 백업하고 `fixed_resolution`을 적용한다.

전진 목표 속도는 0.16m/s, 회전 목표 속도는 0.60rad/s다. 배포본의 실제 주행은 별도 검증이 필요하다. 맵 자산 출처는 [THIRD_PARTY_NOTICES](moon/THIRD_PARTY_NOTICES.md)를 참고한다.
