# 달 탐사 전용 Docker 환경

이 구성은 Ubuntu 24.04, ROS 2 Jazzy, Webots R2025a용입니다. 기존 컨테이너나 다른 체크아웃을 수정하지 않습니다. 소스는 `LUNAR_REPO`에 지정한 체크아웃 하나만 `/workspace/py_bt_ros`로 마운트합니다. 이미지에는 소스를 복사하지 않습니다.

Jazzy Webots 배포 패키지의 importer는 `PIL`을 직접 사용하므로 `python3-pil`을 명시적으로 설치합니다. 또한 Webots와 외부 컨트롤러가 같은 IPC 사용자 경로를 사용하도록 이미지와 Compose에 `USER=lunar`를 설정합니다. 이 값은 컨테이너 프로세스 UID/GID 설정과 별개입니다.

Jazzy의 배포 패키지는 [ROS 공식 Webots 설치 문서](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html)를 따릅니다. Webots 버전은 Ubuntu 24.04를 지원하는 [공식 R2025a 릴리스](https://github.com/cyberbotics/webots/releases/tag/R2025a)의 `webots_2025a_amd64.deb`로 고정합니다. 나머지 apt 패키지는 빌드 시점의 Jazzy 배포 버전이므로 비트 단위 재현 이미지라고 보장하지 않습니다.

## 현재 체크아웃 확인과 빌드

현재 저장소 루트에서 실행합니다. `realpath` 출력이 의도한 새 체크아웃인지 확인하세요. 이 작업의 경로는 `/home/gayeon/lunar_hungarian_project`입니다. 기존 `/home/gayeon/ros2_ws/src/py_bt_ros`나 `limo_jazzy_sim_ws`를 지정하지 않습니다.

```bash
export LUNAR_REPO="$(realpath .)"
export LUNAR_UID="$(id -u)"
export LUNAR_GID="$(id -g)"
export LUNAR_ROS_DOMAIN_ID=63
git -C "$LUNAR_REPO" status --short --branch
docker compose -f docker/lunar/compose.yaml config
docker compose -f docker/lunar/compose.yaml build lunar
```

`config`의 `volumes.source`가 현재 체크아웃인지 확인합니다. `LUNAR_REPO` 미지정이나 존재하지 않는 경로는 오류가 됩니다. UID/GID를 호스트 사용자와 맞춰 생성 파일의 소유권을 유지합니다. 기존 컨테이너에는 `docker exec`하지 않습니다.

```bash
docker compose -f docker/lunar/compose.yaml run --rm lunar
```

이후 컨테이너 안에서:

```bash
source /opt/ros/jazzy/setup.bash
cat /etc/os-release
cat "$WEBOTS_HOME/resources/version.txt"
pwd
git rev-parse HEAD
colcon build --base-paths ros_packages/multirobot_sim ros_packages/moon_mapping_demo --symlink-install
source install/setup.bash
python3 -m pytest -q ros_packages/moon_mapping_demo/tests
moon_shadow_plan --help
```

`build/`, `install/`, `log/`는 이 체크아웃에 생성됩니다. 호스트 Humble 빌드 산출물을 Jazzy에서 재사용하지 마세요. 다른 배포판으로 이미 빌드했다면 별도 빌드 경로를 사용합니다.

```bash
colcon --log-base log/jazzy build \
  --base-paths ros_packages/multirobot_sim ros_packages/moon_mapping_demo \
  --build-base build/jazzy --install-base install/jazzy --symlink-install
source install/jazzy/setup.bash
```

## 창 없는 알고리즘·연결 점검

컨테이너 셸에서 순서대로 실행합니다. `execute:=false`는 경로 표시·검사용이며 로봇 주행을 시작하지 않습니다. Xvfb 사용은 실제 RViz 화면 확인을 대신하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
xvfb-run -a ros2 launch moon_mapping_demo moon_shadow.launch.py rviz:=false execute:=false
```

Compose의 `init: true`를 유지합니다. 직접 `docker run`으로 Xvfb를 시작할 때도 `--init`을 넣어야 합니다. 이번 검사에서는 `xvfb-run`이 컨테이너 PID1인 경우 X 서버가 생성된 뒤 준비 신호 대기에서 멈췄고, `--init`을 추가하자 `xvfb-run -a /usr/local/webots/webots --version`이 0.36초에 `Webots version: R2025a`를 출력하고 종료 코드 0으로 끝났습니다.

다른 터미널에서도 동일한 `LUNAR_REPO`, UID/GID, `LUNAR_ROS_DOMAIN_ID`를 설정하여 새 컨테이너를 실행하면 host 네트워크와 같은 ROS 도메인을 통해 토픽을 확인할 수 있습니다. 확인 명령과 단일·다중 로봇 실행은 [패키지 README](../../ros_packages/moon_mapping_demo/README.md)를 따릅니다.

## Webots·RViz 화면

이전에 이미지를 빌드했다면 호스트 데스크톱의 터미널에서 다음 한 명령으로 현재 소스를 Jazzy로 빌드·검사한 뒤 Webots와 RViz를 실행할 수 있습니다.

```bash
cd /home/gayeon/lunar_hungarian_project
bash docker/lunar/run_demo.sh bt-fixed
```

기본 `bt-fixed`는 TeamBT·RobotBT를 통한 4대 고정 목표 주행입니다. 종료 후 `explore`로 센서 기반 단기 반복 탐사를 검증하고, 그 뒤 `explore-wide`로 넓은 평가를 진행합니다. `single`은 기존 단일 회귀, `preview`와 `multi`는 기존 시간 실행기의 4대 모드입니다. 실행 스크립트는 기본적으로 현재 커널 드라이버에 맞는 GPU 모드를 선택하고 실제 OpenGL 공급자·렌더러를 검사합니다. 단일 주행 확인 후 다중 주행을 실행합니다. 결과는 매번 새 `results/gui_<모드>_<시각>_<PID>/`에 저장하며 Ctrl+C로 종료합니다. 현재 체크아웃만 마운트하고 새 일회성 컨테이너를 사용합니다. Docker·화면·인증 파일에 접근할 수 없으면 실행 전에 원인을 출력하고 종료합니다.

### GPU 렌더링

확인한 장비는 Intel Iris Xe와 NVIDIA MX450이며, 커널 드라이버는 각각 `i915`, `nouveau`입니다. `LUNAR_GPU=auto` 기본 모드는 NVIDIA 독점 드라이버 모듈이 있으면 `nvidia`, 그 외에는 `mesa`를 선택합니다. GPU 서비스의 `LIBGL_ALWAYS_SOFTWARE=0`을 Webots와 RViz가 상속하며 RViz launch의 소프트웨어 렌더링 강제 옵션은 제거했습니다.

- `mesa` → `lunar-dri`: 현재 장비의 nouveau 또는 Intel/AMD Mesa 드라이버용입니다. [Docker 장치·추가 그룹 설정](https://docs.docker.com/reference/compose-file/services/)으로 `/dev/dri`와 실제 장치의 render/video GID를 전달합니다. NVIDIA render 장치가 있으면 해당 PCI 주소를 읽어 [Mesa DRI_PRIME](https://docs.mesa3d.org/envvars.html)에 지정합니다. 현재 주소를 소스에 하드코딩하지 않습니다. `LUNAR_DRI_PRIME=0`으로 화면 기본 GPU를 선택할 수도 있습니다.
- `nvidia` → `lunar-gpu`: NVIDIA 독점 드라이버·Container Toolkit 환경용입니다. [Docker GPU 예약](https://docs.docker.com/compose/how-tos/gpu-support/)과 [NVIDIA graphics/display 기능](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html)을 사용합니다. 하이브리드 화면에는 [PRIME Render Offload](https://download.nvidia.com/XFree86/Linux-x86_64/550.135/README/primerenderoffload.html)를 적용합니다.
- `software` → `lunar-gui`: 명시적으로 CPU 렌더링을 사용하는 기존 모드입니다.

먼저 기존 창을 유지한 채 별도 터미널에서 GPU 연결만 검사할 수 있습니다.

```bash
bash docker/lunar/run_demo.sh gpu-check
```

`glxinfo -B`의 실제 OpenGL 공급자·렌더러를 확인하고 `graphics.json`에 기록합니다. GPU 모드에서 `llvmpipe` 등 CPU 렌더러이면 실행을 중단합니다. `nvidia` 모드에서는 `nvidia-smi`와 NVIDIA OpenGL 공급자도 검사합니다. `mesa` 모드에서는 공급자가 Mesa여도 실제 GPU 렌더러이면 통과합니다. nouveau 환경에 `nvidia-smi`나 NVIDIA Docker 런타임을 요구하지 않습니다. 결과와 Docker 오류는 출력된 `results/gui_gpu-check_.../`에 저장합니다.

MX450은 Turing 계열이며 [Mesa 25.1부터 OpenGL에 NVK+Zink를 사용](https://docs.mesa3d.org/drivers/nvk.html)합니다. Dockerfile에 `mesa-vulkan-drivers`, `libvulkan1`, `vulkan-tools`를 추가했습니다. Mesa 모드에서 이전 이미지의 그래픽 구성 표시가 없으면 같은 검사·실행 명령이 한 번 증분 빌드합니다. `image_build.log`, 그래픽 패키지 버전, Vulkan 진단도 결과 폴더에 저장합니다. 컨테이너 내부에 커널 드라이버를 설치하거나 호스트 드라이버를 바꾸지 않습니다.

GPU 검사가 통과하면 이전 실행을 Ctrl+C로 종료하고 `bt-fixed` 명령을 실행합니다. 이미 생성된 CPU 컨테이너의 GPU 설정은 실행 중에 바뀌지 않습니다. 모드를 직접 지정할 수도 있습니다.

```bash
LUNAR_GPU=mesa bash docker/lunar/run_demo.sh gpu-check
LUNAR_GPU=nvidia bash docker/lunar/run_demo.sh gpu-check
LUNAR_GPU=software bash docker/lunar/run_demo.sh single
```

열려 있는 이 프로젝트의 GUI 컨테이너(`lunar-gui`, `lunar-gpu`, `lunar-dri`)가 1개일 때는 다른 터미널에서 다음 명령을 사용할 수 있습니다. `start`는 정지된 임무를 현재 전체 로봇 위치와 시각에서 다시 계산하고, `stop`은 창을 닫지 않고 임무를 정지합니다.

```bash
bash docker/lunar/run_demo.sh status
bash docker/lunar/run_demo.sh start
bash docker/lunar/run_demo.sh stop
bash docker/lunar/run_demo.sh diagnose
```

`diagnose`는 12초 동안 clock과 4대 odom의 수신 수·최대 수신 간격·시각 차이를 관측하고 `results/gui_input_probe.json`에 저장합니다. 상태 확인·진단은 속도를 발행하지 않습니다. 실행 중인 노드는 소스 파일 수정만으로 바뀌지 않습니다. 실행기 수정은 기존 실행을 Ctrl+C로 끝내고 선택한 모드를 다시 실행해야 반영됩니다.

11월 22일 사용자가 호스트 터미널에서 실행한 뒤, 공유된 로그에서 실제 4대 Webots 연결과 RViz OpenGL 초기화를 확인했습니다. 첫 실행은 입력 갱신 중단으로 정지했고, 독립 진단에서는 약 0.17배속의 시뮬레이션을 관측했습니다. 기본 GUI 실험 설정은 시뮬레이션 odom 나이 0.6초와 실제 시간 수신 중단 3초를 별도로 제한합니다. 목표 출발 전 10 시뮬레이션 초의 대기는 이 속도에서 실제 약 1분이며, 초기 안정화 시간도 추가됩니다. 목표 도착 검증은 [현재 상태 문서](../../docs/shadow_hungarian_requirements.md)를 따릅니다.

Linux의 X11/Xwayland 세션에서 현재 인증 파일을 읽기 전용으로 전달합니다. `XAUTHORITY`가 설정되어 있지 않은 세션은 먼저 해당 세션의 인증 파일 경로를 확인해야 합니다.

```bash
export LUNAR_XAUTHORITY="${XAUTHORITY:?현재 X11 세션의 인증 파일 경로를 지정하세요}"
test -r "$LUNAR_XAUTHORITY"
docker compose -f docker/lunar/compose.yaml run --rm lunar-gui
```

컨테이너 안에서:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch moon_mapping_demo moon_shadow.launch.py execute:=false
```

위의 수동 `lunar-gui` 서비스는 소프트웨어 렌더링이며 `run_demo.sh`는 드라이버에 맞는 GPU 서비스를 선택합니다. `execute:=true` 주행 전에는 시각화와 단일 로봇 확인을 먼저 수행합니다. 기본 실험은 robot1~4입니다. 통합 모드는 `exploration_short.yaml` 또는 `exploration_wide.yaml`을 사용합니다. 일부 대기 로봇도 임무 전체의 안전·몸체 충돌 검사 대상입니다.

## 이전 세션의 환경 확인 (이력)

- 호스트: Ubuntu 22.04.5, ROS 2 Humble, Python 3.10.12. 요청한 Jazzy 환경과 다릅니다.
- 호스트 `/usr/local/webots/resources/version.txt`: `R2025a` 확인.
- Docker Compose v5.6.0 확인. Docker 소켓은 일반 샌드박스에서 접근 거부되었고 승인된 읽기 전용 `docker ps -a` 조회로 기존 컨테이너 네 개가 모두 정지 상태임을 확인했습니다.
- 기존 정지 컨테이너는 이번 체크아웃 검증에 사용하지 않았습니다.
- 로컬 이미지 목록에서 `ros:jazzy-ros-base-noble`과 `limo-jazzy-webots:r2025a`도 발견했습니다. 이 문서의 새 이미지 빌드는 공식 ROS 베이스를 사용했으며 기존 LIMO 이미지와 컨테이너를 수정하지 않았습니다.
- `docker build --progress=plain --tag lunar-shadow-hungarian:jazzy-r2025a --file docker/lunar/Dockerfile .` 실제 실행 성공. 설치 단계 321.8초, 최종 이미지 저장 97.9초였습니다. 빌드 시작 전 여유 디스크는 19GB였습니다.
- 새 이미지의 일회성 컨테이너에 현재 체크아웃을 **읽기 전용**으로 마운트하고, 네트워크 차단·비root UID/GID 1000·읽기 전용 root filesystem·`/tmp` tmpfs 조건에서 확인했습니다. Python 3.12.3, ROS `jazzy`, Webots `R2025a`, `rclpy`·`webots_ros2_driver`·`moon_mapping_demo.shadow_mission`·OpenCV import가 통과했습니다.
- 같은 컨테이너에서 `python3 -m pytest -q -p no:cacheprovider ros_packages/moon_mapping_demo/tests`: **52 passed in 1.57s**, 종료 코드 0. `PYTHONPATH=/workspace/py_bt_ros/ros_packages/moon_mapping_demo`, `PYTHONDONTWRITEBYTECODE=1`을 지정하여 이 체크아웃 소스를 검사했습니다.
- 새 이미지에서 `docker run --rm --init ... xvfb-run -a /usr/local/webots/webots --version`: 실제 Webots 실행 파일의 `R2025a` 출력과 종료 코드 0을 확인했습니다. `--init` 없이 멈췄던 점검용 컨테이너 하나는 종료했습니다.
- 최초 통합 launch에서 `ros2_supervisor`의 `PIL` import 누락과 외부 컨트롤러의 `USER` 미지정 경고를 발견했습니다. `python3-pil` 별도 설치 레이어와 `USER=lunar`를 추가하여 증분 이미지 재빌드에 성공했습니다(설치19.4초·이미지 저장0.5초). 보완 후 새 일회성 컨테이너에서 `ros2_supervisor.py`의 전체 top-level import가 통과했습니다. 통합 launch 재검증은 패키지 README의 실제 결과를 따릅니다.
- 위 결과는 이미지·자동 테스트 검증입니다. RViz 화면과 실제 단일·다중 로봇 주행의 성공 기록으로 간주하지 않습니다. 전체 통합 검증 결과는 [요구사항·상태 문서](../../docs/shadow_hungarian_requirements.md)와 패키지 README에 기록합니다.

## 4대 BT 통합 후속 확인

원본 BT frontier 의존성 `python3-scipy`, 지표용 `python3-matplotlib`, 한글 폰트 `fonts-noto-cjk`를 증분 레이어에 추가했습니다.
이미지의 `io.py_bt_ros.lunar.exploration=team-bt-v1` 표시가 없으면 실행 스크립트가 증분 빌드합니다.
현재 Codex의 Docker 소켓 권한은 거부되므로 이 레이어의 실제 이미지 빌드와 새 4대 Webots/RViz 실행은 검증하지 못했습니다.
기존 nouveau Mesa NVK/Zink와 실제 GPU 렌더러 확인 절차는 유지합니다.
[새 인계](../../docs/bt_exploration_handoff_2026-11-22.md) 및 [검증 JSON](../../docs/shadow_validation_2026-11-22.json)의 최신 항목을 따릅니다.


## 출발 정책 비교

BT 모드에서 `LUNAR_DEPARTURE_POLICY=simultaneous|staggered_pairs`를 선택합니다. 기본값은 기존 simultaneous입니다.
기존 실행을 종료한 뒤 한 모드씩 실행합니다.

```bash
LUNAR_GPU=mesa LUNAR_DEPARTURE_POLICY=staggered_pairs bash docker/lunar/run_demo.sh bt-fixed
# 종료 후 explore, 단기 확인 후 explore-wide 순서
```

새 정책은 4대 배정을 유지하고 출발 시차를 조정합니다. 실제 Webots는 현재 Docker 권한 거부로 미검증이며,
모의 비교에서 넓은 영역 개선도 확인하지 못했습니다. 기존 nouveau Mesa NVK/Zink·렌더러 검증 설정은 변경하지 않았습니다.


## 이른 대피 단계 비교

기본 출발 정책은 simultaneous, YAML의 기본 위험 대응은 candidates입니다. LUNAR_RISK_STAGE=baseline|early|goals|candidates|heading으로 위험 대응 단계를 선택합니다. 이전 launch를 종료하고 같은 월드·시작 상태로 새 실행을 시작해야 비교할 수 있습니다. LUNAR_DEPARTURE_POLICY와 독립적입니다.

~~~bash
LUNAR_GPU=mesa bash docker/lunar/run_demo.sh bt-fixed
# 위 실행 종료 후 단기 확인
LUNAR_GPU=mesa bash docker/lunar/run_demo.sh explore
# 단기 결과 확인 후. 장시간 목표 달성은 아직 미검증/미달성
LUNAR_GPU=mesa bash docker/lunar/run_demo.sh explore-wide
# 기존 위험 대응을 같은 조건으로 비교
LUNAR_GPU=mesa LUNAR_RISK_STAGE=baseline bash docker/lunar/run_demo.sh explore-wide
~~~

[최신 인계](../../docs/bt_exploration_handoff_2026-11-22.md)에 단계별 모의 결과·계산 지연 검사·미해결 사항을 기록했습니다. Docker 소켓 권한 거부로 이번 실제 Webots 실행은 수행하지 않았으며 권한을 우회하지 않았습니다. nouveau Mesa NVK/Zink·GPU 검증 경로는 유지합니다.
