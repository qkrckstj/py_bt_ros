# 그림자 회피·Hungarian 프로젝트 요구사항과 인계 기록

## 최신 상태 요약

- 후속 수정으로 선택형 `staggered_pairs` 정책을 추가했습니다. 4대 Hungarian 배정을 유지한 채 선발2·후발2의 시차를 계획합니다.
  기본값은 기존 `simultaneous`입니다. 좁은 통로 검사·고정 목표는 통과했지만 단기·넓은 탐사 개선은 확인하지 못했습니다.
  최신 수치와 실행법은 인계 문서의 ‘후속 수정’ 절 및 검증 JSON의 `bt_exploration_integration.departure_staggering`이 기준입니다.


- 최신 요구사항은 **robot1~4 모두 탐사 참여**, 유효 목표 부족·안전상 일부 대기 허용입니다. 과거 3대 제한을 대체합니다.
- 고정한 팀원 `TeamBT`·`RobotBT` 원본을 포함하고 navigation 계약에 공동 시간 경로 실행기를 연결했습니다. 새 전용 launch와 `bt-fixed`, `explore`, `explore-wide` 모드를 사용합니다.
- 센서 취득 시각 TF와 광선 가림에 기반한 실제 관측 격자, frontier 목표 생성·반복 배정, 고정 분모 지표·JSON/CSV·그래프·RViz 표시를 추가했습니다. WBT는 안전 계획에만 사용하며 사전 지도/도착은 관측으로 세지 않습니다.
- 원격 업로드·push·PR·reset 없이 기존 미커밋 변경을 보존했습니다. 브랜치는 `feature/shadow-hungarian`, 기준 HEAD는 `97b4759`입니다.
- 실행 순서·계약·평가 영역·출력과 한계는 [4대 BT 탐사 인계](bt_exploration_handoff_2026-11-22.md), 최종 검증 수치와 남은 사항은 [검증 JSON](shadow_validation_2026-11-22.json)의 `bt_exploration_integration`을 기준으로 합니다.
- 최종 회귀117개·합성 ROS7개 통과. WBT+모의 센서의 4대 고정 목표 완료, 단기 탐사2회차·30.64% 관측을 확인했습니다. 넓은 영역은 sim77.2초에 대피 계획 실패로 정지하여 장시간 평가는 미완료입니다.
- 현재 새 코드의 실제 Jazzy/Webots·RViz 검증은 Docker 소켓 권한 거부로 수행하지 못했습니다. 아래 과거 단일·3대 Webots/GPU 성공은 새 4대 BT 검증과 구분합니다.
- GPU는 nouveau 기반 `/dev/dri`·Mesa NVK/Zink를 유지합니다. 독점 드라이버를 강제하지 않았습니다.

## 작업 기준

- 저장소: `qkrckstj/py_bt_ros`, 기준 브랜치 `dev_junyong`, 작업 브랜치 `feature/shadow-hungarian`.
- 조사 시작 시 실제 HEAD: `97b4759786232b77591530c9604326c325e93fbf`.
- 작업 경로: `/home/gayeon/lunar_hungarian_project`. 조사 시작 시 작업 트리는 깨끗했으며 적용되는 `AGENTS.md`는 발견하지 못했습니다.
- `ros_packages/moon_mapping_demo/worlds/moon_terrain.wbt`와 실제 환경 설정을 기준으로 삼습니다. 60×60m 월드, LIMO 4대입니다. 과거 20×20m 지도와 옛 시작 좌표를 실험 입력으로 사용하지 않습니다.
- 로봇 시작 위치는 `config/environment.yaml`, 지형은 WBT, 태양은 `controllers/lunar_sun/sun_config.json`에서 읽습니다.
- 원격 push는 수행하지 않습니다.

## 확정 범위

| 항목 | 확정 요구사항 |
| --- | --- |
| 배정 | 기본 robot1~4 참여. 4개 목표 회차는 공통 t0의 4×4 정수 비용 완전 배정. 목표 부족·안전상 부분집합 배정/대기 허용 |
| 비용 | 공통 절대 시뮬레이션 시각 `t0`에서 Space-Time A*로 계산한 이동+대기 정수 시간 비용 |
| 목적함수 | 선택 비용 합 최소화. 직선거리는 최종 배정 비용에 사용하지 않음 |
| 불가능 조합 | 금지 간선으로 기록. 완전 배정이 없으면 명확히 실패 |
| 그림자 | 진입 금지 영역. 대기, 로봇 몸체와 이동 중 점유 영역 검사 |
| 지형 | 안전 경로는 사전 WBT 보수 모델, 실제 관측 기록과 다음 목표는 센서 관측 격자로 구분 |
| 충돌 | 지형·선택 로봇·대기 로봇 모두 포함. 독립 경로 배정 후 공동 실행 재검사 |
| 시간 | ROS `/clock`과 Webots `Supervisor.getTime()`의 절대 시각 일치 |
| 실행 | 시간 경로·대기를 보존하는 실행기. Nav2 공간 목표만 발행하고 시간 실행으로 간주하지 않음 |
| 실행 이탈 | 위치·시간 이탈, 오래된 관측, 남은 경로 안전성 확인 후 정지·재계획 |
| 비교 | Hungarian 직접 구현, CP-SAT 공통 JSON/CSV 제공. 같은 정수 비용·제약·목적함수 사용 |
| 측정 | 비용 생성·Hungarian·공동 조정 시간 분리. 고정 ROI 센서 관측 면적·탐사율·로봇별 거리/대기/재계획, sim/wall 시간별 JSON/CSV·그래프 |
| 제외 | 통신 거리·기지 연결성·중계, SIPP, 물·얼음 검출, 새 MPC, CP-SAT 자체 구현 |

## 조사한 기존 구조 (통합 전 이력)

`moon_mapping.launch.py`는 LIMO 네 대의 Webots 드라이버와 scan/odom/TF를 실행하지만 robot1만 SLAM·Nav2를 실행합니다. 나머지 세 대가 이미 자율주행 가능한 구성은 아닙니다. 기존 Nav2 설정 파일에는 사용하지 않는 과거 AMCL 초기 위치도 남아 있으므로 실험 초기 위치로 읽지 않습니다.

기존 공장 임무는 `multirobot_bt`의 `/multi_bt/patrol_tasks`, `/multi_bt/rescue_tasks`, `/multi_bt/start`, `/multi_bt/stop`, `/multi_bt/status`를 사용하고, 로봇별 `NavigateToPose`를 호출합니다. 이 계약에는 시간 경로가 없습니다. 따라서 공장 임무의 공간 목표를 달 탐사 시간 경로에 그대로 넣지 않습니다. 새 실행 모드는 기존 `/{robot}/cmd_vel` 드라이버 계약과 `/multi_bt/emergency_stop` 인터페이스를 재사용합니다. 기존 BT/Nav2와 동시에 속도를 발행하지 않도록 전용 launch 모드로 분리합니다.

드라이버는 명령 수신 후 0.5초가 지나면 정지하며, heartbeat를 요구하는 로봇은 `/multi_bt/emergency_stop` 수신이 1초 이상 끊기면 정지합니다. 실제 전진 속도 상한은 0.22m/s, 회전 속도 상한은 0.65rad/s입니다. 계획 속도 설정은 실행 상한과 일치해야 합니다.

기존 TF는 robot1의 `map → robot1/map` 시작 위치 앵커, SLAM의 `robot1/map → robot1/odom`, 드라이버의 `robotN/odom → robotN/base_link`로 이어집니다. 대기 로봇은 `map → robotN/odom` 정적 시작 위치 변환을 사용합니다. 새 사전 지형 실행 모드는 모든 로봇의 `map → robotN/odom` 앵커를 제공하므로 기존 SLAM 앵커와 함께 실행하면 안 됩니다. 오도메트리 누적 오차는 실제 위치 정확도의 한계로 남습니다.

기존 태양 컨트롤러는 시작 시 `origin = robot.getTime()`을 빼는 위상을 사용했습니다. 새 실험에서는 태양과 계획이 모두 절대 Webots 시뮬레이션 시각을 사용해야 합니다. 15° 태양 고도, 240초 주기, 초기 방위각 270°는 태양 JSON에서 읽습니다. 컨트롤러의 로봇 그림자 Bool은 9개 바닥 표본을 쓰는 검증용 정답이며 전신 연속 안전 보장이 아닙니다.

## 구현 구조와 해석

`moon_shadow_plan`은 ROS 없이 사전 지형 경로 비용과 Hungarian 배정을 계산하는 CLI입니다. `moon_shadow_mission`은 ROS 시간, 관측, 시간 경로 추종·재계획을 연결합니다. `moon_shadow.launch.py`는 `moon_mapping.launch.py shadow_mode:=true` 모드와 새 노드를 함께 실행하며 기본 `execute:=false`로 시각화만 준비합니다. 구체적인 인자와 설정 키는 [패키지 README](../ros_packages/moon_mapping_demo/README.md)를 기준으로 합니다.

Space-Time A*의 비용은 정해진 위치·시간 격자와 통행 모델 내 최소 예상 시간입니다. 회전·가감속을 직접 최적화하지 않는 초기 모델이므로 실제 도착 시간이 같다고 보장하지 않습니다. 실행 지연은 남은 경로 재검사와 재계획으로 처리합니다. 사전 지형 모델이 연속 물리 환경을 얼마나 보수적으로 근사하는지는 모델 문서와 테스트를 함께 확인해야 합니다.

Hungarian의 최적성은 개별 로봇 경로 비용 행렬에 대한 완전 배정의 합에 한정됩니다. 충돌 회피를 위해 경로를 조정한 뒤의 팀 전체 공동 주행 시간이나 팀 최적성을 의미하지 않습니다. 초기 `C[i,j]`, 초기 배정 합, 충돌 조정 후 소요 시간과 재계획 결과를 구분해 기록합니다. 서로 다른 동점 배정은 같은 최적 총비용을 가질 수 있습니다.

## 비교 규격

공통 JSON/CSV에는 로봇·목표 ID 순서, 비용 단위, 정수 tick 길이, 금지 조합, 공통 `t0`, 지도·설정 식별 정보를 포함합니다. 두 알고리즘은 같은 정수 행렬을 그대로 읽고 각 행·열을 정확히 한 번 선택합니다. 금지 조합은 선택할 수 없습니다. CP-SAT 팀원이 비용을 재계산하거나 반올림하지 않도록 합니다. 정확한 스키마와 입출력 예시는 패키지 문서와 `exchange.py`가 기준입니다.

## 기존 검증 순서와 당시 환경 (이력)

1. 핵심 알고리즘: Hungarian과 작은 완전탐색 비교, 동점·금지 간선·Hall 조건 실패, A* 이동·대기·도달 불가·몸체·시간·예약 충돌 검사.
2. 시각화: 선택 경로, 시각, 대기 표시를 RViz에서 확인.
3. 단일 로봇: 그림자 경계, 시간 지연, 정지·재계획과 비상정지 확인.
4. 다중 로봇: 교차 경로, 대기 robot4, stale 관측, 충돌 조정 후 비용 확인.

조사 환경은 Ubuntu 22.04.5/Humble/Python 3.10.12이며 요청한 Jazzy 컨테이너가 실행 중이지 않았습니다. Webots R2025a 설치와 Docker Compose v5.6.0은 확인했습니다. 기존 Docker 컨테이너 네 개는 모두 정지 상태이며 이번 검증에 재사용하지 않았습니다. Docker 환경 절차는 [Docker README](../docker/lunar/README.md)에 있습니다.

다음 상태표는 **11월 21일 코드의 기록**입니다. 11월 22일 수정과 검증은 뒤의 후속 기록을 기준으로 합니다. 코드 작성·컴파일이나 창 없는 메시지 검사는 RViz 화면·주행 성공을 대신하지 않습니다.

| 검증 | 상태 | 기록 원칙 |
| --- | --- | --- |
| 알고리즘·교환 형식 자동 테스트 | 통과 | 패키지 53개(Jazzy 3.14초); 기존 시나리오·BT 포함 호스트 회귀 89 passed in 2.51s. Hungarian n≤6 1,500개 완전탐색 대조 포함 |
| 실제 60×60m WBT 비용 행렬 생성 | 통과 | t0=0, C=[[27,21,30],[18,24,27],[30,12,23]], 배정 [2,0,1], 합60tick; 공동 경로70tick(추가10). 행렬0.263초, Hungarian0.000045초 |
| Docker Compose 정적 해석 | 통과 | 기본·GUI profile 해석, source=/home/gayeon/lunar_hungarian_project, 미지정 환경변수 거부 확인 |
| Jazzy 이미지 빌드 | 통과 | 새 lunar-shadow-hungarian:jazzy-r2025a 빌드 exit0; 설치321.8초·최종 저장97.9초 |
| 새 Jazzy 이미지 import·자동 테스트 | 통과 | Python3.12.3/Jazzy/R2025a, ROS·shadow_mission import, 읽기 전용 현재 체크아웃에서 52 passed in 1.57s |
| 새 이미지 Xvfb·Webots 실행 파일 | 통과 | docker run --init 조건에서 Webots version: R2025a, exit0. PID1 xvfb-run 대기 문제를 확인하여 직접 실행에도 --init 필요 |
| Webots importer 런타임 의존성 | 보완·import 통과 | 최초 launch에서 PIL 누락 발견. python3-pil·USER=lunar 추가 후 증분 빌드 및 ros2_supervisor.py 전체 import 통과 |
| 실제 Webots·RViz 연결(Xvfb) | 통과 | 18.44초, 4대 실제 odom·3대 경로·28개 마커 수신, RViz OpenGL 초기화 확인. 화면을 사람이 시각 확인한 검증은 아님 |
| RViz 화면 직접 확인 | 미검증 | Xvfb 연결 검증과 화면 가독성 확인은 구분 |
| 합성 ROS 메시지·안전정지 | 통과 | Jazzy 실제 DDS로 transient 경로/시간 마커, 시각화 zero cmd, robot4 odom 중단 ERROR, clock 역행 STOPPED·정지 확인 |
| 단일 로봇 실제 주행 | 부분 확인·완료 미검증 | 40초 wall 제한까지 robot1 약0.458m 주행. goal=(-27,-26)에 도착 전 timeout; 4대 odom/robot1 경로 수신, 관측상 최소 간격1.5m, 그림자 Bool false. 목표 도착 성공으로 기록하지 않음 |
| 다중 로봇 실제 주행 | 미실행 | 단일 목표 도착까지 미확인. 사용자 배터리10%로 마무리 요청에 맞춰 후속 세션에서 순서대로 진행 |

## 실제 실행 명령·증거

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=.:ros_packages/multirobot_bt:ros_packages/moon_mapping_demo:$PYTHONPATH
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider \
  tests ros_packages/multirobot_bt/tests ros_packages/moon_mapping_demo/tests
```

Jazzy는 기존 `limo-jazzy-webots:r2025a` 이미지를 읽어 만든 새 일회성 컨테이너에서 패키지 빌드·당시 52개 검사·launch 인자 확인을 먼저 수행했습니다. 최종 새 이미지에서 53개 검사와 합성 안전정지 두 경우를 다시 통과했습니다. 이어 새 Dockerfile의 `lunar-shadow-hungarian:jazzy-r2025a` 이미지를 빌드하여 검사했습니다. 기존 컨테이너나 다른 체크아웃에는 쓰지 않았습니다. `build/jazzy`, `install/jazzy`만 현재 체크아웃에 생성했습니다.

실제 Webots 검사는 `--init --network none --user 1000:1000` 컨테이너에서 현재 저장소만 `/workspace/py_bt_ros`로 마운트하고 실행했습니다. 창 없는 실행은 다음 스크립트로 재현합니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/jazzy/setup.bash
xvfb-run -a python3 ros_packages/moon_mapping_demo/tests/webots_smoke.py \
  --mode visualization --timeout 75 --output results/webots_visualization_pil
# 같은 스크립트 --mode single, --mode multi로 단계별 검사
```

- 원본 비교 문제: [JSON](examples/shadow_problem.json), [CSV](examples/shadow_problem.csv). 현재 월드·설정 식별값을 포함합니다.
- 단일 실제 주행 로그: `results/webots_single/{validation.json,launch.log,runtime.json}`. 제한시간은 물리 완료 시간의 상한 검증이 아니며 더 긴 실행과 도착 확인이 필요합니다.
- 로컬 실행 증거: `results/shadow_hungarian/t0_0/`, `results/webots_visualization_pil/{validation.json,launch.log,runtime.json}`. `results/`는 gitignore되지만 이 작업 폴더에 남습니다.
- t0=60 검사에서는 동일 최적 배정 합60이 나왔으나 목표 대기까지 포함하는 공동 계획은 실패하여 CLI exit2를 확인했습니다. 성공으로 기록하지 않습니다.
- 최초 Webots 검사는 Pillow 누락으로 timeout했습니다. `python3-pil`, `USER=lunar`를 Docker에 추가한 뒤 실제 연결 검사를 통과했습니다. `--init` 없이 Xvfb를 PID1로 실행하면 준비 신호를 기다리므로 직접 docker run에도 --init이 필요합니다.
- 강제 SIGINT 종료 과정에서 RViz의 context 종료 예외와 반복 SIGINT에 따른 KeyboardInterrupt 로그가 남았습니다. 정상 실행 중 연결 성공과 종료 안정성은 별개이며 종료 처리 점검이 남습니다.
- 11월 21일 조사 시에는 팀원의 새 BT가 아직 게시되지 않았습니다. 11월 22일 `dev_chanseo@02cda62` 게시를 확인했으며 [BT 조사 기록](bt_integration_review.md)에 결과를 남겼습니다. 어댑터 통합은 미검증이며 현재 start/stop/status·cmd_vel·heartbeat 계약으로 독립 실행합니다.
- 사용자 재확인: **GitHub push·PR 생성·원격 업로드 금지**. 로컬 변경과 문서만 남깁니다.

## 2026-11-22 후속 구현·검증

이번 세션은 Docker API 연결에 `permission denied`, X11 `:0` 연결 실패, DDS UDP 소켓 생성에 `Operation not permitted`가 발생했습니다. 정책을 우회하지 않았으며 Jazzy 컨테이너·Webots 물리 주행은 재개하지 못했습니다. 호스트 Humble은 현재 저장소의 `build/humble`, `install/humble`, `log/humble`만 사용하여 별도로 빌드했습니다. 기존 Jazzy overlay나 다른 체크아웃·컨테이너는 수정하지 않았습니다.

단순 기구학 검사로 수정 전 robot1의 코너 이탈과 재계획 반복을 재현했습니다. 중간 목표의 허용 오차 안에 일찍 도착하면 다음 출발 시각까지 정지하는 기존 동작 때문에 선회 준비를 하지 못했습니다. 대기·조기 도착 시 선속도 0으로 다음 방향을 준비하도록 수정했습니다. 계획 비용에 회전 시간을 추가하거나 최초 Hungarian 결과를 바꾸지는 않았습니다.

실행기의 안전 판단을 `execution.control_step()`으로 분리해 ROS와 기구학 검사가 같은 코드를 사용합니다. 남은 공동 경로는 현재 관측 위치를 시작점으로 다시 구성하여 검사하고, 회전 명령의 원호 여유는 로봇 쌍 검사에도 적용합니다. 수동 stop·clock 역행·cmd_vel 권한 충돌의 최종 상태를 runtime 파일에 저장하도록 보완했습니다.

`webots_smoke.py`는 COMPLETE 메시지 외에도 전체 odom·그림자 관측, 배정 목표와의 오차, 로봇 간격을 검사합니다. 그림자 진입·관측 누락·잘못된 완료는 실패로 처리합니다. 종료 신호는 launch에 먼저 전달하여 자식에게 중복 SIGINT가 전달되는 것을 줄였습니다. **이 스크립트 보완 후 실제 Webots 실행은 미검증**입니다.

| 검사 | 이번 결과 | 범위 |
| --- | --- | --- |
| Humble 분리 빌드 | 1 패키지 성공 | 현재 체크아웃의 별도 overlay |
| 기존 시나리오·BT·달 패키지 회귀 | 95 passed in 2.31s | 코너 선회, 실제 위치 기반 미래 충돌, 대기 로봇 포함 |
| 합성 ROS 시각화 | 통과 | 늦은 경로·마커 구독, zero cmd |
| 합성 ROS 안전정지 4가지 | 통과 | robot4 odom 중단, clock 역행, stop 서비스, cmd_vel 발행자 충돌 및 runtime 상태 저장 |
| 단일 이상적 기구학 | 통과 | 19.0 sim초, 재계획0, 목표 오차 약0.0367m |
| 다중 이상적 기구학 | 통과 | 66.6 sim초, 재계획2, 목표 오차 약0.0372/0.0358/0.0483m, 최소 표본 간격0.7068m |
| 팀원 BT 원본 순수 로직 | 54 passed in 1.39s | `dev_chanseo@02cda62`의 배정·취소·복귀·통신·스윕 관련 4개 파일 |
| 이번 수정의 Jazzy·Webots 물리 주행 | 실행 불가 | 현재 세션 Docker·화면 접근 제한 |
| 새 BT 어댑터 통합 | 미구현·미검증 | 게시 코드·ROS 계약·연결 조건 조사 완료 |

합성 ROS 검사는 한 프로세스의 공유 rclpy context에서 성공했습니다. UDP 소켓 생성은 여전히 거부되므로 다른 프로세스·컨테이너 간 DDS 연결 성공으로 해석하지 않습니다. 이상적 기구학 검사는 실제 WBT 지형을 사용하지만 ROS/DDS·Webots 물리·계산 지연·미끄러짐·오도메트리 오차를 포함하지 않습니다.

결과는 `results/resume_2026-11-22/`와 [이번 검증 요약 JSON](shadow_validation_2026-11-22.json)에 저장합니다. 새 BT의 게시 커밋·알고리즘·메시지 계약과 아직 필요한 어댑터는 [BT 연결 검토](bt_integration_review.md)를 기준으로 합니다. 기본 greedy/Nav2/통신 제약을 현재 Hungarian/시간 경로 실행에 그대로 병합하지 않았습니다.

## 2026-11-22 GUI 실행 점검 후 보완

사용자가 호스트에서 `run_demo.sh single`을 실행한 뒤 `results/gui_single_20261122_010651_7781/`에서 4대 실제 Webots 드라이버 연결과 RViz OpenGL 초기화를 확인했습니다. 사용자는 화면이 열리고 시뮬레이션 시간이 계속 흐른다고 확인했습니다. 이 실행과 t0=69.52의 수동 재시작은 모두 `전체 odom 또는 /clock 갱신 중단`으로 ERROR가 되었고 목표 도착은 기록되지 않았습니다.

`run_demo.sh diagnose`의 독립 구독자는 12.03초 동안 robot1~4 각각 odom 86개와 clock 160개를 수신했습니다. 시뮬레이션/실제 시간 비율은 약 0.17~0.18, 관측된 odom 시각 간격은 최대0.02초, clock은 최대0.01초였습니다. 최초 진단 파일의 `max_wall_gap_seconds`에는 구독 연결 전 초기 탐색 대기도 포함되어 있으므로 메시지 사이의 실제 공백으로 해석하지 않습니다. 이후 진단 스크립트에서는 초기 탐색·마지막 관측 나이·메시지 사이 최대 간격을 별도 필드로 구분했습니다.

후속 수정은 다음과 같습니다.

- 자동 계획 전 전체 입력이 기본3초 동안 시뮬레이션·실제 시간 양쪽에서 안정되는지 확인합니다. 준비 중 입력 공백은 출발 전 대기로 처리합니다. `execution.startup_stable_seconds`로 바꿀 수 있습니다.
- 관측의 시뮬레이션 나이 제한 `odom_timeout=0.6`과 실제 시간 수신 중단 제한 `input_wall_timeout=3.0`을 분리했습니다. 느린 GUI 시뮬레이션에서도 시간표·그림자·충돌 검사는 시뮬레이션 시각을 사용하며, 실제 입력 중단은 별도로 감시합니다. wall 설정이 없으면 기존 odom_timeout 값으로 동작합니다.
- 상태 토픽·runtime에 clock과 로봇별 관측의 두 시간 지연, 허용 제한과 관측 위치를 기록합니다. 상태 전환도 ROS 로그에 남깁니다.
- 실행 스크립트에 현재 프로젝트 GUI 컨테이너의 `status`, `start`, `stop`, `diagnose` 명령을 추가했습니다. 다른 컨테이너를 임의로 선택하지 않으며 여러 개가 실행 중이면 거부합니다.

이 수정 후 호스트 회귀 **104 passed in 3.65s**, 합성 ROS 시각화와 안전정지4가지 검사를 통과했습니다. 느린 시뮬레이션, 실제 wall 입력 중단, sim 관측 지연, 대기 로봇 누락, 미래 시각 입력과 초기 안정화 공백을 검사합니다. 결과는 `results/gui_timeouts_fix/`에 저장합니다.

사용자가 새 버전을 재실행한 폴더는 `results/gui_single_20261122_011837_9508/`입니다. **단일 실제 주행 COMPLETE를 확인했습니다.** t0=13.1에서 출발해 19.1 sim초에 목표에 도착했고, 22.1 sim초에 3초 대기를 마쳤습니다. 재계획은0회, 최종 오도메트리 위치는 약(-26.96733,-26.00000)이며 목표(-27,-26)와의 오차는0.03267m입니다. 완료 시점에 4대 관측은 최신이었습니다. 계획 파일 생성 이후 실제 시간은 약129.8초였습니다. 실제 위치의 별도 정답 센서나 연속 그림자 Bool 기록은 수집하지 않았으므로 이를 별도 검증한 것으로 표현하지 않습니다. `validation_from_runtime.json`에 배정 목표·runtime·최종 odom을 대조한 결과를 저장했습니다.

이후 사용자가 `multi`를 실행했습니다. **3대 실제 주행도 COMPLETE를 확인했습니다.** 폴더는 `results/gui_multi_20261122_012256_10407/`입니다. 초기 배정 합63tick·공동 계획73tick이며 실제 입력 행렬의 완전탐색 최적값도63입니다. 재계획3회 후 robot3=47.88, robot2=64.7, robot1=79.7 sim초에 도착했고82.7 sim초에 전체 대기를 마쳤습니다. 최종 odom 목표 오차는 각각 robot1=0.03725m, robot2=0.03751m, robot3=0.03254m였습니다. 계획 생성 이후 실제 시간은 약711.3초였습니다. 결과 해석 범위는 단일 실행과 동일하며 `validation_from_runtime.json`을 함께 저장했습니다.

## GPU 렌더링 후속 작업

사용자가 기존 Docker의 GPU 사용을 요청했습니다. PCI와 커널 드라이버를 읽어 Intel Iris Xe/i915, NVIDIA MX450/nouveau를 확인했습니다. 기존 구성은 Compose와 RViz launch 모두 소프트웨어 렌더링을 강제했습니다. RViz 강제를 제거하고 드라이버에 따른 GPU 자동 선택, `/dev/dri` 및 render/video GID 전달, Mesa DRI_PRIME 선택을 추가했습니다. NVIDIA 독점 드라이버용 Toolkit 서비스도 별도로 제공합니다. 설정과 명령은 [Docker GPU 문서](../docker/lunar/README.md)를 기준으로 합니다.

사용자가 실행한 첫 Mesa GPU 검사는 `results/gui_gpu-check_20261122_014439_11958/`에서 llvmpipe·Accelerated:no를 기록했습니다. Zink의 Vulkan 초기화가 실패해 GPU 검사도 실패한 것으로 처리했습니다. Mesa25의 Turing OpenGL에 필요한 Vulkan 드라이버·진단 도구를 Dockerfile의 별도 증분 레이어에 추가했으며, 이전 이미지를 쓰는 Mesa GPU 명령은 필요한 경우 자동 빌드하도록 했습니다. 실제 GPU 렌더러와 속도 개선은 다음 검사 결과를 확인할 때까지 통과로 기록하지 않습니다.

후속 검사 `results/gui_gpu-check_20261122_014746_12323/`에서 증분 빌드와 GPU 검사가 통과했습니다. 실제 렌더러는 `zink Vulkan 1.4(NVIDIA GeForce MX450 (NVK TU117) (MESA_NVK))`, OpenGL4.6이며 소프트웨어 렌더링은 false입니다. 이미지에는 Mesa25.2.8의 Vulkan 드라이버가 설치됐습니다.

사용자가 GPU 모드로 다시 실행한 `results/gui_multi_20261122_015002_13096/`도 **COMPLETE**였습니다. 초기 배정63tick·공동 계획73tick, 재계획3회이며 robot3=47.99, robot2=64.87, robot1=79.88 sim초에 도착했고82.89 sim초에 전체 대기를 마쳤습니다. 최종 odom 목표 오차는 robot1=0.03024m, robot2=0.03354m, robot3=0.03962m였고 대기 robot4를 포함한 전체 입력이 최신이었습니다. 같은 폴더의 그래픽 검사도 MX450 하드웨어 렌더러를 기록했습니다.

계획 파일 생성부터 완료 runtime 저장까지의 실제 시간은 소프트웨어711.32초, GPU194.07초였습니다. 이번 두 실행에서는 약3.67배 차이가 났습니다. 이는 이 장비·월드·실행 두 건의 측정이며 모든 환경의 속도 향상 보장은 아닙니다. 실제 정답 위치 센서와 연속 그림자 Bool 별도 기록은 수집하지 않았으므로 최종 판정은 배정 목표·실행기 상태·오도메트리를 대조한 범위입니다. 각 폴더의 `validation_from_runtime.json`과 최신 검증 JSON에 결과를 보존했습니다.

## 다음 세션 인계

GUI 확인 요청에 따라 `docker/lunar/run_demo.sh`를 추가했습니다. 호스트 데스크톱 터미널에서 `bash docker/lunar/run_demo.sh single`을 실행하면 현재 체크아웃을 Jazzy로 빌드·검사한 뒤 Webots·RViz와 단일 목표 주행을 시작합니다. `preview`, `multi`도 지원합니다. 셸 구문과 도움말은 확인했으며 현재 세션의 실제 `single` 실행은 Docker 데몬 접근 거부로 사전 점검에서 종료했습니다. GUI가 열렸다고 기록하지 않습니다.

로컬 변경은 아직 커밋하지 않았으며 원격 push·PR 생성 금지는 유지됩니다. 먼저 현재 `git status`, 위 최신 요약과 검증 JSON, 패키지 README를 읽습니다. 사용자는 저장·인계를 요청한 뒤 계속 진행을 요청했고, 그 이후 GPU 3대 주행 완료까지 확인했습니다. 이미 통과한 검사를 단순히 세션이 바뀌었다는 이유로 전부 반복할 필요는 없습니다. 구현 변경이나 실패가 있으면 해당 범위에 맞춰 다시 검사합니다.

다음 구현 후보는 [팀원 BT 연결 조건](bt_integration_review.md)에 따른 별도 어댑터 통합입니다. 기본 greedy 배정·Nav2 공간 목표·통신/중계 제약을 현재 Hungarian 시간 경로 모드에 그대로 합치지 않습니다. 시간표와 공동 출발·취소·재계획을 보존하고 새 어댑터를 따로 검증해야 합니다. 4대 모두 움직이는 변경은 별도의 목표4개 설정이 필요하며 현재 기본3대 요구사항을 임의로 바꾸지 않습니다.

마지막 GPU GUI 컨테이너는 완료 상태로 창이 열려 있을 수 있습니다. 새 실행 전에 기존 실행 터미널에서 Ctrl+C로 종료하여 같은 도메인63에 중복 발행자가 생기지 않게 합니다. Codex의 직접 Docker/X11 접근은 계속 제한됐으며, 사용자가 호스트 터미널에서 실행한 결과를 공유 경로로 확인했습니다. GPU 재실행 명령은 `bash /home/gayeon/lunar_hungarian_project/docker/lunar/run_demo.sh multi`입니다. 현재 커널에서는 자동으로 Mesa/nouveau 경로를 선택하며, 하드웨어 검사는 `gpu-check`, 명시적 CPU 모드는 `LUNAR_GPU=software ...`입니다. 세부 명령은 Docker README를 따릅니다.
