# 새 BT + 달 맵 + CP-SAT / 공간 A* + RViz

## 처음 내려받은 저장소 준비

ROS 2 Humble, Webots R2025a, Nav2, slam_toolbox, webots_ros2_driver, colcon 및 python3-venv가 설치된 PC에서 저장소 루트 기준으로 한 번 실행한다.

```bash
scenarios/lunar_chanseo_cp_sat/prepare.sh
```

달 월드/센서/그림자·통신 런타임 소스는 `ros_packages/moon_mapping_demo`와 `scenarios/lunar_exploration`에 함께 포함했다. 준비 스크립트는 ROS Python을 쓸 수 있는 별도 `lunar_cp_env`를 만들고 CP-SAT/NumPy/OpenCV/SciPy 의존성을 설치한 뒤, 이 저장소의 `multirobot_sim`과 `moon_mapping_demo`만 `install_lunar`에 빌드한다. 새 PC 전체 의존성 설치와 장시간 주행까지 자동 검증하는 스크립트는 아니다.

현재 PC처럼 기존 형제 폴더 `../py_bt_ros`에 설치가 있으면 그것을 계속 사용할 수 있다. 이 저장소에 `install_lunar`가 준비되면 자체 설치를 우선 사용한다. 경로를 명시하려면 `LUNAR_LEGACY_REPO`를 지정한다. 실행 중인 다른 체크아웃을 덮어쓰거나 전역 Python에 pip 설치하지 않는다.

## 로봇별 탐사·중계 역할과 실제 진행 기록

이번 측정은 **CP-SAT 공동 목표 배정 + 공간 A* 경로 탐색**을 사용한 하나의 계속 실행 중인 실험이다. 네 로봇 모두 탐사 목표 배정 대상이다. CP-SAT은 로봇별 다음 목표/현재 위치 유지를 정하고 통신 연결을 강제하지만, 고정된 `explorer`/`relay` 역할 변수를 따로 최적화하지 않는다. 대기하거나 이동 중인 로봇도 중계가 가능하다.

2026-11-25 **01:50:29 KST** 스냅샷에서 로봇 중심 간 10m 연결은 `robot3 — robot1 — robot4 — robot2`였다. 여기서 중계는 측정 RF가 아니라 거리 기반 그래프에서 해당 로봇을 제거하면 나머지가 분리되는 **현재 위치의 구조적 중계**를 뜻한다.

| 로봇 | 이 시점의 탐사 상태 | 통신에서의 역할 | 누적 관측 이동량 |
|---|---|---|---:|
| robot1 | 새 목표 대기 | robot3와 robot4 쪽을 잇는 구조적 중계 | 57.78m |
| robot2 | 탐사 목표 수행, 이동 허가 있음 | 연결 사슬의 끝 로봇 | 61.45m |
| robot3 | 새 목표 대기 | 연결 사슬의 끝 로봇; 고정 중계로 지정한 것이 아님 | 60.44m |
| robot4 | 새 목표 대기 | robot1과 robot2 쪽을 잇는 구조적 중계 | 61.62m |

이는 한 시점의 상태다. robot1/4가 실험 내내 중계 전용이었다는 의미는 아니며 네 대 모두 지금까지 탐사 이동을 했다. 세 로봇이 대기 중이라는 이유만으로 모두 중계로 분류하지 않는다.

Webots 실행 시작 로그 시각은 **01:08:24 KST**다. 실제 시간은 초기화, 대기, 회전, 디버깅 및 제어부 재시작을 포함하며 시뮬레이션 시간과 다르다.

| 확인 시각 (KST) | 실행 후 실제 경과 | 시뮬레이션 시간 | 60×60m 관측률 |
|---|---:|---:|---:|
| 01:34:50 | 26.44분 | 372.80초 | 19.23% |
| 01:37:53 | 29.49분 | 410.03초 | 21.06% |
| 01:45:01 | 36.61분 | 해당 스냅샷에 미기록 | 25.69% |
| 01:50:29 | **42.09분** | **572.89초 (9.55분)** | **25.89%** |

마지막 스냅샷은 3,600m² 중 약 **932.12m² 관측**, 약 **2,667.88m² 미관측**이다. 장애물로 관측한 셀도 포함하며 바퀴로 주행한 면적과 같지 않다. 수정/정지/대기가 포함된 단일 실행이므로 알고리즘 성능 비교나 완료 시간 보장으로 사용할 수 없다. 특히 마지막 몇 분의 증가가 둔화되어 이전의 선형 남은 시간 추정은 그대로 신뢰할 수 없다. 별도 `space_astar` 방식의 장시간 독립 비교 실험은 아직 하지 않았다. 전체 영역의 접근 가능성과 100% 관측 완료도 검증되지 않았다.

근거 데이터는 [results/20261125_live_progress.json](results/20261125_live_progress.json)에 저장했다. 현재의 실제 진행률은 아래 `manage.py status`로 확인한다.

## 다음 접속 시 터미널 명령

```bash
cd /home/seoyeong/py_bt_ros_chanseo_cp_sat

# 1) 먼저 기존 실행과 현재 관측률을 확인
python3 scenarios/lunar_chanseo_cp_sat/manage.py status

# 2) 실행 중이 아닐 때만 새 실험 시작: Webots + RViz, CP-SAT 배정 + A* 경로
python3 scenarios/lunar_chanseo_cp_sat/manage.py start
```

두 번째 터미널에서 진행률을 계속 보려면:

```bash
cd /home/seoyeong/py_bt_ros_chanseo_cp_sat
python3 scenarios/lunar_chanseo_cp_sat/manage.py status --watch
```

종료는 실행 터미널의 Ctrl+C 또는 아래 명령이다. 아래 명령은 이 작업 폴더의 달 탐사 launch/제어부/지도 표시와 RViz를 찾아 종료 요청을 보낸다. 다른 프로젝트의 ROS 프로세스는 일괄 종료하지 않는다.

```bash
python3 scenarios/lunar_chanseo_cp_sat/manage.py stop
```

별도 공간 A* 탐사 방식으로 새 실험을 시작하려면 기존 실행을 stop한 뒤:

```bash
python3 scenarios/lunar_chanseo_cp_sat/manage.py start --method space_astar
```

`start`는 기존 실행이 있으면 중복 실행하지 않는다. ROS 환경은 내부 run.sh가 준비하므로 별도 source/colcon build는 필요 없다. **새 start는 같은 초기 월드에서 새 실험을 시작하며 이전 관측률/물리 위치/지도를 자동 복원하지 않는다.** 계속 실행 중인 실험을 보려면 stop/start 대신 status만 사용한다. 현재 PC 환경을 전제로 하며 다른 PC의 의존성 설치까지 대신하지 않는다.


기준: `origin/dev_chanseo`의 `02cda62` (2026-11-22 확인).
작업 브랜치: `integration/chanseo-lunar-cp-sat`.
원래 `/home/seoyeong/py_bt_ros`의 `dev_seoyong` 작업 파일과 팀원 원격 브랜치는 수정하지 않았다.

## 연결 내용

- 새 `TeamBT`와 `RobotBT`를 그대로 사용하고 목표 할당기를 주입한다. 기본 Greedy도 계속 사용할 수 있다.
- CP-SAT은 각 로봇의 목표 또는 현재 위치 유지를 선택한다. 동시 배정 수를 먼저 최대화한 뒤 관측 지도 경로 거리와 제한된 정보 이득 비용을 최적화한다.
- 미관측/장애물 셀을 가로지르는 목표, 예약 목표와 겹치는 목표, 재시도 제외 구역은 배제한다. 목표 간격은 기본 2m다.
- 네 로봇의 선택된 목표·이동 중인 기존 목표·대기 위치를 함께 넣고 단일 상품 흐름 제약으로 연결을 강제한다. 대기하는 로봇도 중계 노드가 될 수 있다. 독립적인 역할/스케줄 최적화는 아니다.
- 원래의 60×60m 달 지형, 4대 드라이버, robot1 SLAM, 로봇별 거리 제한 공유 지도, 그림자 예측·Nav2 필터를 재사용한다. robot2~4는 기존 시작점 기준 odometry를 사용하며 새 SLAM을 추가하지 않는다.
- 고정 기지는 두지 않고 로봇 중심 간 10m의 이동 통신망을 유지한다. 계획에는 0.3m 여유를 둔다. 실제 RF 측정이 아니다.
- 각 배정은 최대 3m의 다음 구간이다. 기존 시뮬레이터 그림자 예측으로 경로를 검사하고, 실행 중에도 실제 그림자 토픽·최신 경로·속도 명령·예측·통신 상태를 검사해 이동 허가를 발행한다. 관측 위치 기반 짧은 구간 통신/충돌 검사는 연속 시간의 안전 증명이 아니다.
- 한 대가 도착하면 다른 세 대의 도착을 기다리지 않고 다음 목표를 받는다. 12초의 시뮬레이션 시간 동안 0.12m 이상 진행하지 못하면 해당 목표만 취소·재배정한다.
- 탐사 완료는 지정한 전체 사각형의 관측 여부로 보수적으로 판정한다. 이전 실행의 약 30% 관측 기록이나 단일 Nav2 성공을 전체 탐사 완료로 표시하지 않는다. 이 실행은 자동 기지 복귀를 끈다.

## 현재 PC에서 실행

이 PC는 기존 형제 작업 폴더의 설치를 참조할 수 있다. 새 체크아웃은 위 `prepare.sh`로 포함된 런타임을 먼저 준비한다. `run.sh` 자체는 빌드나 pip 설치를 자동 수행하지 않는다.

```bash
cd /home/seoyeong/py_bt_ros_chanseo_cp_sat
scenarios/lunar_chanseo_cp_sat/run.sh path_planner:=astar rviz:=true
```

기본 ROS 배포판은 호스트에 설치된 Humble, ROS 도메인은 64다. 이 명령은 **같은 60m 맵의 시작 위치에서 새 실험을 시작**하며 어제의 물리 상태/SLAM/주행 거리를 자동 복원하지 않는다. 어제 체크포인트와 지도는 오프라인 검증에 사용한다.

기존 `explore.py`/`rolling_dispatch.py`와 이 BT 연결부를 동시에 실행하면 안 된다. 이 launch는 기존 executor를 `explore:=false`로 두고 새 BT만 실행한다. 통신 스레드의 블로킹을 줄이도록 `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`를 설정한다.

중지는 실행 터미널에서 Ctrl+C다. `/exploration/stop`에 `std_msgs/msg/Empty`를 보내면 BT만 정지할 수 있다. 종료 시 이동 허가를 내리고 Nav2 취소를 요청하며, 드라이버와 radio agent의 기존 watchdog도 유지한다.

실행 상태는 `/exploration/team_status`, 로봇별 `/robotN/exploration/state`, 그리고 이 작업 폴더의 `artifacts/lunar_chanseo_cp_sat/live_status.json`에서 확인한다. `travel_m`은 이 BT 프로세스가 관측한 TF 이동량이며 실제 배터리 소비가 아니다.

## 할당기만 새 시뮬레이션에 적용

새 패키지의 기본 Greedy는 유지한다. 같은 Python 환경에 아래 의존성을 설치하고 기존 패키지를 빌드한 뒤 명시적으로 선택한다.

```bash
python3 -m pip install -r simulation/webots_ros2_limo/requirements_cp_sat.txt
ros2 launch webots_ros2_limo exploration_flat_launch.py world:=moon allocator:=cp_sat
```

이 명령의 `world:=moon`은 팀원이 만든 별도 테스트 월드이며 **어제의 지형을 실행하는 명령은 위의 run.sh**다. `moon_radar` 전략은 frontier 할당 전용 CP-SAT을 스윕 자체에 적용하지 않으므로 여기서는 사용하지 않는다.

CP-SAT은 로봇당 비용 상위 32개 후보를 사용한다. 기본 탐색 시간 0.15초는 솔버에만 적용하며 거리장 계산·경로/그림자 검사·모델 구성 시간은 별도다. 추가 BT 제약에 걸린 해는 제외하고 다시 풀며, 검증된 해를 못 찾으면 해당 주기에 신규 목표를 보내지 않는다. 전 지도/연속 시간/모든 후보에 대한 최적성을 주장하지 않는다.

## 저장 지도 재현 검증

```bash
cd /home/seoyeong/py_bt_ros_chanseo_cp_sat
PYTHONPATH=simulation/webots_ros2_limo \
  ../py_bt_ros/lunar_cp_env/bin/python scenarios/lunar_chanseo_cp_sat/replay.py \
  --map ../py_bt_ros/artifacts/lunar_shutdown_20261121_222507/robot1_team_map.npz \
  --checkpoint ../py_bt_ros/artifacts/lunar_shutdown_20261121_222507/lunar_exploration_checkpoint.json \
  --output artifacts/lunar_chanseo_cp_sat/replay.json
```

이 스냅샷에서 Greedy와 CP-SAT 모두 네 대에 도달 가능한 관측 공간 목표를 배정하고, CP-SAT의 공동 목표는 10m 통신망을 유지했다. 이는 저장 지도 한 장에 대한 할당 검증이며 속도 비교 벤치마크, 그림자 주행 검증 또는 탐사 완료 결과가 아니다. 앞서 실행한 Greedy가 거리장 캐시를 예열하므로 두 실행 시간은 직접 비교하면 안 된다.

검증 명령:

```bash
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD/simulation/webots_ros2_limo:$PYTHONPATH" \
  ../py_bt_ros/lunar_cp_env/bin/python -m pytest -q simulation/webots_ros2_limo/tests
```

실제 실행에서 네 대의 이동량, 도착 후 후속 목표, 연결 상태를 로그로 확인한다. 전체 영역 탐사·그림자 사고율·장시간 정체 복구는 별도의 지속 실험이 필요하다.

2026-11-22 실제 실행 중간 확인: 새 BT 프로세스 기준 robot1 약 1.10m, robot2 2.96m, robot3 0.82m, robot4 1.55m 이동을 관측했다. 이 표본에서 네 대의 10m 연결이 유지됐고 그림자 토픽은 모두 false였으며, 목표 결과 3건(도착과 취소 포함)을 기록했다. 원시 확인 결과는 `artifacts/lunar_chanseo_cp_sat/live_result.json`이다. 이는 짧은 실행 확인이고 그림자 회피 성능의 정량 검증은 아니다.

## 2026-11-22 정지 재발 수정

원인 1: 두 로봇 중심 간격이 약 0.83m가 되자 0.85m 여유 조건이 초기 상태부터 실패해 모든 로봇의 이동을 막았다. 연결부에서만 `recovery_floor=0.65`를 사용한다. 기존 0.85m 이상 간격인 쌍은 그대로 보호하며, 0.65~0.85m의 쌍은 현재 간격을 줄이지 않는 명령만 허용한다. 0.65m 미만, 통신 단절, 오래된 입력은 계속 정지한다. 기본 공용 interlock 동작은 바뀌지 않는다.

원인 2: 동일 ROS 콜백에서 지도 거리장·그림자 경로·CP-SAT을 계산해 주행 명령 lease 갱신이 지연됐다. `PlanningWorker`로 단일 배경 작업만 실행한다. 활성 작업 ID가 바뀐 결과는 버리고, 결과를 적용하기 전에 최신 지도·위치·그림자·공동 통신 연결을 다시 검사한다. 계산 중에도 진행 중인 목표와 이동 허가는 갱신한다.

그림자 때문에 모든 frontier 목표에 안전한 정지 구간이 없을 때는 관측된 안전 공간의 중간 위치로 재배치를 시도한다. 이때도 동일한 경로·그림자·통신·간격 검사를 거친다. 그마저 불가능하면 `WAITING_FOR_SHADOW_WINDOW`와 다음 햇빛 시간 `solver.next_sunlit_sim_s`를 기록한다. 이 값은 시뮬레이션 시간이다. 다른 로봇의 안전한 주행은 계속 유지한다.

정지 원인을 확인할 때는 `motion_gates`, `physical_blocked`, `requested_commands`, `planning_pending`도 함께 본다. 이동 허가는 주행 명령의 허용 여부이며 실제 이동량은 `travel_m`과 위치 변화로 확인한다. 수정 전 상태와 수정 후 관찰은 `artifacts/lunar_stop_fix_20261122/`에 보관했다.


## 두 탐사 방식과 RViz 실행

`method:=cp_sat`은 CP-SAT 공동 목표 배정과 기존 거리장/Nav2 Dijkstra 경로를 사용한다. `method:=space_astar`은 탐사 후보를 최단 지도 거리와 제한된 정보 이득으로 순차 배정하고 공간 A*로 경로를 찾는다. A*는 목표 공동 배정 솔버가 아니며 Space-Time A*의 시간 예약도 포함하지 않는다. 두 방식 모두 같은 지도, BT, 그림자 예측, 통신망, 목표 간격, 실제 주행 검사를 사용한다.

```bash
cd /home/seoyeong/py_bt_ros_chanseo_cp_sat
# 각각 Webots와 RViz를 함께 실행한다. 종료는 Ctrl+C.
scenarios/lunar_chanseo_cp_sat/run.sh method:=cp_sat rviz:=true
scenarios/lunar_chanseo_cp_sat/run.sh method:=space_astar rviz:=true
# CP-SAT 배정과 A* 경로를 함께 쓰는 조합도 가능하다.
scenarios/lunar_chanseo_cp_sat/run.sh method:=cp_sat path_planner:=astar rviz:=true
```

바탕화면 실행 아이콘:

- `달 탐사 - CP-SAT`: CP-SAT 탐사와 RViz.
- `달 탐사 - Space A-star`: 공간 A* 탐사와 RViz.
- `달 탐사 - 두 방식 순서대로 비교`: CP-SAT을 실행하고 종료한 뒤, 같은 초기 월드에서 A*를 새로 실행한다. 각 실행은 입력 준비 후 실제 시간 120초 동안 시연한다. 관측률/로봇별 이동량과 로그를 `artifacts/lunar_live_demo_시간/comparison.json`에 보관한다.

비교 실행을 터미널에서 시작할 수도 있다.

```bash
python3 scenarios/lunar_chanseo_cp_sat/demo.py compare --seconds 120
```

같은 환경에서 두 제어 프로세스를 동시에 실행하지 않도록 `run.sh`에 파일 잠금을 둔다. 비교 런처는 자신이 시작한 프로세스 그룹만 종료하고 두 번째 월드를 시작한다. 기존 물리 상태나 SLAM은 복원하지 않는다. 두 실행에서 지도/캐시를 새로 구성하지만 시뮬레이션의 비결정성까지 제거한 통계 벤치마크는 아니다.

`path_planner:=auto`가 기본이며 CP-SAT에는 기존 거리장, A* 방식에는 A*를 선택한다. `astar`는 관측 지도 기반 4방향 경로를 사용하고 네 로봇의 Nav2 `GridBased.use_astar`도 켠다. 두 경로 방식은 미관측/장애물 통과 금지와 기존 로봇 몸체 시작 셀 예외를 공유한다. BT 검증 경로와 최신 비용 지도로 다시 계산하는 실제 Nav2 경로는 다를 수 있다.

RViz는 기본으로 자동 실행하며 `map` 좌표계에 다음을 표시한다.

- `Observed area`: 관측한 영역의 반투명 보라색 오버레이. 바탕 지도에서는 미관측 영역이 회색이다.
- `Observed percentage`: 60×60m 임무 영역의 관측률과 현재 탐사/경로 방식.
- `Exploration goals`: 현재 배정된 탐사 목표.
- `robotN selected exploration route`: 마지막 배정 시 검증한 경로. 완료/취소한 목표의 표시는 지운다.
- 기존 로봇별 센서와 `robotN path`: 실제 Nav2 경로.

관측 영역은 공유 지도에서 센서로 관측한 영역이며 관측된 장애물도 포함한다. 목표 전송/계획 경로만으로 관측률을 늘리지 않는다. 지도 바깥의 임무 영역도 미관측으로 계산하고 지도 회전을 반영한다. 관측률은 지도 해상도의 표본 추정이며 탐사 완료는 기존 BT의 엄격한 판정을 유지한다.

데이터 토픽: `/exploration/coverage`, `/exploration/coverage_status`, `/exploration/coverage_label`, `/exploration/robotN/planned_path`. `live_status.json`과 `live_status_cp_sat.json`/`live_status_space_astar.json`에 사용 방식, 관측률, 이동량을 기록한다.

GUI 시연은 로그인한 데스크톱 세션에서 실행해야 한다. 현재 Codex 샌드박스는 X11 소켓 연결을 `Operation not permitted`로 차단한다. 이 환경에서 창이 실제 열렸다고 보고하지 않는다. 바탕화면 런처는 정상 데스크톱 환경에서 Webots/RViz를 실행하며, 화면 접속이 안 되면 실행 전에 이유를 출력한다.


## 현재 실시간 RViz 장애물 지도 설정

`Fixed Frame`은 `map`이다. 이 PC의 OccupancyGrid 렌더러에서 GLSL sampler 오류가 발생해 기본 지도는 같은 실시간 `/lunar/robot1/team_map` 데이터를 MarkerArray로 표시한다. 검정은 점유 셀(65 이상), 짙은 회색은 불확실한 장애물 셀(25~64), 흰색은 관측 자유 공간(0~24), 회색 바탕은 미관측 영역이다. 지도 원점과 회전을 유지하고 경로 계산에 가상의 장애물을 추가하지 않는다.

- `Lunar obstacles and observed free space`: `/exploration/obstacle_map`, MarkerArray, Transient Local.
- `Robot positions and headings`: `/exploration/robot_markers`, MarkerArray, Transient Local.
- `Views`에는 로봇 시작 부근 기본 보기와 `Whole lunar mission (60m)` 전체 보기를 저장한다.
- 표준 Map 표시를 직접 쓰려면 `OccupancyGrid map (optional GPU renderer)`를 켜고 `/lunar/robot1/team_map`, Color Scheme `map`, Transient Local을 사용한다. 현재 PC의 그래픽 오류가 재발하면 기본 MarkerArray 표시를 사용한다.

2026-11-25 화면 접속 권한 변경 후 Webots/RViz 실제 창 실행, 네 로봇 이동, 실시간 지도 마커의 자유 공간/장애물 및 네 로봇 표식 수신을 확인했다. 이전 제한 환경의 화면 접속 실패는 현재 실행 상태를 의미하지 않는다.


## 느린 주행/제자리 회전 조정 (2026-11-25)

0.02m/s로 보인 측정 구간에서는 잦은 방향 정렬로 직진 명령이 대부분 0이었고, 실제 시간 대비 시뮬레이션 진행률도 약 0.13이었다. RViz의 강제 CPU 렌더링을 제거해 Intel GPU를 사용하고, 네 Nav2 컨트롤러의 lookahead를 0.35m에서 0.6m로, 제자리 방향 정렬 시작 각도를 0.25rad에서 0.6rad로 조정했다. 현재 실행에도 SetParameters 성공을 확인해 적용했다. 조정 후 15초 표본에서 네 로봇의 odometry 직진 속도 평균은 약 0.218~0.220m/s였고, 최종 주행 명령에 완전 정지는 없었다. 실제 시뮬레이션 진행률은 약 0.20으로 여전히 실시간보다 느리다. 표본이 다른 위치/회전 상황이므로 통계적 성능 비교는 아니다.

장애물 지도의 셀별 큐브를 가로로 이어진 구역당 두 삼각형으로 바꿔 화면 표시 비용을 낮췄다. 지도 셀을 다운샘플링하지 않으며 랜덤/전체/빈 마스크를 재구성해 원본과 동일함을 확인했다. 드라이버의 0.22m/s 상한, 그림자 예측 속도 범위, 통신/충돌 검사와 watchdog은 유지했다. `artifacts/lunar_live_20261125/speed_before.json`, `speed_after.json`, `speed_compact.json`에 측정 결과를 남긴다.


## 탐사 확장 목표 평가 수정 (2026-11-25)

CP-SAT과 공간 A* 방식의 frontier 순위는 이제 잘라낸 다음 이동 구간의 거리 대신 원래 frontier까지 관측 지도에서의 전체 경로 거리를 사용한다. 통신/그림자 검사 때문에 먼 frontier의 허용 구간이 짧아졌다는 이유로 그 후보가 유리해지는 문제를 고쳤다. 실제 명령은 여전히 최대 3m의 검증된 짧은 구간이며 미관측 영역을 가로지르는 직선 명령은 추가하지 않는다.

이 연결부는 최근 180초(실제 시간)에 방문 완료한 탐사 목표의 1.25m 이내를 새 정지 목표로 즉시 재선택하지 않는다. 해당 지역을 통과하는 경로는 계속 허용하고 제한은 만료된다. 경로 탐색/통신/그림자 검사는 계속 적용한다. 목표 부족 시 관측 공간 재배치가 여전히 필요할 수 있으며 이 수정만으로 전체 탐사 완료를 보장하지 않는다.

현재 월드를 유지한 제어부 재시작은 `bridge.py --resume-status <현재 live_status 복사본>`으로 이동량과 완료 이력을 이어받는다. 기존 활성 Nav2 목표를 먼저 취소하고 새 세션에서 재배정한다. 이는 같은 월드에 제어부만 재접속하는 용도이며 다른/초기화된 월드의 물리 상태나 지도를 복원하는 기능은 아니다. 2026-11-25 재접속 전 상태는 `artifacts/lunar_live_20261125/frontier_fix_before.json`에 보관했다.
