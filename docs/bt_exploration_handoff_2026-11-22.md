# 4대 BT·센서 반복 탐사 인계 (2026-11-22)

이번 사용자의 확정 요구사항이 과거의 “3대 활동 + robot4 대기”를 대체합니다.
기본은 robot1~4 모두 참여하며, 목표 부족·안전·유한 계산 예산 때문에 일부가 대기할 수 있습니다.
작업 경로는 `/home/gayeon/lunar_hungarian_project`, 브랜치는 `feature/shadow-hungarian`입니다.
기존 미커밋 변경을 보존했습니다. reset·원격 업로드·push·PR 생성은 수행하지 않았습니다.
상위 디렉터리와 저장소에서 적용되는 `AGENTS.md`를 발견하지 못했습니다.

## 실행 순서

기존 Webots·RViz·BT·Nav2 launch를 Ctrl+C로 종료한 뒤 Docker와 데스크톱 권한이 있는 호스트 터미널에서 실행합니다.
서로 다른 모드를 동시에 실행하지 않습니다. 현재 nouveau의 Mesa NVK/Zink 방식을 그대로 사용합니다.

```bash
cd /home/gayeon/lunar_hungarian_project
bash docker/lunar/run_demo.sh bt-fixed
```

`bt-fixed`가 실제로 `COMPLETE / fixed_goals_observed`인지 확인한 뒤 종료하고:

```bash
bash docker/lunar/run_demo.sh explore
```

단기 센서 탐사·화면·지표를 확인한 다음에만 종료 후 넓은 프로필을 실행합니다.

```bash
bash docker/lunar/run_demo.sh explore-wide
```

새 기본 모드는 `bt-fixed`입니다. `single`은 기존 단일 로봇 회귀 검사이고, `multi`는 BT가 없는 기존 실행기의 4대 모드입니다.
이미지의 BT 의존성 표시가 없으면 스크립트가 SciPy·Matplotlib·한글 폰트를 포함하도록 증분 빌드합니다.
독점 NVIDIA 드라이버 설치를 요구하지 않습니다. Docker/화면 권한이 없으면 실행을 중단합니다.

다른 터미널의 상태·시작·정지 명령은 기존과 같습니다.

```bash
bash docker/lunar/run_demo.sh status
bash docker/lunar/run_demo.sh stop
bash docker/lunar/run_demo.sh start
```

컨테이너의 빌드된 Jazzy overlay에서 직접 실행할 때:

```bash
ros2 launch moon_mapping_demo moon_exploration.launch.py mode:=fixed
ros2 launch moon_mapping_demo moon_exploration.launch.py mode:=explore
```

위 두 launch는 순서대로 종료·실행해야 합니다. `execute:=false`에서는 정지만 유지하며 비용 계획을 자동 실행하지 않습니다.

## 실제 재사용과 연결 계약

`qkrckstj/py_bt_ros`의 `dev_chanseo@02cda62197e30858f295de25567003f02242ad64`에서 가져온
`exploration_bt.py`, `exploration_runtime.py`, `exploration_policy.py`, `exploration_sweep.py`를
`moon_mapping_demo/vendor/team_bt/`에 수정 없이 보존했습니다. 원본 LICENSE와 blob SHA는 같은 위치의 `UPSTREAM.json`에 있습니다.
새 어댑터는 이 원본의 `TeamBT`·`RobotBT` 트리, 명령 lease, session/task 재전송·중복 차단, 상태·방문 이력을 재사용합니다.

`HungarianTeamBT`는 원본의 greedy 배정 leaf와 완료 판정을 확장합니다.
관측 frontier 후보 선정 → 공통 t0 비용 행렬 → 직접 구현 Hungarian → 공동 시간 경로 → 전체 준비 → 주행·관측 → 다음 회차를 관리합니다.
`ObservingRobotBT`는 원본 `ready/start/poll/cancel` navigation 계약에 `TimedNavigation`을 주입합니다.
Nav2 액션은 발행하지 않으며, 실제 이동은 기존 `control_step`과 단일 노드의 네 `cmd_vel` 발행자를 사용합니다.

각 명령은 원본 `session`, `task_id`, `mode`, `goal: [x,y,yaw]`, `purpose`에 `plan_id`를 추가합니다.
모든 배정 로봇의 navigation.start가 준비 장벽에 도달해야 공동 경로를 실행합니다. 선형 이동은 공통 t0 전에는 시작하지 않습니다.
계획 내부의 이동·회전·그림자 대기 때문에 각 로봇의 첫 선형 이동 시각은 다를 수 있습니다.
단일 취소·준비 실패·실행 이탈은 전체 정지 후 새 t0·행렬·task 번호로 재계획합니다.
지연 worker 결과와 상태 보고는 session/task/plan 조합으로 차단하며 취소된 계산을 금지 간선으로 바꾸지 않습니다.
도착 이후 원본 관측 시간이 지났어도 도착 이후 취득한 실제 유효 스캔이 최소 3개 없으면 `VISITED`가 되지 않습니다.

원본 `/exploration/start`, `/exploration/stop`의 `std_msgs/Empty` 토픽과
기존 `/shadow_mission/start`, `/shadow_mission/stop`의 Trigger 서비스를 지원합니다.
`/robotN/exploration/command`는 실제 내부 명령의 모니터링용 단방향 출력입니다. 외부 RobotBT를 추가 실행하지 않습니다.
`/exploration/team_status`, `/robotN/exploration/state`, `/shadow_mission/status`에서 상태를 확인합니다.
통신 거리·기지·중계 제약과 자동 기지 복귀는 사용하지 않습니다.

전용 `moon_exploration.launch.py`는 기존 `moon_mapping.launch.py shadow_mode:=true exploration_mode:=true`의
네 드라이버·센서·시작 위치 TF 앵커와 새 노드 하나를 실행합니다. SLAM·Nav2·옛 shadow_mission·팀원 launch를 중복 실행하지 않습니다.
다른 `cmd_vel` 발행자를 발견하면 시작을 거부하거나 전체 정지합니다.

## 관측, 목표, 배정의 구분

WBT 지형·태양 모델은 경로 계획과 안전 검사에만 사용합니다. 초기 관측 격자는 전부 미관측입니다.
기존 `/scan`의 `0=invalid` 계약은 유지하고, `/robotN/scan_observation`에서 실제 Webots 장치의
범위·FOV·취득 시각과 `+inf=유효 무반사`, `NaN=무효`를 별도로 보존합니다.
ROS 어댑터는 스캔 취득 시각의 `map → laser_link` TF를 사용합니다. 최신 자세로 대체하지 않습니다.
TF를 기다리되 시뮬레이션 나이 1초·실제 대기 3초를 넘으면 버립니다.
스캔·clock 콜백 도착 순서의 최대 50ms 선행 허용은 입력 수용과 센서 신선도 검사에 동일하게 적용합니다. 세션 이전·잘못된 프레임·유의미한 센서 기울기도 제외합니다.

각 유효 광선이 실제 통과한 셀과 첫 반사점만 관측합니다. 센서 최대 범위와 설정 상한 중 작은 값을 사용하며,
장애물 뒤나 광선 사이의 부채꼴을 임의로 채우지 않습니다. 2D 격자 해상도에 따른 근사이며 3D 지표면적이 아닙니다.
늦은 스캔은 최신 점유 상태를 덮지 않습니다. 로봇 간 중복 관측은 팀 면적에서 한 번만 셉니다.
로봇별 신규 면적은 **융합 처리 순서의 최초 기여도**입니다. 취득 시각의 전역 정렬이나 공평한 기여 분배를 뜻하지 않습니다.

후보는 관측 자유 공간과 미관측 셀 사이 frontier로부터, 관측 자유 공간의 연결 경로를 따라 제한 거리만 전진하도록 선정합니다.
미관측 셀 원판의 정보 이득 상한과 목표 간격은 후보 선택 휴리스틱이며 실제 관측 면적으로 누적하지 않습니다.
선정 뒤에 Hungarian을 적용합니다. 4개 목표를 모두 배정하는 시도는 같은 t0의 4×4 정수 tick 행렬입니다.
실패하면 더 작은 실제 로봇·목표 부분집합을 유한 횟수로 탐색하고, 미배정 몸체를 정적 장애물로 넣어 비용을 다시 계산합니다.
부분집합 선택, 공동 경로 순서 선택, frontier 선택은 전체 탐사 최적화가 아닙니다.

독립 A* 비용은 기존 이동·대기 모델입니다. BT 공동 실행에서는 경로 방향 변화에 필요한 정수 tick 회전 대기를 추가하고,
바뀐 시각으로 그림자·예약·목표 hold를 다시 검사합니다. 회전 추가분은 공동 비용에 포함하며 원래 Hungarian 최적값과 구분합니다.
통합 계획과 실행의 몸체 반경은 설정값 0.26m입니다. 실행 중 실제 odom 위치에서 남은 경로와 투영 명령을 다시 검사합니다.
명목 경로의 안전성이 오도메트리 자체의 절대 위치 오차나 실제 지형 모델의 오차를 보장하지 않습니다.

계산·출발 대기는 계획 시작부터 t0까지, 미배정 로봇은 해당 공동 임무 종료까지 안전해야 합니다.
관측·대기의 접근 그림자를 예측하고 필요하면 관측 완료와 구별한 대피 목표를 생성합니다.
대피 목표의 미래 안전 구간, 공동 경로와 회전 시간도 다시 검사합니다. 안전한 대피 계획을 찾지 못하면 명시적으로 정지합니다.
현재 회차의 검증된 hold 이후에는 새 임무/대피를 선정합니다. 종료 뒤 계속 정지한 몸체의 무한 시간 안전성은 보장하지 않습니다.

## 고정 분모와 출력

| 프로필 | 사전 고정 평가 사각형 | 분모 | 제한 |
| --- | --- | --- | --- |
| `exploration_short.yaml` | x,y 모두 [-29,-9), 0.25m 셀 | 400m² | 180 sim초, 최대 4회차, 전진 후보 1.5m·관측 4초 |
| `exploration_wide.yaml` | x,y 모두 [-29,29), 0.25m 셀 | 3364m² | 1800 sim초, 최대 100회차, 전진 후보 3m·관측 7초 |

분모는 장애물·관측 불가능 셀도 포함한 고정 ROI의 셀 수×셀 면적입니다. 실험 중 관측 범위에 맞춰 줄이지 않습니다.
사전 지도 조회, 목표 생성, 목표 도착, `VISITED` 자체는 면적을 증가시키지 않습니다.
기본 탐사율 목표는 85%이며 시간·회차 제한이나 진행 정체는 `PARTIAL_COMPLETE`와 구체적인 종료 사유로 기록합니다.
`fixed_goals_observed`의 `COMPLETE`는 고정 목표 연결 검증의 완료이며 전체 ROI 탐사 완료를 뜻하지 않습니다.

RViz에는 미관측 회색·관측 자유 흰색·관측 점유 검정 지도, 네 시간 경로·시각, 탐사율·팀 BT·로봇 BT 텍스트를 표시합니다.
관측 격자 토픽은 `/exploration/observations`, BT 표시는 `/exploration/bt_markers`입니다.

각 실행은 `results/gui_<mode>_.../latest.json`이 가리키는 session 하위 폴더에 저장합니다.
수동 재시작도 새 session을 만들어 이전 결과를 보존합니다.

- `runtime.json`: 현재 팀·개별 BT 트리와 상태, session/plan/회차, 관측 요약·종료 사유.
- `metrics.json`, `metrics.csv`: 시간별 탐사율, 로봇별 최초 신규 면적·오도메트리 이동거리·대기·관측·회전 시간, 재계획 횟수와 사건.
- `observations.npz`: 관측/미관측/점유 셀, 셀별 최초·최신 관측 sim 시각, 원점과 해상도.
- `scan_poses.csv`: 융합한 스캔의 취득 sim 시각·센서 자세·범위·각도·새 면적. ROS 실행에서 기록.
- `round_*/plan_*/attempt_*/problem.json`, `problem.csv`: 공통 t0 정수 비용·금지 간선·선택 순서·계산 시간.
- `batch_plan.json`: 실제 공동 시간 경로, 원래 배정값과 회전·충돌 조정 후 값.
- `coverage.png`, `coverage.svg`: 시뮬레이션 시간별 탐사율, 실제 소요 시간별 탐사율, 로봇별 신규 면적. 종료·shutdown 후 생성.

관측·거리·대기 지표의 시간은 sim 기준이며 실제 경과 시간은 monotonic 기준으로 별도 열·그래프에 저장합니다.
대기에는 계획·시간표·미배정 정지를 포함하며 관측·제자리 회전은 별도 누적합니다. 제어 주기의 이산 근사입니다.
거리는 휠/IMU odom 기준이며 시간 역행·급격한 위치 점프를 누적 거리에서 제외합니다.
그래프를 다시 생성하려면 빌드된 overlay에서 `moon_exploration_plot <session>/metrics.json`을 실행합니다.

## 검증과 다음 작업

최종 실행 결과와 미해결 사항은 [검증 JSON](shadow_validation_2026-11-22.json)의 `bt_exploration_integration` 항목이 기준입니다.
이전 단일·3대 Webots 성공 기록을 이번 새 4대 BT·센서 통합의 실제 실행 성공으로 재사용하지 않습니다.
이번 Codex 환경에서는 Docker 소켓 접근이 거부되었습니다. 우회·권한 변경을 하지 않았습니다.
호스트 Humble의 프로세스 내 합성 ROS 검사는 네트워크 DDS·Jazzy·실제 Webots 물리·RViz 화면 검사를 대체하지 않습니다.
WBT 기구학 검사의 센서는 모의 광선 추적이며 실제 센서 결과와 명확히 구분합니다.

다음 담당자는 먼저 호스트에서 `bt-fixed`의 네 로봇 실제 주행과 관측 완료를 확인하고,
`explore`의 다음 회차 자동 생성·관측 지도·CSV/그래프를 확인한 뒤 넓은 평가를 진행해야 합니다.
장시간의 모델/물리 종료 사유와 센서 시각 TF 지연, 오도메트리 드리프트, 보수적 대피 탐색 실패를 별도로 분석해야 합니다.

## 최종 수치와 미해결 항목

- 원본 네 모듈과 LICENSE의 blob SHA가 고정 커밋의 기록과 모두 일치합니다.
- 호스트 Humble 빌드, 패키지·기존 BT/드라이버 회귀 **117개**, 합성 ROS 계약 **7개**가 통과했습니다. 자세한 결과는 검증 JSON에 기록합니다.
- 실제 WBT + 이상 기구학 + **모의 센서**의 4대 고정 목표 검사는 sim 56.1초에 `COMPLETE`, 재계획 0회였습니다.
- 같은 모델의 단기 반복 탐사는 180 sim초 동안 탐사 2회차를 마쳤고, 400m² 중 122.5625m²(30.640625%)를 관측했습니다.
  네 로봇 모두 탐사 목적의 임무를 받았습니다. 일부 회차는 유효 목표/공동 계획 제약으로 3대 배정·1대 대기였습니다.
  탐사율 목표 85%에는 도달하지 못했으며 `PARTIAL_COMPLETE / sim_duration_limit`입니다. 그래프는 모의 센서임을 제목에 표시합니다.
- 넓은 3364m² 프로필은 1800 sim초를 요청했지만 **sim 77.2초**, 탐사 1회차 후 대피 재계획에서
  `no_safe_evacuation: NO_SAFE_ASSIGNMENT`로 정지했습니다. 119.5625m²(3.5542%) 관측, 재계획 2회입니다.
  이는 현재 후보·회전·우선순위 예약의 유한 탐색에서 실패한 것이며, 실제로 대피가 불가능하다는 증명이 아닙니다.
  장시간·넓은 탐사 평가는 **미완료**입니다. 경로 실행을 강행하거나 이 실패를 탐사 완료로 바꾸지 않았습니다.
- 다음 개선 대상은 대피 목표 후보를 여러 개 비교하는 탐색, 그림자 진입 전 회전·계산 시간을 포함한 더 이른 대피,
  명목 경로와 실제 odom 위치 사이의 여유 처리입니다. 임의의 여유 반경 증가가 공동 경로를 막는 실험도 있어 기본 반경을 바꾸지 않았습니다.
- 실제 새 Jazzy 이미지 의존성 빌드·4대 Webots 물리 주행·센서 누적·RViz 화면 확인은 Docker 권한 때문에 아직 수행하지 못했습니다.

최종 모델 결과는 `results/bt_fixed_verified_20261122/`, `results/bt_explore_final_v3_20261122/`,
`results/bt_wide_final_v2_20261122/`에 있습니다. 중간 실패·시도 결과도 별도 폴더에 보존했습니다.
`results/`는 저장소의 기존 gitignore 대상이므로 소스만 복사하면 실험 산출물이 포함되지 않는 점에 유의합니다.

## 후속 수정: 4대 배정 유지·2대 우선 출발 정책 (2026-11-22)

이 절과 검증 JSON의 `bt_exploration_integration.departure_staggering`이 출발 정책의 최신 상태입니다.
기존 `simultaneous` 정책을 기본값으로 유지하고 `staggered_pairs`를 선택형으로 추가했습니다.
최초 4×4 정수 행렬·Hungarian 배정은 동일하며, 그 뒤 공동 경로에서 선발 2대와 후발 2대를 정합니다.
4개 목표 시도의 시차 탐색이 실패했다고 2대 배정으로 바꾸지 않습니다. 실제 목표가 3개 이하인 경우의 기존 부분 배정과 구분합니다.

### 기존 실패에서 확인한 사실

기존 넓은 영역 실패를 새 폴더에서 재현했습니다. 정확한 sim 시각 77.10000000000001의 재계획 직전 상태를 저장해 확인한 결과,
robot3의 남은 이동 선분이 정적 장애물이 아닌 시간 금지 영역(보수적 그림자 모델)에 걸렸습니다. 남은 공동 경로 간 충돌은 검출되지 않았습니다.
다음 공통 t0=80.10000000000001에서는 robot3의 출발점이 `start_forbidden`이 되어 네 비용이 모두 null이었습니다.
기록된 최소 표본 간격은 0.7432m로 0.52m 몸체 충돌 기준보다 큽니다.
사용자가 화면에서 관찰한 이동 겹침은 검토할 가설이며, 이 모의 실패의 확정 원인으로 단정하지 않습니다.

### 구현

- `departure.py`: 현재 위치, 목표 방향의 초기 통로 몸체 점유, 그림자 접근으로 선발 조합 순위를 정합니다.
  조합·선발/후발 내부 순서·추가 시차·대기 위치를 섞어 최대 24개 후보와 기존 실제 계산 예산 안에서 탐색합니다.
- 후발의 초기 대기는 공통 t0부터 경로에 명시합니다. 해제 시각에서 Space-Time A*를 다시 실행하며 기존 경로의 시각만 이동하지 않습니다.
- 선발은 최종 목표 도착을 요구하지 않고 초기 구역을 벗어나면 후발을 해제합니다. 이후 이동 로봇 수를 2대로 제한하지 않습니다.
- 기본 해제는 출발점에서의 변위 기준이며 짧은 목표에서는 목표 거리 절반으로 제한합니다. `release_region`을 지정하면
  xmin,xmax,ymin,ymax 사각형을 몸체까지 벗어나는 조건을 사용합니다. 자동 통로 분할의 정답이라고 해석하지 않습니다.
- 예정 해제에 1 tick 여유를 두고, 실행 시 실제 odom의 통과도 확인합니다. 미확인 시 전체 정지·새 plan으로 재계획합니다.
  명시적 release_region은 목표 도착 상태만으로 실제 통과 검사를 우회하지 않습니다.
- 초기 위치의 대기가 불가능하면 다른 조합/시차를 검토합니다. 선발 해제 후 전방 대기점을 A*로 경유하는 후보도 포함하며,
  이 대기점과 추가 대기의 안전성도 검사합니다. 후발을 t0에 먼저 이동시켜 초기 대기를 우회하지 않습니다.
- 대기 몸체·그림자·선발/후발 이동·회전·관측 hold를 검사합니다. 반경 0.26m, 입력 신선도, 센서 광선·고정 분모 기준을 유지합니다.
- `WAITING_DEPARTURE`를 BT 상태에 추가하고 RViz에는 ‘출발 대기’, 선발/후발, 예정 이동 시각을 표시합니다.
  대기 중에도 원본 BT의 명령 lease, 실제 스캔 누적, 입력 감시를 수행합니다.
- session/task_id/plan_id 차단·전체 준비·취소·재계획과 단일 cmd_vel 소유권을 유지합니다.
- JSON/CSV에 최초 Hungarian 값, 공동 조정 값, 초기/대기점 대기·회전·나머지 경로 비용 차이,
  선발/후발, 예정 해제/이동 시작, 최초 선형 명령, odom 이동 시작, plan/task ID를 구분해 기록합니다.
  실제 이동 시작은 해당 plan 시작 위치에서 유효 odom 변위가 0.01m 이상인 최초 취득 시각입니다.
  경로·기타 대기 비용 차이는 원래 경로의 대기가 대체될 수 있으므로 부호 있는 값입니다.

### 선택과 재현

기존 launch를 종료한 뒤 호스트 터미널에서 새 정책을 선택합니다. GPU 설정은 기존 nouveau Mesa NVK/Zink를 유지합니다.

```bash
cd /home/gayeon/lunar_hungarian_project
LUNAR_GPU=mesa LUNAR_DEPARTURE_POLICY=staggered_pairs bash docker/lunar/run_demo.sh bt-fixed
# 위 실행 종료 후 단기 확인
LUNAR_GPU=mesa LUNAR_DEPARTURE_POLICY=staggered_pairs bash docker/lunar/run_demo.sh explore
# 넓은 영역은 현재 모의 비교에서도 실패하므로 완료 검증으로 간주하지 않음
LUNAR_GPU=mesa LUNAR_DEPARTURE_POLICY=staggered_pairs bash docker/lunar/run_demo.sh explore-wide
```

비교용 기존 정책은 `LUNAR_DEPARTURE_POLICY=simultaneous`입니다. 직접 launch는
`departure_policy:=staggered_pairs`, YAML은 최상위 `departure.mode`로 선택합니다.
설정 파일의 기본값도 simultaneous입니다.

모의 전체 비교를 다시 수행하려면 반드시 새 결과 폴더를 지정합니다.

```bash
PYTHONPATH=ros_packages/moon_mapping_demo python3 ros_packages/moon_mapping_demo/tests/departure_comparison.py --output results/새로운_정책비교_폴더
```

위 도구의 exit0은 비교 산출물 생성 완료이며 모든 실험 성공을 뜻하지 않습니다. 각 행의 passed·종료 사유를 확인합니다.

### 같은 조건의 비교 결과 (모의 센서·이상 기구학)

모든 프로필에서 최초 로봇/목표 순서·t0·정수 행렬·Hungarian 배정이 두 정책에서 일치했습니다.
시간은 아래 표에서 **경과 sim 시간**입니다. 이전 절의 절대 sim 시각과 1초 차이가 있습니다.

| 프로필 | 기존 simultaneous | 새 staggered_pairs |
| --- | --- | --- |
| 4대 고정 목표 | 완료 55.1초, 재계획0 | 완료 69.1초, 재계획0 |
| 단기 400m² | 180초, 탐사2회차, 관측30.640625%, 시간 제한 종료 | 84.3초, 탐사1회차, 관측25.75%, 대피 계획 실패 |
| 넓은 3364m² | 76.2초, 탐사1회차, 관측3.5542%, 대피 재계획 실패 | 14.4초, 탐사0회차, 관측2.7460%, 공동 계획 탐색 실패 후 부분 종료 |

고정 목표의 Hungarian 값은 양쪽 모두 57tick입니다. 기존 공동 값83tick(회전26), 새 공동 값111tick(회전26+초기 대기28)입니다.
**좁은 통로 기구 검사에서는 선발 통과·후발 대기·목표 도착 전 후발 출발이 검증됐지만, 현재 넓은 영역 실패의 개선은 확인하지 못했습니다.**
추가 대기는 공동 임무 종료와 관측 hold를 늦춰 그림자 제약을 더 어렵게 만들 수 있습니다. 새 정책을 기본값으로 바꾸지 않았습니다.
실패 시점의 위치·공통 t0를 고정한 재검사에서는 양쪽 모두 robot3의 네 비용이 null이므로 출발 조정만으로 해당 상태를 복구하지 못했습니다.
다음 개선은 더 이른 대피·관측 hold·목표 선택과 회전 비용을 함께 고려해야 하며 안전 검사를 약화시키면 안 됩니다.

검증: 회귀128개, 새 정책 관련11개를 포함한 관련 검사31개, 새 정책 합성 ROS7개와 기존 정책 ROS 연결1개를 확인했습니다.
ROS는 공유 rclpy context의 합성 입력이며 네트워크 DDS·실제 물리 검증을 대체하지 않습니다.
Docker 소켓 접근은 다시 permission denied였으며 실제 Webots bt-fixed → explore → explore-wide는 미수행입니다.

새 결과 위치:
- `results/staggered_departure_20261122_181339/final_comparison/comparison.json`, `comparison.csv`
- 같은 폴더의 `departure_timeline.png`, `coverage_comparison.png`
- 상위 폴더의 `exact_failure_analysis.json`, `historical_failure_analysis.json`, `initial_inventory.json`
- 상위 폴더의 `ros_pairs_verified/`, `ros_simultaneous_connection/`, 검사 로그

이전 실험 폴더를 덮어쓰거나 삭제하지 않았습니다. 기존 미커밋·미추적 파일을 보존했고 reset·clean·원격 업로드·push·PR은 수행하지 않았습니다.

## 추가 수정: 이른 대피·관측 후 안전 시간·복수 대피 후보

이 절이 위험 대응의 최신 기록입니다. 앞의 staggered_departure 결과는 과거 비교로 보존합니다.
브랜치/HEAD: feature/shadow-hungarian / 97b4759786232b77591530c9604326c325e93fbf.
새 결과 루트: results/early_escape_20261122_191957/.
**기본 출발 정책은 simultaneous이며 staggered_pairs의 탐사 개선을 가정하지 않습니다.**

### 재현으로 확인한 사실과 모델의 한계

- 기존 넓은 영역을 같은 월드·초기 위치·시작 sim=1·센서·3364m² 분모로 다시 실행해 절대 sim77.2초에 같은 대피 배정 실패를 재현했습니다.
- 정확한 replan trace의 감지 시각은77.10000000000001초입니다. 그 순간 네 로봇의 실제 몸체 위치는 금지가 아니며 정적 지형 충돌도 아닙니다.
- robot3의 실제 위치에서80.10000000000001초까지 정지하면 그림자 금지입니다. 모델상 정지 안전 기한 하한은 약79.744초, 최초 금지 상한은79.751초입니다. 이후 공통 t0에서 robot3의 네 비용이 start_forbidden/null이 됩니다.
- 먼저 실패한 검사는 robot3의 **남은 이동 구간**(77.1→80.1초)입니다. 정적 금지=false, 동료 공동 충돌 목록=[]입니다. 이 고정 실패 자체는 계산 예산 초과나 최초 관측 hold 실패가 아닙니다.
- 앞선 계획에 회전/대기가 포함됐지만 이 한 실패를 회전만의 원인으로 단정하지 않습니다. 별도② 단계의 대피 공동 계획에서24개 예약 순서 모두 회전 후처리 재검사에 실패한 근거를 확인한 뒤 방향 A* 비교를 추가했습니다.
- 반올림 시각과 마지막 위치만 사용한 과거 재구성에서는 남은 이동 금지가 재현되지 않았습니다. 원인 진단은 저장된 정확한 trace를 사용합니다.
- geometry_final/failure_geometry.png 및 .svg는 원본 WBT 삼각형의 지면 투영 그림자와 현행 기둥·구간 AABB 감쌈을 대조합니다. 남은 이동의 동시 시각301개 표본에서는 금지가 검출되지 않았지만 전체 구간 감쌈은 금지였습니다. **공간·시간 감쌈의 보수성이 관여할 수 있다는 진단이며, 표본이나 투영 그림으로 연속 안전을 증명한 것은 아닙니다.** 안전 oracle·0.25초 검사 설정·0.26m 몸체는 바꾸지 않았습니다. 실제 Webots 그림자와의 대조는 미검증입니다.

근거는 새 루트의 failure_diagnosis.json, geometry_final/geometry_diagnosis.json, 기존 루트의 exact_failure_analysis.json 및 정확한 replan trace입니다.

### 구현한 동작

1. risk.py가 현재 몸체의 정지 안전 구간과 실제 위치에서 시작하는 남은 이동·회전/대기·도착 후 유한 hold의 최초 금지를 예측합니다.90초 유한 범위,0.5초 주기이며 경고에만0.05m 위치 여유를 추가합니다. 실제 검사 반경을 줄이지 않습니다.
   준비 추정은 계획5 + 교체0.5 + 준비3 + 회전6.783 + 3.5m 탈출19.444 + 여유5 = 약39.728 sim초입니다. 거리·회전 추정은 경고용 휴리스틱이며 탈출 가능성의 증명이 아닙니다. 실제 A*와 공동 검사로 별도 검증합니다.
2. 계산 시작 **전에** 현재부터 새 t0까지 네 몸체의 정지 안전성을 검사합니다. 일반 lead10초를 유지하고 대피 lead는 계산5+교체0.5+준비3을 수용하는 최소8.5초로 잡습니다. 고정 대기를 무조건 줄이지 않습니다.
   계산 중에도 정지 구간·입력·명령 권한을 감시합니다. sim 계산 기한5초와 monotonic 실제 예산8초를 별도 기록하고 초과/취소를 구분합니다. 설치 직전 session/plan ID, 현재 시각, 스냅샷 대비 위치·방향, 남은 정지 구간을 재검사합니다. 이전 결과를 새 task와 혼용하지 않습니다.
3. 목표의 관측 시간과 추가12초 후속 계획/준비 대기를 A* 목표 서비스 시간대로 검사합니다. 최소3개의 도착 이후 유효 scan과 원래 관측 시간은 유지합니다. 첫 도착이 불리해도 더 늦은 도착을 유한 horizon 안에서 탐색합니다.
   가장 늦은 동료를 기다리는 조기 도착 로봇의 hold도 검사하고 필요하면 도착 시각을 재탐색합니다. 추가12초는 다음 계산용 예약이며 회차 종료를12초 늦추지 않습니다. 후보 순위에 유한 안전 여유를 반영하지만 관측 판정/분모는 바꾸지 않습니다.
4. 로봇별 최대3개 대피 후보에서 최대8개 조합을 공유 계산 예산 안에서 비교합니다.4대 전체를 배정하고 정지 안전 기한이 짧은 로봇부터 예약 순서를 시도합니다. 후보마다 공통 t0의 비용을 재계산하고 Hungarian 뒤 공동 검사를 수행합니다.
   완전히 검증된 후보 중 유한 목표 대기 여유를 우선하고 공동 비용을 차순으로 사용합니다. 탐색 예산 종료 때 검증된 후보가 있으면 그 후보와 중단 사유를 반환합니다. 명시적 취소라면 기존 후보도 폐기합니다. 후보·예약 순서는 유한 휴리스틱입니다.
5. risk_stage:=heading 비교 옵션은 Space-Time A* 상태에 방향을 포함하고 회전·대기·이동을 처음부터 계산합니다. 동일 회전 속도0.5rad/s와 정착 여유0.5초를 사용하며 회전 시간을 삭제하지 않습니다.
   이때 독립 정수 행렬에도 회전이 들어가므로 기존 이동·대기 행렬과 같은 모델로 취급하면 안 됩니다. JSON/CSV에서 독립 배정 회전, 공동 회전, 추가 회전, 대기·나머지 경로 차이를 구분합니다. 실제 turning 시간과 예정/odom 출발 시각도 별도입니다. 취소된 경로의 미실행 회전까지 포함한 계획 tick 합을 실제 회전 시간으로 취급하지 않습니다.

네 로봇 참여,4개 목표 시도 시 공통 t0의4×4 정수 행렬, Hungarian, Space-Time A*, 기존 TeamBT·RobotBT·TimedNavigation, 전체 준비/취소/정지/재계획, session/task_id/plan_id 지연 차단, 단일 cmd_vel을 유지했습니다.
입력·scan/clock50ms 허용, 실제 센서 광선·중복 제거·ROI·관측 시간도 유지했습니다. 통신 제약은 추가하지 않았습니다.
Hungarian 최적성은 **그때 고른 로봇/목표의 고정 정수 행렬 합**에만 해당합니다.

### 같은 조건의 단계별 모의 비교

아래 시간은 절대 sim에서 시작 시각1초를 뺀 **경과 sim초**입니다. 계산 중 sim이 멈추는 이상 기구학·합성 센서 모델이며 물리 주행이 아닙니다.
동일 프로필 내 월드·초기 위치·시작 시각·72광선/0.5초 센서·ROI·요청 기간을 유지했습니다.
②부터 후보 순위와 목표/비용이 달라지며④는 회전 포함 시간 모델도 다릅니다. 모두 같은 경로를 실행한 비교는 아닙니다.

| 넓은 영역 단계 | 지속 시간 | 관측률 | 탐사 완료 회차 | 종료 |
| --- | ---: | ---: | ---: | --- |
| 기존 simultaneous | 76.2초 | 3.5542% | 1 | robot3 금지 출발점으로 대피 배정 실패 |
| ① 이른 대피만 | 122.5초 | 3.4836% | 0 | 후속 계획 예산3회 초과, 부분 종료 |
| ② 관측·후속 시간/후보 순위 | 117.6초 | 3.8069% | 1 | 대피 공동 경로 실패, 회전 후처리 영향 확인 |
| ③ 복수 후보 비교 | 957.9초 | 10.5789% | 7 | 후속 계획 예산3회 초과, 부분 종료 |
| ④ 방향 A* 비교 | 641.5초 | 7.7698% | 5 | 재계획 시 정지 안전 구간 부족으로 정지 |

①은 sim36.5초에 robot3의 예정 관측/후속 hold 위험을 예측했습니다. 위험까지39.47초와 준비 추정39.73초를 비교해 대피했습니다. 기존 금지 출발 상태를 억지로 성공시키지 않았습니다. 기본③ 넓은 실행의 start_forbidden/null 행은0회(기존1회)였습니다.
기존과① 최초 목표·t0·행렬은 같고 Hungarian78/공동132tick입니다.②·③은 첫 목표가 달라져72/122tick,④는106/120tick입니다.
같은75초에서 기존3.5542%,③3.5802%입니다.③의180초 표본은179.2초의4.3512%,600초 표본은599.5초의8.7786%입니다. 종료 후 관측률을 외삽하지 않습니다.

짧은 프로필의 결과도 함께 해석해야 합니다.

- 고정4목표: 기존55.1초 → 기본③52.1초 완료, 양쪽 재계획0. 목표/배정57tick은 같고, 서비스 시간대 탐색의 동률 경로 선택이 달라 공동 회전26→23tick, 공동 비용83→80tick입니다.
- 단기180초: 기존30.640625%·완료2회차 → 기본③30.984375%·완료1회차·재계획4. **회차 개선은 확인하지 못했습니다.**
- 결과 공개를2 sim초 늦춘 지연 주입: 고정③/④ 모두52.1초 완료. 단기③은180초29.03125%·완료0회차로 부분 종료하여 계산 지연에 대한 취약성이 남습니다.
- 6 sim초 지연:5초 계산 기한 초과로 경과12.1초에 정지, 배정 실행0회. 이전 결과가 설치되지 않았습니다.

**③도 요청한 넓은1800초/85% 목표를 달성하지 못했습니다.** 도구의 passed=true는 최소 회차 등의 smoke 조건이며 전체 탐사 성공이 아닙니다. 비교 도구 exit0도 산출물 집계 완료만 뜻합니다.
동일 경과 시각의 관측률, 정지 구간, 최초 위험, 계산 sim/wall, 비용·실제 대기, 종료 사유는 comparison_final/comparison.json, comparison.csv와 각 *_details.json에 있습니다.
coverage_comparison.png/.svg는 sim/실제 시간 축을 구분합니다. 첫 유효 대피 후보만 채택한 중간 실행과 목표 시간대 캐시/순위 보완 전 실패도 별도 폴더에 보존했습니다.

### 검증 및 실행

ROS Humble에서 패키지2개 빌드, 회귀140개(새 위험 대응12개 포함), 기본③ 합성 ROS7경우를 검사했습니다.
추가로 baseline 연결1개와 기본③+staggered_pairs 연결1개를 통과했습니다. 기본③의 staggered_pairs 고정 목표 모의 주행도4대 모두 경과66.1초에 완료했습니다(기본 simultaneous는52.1초).
합성 ROS는 취소·정지, odom/scan 중단, clock 역행, cmd_vel 충돌, 취득 시각 TF 누락과4대 연결을 포함합니다.
UDP 접근 오류가 있어 같은 rclpy context의 메시지/서비스 계약 검사 결과이며 네트워크 DDS 검증은 아닙니다.
Docker 소켓 permission denied를 다시 확인했고 권한 우회 없이 실제 Webots bt-fixed → explore → explore-wide는 미수행입니다.
Mesa NVK/Zink·nouveau 및 Docker GPU 구성은 변경하지 않았습니다.

현재 YAML의 기본 위험 대응은③(candidates)입니다.④는 이번 실행에서 더 일찍 정지해 기본으로 채택하지 않았습니다. 기존 출발 정책 비교와 독립적으로 선택합니다.

~~~bash
cd /home/gayeon/lunar_hungarian_project
# 화면·Docker 권한이 있는 호스트 터미널에서, 이전 launch 종료 후 순서대로 실행
LUNAR_GPU=mesa bash docker/lunar/run_demo.sh bt-fixed
# 위 실행 종료 후
LUNAR_GPU=mesa bash docker/lunar/run_demo.sh explore
# 단기 결과 확인 후. 장시간 목표는 아직 미달성임
LUNAR_GPU=mesa bash docker/lunar/run_demo.sh explore-wide

# 동일 조건을 새 실행으로 다시 시작하여 기존/단계별 비교
LUNAR_GPU=mesa LUNAR_RISK_STAGE=baseline bash docker/lunar/run_demo.sh explore-wide
LUNAR_GPU=mesa LUNAR_RISK_STAGE=early bash docker/lunar/run_demo.sh explore-wide
LUNAR_GPU=mesa LUNAR_RISK_STAGE=goals bash docker/lunar/run_demo.sh explore-wide
LUNAR_GPU=mesa LUNAR_RISK_STAGE=candidates bash docker/lunar/run_demo.sh explore-wide
LUNAR_GPU=mesa LUNAR_RISK_STAGE=heading bash docker/lunar/run_demo.sh explore-wide
~~~

직접 launch는 risk_stage:=baseline|early|goals|candidates|heading입니다. 출발 옵션은 별도로 departure_policy:=simultaneous|staggered_pairs이며 기본 simultaneous를 유지합니다.
저장 결과 집계는 PYTHONPATH=ros_packages/moon_mapping_demo python3 ros_packages/moon_mapping_demo/tests/risk_comparison.py --root results/early_escape_20261122_191957 --output results/새로운_집계_폴더 입니다.
모의 재실행은 같은 PYTHONPATH로 python3 ros_packages/moon_mapping_demo/tests/bt_kinematic_smoke.py --mode explore --config ros_packages/moon_mapping_demo/config/exploration_wide.yaml --risk-stage candidates --output results/새로운_실행_폴더 입니다. --planning-delay-sim 2로 지연을 주입합니다.

### 남은 문제와 다음 순서

- 기본③은 이전 실패 상태를 피하고 지속 시간을 늘렸지만 후속 탐색 예산 초과로 종료했습니다. 호스트 부하에 따라 계산 시간이 달라지므로 한 실행을 통계적 성능 보장으로 표현하면 안 됩니다.
- 현재는 안전한 정지 구간을 먼저 확보합니다. 검증된 기존 이동 구간을 실행하며 계산하는 계획 교체나 사전 비상 경로는 검토했지만 **구현하지 않았습니다**. 안전 구간 부족 시 같은 자리에서 계산을 기다리지 않고 오류 종료합니다. 특히④의 실행 중 충돌/시간 경로 재검사 뒤에는 이미 정지 기한이 짧아지는 사례가 남습니다. 이 상태에서 계속 움직이도록 검사를 완화하지 않았습니다.
- 관측 후 별도 대기점으로 이동하는 복합 경로는 구현하지 않았습니다. 현재는 관측·유한 후속 대기가 가능한 목표/도착을 채택하고 불가능하면 다른 후보/대피를 시도합니다.12초 이후나 종료 뒤 무한 정지 안전을 보장하지 않습니다.
- 후보 각3개/조합8개와 우선순위 예약의 유한 탐색입니다. 후순위 시작점을 영구 예약하는 보수성으로 가능한 해를 놓칠 수 있습니다. 서비스 시간대/회전·실제 추종 편차와 다른 예약 순서의 효과를 추가 분석해야 합니다.
- 단기 회차 감소와2초 지연 시 관측 미완료를 개선해야 합니다.3.5m 탈출·최대180도 회전 준비 추정은 실제 후보 경로 길이/방향을 미리 보증한 값이 아닙니다.
- 다음은 실제 Webots 고정→단기→넓은 검증, 계산 중 /clock 진전·실제 회전/추종 편차 측정, 안전한 기존 경로 구간/사전 대피 경로의 교체, 동일 조건 반복 실험 순서입니다. 분모·반경·금지 검사를 완화하는 방식은 제외합니다.

기존 파일 누락 여부와 수정 목록은 preservation_audit.json에 기록합니다. 미커밋/미추적 파일·과거 실험 결과를 보존했으며 reset·clean·원격 업로드·push·PR을 수행하지 않았습니다.


## 전원 종료 전 재개 기록 (2026-11-23 09:33 KST)

코드·실험 결과를 보존하고 루트 NEXT_SESSION.md와 docs/resume_prompt_2026-11-23.txt를 추가했습니다.
별도 로컬 백업 위치는 /home/gayeon/lunar_hungarian_checkpoints/20261123_093335 입니다. 원래 작업 폴더에서 그대로 재개하며 커밋/원격 업로드는 하지 않았습니다.
이 저장 작업에서 구현 변경이나 실험 재실행은 하지 않았습니다. 검증140개·합성 ROS9개는 앞선 실행 기록이며 실제 Webots는 미검증입니다.
살아 있는 시뮬레이터의 메모리 체크포인트가 아니라 소스·Git 상태·결과·재개 지침의 보관입니다.

## 2026-11-23 후속 계획의 실제 계산 예산 초과

**확인한 사실.** 넓은 영역의 기존 simultaneous + candidates 실행에서 마지막 세 요청(38·39·40번)은 정지 안전 검사를 통과한 뒤 worker의 실제 계산 예산 8초를 소진했다. 마지막 40번은 절대 sim 957.7초에 제출됐고 공통 출발 시각은 967.7초였다. 제출부터 결과 수신까지 기존 기록은 실제 8.149초, sim 0.1초였다. 이 모의 실행은 계산 중 sim이 멈추고 다음 제어 tick에 결과를 받는다. sim 기한 5초 초과, 명시적 취소, 안전한 정지 구간 부족은 각각 다른 종료 사유다. 세 요청은 모두 일반 탐사였으며 초기 외부 예약과 대기 로봇은 없었다.

[결과 폴더](../results/followup_budget_20261123/)의 inputs에는 세 요청의 위치·방향·시각, 당시 후보·로봇 순서, 설정·월드·태양, 원본 소스 해시를 저장했다. 관측 격자는 로봇들이 같은 위치와 방향으로 정지한 구간의 최종 NPZ에서 복원했다. 최후 신규 셀 관측은 sim 943.5초였다. 각 요청의 마지막 scan까지 시각을 제한하고 방문 목표의 일시 제외 상태를 복원하면 후보 ID·좌표·순서·방향·이득이 세 기록 모두 일치한다. 호출된 worker는 당시 선택한 목표를 입력받는다. 목표 서비스 시간 22초는 관측·준비 10초와 후속 대기 12초의 합이다.

**병목과 변경.** 원래 40번은 4×4 행렬의 마지막 원소인 robot2 → frontier_89_8을 탐색하다가 중단됐다. cProfile 8초에서 선분과 사각형 교차 함수의 자체 시간은 5.28초, 포함 시간은 5.96초였다. 지형 질의 53,050회 중 입력이 같은 추가 호출은 2,076회였다. 실제 변경은 terrain.py의 그림자 충돌 판정 한 곳이다. 기존 코드는 시간 구간의 그림자 사각형 4,398개를 선분마다 검사했다. 변경 코드는 선분을 감싸는 1m 단위 공간 범위와 16초 단위로 바깥쪽으로 확장한 시간 범위에서 후보 기둥을 고른다. 몸체 반경으로 팽창한 상자가 겹치면 원래 시간 구간과 원래 선분으로 정밀 검사를 수행한다. 캐시에는 같은 지형·태양 모델의 후보 인덱스만 담는다. 시간·반경·양끝점이 속한 공간 범위가 키에 들어가며 정적 몸체·동료 예약 검사는 기존 경로를 따른다.

[단계별 계측](../results/followup_budget_20261123/report.json)은 후보 생성부터 결과 저장까지 호출 수, 포함 시간, 측정한 자식 단계를 뺀 자체 시간을 담는다. 별도 단계 계측에서 행렬 생성은 수정 전 8.000초에 중단됐고 수정 후 6.263초에 완료됐다. 수정 후 공동 조정은 0.103초였고 Hungarian 1회, 회전 후처리 4회, 예약 순서 1회였다. 수정 후에도 가장 많은 시간은 행렬의 A*에 든다. cProfile 원본과 함수별 자체 시간은 before_profile·after_profile에 있다. 두 종류의 계측은 실행량과 완료 시점에 영향을 주므로 성공 여부는 아래 비계측 실행으로 판정했다.

| 고정 요청 | 수정 전 실제 초 | 수정 후 실제 초 | 배정 / 공동 비용(틱) | 수정 전 전체 지형 검사로 재검증 |
| --- | --- | --- | --- | --- |
| 40 (각 3회) | 8.003 부근, 3회 초과 | 5.248 / 5.238 / 5.245 | 50 / 86 | 3회 통과, sim 1017.7까지 |
| 38 | 8.003 초과 | 5.780 | 50 / 86 | 통과, sim 1011.5까지 |
| 39 | 8.003 초과 | 7.650 | 61 / 115 | 통과, sim 1020.6까지 |

각 실행은 독립 프로세스에서 빈 지형·계획 캐시로 시작했다. 기존 행렬·경로 파일을 쓰는 시간도 작업 시간에 포함했다. 프로파일 덤프와 독립 검증은 타이머 밖이다. 검증은 원본 지형 모델로 시작 전 정지, 모든 이동·회전, 팀 종료까지 목표 대기, 연속 동료 원판 간격과 출발·도착 좌표를 확인했다. Hungarian 비용은 해당 고정 정수 행렬에서 선택한 배정 합이다. 관련 회귀 40개가 통과했다. 여기에는 원래 전체 사각형 판정과 새 판정을 비교한 1,600개 질의가 포함된다.

| 모의 탐사 | 경과 sim 초 | 관측률 | 완료 회차 | 재계획 | 실제 예산 초과로 계획 반환 실패 |
| --- | --- | --- | --- | --- | --- |
| 단기 기존 → 수정 | 180 → 180 | 30.9844 → 33.8125% | 1 → 1 | 4 → 4 | 1 → 0 |
| 넓은 기존 → 수정 | 957.9 → 581.1 | 10.5789 → 8.9402% | 7 → 4 | 8 → 6 | 4 → 0 |

단기는 시간 제한으로 종료했다. 넓은 실행은 경과 60초에 양쪽 모두 3.5505%, 300.3초에 5.3861 → 5.8561%, 539.5초에 8.5854 → 8.8251%였다. 수정 후에는 24번 대피 요청에서 후보 조합 8개 모두 Hungarian 배정은 가능했지만 예약 순서 192개가 회전 후처리 안전 검사에서 탈락했다. 종료 사유는 NO_SAFE_ASSIGNMENT였다. 제한된 탐색에서 해를 찾지 못한 결과다. 종료 후 관측률은 외삽하지 않았다. 정확히 같은 경과 시각의 모든 표본과 첫 배정 차이는 [탐사 비교](../results/followup_budget_20261123/comparison.json)에 있다.

**해석과 미검증.** 넓은 실행의 세 번째 배정에서 이미 대피 목표·경로가 달라졌다. 기존 5번 계획은 검증된 후보를 확보하고 공유 예산 소진으로 탐색을 멈췄고 수정 실행은 후보 한도까지 비교했다. 이후 서로 다른 위치에서 계획을 요청한 것이 전체 성능 차이에 기여했을 수 있다. 넓은 영역 지속 시간은 이번 한 쌍의 모의 실행에서 감소했다. 호스트 부하가 달라졌을 때의 성공률, 넓은 영역 1800초/85%, 실제 Webots에서 진행하는 /clock과 추종 오차는 미검증이다. Docker API 소켓 접근은 permission denied였다.

고정 입력은 저장 폴더의 도구로 재생한다. 아래 명령은 프로젝트 루트에서 새 출력 폴더를 만든다.

~~~bash
R=results/followup_budget_20261123
D=$(mktemp -d "$R/recheck_XXXXXX")
PYTHONPATH="$R/baseline" python3 "$R/replay.py" 40 "$D/before"
PYTHONPATH=ros_packages/moon_mapping_demo python3 "$R/replay.py" 40 "$D/after"
~~~

실제 Webots는 호스트에서 각 실행을 종료한 후 다음 명령을 실행한다.

~~~bash
cd /home/gayeon/lunar_hungarian_project
export LUNAR_GPU=mesa LUNAR_DEPARTURE_POLICY=simultaneous LUNAR_RISK_STAGE=candidates
bash docker/lunar/run_demo.sh bt-fixed
bash docker/lunar/run_demo.sh explore
bash docker/lunar/run_demo.sh explore-wide
~~~

각 실행이 출력한 results/gui_<모드>_<시각>_<PID>/의 graphics.json에서 렌더러를 확인한다. latest.json의 session 값으로 같은 폴더 아래 <session>/runtime.json을 열면 phase와 reason을 확인할 수 있다. <session>/metrics.json의 events 중 planning_timing에 sim·실제 경과와 상태가 있고, samples에 같은 경과 시각 비교용 관측률이 있다. <session>/plan_failure_*.json과 <session>/round_*/plan_*/ 아래의 attempt_*/problem.json 및 batch_plan.json에 실패·배정·시간 경로가 남는다. 복수 대피 후보의 시도별 파일은 plan_*/candidate_set_*/ 아래에 있다.

## 2026-11-23 요청 24번의 첫 이동 실패 진단과 제한 보정 실험

결과는 `results/escape24_20261123_163050/`에 저장했다. **요청 24번은 같은 입력과 기존 격자에서 해결되지 않았다.** 회전 후 잔여 경로를 한 번 재탐색하는 보정을 구현해 비교했으나 유효한 공동 계획을 반환하지 못했다. 실험 소스와 patch는 결과 폴더에 보존했다. 실행 소스 64개와 설정의 SHA-256은 이번 작업 시작 상태와 일치하며 이전 그림자 검사 최적화도 보존됐다.

### 고정 입력과 원본 재현

`request24.json`의 위치·방향은 기존 `metrics.json`에서 plan_id=24인 `planning_timing.snapshot_poses`를 읽었다. 제출 시각은 절대 sim 582.1초, 공통 t0는 590.6초다. 최종 runtime 자세를 사용하지 않았다. 로봇별 odom 취득 시각은 원본에 저장되지 않았다. 이상 기구학 검사에서 매 step의 now와 poses가 함께 공급된다는 근거로 해당 tick의 자세로 해석했으며 실제 센서의 odom stamp로 표현하지 않는다.

`inputs/`에 설정·월드·태양을 복사했고 `initial_hashes.json`에 소스 식별값을 저장했다. 전체 대피 후보 풀은 원본에 없으므로 기존 함수로 복원한 결과를 `candidate_pools.json`에 저장했다. 이 풀에서 선택한 8개 조합의 후보 ID·순위·좌표·방향과 비용 행렬 8개가 원본과 일치했다. 192개 최초 실패의 로봇·예약 순서도 일치했다. 네 로봇 전체 배정이며 최초 Hungarian 합은 조합 순서대로 22, 21, 22, 22, 24, 22, 21, 22틱이다. 실제 파일 값은 `original_failure.json`의 각 attempts.assignment.total_cost로 확인한다.

새 계획의 초기 외부 예약·대기 로봇은 없다. 순서별 선계획 동료 경로와 후순위 출발점 몸체는 상세 진단에 저장했다. 관측·준비 10초와 후속 대기 12초의 목표 서비스 조건, 실제 예산 8초, sim 기한 5초, horizon 180초, 반경 0.26m와 안전 시간 간격 0.25초를 사용했다. 대피 worker에는 관측 격자를 전달하지 않는다.

### 처음 실패한 구간

[집계 표](../results/escape24_20261123_163050/failure_table.md), [192개 상세 CSV](../results/escape24_20261123_163050/before_diag/first_failures.csv), [원래·회전 반영 경로와 예약 JSON](../results/escape24_20261123_163050/before_diag/first_failures.json)에 좌표와 시각을 저장했다.

| 후보 조합 | 해당 순서 수 | 첫 실패 로봇 | 첫 이동의 원래 시각 | 회전 반영 시각 | 실패 제약 |
| --- | --- | --- | --- | --- | --- |
| 0·1·2·3·5·6·7 | 168 | robot1 | 590.6 → 592.6 | 594.6 → 596.6 | 그림자 |
| 4 | 24 | robot1 | 590.6 → 592.6 | 595.6 → 597.6 | 그림자 |

168개는 (-20.426477530080394, -12.744170920895467)에서 (-20.5, -13.0)으로 이동하는 선분이다. 나머지 24개는 같은 출발점에서 (-20.5, -12.5)로 향한다. 192개 모두 회전 대기 자체는 통과했으며 그 뒤의 첫 이동이 금지됐다. 정적 지형·월드 경계, 후순위 몸체, 이미 계획한 동료의 시간 예약, horizon 초과는 이 최초 실패의 원인이 아니었다. 관측·최종 대기 검사는 이 단계에서 도달하지 못했다.

### 잔여 경로 보정 실험과 추가 근거

보정은 실패 전이 직전까지 검증된 prefix를 남기고, 그 위치·방향·시각과 같은 예약에서 기존 방향 Space-Time A*로 잔여 경로를 한 번 탐색했다. 회전 속도 0.5rad/s와 정착 0.5초를 사용하고, 잔여 horizon은 원래 종료 시각에서 이미 사용한 시간을 뺀 값으로 제한했다. 실제 예산·취소 callback도 기존 것을 전달했다. 24번은 첫 회전 뒤 첫 이동이 실패하므로 보정 시작점은 실제 제출 자세와 t0였다.

같은 8초 예산의 비계측 실행은 원래 코드 4.318 / 4.399 / 4.386초, 보정 실험 5.006 / 5.287 / 5.306초였다. 여섯 번 모두 후보 8개를 처리하고 `NO_SAFE_ASSIGNMENT`로 종료했다. 공동 계획과 공동 비용은 반환되지 않았다. 상세 진단은 별도 실행에서 4.421초가 들었다. 원본 metrics의 wall/sim_elapsed=0은 동기 ImmediatePool 호출 중 engine.wall과 now가 갱신되지 않은 기록이다. 이 실험에서는 worker를 직접 monotonic으로 측정했다. 계산 중 sim은 정지한 상태이며, worker의 sim 경과는 0이다.

추가로 이전 전체 사각형 지형 oracle을 사용해 동료 몸체와 예약을 모두 제외한 더 허용적인 조건을 검사했다. robot1의 출발점에서 첫 격자점으로 가는 네 연결과 대기 0~5틱의 조합 24개가 전부 금지였다. 595.6→596.6초 대기도 금지였고 정지 안전 하한은 약 596.4997초였다. robot1의 후보 세 곳에 대한 방향 A*도 각각 시작점의 대기 상태 6개만 확장하고 `no_path_within_horizon`을 반환했다. [최초 전이 전수 검사](../results/escape24_20261123_163050/start_transition_certificate.json)에 근거를 저장했다.

**확인 범위는 현재 0.5m 격자의 연결 규칙과 금지 oracle이다.** 연속 공간의 모든 물리 경로에 대한 불가능 증명은 아니다. 회전 후 재탐색을 생략하는 기존 구현이 다른 입력에서 가능한 경로를 놓칠 가능성은 있으나, 이 고정 입력에서는 재탐색만으로 첫 이동을 만들 수 없었다. 더 이른 시각·다른 실제 위치에서 계획을 요청하거나 다른 안전한 최초 연결이 존재하는지를 살피는 작업에는 추가 근거가 필요하다. 이번 비교에서는 이 조건들을 바꾸지 않았다.

고정 요청에서 효과가 확인되지 않아 검사 확대 조건에 도달하지 않았다. 요청 38·39·40 재실행 및 단기·넓은 탐사 결과를 새 성공 기록으로 추가하지 않았다. 현재 실행 소스는 이번 작업 시작 상태와 SHA-256으로 대조했다. 보정은 `prototype/`와 `bounded_repair_experiment.patch`에 실험용으로 보관했다. 동작 채택 여부는 false다.

### 재생과 호스트 실행

프로젝트 루트에서 아래 명령으로 원본과 보정 실험을 재생할 수 있다. 출력은 매번 새 디렉터리다.

~~~bash
R=results/escape24_20261123_163050
D=$(mktemp -d "$R/recheck_XXXXXX")
PYTHONPATH="$R/baseline" python3 "$R/replay24.py" "$D/original"
PYTHONPATH="$R/prototype" python3 "$R/replay24.py" "$D/prototype"
# 상세 계측. 위 성능 비교와 별도 출력 디렉터리 사용
PYTHONPATH="$R/baseline" python3 "$R/replay24.py" "$D/diagnosis" --diagnose
~~~

Docker API 소켓은 이번에도 permission denied였다. 실제 Webots는 미수행이다. 호스트에서는 기존 nouveau Mesa NVK/Zink 구성으로 각 실행을 종료한 뒤 다음 명령을 순서대로 실행한다.

~~~bash
cd /home/gayeon/lunar_hungarian_project
export LUNAR_GPU=mesa LUNAR_DEPARTURE_POLICY=simultaneous LUNAR_RISK_STAGE=candidates
bash docker/lunar/run_demo.sh bt-fixed
bash docker/lunar/run_demo.sh explore
bash docker/lunar/run_demo.sh explore-wide
~~~

각 실행이 출력한 `results/gui_<모드>_<시각>_<PID>/graphics.json`에서 렌더러를 확인한다. 같은 폴더의 `latest.json`에 기록된 session 디렉터리에서 `runtime.json`의 phase/reason, `metrics.json`의 planning_timing 및 samples, `plan_failure_*.json`을 확인한다. 비용 행렬과 각 경로 시도는 `round_*/plan_*/candidate_set_*/attempt_*/problem.json`과 `batch_plan.json`에 남는다. 고정 목표·일반 탐사 계획은 candidate_set 디렉터리가 없는 경로도 사용한다.


## 세션 마무리 저장 (2026-11-23T16:40:00+09:00)

NEXT_SESSION.md와 docs/resume_prompt_2026-11-23.txt를 현재 상태로 갱신했습니다. 요청 24번은 미해결이며 보정 실험은 결과 폴더에 보관되어 있습니다. 이번 저장에서는 구현 변경이나 검증 재실행을 수행하지 않았습니다. 최신 로컬 체크포인트는 /home/gayeon/lunar_hungarian_checkpoints/20261123_164000_escape24입니다.


## 2026-11-24 단기 후속 계획과 통신 제약 없는 공간 구역 분담

사용자는 호스트에서 고정 목표 완료와 단기 1회 이동 후 오류 정지를 확인했고, 더 적극적인 탐사와 네 로봇의 구역 분산을 요청했습니다.
호스트 단기 기록 `results/gui_explore_20261123_234613_7101/b59304f1ef774b89bee452f0e0df5c66/`의
두 번째 계획은 실제 8.000651초, sim 3.2초 후 `planning_wall_budget_exceeded`였습니다.
RViz 경고와 별도로 이 계산 종료 사유를 재현했습니다.

### 채택한 변경

일반 탐사는 45초 경로 안에서 다음 목표를 선택하고 단기/넓은 전진 후보를 3/4m로 늘렸습니다.
동일 대기 로봇 조건의 독립 로봇–목표 비용과, 동일 필드·설정·출발·미계획 원판·예약 시간 경로의 공동 계획 앞부분을 재사용합니다.
그림자와 로봇 충돌 검사를 생략하지 않습니다. 공동 경로는 회전을 포함해 찾고 목표 관측/후속 대기는 0.05m 예측 여유를 검사합니다.
worker는 실제 8초 안에 결과 전달 여유 0.4초를 남기며 sim 5초 제한도 적용됩니다.
대피는 짧은 45초 경로를 먼저 찾고 실패하면 남은 동일 예산으로 기존 horizon을 사용합니다.

넓은 영역은 `work_regions.py`의 공간 frontier 군집과 직전 담당의 연속성 비용으로 진행 목표를 만듭니다.
실제 그림자 시간 경로 비용 Hungarian 배정에서는 한 작업 구역에 한 로봇만 허용하고 필요하면 담당을 바꿉니다.
현재 팀 몸체의 라이다 반사는 후보용 지도 복사본에서 분리하며 실제 관측 셀/면적은 보존합니다.
목표 방향을 고려하는 BFS 동률 처리로 모든 후보가 같은 축을 따라 전진하는 현상을 줄였습니다.
그림자 때문에 한 로봇의 정지를 유지할 수 없으면 그 로봇의 `support` 보호 이동과 동료의 탐사 목표를 함께 배정합니다.
보호 이동만 수행한 회차는 탐사 완료 회차에 포함하지 않습니다.
RViz `/exploration/bt_markers`와 runtime/후보/배정 이벤트의 `work_regions`에서 구역을 확인합니다.

### 최종 검증과 남은 한계

`results/spatial_regions_20261124_001/final/report.json`과 `checks.json`, `source_sha256.json`을 기준으로 합니다.
회귀 127개, 고정 목표 완료, 단기 180초 3회차/49.84375%/재계획0을 확인했습니다.
실제 단기 실패의 같은 위치·목표·t0·hold 입력도 새 프로세스/빈 캐시 3회 모두 실제 8초 내에 반환했고 원래 전체 사각형 oracle을 통과했습니다.
반환 계획 38개(넓은33/단기4/고정1)를 동일 원래 oracle로 독립 검사했습니다.
이 수치는 동시 이동/회전/도착 후 대기와 동료 원판 간격 검사를 포함하며 실제 Webots 물리 검증을 대신하지 않습니다.

같은 모의 120초의 로봇 쌍 평균 거리는 구역 분할 전3.1879m→후5.0337m,
최소 쌍 거리는1.0000m→2.6040m였습니다. 새 버전의300초 평균은10.2737m였습니다.
단일 전후 비교이며 호스트 부하와 유한 계산 예산의 후보 선택 영향을 포함합니다.
구역 추가 직전 비교 실행은127.4초 경과에 대피 배정 실패로 끝났고 이후 값을 외삽하지 않았습니다.
새600초 시도는507.7초 경과(절대sim508.7)에 `no_safe_plan_within_search_limits`로 부분 종료했습니다.
그때 완료7회차, 재계획3회, 관측10.894768%였습니다. **600/1800초 지속 및85% 목표는 달성하지 못했습니다.**
이전 고정 대피 요청24의 성공은 재검증하지 않았으며 해결됐다고 기록하지 않습니다.

구역 소유권을 강제한 중간 실험과 전원 대피를 반복한 중간 정책은 성능 저하가 있어 기본에 채택하지 않았습니다.
`wide_regions_v1`~`v7` 등 중간 결과를 최종 성공으로 혼동하지 않습니다. 최종 실행은 `final/`입니다.
새 코드의 실제 Webots/ROS 전달은 사용자 호스트에서 다시 확인해야 합니다. 기존 실행을Ctrl+C로 종료한 뒤 실행합니다.

```bash
cd /home/gayeon/lunar_hungarian_project
LUNAR_GPU=mesa LUNAR_DEPARTURE_POLICY=simultaneous LUNAR_RISK_STAGE=candidates bash docker/lunar/run_demo.sh explore-wide
```

소스/설정 변경 전 자료는 `results/progressive_exploration_20261124_001/before/`와
`results/spatial_regions_20261124_001/before/`에 보존했습니다. 기존 미커밋 변경을 보존했고 reset/clean/push/PR/원격 업로드는 하지 않았습니다.


## 2026-11-24T00:50:10+09:00 로컬 저장

사용자 요청으로 현재 소스·Git 상태·전체 실험 결과를 `/home/gayeon/lunar_hungarian_checkpoints/20261124_005010_spatial_regions`에 저장합니다. 검증 근거는 해당 폴더의 verification.json과 SHA256SUMS입니다. 이번 저장에서는 구현 변경이나 검증 재실행 없이 기존 결과를 보존했습니다. NEXT_SESSION.md와 docs/resume_prompt_2026-11-24.txt에 재개 방법을 정리했습니다.

## 2026-11-24 실제 첫 이동 이후 계산 실패 수정

실제 `gui_explore-wide_20261124_123356_5661`은 1회차 뒤 탐사 `ATTEMPT_LIMIT`와
대피 `PLANNING_BUDGET`으로 종료했습니다. 회전 없는 독립 배정과 회전 있는 공동 경로의 불일치,
대피의 사후 회전 추가 실패, 다음 준비 예측보다 짧은 12초 후속 안전 구간을 확인했습니다.

독립 비용과 공동 경로의 방향·회전·목표 관측 반경을 맞추고 반복 탐사의 후속 안전 구간을
준비·회전·탈출 예측 이상으로 확보했습니다. 미배정 로봇의 대기가 막히면 이른 보호 이동으로 전환합니다.
불가능한 행렬도 공동 경로 24회 제한을 소모하던 단기 모드의 계수도 수정했습니다.
실제 8초/sim 5초, 결과 전달 여유 0.4초, 몸체 0.26m, 시간 검사 0.25초는 유지합니다.

최종 근거는 `results/followup_paths_20261124_001/report.json`, `checks_final.json`, `source_sha256.json`입니다.
회귀 131개와 원래 전체 사각형 oracle의 계획 25개 안전 검사를 통과했습니다.
실제 첫 좌표·방향·후보·t0에서 시작한 모의는 기존 코드가 62.2초 경과에 1회차 뒤 종료했고,
수정본은 결과 전달 지연 3 sim 초 조건에서 300초·탐사 4회차·보호 이동 2회까지 진행했습니다.
단기 모의 180초·3회차·47.734375%, 고정 목표 모의 완료도 확인했습니다.

실제 후속 스캔·Webots 물리·ROS 전달은 검증하지 않았습니다. Docker API 소켓 권한이 없습니다.
이미 위험이 임박한 실제 요청4/5의 복구나 대피 요청24 성공, 600/1800초·85% 목표 달성을 뜻하지 않습니다.
기존 실행을 Ctrl+C로 종료한 뒤 NEXT_SESSION.md의 같은 `explore-wide` 명령으로 재실행해야 새 코드가 적용됩니다.
