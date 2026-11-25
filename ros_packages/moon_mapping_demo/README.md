# ROS 2 LIMO 달 탐사: 그림자 회피와 Hungarian 배정

현재 `worlds/moon_terrain.wbt`의 **60×60m 지형과 LIMO 4대**를 사용하는 사전 지형 기반 실험입니다. 기본 실험 로봇은 robot1~4 모두이며 목표 부족·안전상 일부 대기를 허용합니다. 시작 위치는 `config/environment.yaml`과 실제 WBT를 대조합니다. 과거 월드 좌표를 사용하지 않습니다.

`moon_mapping.launch.py`의 기본 동작은 기존과 같이 **robot1만 SLAM/Nav2**, 나머지는 대기입니다. 새 `moon_shadow.launch.py`는 별도 시간 경로 실행기를 사용합니다. 이 모드는 선택된 로봇의 드라이버를 활성화하고 모든 로봇에 `map → robotN/odom` 시작 위치 앵커를 제공합니다. SLAM·Nav2·기존 공장 BT는 함께 띄우지 않습니다.

## 4대 BT 반복 탐사 실행

기존 launch를 종료하고 호스트 데스크톱 터미널에서 실행합니다.

```bash
bash docker/lunar/run_demo.sh bt-fixed
# 고정 4대 목표의 실제 연결을 확인하고 Ctrl+C로 종료한 뒤
bash docker/lunar/run_demo.sh explore
# 단기 센서·지표 검증 후에만
bash docker/lunar/run_demo.sh explore-wide
```

새 기본 `bt-fixed`와 `explore`는 전용 통합 launch를 사용합니다. 옛 BT/Nav2를 중복 실행하지 않습니다.
짧은 평가는 400m², 넓은 평가는 3364m²의 고정 분모입니다. `/scan_observation`의 취득 시각 TF·범위·가림을 반영한
광선만 실제 면적에 포함합니다. 도착/사전 지도는 면적으로 누적하지 않습니다.
RViz의 관측 격자와 BT 상태, session 폴더의 JSON/CSV/PNG/SVG를 함께 확인합니다.
[인계 문서](../../docs/bt_exploration_handoff_2026-11-22.md)와
[검증 JSON](../../docs/shadow_validation_2026-11-22.json)의 최신 항목이 아래 과거 검증 기록보다 우선합니다.

### 2026-11-24 원본 삼각형 그림자와 직선 경로 — 현재 기본

기존 금지 영역은 0.5m 기둥의 높이로 그림자를 만든 뒤 전체 이동/대기 구간의 큰 사각형으로
감쌌습니다. 실제 삼각형 그림자 바깥과 시간대가 다른 위치까지 막는 문제가 있어 경로 생성과 실행
검사 모두 `terrain_geometry: mesh`로 교체했습니다. 이전 모델은 `legacy_columns`로 비교할 수 있습니다.

- 지형 장애물은 원본 비평탄 삼각형의 XY 형상, 그림자는 원본 3D 삼각형의 투영 형상을 씁니다.
  수신 높이 `shadow_plane_height: 0.02`는 Webots 지형 그림자 모니터의 최소 발자국 높이와 같습니다.
  수신 평면 아래 부분을 잘라내며 경사 주행을 허용하는 변경은 아닙니다.
- 사각형과 공간 검색 트리는 후보를 빠르게 제외하는 데만 씁니다. 최종 충돌은 실제 삼각형과
  원형 몸체 사이의 유클리드 거리로 판정하여 사각형 모서리의 과도한 여유를 없앱니다.
- 이동 중에는 같은 시간의 몸체와 그림자를 검사합니다. 중간 시각에서 각 꼭짓점의 회전 이동량
  상한을 더하고 모호한 구간을 최대0.1초까지 나눕니다. 긴 회차 전체의 그림자를 한 사각형으로 합치지 않습니다.
- `direct_goal_links: true`: 각 탐색 위치에서 실제 목표 좌표까지 직선 연결도 시도합니다.
  회전·이동 시간·그림자·지형·예약된 동료 경로를 모두 검사합니다. 통과한 직선을 격자 축에 맞춰 꺾지 않습니다.
  대체 경로의 안내 격자는0.5m, 시간 tick은0.5초입니다. 0.25m 안내 격자는8초 예산에서 계산량이 커 기본에서 제외했습니다.
- 계획 몸체 반경은0.26→0.22m, 목표 예측 여유는0.05→0.02m입니다. 현재 PROTO의 바퀴 충돌 형상은
  중심 이동과 원통 외접구를 합해 약0.184m 안에 들며0.22m 원판은 그보다 약3.6cm 큽니다.
  실제 로봇의 추가 장착물이나 다른 PROTO에 그대로 적용할 수 있다는 검증은 아닙니다.
- RViz `/exploration/bt_markers`의 `terrain_shadow_mesh`는 검사에 쓰는 같은 원본 삼각형의 현재 시각 그림자를 표시합니다.
  표시 자체에는 몸체 반경과 미래 이동량 여유를 더하지 않습니다.

[동일 시각 비교 그림](../../results/mesh_shadow_20261124_001/shadow_geometry_comparison.png)은 시작 구역
(-29,-29)~(-9,-9)의 평지 그림자를0.1m 격자 중심에서 표본 측정합니다. sim30초는32.53→6.19m²,
sim90초는104.59→41.60m²입니다. 전 월드/모든 태양 시각의 면적 감소율로 일반화하지 않습니다.

최신 근거는 `results/mesh_shadow_20261124_001/report.json`, `checks.json`, `source_sha256.json`입니다.
회귀146개, 단기 탐사4회차(절대sim148.9초, 추가 재계획0), 고정 목표 완료(절대sim46.1초)를 확인했습니다.
넓은600초 시도는 절대sim299.4초/경과298.4초·탐사6회차·456.5625m²에서 대피 계산 예산 초과로 종료했습니다.
600/1800초 지속과85% 관측 목표는 달성하지 못했습니다. 실제8초·sim5초 계산 예산을 늘리지는 않았습니다.

원본 3D 삼각형의 별도 Möller–Trumbore 광선 검사로 준비·이동·대기 경로를 대조합니다.
이 검사는0.2초·원판 중심/둘레33점 표본이며 연속 그림자 안전 증명이나 실제 Webots 주행으로 표현하지 않습니다.
연속 구간 판정의 이동량 상한과 형상/시간 검색은 별도 단위 검사로 확인합니다. 이전 전체 사각형 oracle은
이제 과대 차단 비교 자료이며 새 경로의 안전 정답으로 쓰지 않습니다. 실제 Webots/ROS 재검증은 Docker API 접근 거부로 미수행입니다.

### 2026-11-24 정지 여유를 줄인 적극 탐사 설정 (이전 기록)

사용자의 요청으로 `explore`와 `explore-wide`의 안전 예측 여유와 불필요한 회차 대기를 줄였습니다.
아래 이전 수정 기록의 약 49.73초 안전 구간은 변경 전 값입니다.

| 넓은 탐사 항목 | 변경 전 | 현재 |
| --- | --- | --- |
| 일반 출발 준비 | 10초 | 7초 |
| 도착 후 실제 관측 | 7초 | 4초·유효 스캔 최소 3개 |
| 관측·후속 정지 안전 검사 구간 | 약 49.73초 | 약 37.17초 |
| 대피 목표의 관측·다음 계획 준비 구간 | 약 49.73초 | 약 23.28초 |
| 탈출 거리 예상 / 추가 시간 여유 | 3.5m / 5초 | 2.5m / 3초 |

- `finish_round_after_observation: true`: 배정된 로봇 모두가 실제 도착·관측 시간·유효 스캔 수를
  충족하면 추가 정지 시간을 채우지 않고 다음 회차로 넘어갑니다. 고정 목표 모드의 종료 조건은 유지합니다.
- `allow_partial_escape: true`: 최소 서비스 구간 동안 정지할 수 없는 로봇만 먼저 대피시킵니다.
  남은 로봇은 실제 계획 종료 시점까지 대기·몸체·동료 충돌 검사를 통과해야 합니다.
  부분 대피가 실패하면 같은 전체 계산 예산의 남은 시간으로 전체 대피를 시도합니다.
- `arrival_window_escape: true`: 대피점이 출발 시점부터 계속 안전해야 한다는 후보 제한을 줄입니다.
  도착 예상 뒤 안전 시간대가 있는 후보도 허용하고 실제 도착·회전·이동·대기는 공동 계획에서 다시 검사합니다.
- `short_escape_hold: true`: 대피 후 다음 계산·준비·회전 여유를 확보하되, 그 자리에서 또 한 번의
  탈출 이동 예상까지 정지로 보장하지는 않습니다. 국소 대피 후 더 일찍 새 계획이 필요합니다.

몸체 반경 0.26m, 목표 예측 위치 여유 0.05m, 시간 검사 0.25초, 그림자 진입 금지와 동료 충돌 검사는
그대로입니다. 계산은 실제 8초·sim 5초이며 통신 거리 제약은 없습니다. 이번에는 계산 중 팀 정지와
공동 회차 구조를 유지했습니다. 이동하면서 다음 계획을 계산하는 구조는 아직 구현하지 않았습니다.

최신 근거는 `results/aggressive_exploration_20261124_001/report.json`입니다.
회귀 137개 통과, 3 sim초 결과 전달 지연을 넣은 단기 180초·3회차와 고정 목표 완료를 확인했습니다.
동일 240초 모의 비교에서 관측 면적은 260.1875→284.375m², 완료 탐사는 3→4회차였습니다.
다만 적극 설정의 넓은 600초 시도는 절대 sim291.3초(경과290.3초), 탐사 5회차·351.375m²에서
대피 계산 예산 초과로 종료했습니다. 변경 전 비교는 300초까지 진행했으므로 **장시간 지속성 개선은
확인하지 못했습니다.** 더 빠른 탐사와 줄어든 대피 여유의 교환 관계를 포함한 설정입니다.
실제 Webots/ROS 주행은 Docker API 접근 거부로 재검증하지 못했습니다. 초기 실패 실험은
`after_wide_300`, `after_v2`~`after_v5`에 보존하며 최종 실행은 `after_v7_wide_600`입니다.

### 2026-11-24 실제 첫 이동 이후 계획 실패 수정 (이전 기록)

호스트 실행 `results/gui_explore-wide_20261124_123356_5661/`은 첫 회차를 완료한 뒤
후속 탐사가 `ATTEMPT_LIMIT`, 대피가 `PLANNING_BUDGET`으로 끝났습니다.
후속 탐사에서는 회전을 제외한 독립 비용으로 선택한 배정이 공동 경로에서 반복 탈락했고,
대피에서는 이동 경로에 회전을 나중에 더한 결과가 반복해서 금지 구간에 걸렸습니다.
목표의 후속 대기 12초도 다음 계산·준비·회전·탈출 예측 약 39.73초보다 짧았습니다.

- `turn_aware_coordination: true`이면 탐사·대피·고정 목표의 독립 비용과 공동 경로에 같은
  방향·회전 모델을 적용합니다. 독립 비용에서도 목표 관측 반경의 위치 여유를 검사합니다.
- 반복 탐사의 후속 안전 구간은 `max(followup_wait_seconds, 계산·준비·회전·탈출 예측)`입니다.
  넓은 기본에서 관측 10초를 합한 안전 검사 구간은 약 49.73초입니다.
  실제 관측 완료 후 이 시간 전체를 기다리는 것은 아닙니다.
- 안전한 탐사 경로가 있어도 미배정 로봇을 회차 끝까지 세울 수 없으면 보호 이동을 먼저 계획합니다.
  보호 이동만 수행한 회차는 탐사 완료 횟수에 포함하지 않습니다.
- 배정 불가능한 비용 행렬은 공동 경로 탐색 횟수를 소모하지 않습니다.
  전체 실제 8초·sim 5초 예산과 결과 전달 여유 0.4초는 그대로 적용합니다.

재생 입력·변경 전 소스·최종 검사 근거는 `results/followup_paths_20261124_001/`에 있습니다.
`report.json`과 `checks_final.json`을 기준으로 보며 중간 실험을 최종 결과와 구분합니다.
실제 시작 좌표·방향·첫 후보를 사용하는 모의 재생도 센서·물리·ROS 전달은 합성입니다.
새 코드의 실제 Webots 주행은 기존 실행을 Ctrl+C로 종료하고 다시 실행해 확인해야 합니다.

### 2026-11-24 후속 계획과 탐사 적극성 수정

2026-11-23 호스트 실행에서 고정 목표는 `fixed_goals_observed`로 완료했습니다.
단기 탐사는 첫 회차를 완료한 뒤 두 번째 계획의 실제 8초 예산 초과로 정지했습니다.
이 실패 입력을 보존하고 다음 변경을 적용했습니다. 변경 후 실제 Webots 주행은 다시 확인해야 합니다.

- 전진 후보 거리는 단기 1.5→3m, 넓은 영역 3→4m입니다. 관측 자유 공간의 경로를 따라 후보를 만들고,
  예상 신규 면적/이동 시간과 이미 선정한 후보의 관측 원판 중복을 고려해 최대 6개를 고릅니다.
  이 예상 면적은 후보 순위에만 쓰고 관측률은 실제 센서 광선으로 계산합니다.
- 일반 탐사는 회전·이동을 포함한 경로를 45초 범위에서 찾습니다. 오래 기다려야 하는 목표는 다른 후보로 교체합니다.
  고정 목표는 기존 범위를 쓰고, 대피는 45초 경로를 먼저 찾은 뒤 필요하면 남은 동일 계산 예산으로 기존 180초 범위도 탐색합니다.
- 독립 비용 행렬의 로봇·목표 쌍은 같은 대기 로봇 조건 안에서 재사용합니다. 공동 계획에서는 미계획 로봇과
  이미 예약한 경로가 같은 순서 앞부분의 결과를 재사용합니다. 취소된 탐색은 재사용하지 않습니다.
- 공동 경로는 회전 시간을 포함해 탐색합니다. 위 실제 후속 실패 수정 이후에는
  `turn_aware_coordination: true`인 독립 Hungarian 행렬에도 회전 시간을 포함합니다.
  목표의 관측·후속 대기는 예측 위치 여유 0.05m도 검사합니다.
- 이동 중에는 남은 경로를 검사하고 계산 중에는 출발까지의 정지를 검사합니다. 다음 계획으로 교체할 경로를
  목표에서 오래 정지하는 것으로 연장해 불필요하게 취소하지 않습니다. 그림자가 접근하면 검증 가능한 탐사 경로도 먼저 시도합니다.
- 실제 8초/시뮬레이션 5초 제한 안에서 worker는 결과 전달용 0.4초를 남깁니다.
  몸체 반경 0.26m, 시간 안전 검사 0.25초, 동료 간격, 입력 신선도, 도착 후 관측 시간과 유효 스캔 3개 조건을 적용합니다.

기존 실행을 Ctrl+C로 종료한 후 호스트 데스크톱 터미널에서 실행합니다.

```bash
cd /home/gayeon/lunar_hungarian_project
LUNAR_GPU=mesa LUNAR_DEPARTURE_POLICY=simultaneous LUNAR_RISK_STAGE=candidates bash docker/lunar/run_demo.sh explore
```

변경 전후 모의 비교, 실제 실패 입력 재생, 원래 전체 그림자 사각형 oracle의 독립 검사는
`results/progressive_exploration_20261124_001/`에 보존합니다. 모의 검사는 실제 Webots 물리·ROS 전달을 검증하지 않습니다.

### 통신 거리 제약 없는 공간 구역 분담

넓은 영역의 기본 `spatial_regions: true`는 팀 전체 관측 지도의 frontier를 최대 4개의 공간 군집으로 나눕니다.
멀리 떨어진 대표점을 고른 뒤 군집의 관측 자유 셀을 대표점으로 사용하고, 이동 거리와 직전 담당 구역의 연속성으로
로봇별 진행 목표를 만듭니다. 같은 BFS 거리의 경로에서는 목표 방향을 고려하여 이웃 나열 순서 때문에 모두 같은 축으로 전진하는 현상을 줄입니다.

최종 후보 조합은 **한 작업 구역에 한 로봇**만 허용합니다. 실제 로봇–목표 배정은 그림자 시간 경로의 정수 비용 Hungarian으로
다시 정하므로 접근할 수 없는 이전 담당에게 구역을 강제로 고정하지 않습니다. 구역 분할·대표 목표 선정은 휴리스틱이며,
Hungarian 최적성은 선택한 구역/목표/활동 로봇의 고정 비용 행렬에만 해당합니다.

현재 팀 몸체가 다른 로봇의 스캔에 찍혀 시작점을 막는 반사는 후보 경로용 지도 복사본에서 분리합니다.
실제 관측 셀·최초 관측 시각·면적에는 손대지 않으며, 동료 몸체는 공동 예약과 연속 원판 충돌 검사에 포함합니다.
일부 로봇의 현재 위치가 다음 회차 끝까지 안전하지 않으면 그 로봇에 `support` 보호 이동 후보도 제공합니다.
다른 로봇은 각자의 frontier 탐사를 계속할 수 있고, 보호 이동만 한 회차는 완료 탐사 회차로 세지 않습니다.
보호 지점은 기존 사전 지형 안전 모델에서 생성하며 실제 관측 면적은 도착 후 센서 광선만으로 갱신합니다.

`work_regions`는 `runtime.json`, 상태 토픽, 후보/배정 이벤트에 남습니다.
RViz의 `/exploration/bt_markers`에는 로봇별 작업 구역과 방향 화살표, 보호 이동 상태를 표시합니다.
구역 후보가 부족하거나 물리 통로가 하나이면 가능한 작업 수만 배정합니다. 통신 거리 제한은 두지 않습니다.

최종 순차 검사(`results/spatial_regions_20261124_001/final/`)는 회귀 **127개**, 고정 목표 완료,
단기 180초 **3회차·49.84375%**, 실제 단기 실패 입력 3회 재생을 통과했습니다.
같은 모의 120초에서 구역 분할 전/후 평균 로봇 쌍 거리는 **3.188→5.034m**였고 새 버전의 300초 값은 **10.274m**였습니다.
반환 계획 38개를 원래 전체 그림자 사각형 oracle로 재검사했습니다.
넓은 영역의 600초 시도는 **507.7초 경과(절대 sim 508.7), 7회차, 10.89477%**에서
`no_safe_plan_within_search_limits`로 부분 종료했습니다. 600초/1800초 지속과 85% 목표는 달성하지 않았습니다.
이 비교는 한 쌍의 모의 실행이며 계산 예산에 따른 후보 선택과 호스트 부하의 영향을 포함합니다.

이전 실행을 Ctrl+C로 종료한 뒤 새 코드를 다시 빌드·실행합니다.

```bash
cd /home/gayeon/lunar_hungarian_project
LUNAR_GPU=mesa LUNAR_DEPARTURE_POLICY=simultaneous LUNAR_RISK_STAGE=candidates bash docker/lunar/run_demo.sh explore-wide
```

## 구현 범위와 모델

- Space-Time A*: `(x, y, 시간 tick)` 상태에서 8방향 이동과 대기를 탐색합니다. 이동 시간은 `ceil(거리 / 속도 / tick 길이)` tick이며 정확한 출발·목표 좌표 연결도 비용과 충돌 검사를 거칩니다. 직선거리는 허용적 휴리스틱으로만 사용합니다.
- 지형: 실제 WBT의 지면·언덕·크레이터 삼각형, 배치 변환, 경계벽을 읽습니다. 현재 월드는 74,808개 삼각형입니다. 높이가 기준 평면에서 1mm보다 벗어나는 지형을 격자 기둥으로 감싸서 금지하므로 **평지 통로만 주행**합니다. 경사 주행은 지원하지 않습니다. 새 통합 모드는 이 사전 안전 모델과 실제 센서 관측 격자를 분리합니다.
- 그림자: 태양 설정을 읽어 기둥의 그림자를 보수적인 사각형으로 감쌉니다. 이동·대기 구간의 태양각 전체에 대해 sin/cos의 극값을 사용하고 구간을 바깥으로 양자화합니다. 시간 표본 사이의 그림자를 통과시키지 않습니다. 몸체 반경으로 팽창한 금지 영역과 이동 선분 전체를 검사합니다. 실제 그림자보다 넓게 차단될 수 있습니다.
- 시간: 태양 컨트롤러와 계획기는 Webots `getTime()`의 **절대 시뮬레이션 초**를 사용합니다. ROS 실행기는 `use_sim_time=true`로 같은 `/clock`을 읽습니다. 태양의 초기 방위각·고도·주기는 `controllers/lunar_sun/sun_config.json`에서 읽습니다.
- 배정: 행/열 잠재치와 증대 경로를 사용하는 O(n³) **Hungarian 직접 구현**입니다. SciPy를 호출하지 않습니다. `None`인 금지 간선을 선택하지 않으며 완전 배정이 없으면 실패합니다.
- 실행: 시간표를 유지하는 차동 구동 추종기가 `cmd_vel`을 발행합니다. `NavigateToPose`만 보내거나 Nav2가 경로를 다시 계산하도록 맡기지 않습니다. 기존 드라이버의 `/multi_bt/emergency_stop` heartbeat와 0.5초 명령 watchdog을 재사용합니다.

통신 거리·기지 연결성·중계 제약, SIPP, 물·얼음 검출, 새 MPC 및 CP-SAT 구현은 포함하지 않습니다. 팀원 BT는 `dev_chanseo@02cda62`에서 확인했고 원본 순수 로직 테스트 54개를 통과했습니다. [BT 조사와 연결 조건](../../docs/bt_integration_review.md)을 참고하세요. 새 `moon_exploration.launch.py`는 고정한 원본 TeamBT·RobotBT를 재사용하는 시간 경로 어댑터입니다. **실제 새 4대 Jazzy/Webots 실행은 권한 제한으로 미검증**입니다. [4대 탐사 인계](../../docs/bt_exploration_handoff_2026-11-22.md)에 반복 탐사·센서 관측·평가 분모·출력·실행 순서를 정리했습니다.

## 최적화와 충돌 조정

모든 `C[i,j]`는 하나의 공통 `t0`에서 계산한 A* 최소 이동+대기 tick입니다. 한 로봇에 한 목표, 한 목표에 한 로봇을 배정하여 `Σ C[i,j]`를 최소화합니다. 단위가 초인 실수 비용을 알고리즘별로 따로 반올림하지 않습니다.

도달 불가 조합은 JSON `null`과 금지 조합 목록으로 기록합니다. A* 실패는 설정한 격자·시간·유한 horizon 및 보수적인 지형 모델 안에서의 실패이며 물리 세계에서 영원히 도달할 수 없다는 증명은 아닙니다. `SpaceTimePlanner.last_status/last_reason`으로 원인을 확인할 수 있습니다.

독립 경로는 로봇 간 교차·마주침·도착 후 점유를 연속 선분의 최소 거리로 다시 검사합니다. 고정된 배정을 유지하며 우선순위 예약으로 재계획합니다. 대기 로봇과 아직 계획하지 않은 로봇의 시작점도 점유합니다. 최대 6대까지 순서를 바꿔 시도하고 그 이상은 설정 순서만 시도합니다. 후순위 시작점을 영구 장애물로 두는 보수적인 방식이므로 시작점 교환 같은 가능한 문제도 실패할 수 있습니다. 이 경우 **배정 가능·공동 계획 실패**를 구분해 보고합니다.

Hungarian의 최적성은 **개별 비용 행렬의 배정 합**에 한정됩니다. 충돌 조정 후 공동 주행의 팀 최적성을 의미하지 않습니다. `execution_plan.json`은 최초 배정 합, 공동 계획 합, 추가 tick을 따로 보존합니다. 동점 배정이 달라도 합계가 같으면 같은 최적값일 수 있습니다.

## 환경과 빌드

대상 환경은 Ubuntu 24.04, ROS 2 Jazzy, Webots R2025a입니다. 새 체크아웃만 마운트하는 [Docker 구성과 명령](../../docker/lunar/README.md)을 먼저 확인하세요. 호스트의 다른 저장소나 예전 컨테이너에 있는 패키지를 덮어쓰지 않습니다.

필수 ROS 의존성은 `package.xml`에 선언했습니다. `rclpy`, 메시지·TF·launch, `webots_ros2_driver`, 기존 모드용 SLAM Toolbox/Nav2/RViz가 필요합니다. 순수 알고리즘에는 Python 3.10 이상, NumPy, PyYAML이 필요하고 자동 검사는 pytest를 사용합니다. SciPy·OR-Tools는 필요하지 않습니다.

저장소 루트 또는 컨테이너 `/workspace/py_bt_ros`에서:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros_packages/moon_mapping_demo \
  --build-base build/jazzy --install-base install/jazzy --symlink-install
source install/jazzy/setup.bash
```

`package.xml`의 기존 `multirobot_sim` 의존성도 처음부터 준비한다면 다음처럼 함께 빌드할 수 있습니다. 새 모듈은 `moon_mapping_demo` 내부 드라이버·센서를 사용합니다.

```bash
colcon build --base-paths ros_packages/multirobot_sim ros_packages/moon_mapping_demo \
  --build-base build/jazzy --install-base install/jazzy --symlink-install
```

## 검증 순서와 실행

1. 핵심 알고리즘과 교환 형식을 검사합니다. 불필요한 호스트 pytest 플러그인은 자동 로딩하지 않습니다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=ros_packages/moon_mapping_demo \
  python3 -m pytest -q -p no:cacheprovider ros_packages/moon_mapping_demo/tests

# ROS·Webots 없이 실제 월드 파일로 행렬과 공동 계획을 생성합니다.
PYTHONPATH=ros_packages/moon_mapping_demo python3 -m moon_mapping_demo.experiment \
  --config ros_packages/moon_mapping_demo/config/shadow_experiment.yaml \
  --t0 0 --output results/shadow_hungarian/t0_0
# 빌드 후에는 같은 CLI를 사용할 수 있습니다.
ros2 run moon_mapping_demo moon_shadow_plan --t0 0 --output results/shadow_hungarian/t0_0
```

성공 종료 코드는 0, 완전 배정 또는 공동 계획 실패는 2입니다. `problem.json`, `problem.csv`, `execution_plan.json`을 생성합니다. t0가 바뀌면 그림자와 최종 대기 안전성이 달라질 수 있으며 배정이 가능해도 공동 계획은 실패할 수 있습니다.

2. Webots·RViz에서 시각화를 먼저 확인합니다. 기본은 정지한 시각화 모드입니다.

```bash
ros2 launch moon_mapping_demo moon_shadow.launch.py execute:=false
```

RViz 고정 좌표계는 `map`입니다. 로봇별 `/{robot}/shadow_path`와 `/shadow_mission/markers`를 표시합니다. 경로의 각 `PoseStamped`에는 예정 시각이 있고, 텍스트에는 절대 도착 초와 대기 시간이 있습니다. 둘 다 transient-local이므로 나중에 RViz를 연결해도 수신합니다. RViz의 일반 2D Goal Pose 도구는 시간 경로 모드에서 제거합니다.

화면 없이 메시지 계약을 검사할 수도 있습니다. 아래 입력은 합성 clock/odom이며 **물리 주행 검사가 아닙니다**.

```bash
ROS_DOMAIN_ID=187 python3 ros_packages/moon_mapping_demo/tests/ros_smoke.py
ROS_DOMAIN_ID=188 python3 ros_packages/moon_mapping_demo/tests/ros_smoke.py --execute
```

`--execute`는 robot4의 odom 중단, clock 역행, stop 서비스, 다른 cmd_vel 발행자를 각각 검사하며 정지 상태가 `runtime.json`에도 저장되는지 확인합니다. `--output-root results/ros_safety`로 결과 저장 위치를 지정할 수 있습니다.

Webots를 사용할 수 없을 때는 같은 제어·안전 판단을 이상적인 차동 구동 모델로 확인할 수 있습니다. ROS/DDS, 물리 접촉·미끄러짐·오도메트리 오차를 검증하는 검사는 아닙니다.

```bash
PYTHONPATH=ros_packages/moon_mapping_demo python3 \
  ros_packages/moon_mapping_demo/tests/kinematic_smoke.py \
  --mode single --output results/kinematic_single
PYTHONPATH=ros_packages/moon_mapping_demo python3 \
  ros_packages/moon_mapping_demo/tests/kinematic_smoke.py \
  --mode multi --output results/kinematic_multi
```

3. 짧은 단일 로봇 설정으로 먼저 주행합니다. robot2~4는 충돌 검사에 남습니다.

```bash
ros2 launch moon_mapping_demo moon_shadow.launch.py execute:=true \
  config:="$PWD/ros_packages/moon_mapping_demo/config/shadow_single.yaml" \
  output:=results/shadow_single
```

4. 단일 로봇 로그를 확인한 뒤 기본 다중 로봇 실험을 실행합니다.

```bash
ros2 launch moon_mapping_demo moon_shadow.launch.py execute:=true \
  output:=results/shadow_multi
ros2 topic echo /shadow_mission/status
ros2 service call /shadow_mission/stop std_srvs/srv/Trigger '{}'
# 정지 후 현재 위치와 현재 clock에서 새 비용 행렬·배정을 생성합니다.
ros2 service call /shadow_mission/start std_srvs/srv/Trigger '{}'
```

실행 시 모든 로봇의 최신 odom이 필요합니다. 선택한 로봇만 관측하고 대기 로봇을 누락시키면 실행하지 않습니다. 다른 `cmd_vel` 발행자가 발견되면 실행을 중단합니다. `/clock` 역행은 STOPPED, clock 또는 odom 중단은 ERROR 및 정지로 처리합니다.

자동 계획 전에는 전체 odom과 clock이 시뮬레이션·실제 시간 양쪽에서 기본 3초 동안 연속으로 안정되는지 확인합니다. 초기 GUI·드라이버 준비 중 입력이 끊기면 정지 상태에서 관측을 다시 시작합니다. 선택 설정 `execution.startup_stable_seconds`로 기간을 바꿀 수 있습니다. `execution.odom_timeout`은 시뮬레이션 내 관측 나이, `execution.input_wall_timeout`은 실제 수신 중단 시간입니다. 기본 GUI 실험은 각각 0.6초와 3초를 사용하며, wall 설정이 없으면 odom_timeout 값을 사용합니다. 상태 토픽과 `runtime.json`의 `input_health`에 두 제한과 로봇별 odom 시각·수신 지연, clock 수신 지연을 기록합니다. 오류 후에는 `start`로 수동 재시작합니다.

기존 SLAM/Nav2 모드는 별도 실행합니다.

```bash
ros2 launch moon_mapping_demo moon_mapping.launch.py
```

## 설정과 실제 실행의 한계

`config/shadow_experiment.yaml`에서 로봇 목록, 목표 ID·좌표, 속도, 몸체 반경, 격자·시간 해상도, horizon, 시간 안전 여유 단위를 바꿉니다. 목표 수는 로봇 수와 같아야 합니다. 시작 좌표·전체 로봇 목록은 `environment.yaml`에서 읽으며 WBT와 다르면 거부합니다. `shadow_single.yaml`은 짧은 단일 로봇 확인용입니다. 속도는 기존 드라이버 상한 0.22m/s를 넘길 수 없습니다.

실행기는 실제 위치와 예정 시각의 오차, 끝점 도착 지연, 남은 경로와 목표 대기의 안전성을 검사합니다. 이탈 시 정지하고 같은 배정 목표에 대해 현재 시각에서 다시 계획합니다. 계산 중 출발 시각이 지나도 기존 경로를 단순히 시간 이동시키지 않고 재계획합니다. 재계획 횟수 상한, 관측 유효 시간, 제어 주기와 허용 오차는 `execution` 설정에 있습니다. 초기 행렬·배정은 보존하고 실행 이력은 `runtime.json`에 별도 저장합니다.

중간 지점에 일찍 도착하거나 명시적으로 대기하는 동안에는 선속도 0을 유지하며 다음 이동 방향으로 선회합니다. 남은 경로의 로봇 충돌 검사는 현재 관측 위치에서 미래 시간표로 복귀하는 선분을 검사합니다. 원래 계획끼리 안전했더라도 현재 위치 오차 때문에 미래 충돌이 생기면 전체 명령을 보류하고 재계획합니다.

계획 비용에는 **회전·가감속 시간이 없습니다**. 추종기는 출발 전 선회할 수 있지만 코너 회전과 접지 오차가 지연·재계획을 유발할 수 있습니다. 정지·비상정지·계산 중에는 로봇이 스스로 다가오는 그림자를 피한다는 보장이 없습니다. 도착 후 그림자 검사는 팀 최종 도착+설정된 목표 대기 시간까지이며 무기한 안전 점유가 아닙니다.

전용 실행 모드의 위치는 시작 앵커+바퀴/IMU 오도메트리입니다. 장거리 누적 오차, 기울기·물리 접촉, 실제 지형 모델과 보수적 근사의 차이는 남습니다. 모델 안에서의 연속 충돌 검사는 실제 물리 주행의 절대 안전 보증이 아닙니다. 컨트롤러의 `/robotN/in_terrain_shadow`는 별도의 9점 표본 정답 표시이며 주행 금지 판정 자체로 사용하지 않습니다.

## CP-SAT 공통 비교 규격

[비교 규격](../../docs/shadow_assignment_format.md)의 JSON 또는 CSV를 그대로 사용합니다. 동일한 행·열 순서, `t0`, 정수 tick 비용, 금지 조합과 목적함수를 공유합니다. CP-SAT 팀에서 공간 경로나 실수 시간을 다시 계산하지 않습니다.

JSON에는 `robot_ids`, `goal_ids`, `costs`, `forbidden_pairs`, `time_step_seconds`, `t0_sim_seconds`, 지도·태양 SHA-256 `metadata.map_id`, 실험 설정 SHA-256 `metadata.config_id`를 기록합니다. 비용 행렬 생성 시간과 배정 시간은 wall-clock 초로 분리합니다. 최초 경로 비용과 충돌 조정 결과는 별도 파일에 있습니다.

## 검증·인계

2026-11-22 수정 후 호스트 Humble 회귀 **104개**와 합성 ROS 연결·안전정지 **5가지** 검사를 통과했습니다. 단일 기구학 검사는 19.0초에 목표 대기까지 완료했고, 다중 검사는 재계획 2회 후 66.6초에 완료했습니다. 최종 목표 오차는 모두 설정한 0.05m 이내였고 다중 최소 표본 간격은 약 0.707m였습니다. 모두 시뮬레이션 초이며 Webots 물리 주행 결과가 아닙니다.

11월 21일의 Jazzy 53개 검사·실제 Webots/RViz 연결·약0.46m 이동은 이전 코드 검증 기록입니다. 11월 22일에는 사용자가 호스트 터미널에서 GUI를 실행했고, 초기 안정화와 별도 wall 제한을 반영한 **단일·3대 실제 주행 모두 COMPLETE**를 확인했습니다. 단일 목표 오차는0.03267m·재계획0회였습니다. 소프트웨어 렌더링 다중은82.7 sim초에 전체 대기를 마쳤고 목표 오차는3.3~3.8cm, 공동 재계획3회였습니다.

이어 **MX450 GPU 모드의 3대 주행도 COMPLETE**를 확인했습니다. 실제 OpenGL 렌더러는 Mesa NVK/Zink의 NVIDIA GeForce MX450이었고,82.89 sim초에 전체 대기를 마쳤으며 목표 오차는3.0~4.0cm·재계획3회였습니다. 계획 생성 이후 실제 시간은 소프트웨어711.32초에서 GPU194.07초로 줄었습니다(이번 실행 약3.67배). GPU 자동 선택·검사·누락 Vulkan 드라이버의 증분 빌드는 [Docker README](../../docker/lunar/README.md)를 따릅니다.

결과는 `results/gui_single_20261122_011837_9508/`, `results/gui_multi_20261122_012256_10407/`, `results/gui_multi_20261122_015002_13096/`의 runtime 및 `validation_from_runtime.json`에 있습니다. 판정은 실행기 상태와 최종 odom을 배정 목표와 대조한 범위이며, 실제 정답 위치·연속 그림자 Bool의 독립 기록은 수집하지 않았습니다. 이 결과는 BT 어댑터 추가 이전의 기록입니다. Codex의 직접 Docker·화면 접근은 여전히 제한되므로 사용자가 실행하고 공유된 로컬 결과를 읽었습니다.

실제 실행한 검사와 남은 작업은 [요구사항·인계 기록](../../docs/shadow_hungarian_requirements.md)을 기준으로 합니다. 코드·단위검사·합성 ROS 메시지 검사는 Webots 실제 주행이나 RViz 화면 확인과 구분해서 기록합니다.

외부 계약 참고: [Nav2 NavigateToPose API](https://api.nav2.org/actions/humble/navigatetopose.html)는 공간 목표를 받으며 이 실험의 시간 경로 계약을 제공하지 않습니다. 라이선스는 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)를 확인하세요.


## 선택형 선발 2대·후발 출발 정책

기존 `simultaneous`를 기본으로 보존하고 `staggered_pairs`를 추가했습니다. 네 목표의 Hungarian 배정을 유지한 뒤
공동 경로에서 선발 조합·시차·안전한 대기 위치를 유한 탐색합니다. 후발도 같은 t0의 명시적 대기 경로를 받으며,
해제 시각에서 A*를 다시 실행합니다. 선발의 목표 도착을 요구하거나 실험 내내 이동 로봇을 2대로 제한하지 않습니다.

```bash
LUNAR_GPU=mesa LUNAR_DEPARTURE_POLICY=staggered_pairs bash docker/lunar/run_demo.sh bt-fixed
# 기존 정책과 비교하려면 LUNAR_DEPARTURE_POLICY=simultaneous
```

직접 launch에는 `departure_policy:=staggered_pairs`, YAML에는 `departure.mode: staggered_pairs`를 사용합니다.
`clearance_distance_m`, `delay_candidates_seconds`, `max_schedule_attempts`, `staging_distance_m`, 선택적 `release_region`을 설정할 수 있습니다.
기본 거리 기준 해제는 통로 자동 인식의 정답이 아니며, 명시적 사각형은 xmin,xmax,ymin,ymax 순서입니다.
실제 통과가 확인되지 않으면 이전 일정을 이동하지 않고 전체 정지·재계획합니다. RViz에 선발/후발·출발 대기·예정 이동 시각을 표시합니다.

모의 비교에서는 고정 목표가 양쪽 모두 완료됐지만 새 정책이 14초 느렸고, 단기·넓은 탐사 실패의 개선은 확인하지 못했습니다.
검증과 비용·예정/실제 출발 지표는 [최신 인계 절](../../docs/bt_exploration_handoff_2026-11-22.md)과
[검증 JSON](../../docs/shadow_validation_2026-11-22.json)의 `bt_exploration_integration.departure_staggering`을 확인합니다.


## 이른 대피와 관측 후 안전 시간

최신 수정은 [인계 문서의 추가 수정 절](../../docs/bt_exploration_handoff_2026-11-22.md)과 검증 JSON의 bt_exploration_integration.early_escape_and_goal_windows를 기준으로 합니다.
기본 출발 정책은 simultaneous, 기본 위험 대응은 candidates입니다. 현재 위치에서 계산·준비를 안전하게 기다릴 수 있는지 먼저 확인하고 관측 이후 유한 후속 대기와 복수 대피 후보를 검사합니다. 반경0.26m·센서 관측·고정 분모는 유지합니다.

직접 launch의 risk_stage:=baseline|early|goals|candidates|heading으로 단계별 비교합니다. heading은 회전 방향을 A* 상태에 포함하는 비교 옵션이며 기본값이 아닙니다. Docker에서는 LUNAR_RISK_STAGE로 선택하며 출발 정책과 독립적입니다.

새 모의 넓은 영역에서 기본 candidates는 경과957.9초·관측10.5789%·완료7회차 뒤 후속 계획 예산 초과로 부분 종료했습니다. 요청한1800초/85% 목표는 미달성입니다. 단기 회차 감소와 계산 지연 시 관측 미완료도 남았습니다. 실제 Webots 검증은 Docker 권한 제한으로 미수행입니다. 비교 그림은 results/early_escape_20261122_191957/comparison_final/에 있습니다.
