# 도시 재난 다중 로봇 할당 실험 환경

10대 LIMO가 알려진 도시 지도에서 7개 작업을 나누어 수행한다. 작업마다 최소 인원·작업량이 다르고, 미진압 화재는 시간이 지날수록 작업량이 증가한다. **할당 알고리즘을 교체해 같은 조건에서 완료 시간과 성공률을 비교**하기 위한 환경이다.

- Webots: 실제 바퀴·충돌 물리 + 로봇별 Nav2 + 팀/로봇 BT.
- 경량 Python: 동일 작업·배터리 상태 전이 + 격자 경로 이동. PPO 학습과 반복 평가용.
- 시각화: 작업 위 반투명 구, 진행도 바, 현장 인원/필요 인원, 실제 작업 배속, 전체 경과 시간·완료 수.
- 상태 관리자: `/disaster/observation` 및 `disaster_city/runtime/latest.json`. 할당기와 화면은 같은 공개 상태를 읽는다.
- 할당기: 최소 팀 집결 시간 기준선, MaskablePPO, 사용자 정의 Python 클래스.

## 시작

아래 명령은 **이 README가 있는 폴더**에서 실행한다. 원본 저장소 전체 작업 환경을 덮어쓸 필요가 없다.

### 1. 경량 학습·평가

Ubuntu 24.04 / Python 3.12 기준:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r disaster_city/rl/requirements.txt
# 같은 조건의 기준선·저장된 PPO 모델 비교
bash scripts/disaster_ppo.sh compare \
  --profile disaster_city/profiles/junyong_10_damage \
  --model models/ppo_allocation.zip --randomize --seed 2000 --count 50 \
  --out disaster_city/rl/results/comparison
# 새 학습
bash scripts/disaster_ppo.sh train \
  --profile disaster_city/profiles/junyong_10_damage \
  --steps 200000 --seed 0 --out disaster_city/rl/results/my_model
```

`--resume models/ppo_allocation.zip`을 추가하면 `--steps`만큼 추가 학습한다. 기본 CLI 프로필은 과거 `junyong_10_known`이므로 현재 실험은 위처럼 `--profile`을 명시한다. 설치된 가상환경을 쓰려면 `CITY_RL_PYTHON=/path/to/python`을 설정한다.

### 2. Webots 시연

ROS 2 Jazzy, Nav2, Webots R2025a, `webots_ros2_driver`, RViz가 필요하다.

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-webots-ros2-driver ros-jazzy-rviz2 \
  python3-pil python3-yaml python3-numpy python3-scipy python3-tk
export WEBOTS_HOME=/path/to/webots
# 기준선 — 자동 준비 후 시작
bash scripts/run_disaster_city.sh
# 학습된 정책
bash scripts/run_disaster_ppo.sh models/ppo_allocation.zip
# 시연 녹화: 1920×1080, 4배속 + 이벤트별 PNG/JSON
CITY_EXIT_AFTER_RECORDING=1 bash scripts/run_disaster_ppo.sh models/ppo_allocation.zip recordings/demo
```

배포 패키지 대신 소스 설치한 Webots ROS 드라이버를 쓰면 `CITY_ROS_OVERLAY=/path/to/install/setup.bash`를 설정한다. Python 코드와 로봇 드라이버는 이 폴더의 `PYTHONPATH`로 로드하며 별도 colcon 빌드가 필요 없다. ROS 도메인 기본 168, Webots 포트 1257. 같은 PC의 중복 실행은 잠금으로 막는다. Ctrl+C 또는 Webots 종료로 전체 종료한다. 선택 창: `CITY_TASK_WINDOW=1`.

녹화는 Webots의 동영상 인코더를 사용한다. ffmpeg를 찾지 못하면 설치 후 PATH에 추가한다. `recordings/demo/recording_status.json`의 `ready=true, failed=false`로 저장 완료를 확인한다.

## 시나리오 변경

현재 프로필: `disaster_city/profiles/junyong_10_damage/scenario.json`.

| 설정 | 의미 |
|---|---|
| `starts` | 시작 좌표 배열. **원소 수가 로봇 대수**이며 Nav2·BT·화면에 반영 |
| `initial_battery` | 로봇별 초기 잔량. starts와 같은 길이 |
| `queue_positions` | 충전 대기 좌표. 로봇 수 이상의 위치 확보 |
| `tasks[].xy` | 작업 시각화 중심 좌표 |
| `tasks[].work_positions` | 실제 접근·정차 위치. maximum 이상 슬롯 확보 |
| `tasks[].minimum`, `maximum` | 작업 시작 최소 인원·배치 가능한 최대 인원 |
| `tasks[].seconds` | 최소 인원 기준 작업량(작업초) |
| `fire_growth_per_s` | 진압 정족수 미달 시 초당 증가 작업량. 현재 0.6 |
| `fire_growth_range` | 학습·무작위 평가의 증가율 범위. 현재 0.3~0.9 |
| `decision_period_s` | 할당 주기. 현재 5초 |
| `speed_mps`, `battery_rates`, `reserve` | 주행 속도·상태별 에너지 변화·복귀 여유 |
| `team_wait_release_s`, `team_arrival_grace_s`, `team_retry_delay_s` | 대기팀 해제 조건과 재배정 보류 |
| `horizon_s` | 임무 제한 시간. 현재 2400초 |

현재 작업은 화재 2~4대/30초, 배전함 2~3대/20초, 소화전 2~4대/20초, 작은 잔해 1~3대/10초, 큰 잔해 4~6대/50초, 나무 3~5대/30초, 통제 1~2대/10초다.

최소 인원이 도착·정차하면 1배속, 추가 한 대마다 +0.5배속으로 작업한다. 인원이 부족하면 진행을 보존하고 중단한다. 화재는 작업 중 증가가 멈추며 중단 시 다시 증가한다. 이 증가율은 작업량 모델의 설정값이다.

새 프로필은 기존 폴더를 복사하고 `CITY_SCENARIO_DIR=/absolute/profile` 또는 학습 CLI `--profile`로 선택한다. 지도는 `city.pgm`/`city.yaml`, 3D 환경은 `city.wbt`를 함께 유지한다. 월드의 `@CITY_ROOT@`는 실행 때 현재 폴더로 치환한다. 시작·정차·충전 위치는 장애물 여유를 확보해 지정한다.

현재 후보 생성기는 한 번에 1~6대를 파견하고 학습 인코더는 제공된 7종 작업명을 사용한다. 로봇 수·작업 수·타입 순서 등을 바꾸면 관측/행동 의미가 바뀌므로 **PPO를 새로 학습**한다. 저장 모델은 제공한 10대/7작업 프로필용이다.

## 다른 알고리즘 연결

`disaster_city/examples/allocator.py`를 복사하여 다음 인터페이스를 구현한다.

```python
class Allocator:
    policy_name = 'my_algorithm'
    def __init__(self, grid, cfg):
        self.grid, self.cfg = grid, cfg
    def choose(self, observation, feasible_actions):
        # feasible_actions: {action_id: [robot assignment proposals]}
        # 0: 현재 배정 유지. 그 밖의 ID: 팀 파견 / 증원 / 충전 / 대기팀 해제.
        return 0
```

```bash
# Webots에서 교체
CITY_ALLOCATOR=disaster_city.examples.allocator:Allocator bash scripts/run_disaster_city.sh
# 같은 알고리즘의 경량 평가
bash scripts/disaster_ppo.sh baseline \
  --profile disaster_city/profiles/junyong_10_damage \
  --allocator disaster_city.examples.allocator:Allocator \
  --randomize --seed 2000 --count 50 --out disaster_city/rl/results/my_algorithm
```

`observation`에는 `time_s`, `phase`, `robots`, `tasks`, `stations`가 있다. 로봇은 위치·배터리·역할·목표·준비 상태, 작업은 필요/배정/도착 인원·진행·잔여 작업량을 제공한다. `grid.distance(a,b)`는 장애물을 고려한 경로 거리다.

후보는 `(robot_id, role, target_xy, task_id, slot)` 제안 묶음이다. `choose()`는 후보 ID만 반환하며 상태를 직접 수정하지 않는다. 실행 불가능한 ID는 오류 처리한다. 배터리 복귀 안전 조건은 공통 실행기에 유지된다. `CITY_ALLOCATOR`와 `CITY_PPO_MODEL`은 동시에 지정하지 않는다.

후보 생성 자체를 연구하려면 `allocation.py`의 `TeamActions.candidates()`와 `safety()`를 수정한다. 기본 후보는 최소 팀을 한 번에 확보하고, 기존 참여자 유지·증원·슬롯 중복·배터리 여유를 검사한다. 경량/실주행에 같은 코드를 사용한다.

## 구조와 실험 기록

`engine.py`는 작업/배터리 상태를 소유한다. 팀 BT는 준비 상태 확인 후 할당기를 주기적으로 호출하며, 각 로봇 BT는 비동기 Nav2 명령을 실행한다. 이동 중 정체는 실패·재배정으로 전달한다. `supervisor.py`는 시뮬레이터 위치와 휠 오도메트리로 TF를 정렬하고 화면을 갱신한다.

경량 환경은 회전·로봇 간 혼잡·국소 회피를 단순화한다. Webots는 실제 바퀴와 Nav2로 주행한다. 이번 시나리오는 작업을 처음부터 공개하며 통신·그림자 제약은 사용하지 않는다. 위치는 시뮬레이터 정답 위치로 보정한다.

| 2026-11-28 결과 | 기준선 | PPO |
|---|---:|---:|
| 경량 환경, 새 평가 seed 2000~2049, 성공 | 50/50 | 50/50 |
| 같은 50개 조건 평균 완료 시간 | 422.22초 | 268.98초 |
| Webots 고정 조건 실주행 | 미측정 | 308.4초 / 7개 완료 |

기준선은 `nearest_feasible_quorum_v1`: 최소 팀이 가장 빨리 집결할 수 있는 작업을 선택하며 작업 종류별 우선순위는 없다. PPO도 화재 우선 규칙 없이 전체 완료 시간을 학습했다. 제공 모델은 누적262,144스텝, 단일 지도·작업 위치·학습 시드의 결과다. 추가학습 검증 평균은 272.9→270.5초로 개선 폭이 작았다. Webots 한 번의 성공과 경량 50회 비교를 구분해서 해석한다.

- `results/heldout_50.json`: 각 시드 결과·이벤트·성공 상태·시간, 설정/모델 해시.
- `results/webots_result.json`: 실주행 결과. `presentation/`에 영상·PNG·그래프.
- `disaster_city/runtime/events.jsonl`: 현재 실행 이벤트. `latest.json`은 공개 상태.
- 종료 상태: `COMPLETE`, `TIME_LIMIT`, `BATTERY_FAILURE`.

검증: `PYTHONPATH=. .venv/bin/python -m pytest -q tests` (pytest 별도 설치).

## 출처

도시 지도: [qkrckstj/py_bt_ros dev_junyong](https://github.com/qkrckstj/py_bt_ros/tree/dev_junyong/urban_disaster_30m), 가져온 커밋 `fd8ecd1`. 원본과 자료는 `profiles/junyong_30m/upstream/`에 유지한다. LIMO 모델·기본 드라이버 기반: [inmo-jang/webots_ros2_limo](https://github.com/inmo-jang/webots_ros2_limo), Apache-2.0 LICENSE 포함. 원본 자산의 권리는 각 출처를 따른다.
