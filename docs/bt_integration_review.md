# 팀원 달 탐사 BT 확인과 연결 조건

## 후속 4대 통합 구현

아래는 통합 전 조사 이력입니다. 이후 사용자가 4대 참여·센서 반복 탐사를 확정하여,
고정 원본을 `moon_mapping_demo/vendor/team_bt/`에 포함하고 실제 트리를 확장하는 시간 경로 어댑터를 구현했습니다.
원본 네 모듈·LICENSE의 blob SHA를 확인했습니다. 배포 패키지에 들어가는 사본과 이전 검사용 사본을 구분합니다.
현재 연결·전용 launch·센서/지표·검증/제한은 [새 인계 문서](bt_exploration_handoff_2026-11-22.md)와
[검증 JSON](shadow_validation_2026-11-22.json)의 `bt_exploration_integration`이 기준입니다.


## 2026-11-22 GitHub 확인 (통합 전 조사)

`qkrckstj/py_bt_ros`의 `dev_chanseo`에 새 탐사 BT가 게시되어 있습니다. 확인한 최신 커밋은 [`02cda62197e30858f295de25567003f02242ad64`](https://github.com/qkrckstj/py_bt_ros/commit/02cda62197e30858f295de25567003f02242ad64)입니다.

- [`8e4a492`](https://github.com/qkrckstj/py_bt_ros/commit/8e4a4924baa5a1a7c7ac89894dfce55efbbb72c5): 4대 달 탐사 BT와 배정 시뮬레이션 추가. 한국 시간 11월 22일 00:25:56.
- [`02cda62`](https://github.com/qkrckstj/py_bt_ros/commit/02cda62197e30858f295de25567003f02242ad64): 실물 LIMO 탐사와 pose twin 연동 추가. 한국 시간 11월 22일 00:42:55.
- `dev_junyong`은 여전히 `97b4759786232b77591530c9604326c325e93fbf`입니다. 현재 로컬 `feature/shadow-hungarian`의 기준과 같습니다. 새 BT는 이 브랜치에 합쳐져 있지 않습니다.

원격 조회와 검사용 파일 다운로드만 수행했습니다. pull/merge, push, PR 생성은 하지 않았습니다. 확인한 파일과 blob SHA는 `results/resume_2026-11-22/upstream_bt/snapshot.json`에 있습니다. 이 검사용 사본은 배포 패키지에 포함하지 않습니다.

## 확인한 구현

[`EXPLORATION.md`](https://github.com/qkrckstj/py_bt_ros/blob/02cda62197e30858f295de25567003f02242ad64/simulation/webots_ros2_limo/EXPLORATION.md)는 로봇별 SLAM 지도, frontier 배정, 팀/개별 BT, 선택적 거리 기반 통신 제약과 기지 복귀를 설명합니다. 그림자 회피는 포함하지 않는다고 명시합니다.

[`exploration_runtime.py`](https://github.com/qkrckstj/py_bt_ros/blob/02cda62197e30858f295de25567003f02242ad64/simulation/webots_ros2_limo/webots_ros2_limo/exploration_runtime.py)의 `TeamBT`는 목표 발행·상태 수집·완료 확인·복귀를 관리하며, `RobotBT`는 주입받은 navigation 객체로 실행·취소·관측을 처리합니다. [`exploration_policy.py`](https://github.com/qkrckstj/py_bt_ros/blob/02cda62197e30858f295de25567003f02242ad64/simulation/webots_ros2_limo/webots_ros2_limo/exploration_policy.py)의 기본 배정은 도달 가능한 격자 경로 거리에서 정보 이득 항을 빼고 순서대로 선택하는 greedy 방식입니다.

[`exploration_ros.py`](https://github.com/qkrckstj/py_bt_ros/blob/02cda62197e30858f295de25567003f02242ad64/simulation/webots_ros2_limo/webots_ros2_limo/exploration_ros.py)의 `Navigation.start()`는 `NavigateToPose` 공간 목표를 발행합니다. 로봇 명령에는 `session`, `task_id`, `mode`, `goal: [x,y,yaw]`가 있으며, 시간 경로와 공통 출발 시각은 포함하지 않습니다.

| 항목 | 팀원 BT 기본 구성 | 현재 Hungarian 실험 |
| --- | --- | --- |
| 패키지 | `simulation/webots_ros2_limo` | `ros_packages/moon_mapping_demo` |
| 임무 | 관측 지도에서 frontier 탐사·기지 복귀 | 사전 WBT 지형에서 설정한 목표 1:1 배정 |
| 로봇 | 기본 4대 | 기본 robot1~3, robot4는 대기·충돌 관측 |
| 비용 | 격자 경로 거리와 정보 이득 | 공통 t0의 이동+대기 정수 tick |
| 배정 | greedy, 목표 예약·간격 고려 | Hungarian 완전 배정, 금지 간선 구분 |
| 주행 | Nav2 `NavigateToPose` | 시간표를 유지하는 `cmd_vel` 실행기 |
| 그림자 | 미포함 | 이동·대기·몸체 점유 금지 검사 |
| 통신 | 선택적 거리·기지·중계 연결성 | 이번 요구사항에서 제외 |

## 재사용할 수 있는 경계

`RobotBT.navigation`은 `ready()`, `start(goal)`, `poll()`, `cancel()`을 요구하므로 시간 경로 실행 어댑터를 넣을 수 있는 지점입니다. 다만 같은 함수 이름만 맞추는 것으로 공동 시간 경로가 실행되지는 않습니다.

1. 배정 단계는 설정된 동일 개수의 목표와 로봇을 한 묶음으로 처리하고, 기존 Hungarian 결과를 사용해야 합니다. 기본 greedy 배정이나 frontier 자동 생성으로 교체하면 현재 비교 문제와 달라집니다.
2. navigation 어댑터는 이미 공동 충돌 검사를 통과한 시간 경로·배정 ID·t0를 참조해야 합니다. 각 `start(goal)`에서 별도 Nav2 목표를 발행하거나 시간을 새로 잡으면 안 됩니다. 배치 전체의 준비 상태와 출발을 조정해야 합니다.
3. 한 로봇이 취소·통신 lease 만료·입력 이상으로 멈추면 공동 계획 전체를 정지하고 현재 전체 위치에서 재계획해야 합니다. 이전 `session/task_id`의 지연 완료를 새 계획 완료로 처리하지 않습니다.
4. 이동과 그림자 예측은 ROS `/clock` 기준을 유지합니다. BT의 monotonic 시계 기반 명령 lease는 유효성 감시이며 이동 비용으로 사용하지 않습니다.
5. `radio_enabled=false`, 중계·기지 연결성 미사용을 유지합니다. 선택 대상이 3대여도 대기 robot4의 관측·충돌 검사는 유지합니다. 팀원 launch 전체를 현재 launch 위에 함께 띄우면 Nav2·TF·속도 발행자가 겹칠 수 있습니다.

현재 구현은 `/shadow_mission/start`, `/shadow_mission/stop`의 Trigger 서비스와 `/shadow_mission/status`를 제공합니다. 새 BT의 ROS 계약은 `/exploration/start`, `/exploration/stop`의 Empty 토픽, `/exploration/team_status` 및 로봇별 `exploration/command/state`입니다. 이름과 메시지 타입이 모두 같지 않으므로 직접 연결되었다고 간주하지 않습니다.

## 실제 검사와 남은 검증

고정한 최신 커밋의 다음 원본 테스트를 별도 검사용 사본에서 실행하여 **54개 통과(1.39초)**를 확인했습니다.

- `test_exploration_allocation.py`
- `test_exploration_return.py`
- `test_exploration_radio.py`
- `test_exploration_sweep.py`

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=results/resume_2026-11-22/upstream_bt \
python3 -m pytest -q -p no:cacheprovider \
  results/resume_2026-11-22/upstream_bt/tests
```

이는 원본 BT의 선택한 순수 로직 검사입니다. 새 BT 전체의 Jazzy/Webots 실행, 실물 LIMO 주행, 현재 Hungarian 실행기와의 어댑터 통합은 수행하지 않았습니다. 게시 여부와 테스트 통과를 하드웨어·그림자 회피 통합 완료로 표현하지 않습니다. 이번 세션에서는 연결 조건 조사까지 완료했으며 BT 어댑터를 추가하거나 원격 코드를 병합하지 않았습니다.
