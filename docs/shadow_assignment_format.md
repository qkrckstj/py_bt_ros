# Hungarian·CP-SAT 동일 문제 비교 규격

비교 입력은 `moon_shadow_plan`이 생성한 `problem.json` 또는 같은 폴더의 `problem.csv`입니다. 구현은 `moon_mapping_demo/exchange.py`의 `limo-shadow-assignment/v1`을 기준으로 합니다. CP-SAT 자체 구현은 이 변경에 포함하지 않습니다.

현재 월드 t0=0의 실제 [JSON 예제](examples/shadow_problem.json)와 [CSV 예제](examples/shadow_problem.csv)를 함께 제공합니다.

## 최적화 정의

`robot_ids[i]`와 `goal_ids[j]` 순서를 보존합니다. 허용 조합에 이진 변수 `x[i,j]`를 두고 각 행 합=1, 각 열 합=1, 목적함수 `min Σ costs[i][j] * x[i,j]`를 사용합니다. 금지 간선은 변수를 만들지 않거나 0으로 고정합니다. 완전 배정이 없으면 INFEASIBLE로 보고합니다.

`costs`는 이동과 대기를 포함한 **정수 tick**입니다. 한 tick은 `time_step_seconds` 시뮬레이션 초이며 모든 쌍은 `t0_sim_seconds`에서 출발합니다. 두 배정기는 같은 정수 배열을 그대로 사용하고 재계산·재반올림하지 않습니다. 예를 들어 tick 길이 0.5초의 비용 7은 3.5초입니다.

## JSON 필드

| 필드 | 의미 |
| --- | --- |
| schema_version | `limo-shadow-assignment/v1` |
| robot_ids / goal_ids | 행·열의 고유 ID 목록 |
| costs | 정수 또는 `null`의 정사각 행렬; `null`은 금지 |
| cost_unit / time_step_seconds | `tick` / tick 길이(초) |
| t0_sim_seconds / time_origin | 공통 출발 시각 / `ros_clock_sim_seconds` |
| objective | `minimize_sum_cost_ticks` |
| constraints | one_goal_per_robot, one_robot_per_goal, complete_assignment 모두 true |
| forbidden_pairs | `{robot_id, goal_id}` 목록; null 조합과 정확히 일치 |
| metadata.map_id | 실제 WBT+태양 JSON의 SHA-256 |
| metadata.config_id | 실험 YAML 의미 내용을 정렬 JSON으로 변환한 SHA-256 |
| metadata | 계획 설정, 목표, 전체 실제 로봇 위치, 대기 ID 등 추가 정보 |
| timings.matrix_generation_seconds | 경로 비용 행렬 생성의 wall-clock 초 |
| timings.assignment_seconds | 배정 알고리즘만의 wall-clock 초 |
| assignment | columns, total_cost_ticks 또는 null |

간략 예시는 다음과 같습니다. 실제 파일에는 위 표의 나머지 필드도 있습니다.

```json
{
  "robot_ids": ["robot1", "robot2"],
  "goal_ids": ["a", "b"],
  "costs": [[7, null], [4, 6]],
  "cost_unit": "tick",
  "time_step_seconds": 0.5,
  "t0_sim_seconds": 12.0,
  "forbidden_pairs": [{"robot_id": "robot1", "goal_id": "b"}],
  "assignment": {"columns": [0, 1], "total_cost_ticks": 13}
}
```

`read_problem(path)`은 스키마, 정수, ID, 금지 조합 일치와 결과 배정의 실현 가능성·합을 검사합니다. 결과 최적성은 별도로 검증해야 합니다.

## CSV 필드

모든 N² 조합을 한 행씩 씁니다. 헤더는 다음과 같습니다.

```text
schema_version,robot_index,goal_index,robot_id,goal_id,allowed,cost_ticks,cost_unit,time_step_seconds,t0_sim_seconds,time_origin,objective,map_id,config_id
```

`allowed=0`인 행의 `cost_ticks`는 빈칸이고 선택할 수 없습니다. 빈칸을 0 또는 큰 유한 비용으로 바꾸지 않습니다. 인덱스로 원래 순서를 복원하고 금지 간선까지 포함한 전체 행 수가 N²인지 확인합니다. CSV는 간단한 공통 입력이며 상세 메타데이터·시간 측정은 같은 폴더의 JSON을 함께 보존합니다.

## 시간 측정과 결과 해석

공통 행렬을 한 번 생성한 뒤 Hungarian과 CP-SAT의 배정 연산을 각각 측정합니다. 파일 읽기, A*·그림자 계산과 공동 충돌 조정은 배정 알고리즘 시간에 넣지 않습니다. CP-SAT은 최적성 증명 상태, 목적값, 최선 하한, 제한 시간, 실행 시간, 라이브러리 버전과 worker 수를 함께 보고하면 비교할 수 있습니다. 제한 시간 내 FEASIBLE을 OPTIMAL로 취급하지 않습니다.

서로 다른 동점 배정이라도 총비용이 같으면 같은 최적값일 수 있습니다. 처음부터 특정 열 순서를 강제하여 Hungarian 결과와 동일하게 만들 필요는 없습니다.

`execution_plan.json`에는 독립 경로, 배정 결과, 충돌 조정 후 경로, `original_assignment_total_ticks`, `coordinated_total_ticks`, `coordination_extra_ticks`가 따로 있습니다. `runtime.json`은 실제 실행 중 재계획 이력을 기록합니다. 두 배정기의 같은 행렬에 대한 최적값 비교와 다중 로봇 공동 실행 성능 비교는 구분합니다. 공동 충돌 조정은 Hungarian의 배정 문제 밖에 있으며 공동 최적성을 보장하지 않습니다.

## 4대 BT 반복 탐사의 회차별 교환

전용 통합 모드는 각 session의 `round_*/plan_*/attempt_*`에 같은 스키마의 `problem.json/csv`를 저장합니다.
4개 목표 회차는 공통 t0의 4×4 정수 행렬입니다. 부분 배정 시도는 실제 활동 부분집합의 k×k 행렬이며,
대기 로봇 몸체를 포함해 비용을 다시 계산합니다. 서로 다른 부분집합 행렬의 비용을 하나의 전역 최적값으로 비교하지 않습니다.
현재 통합 계획 몸체 반경은 설정값 0.26m이며 각 문제의 `config_id`와 계획 설정으로 식별합니다.
독립 A* 이동·대기 행렬의 Hungarian 최적 배정과, 회전 대기·로봇 충돌 조정 후 공동 실행 비용을 별도로 기록합니다.
계산 취소·계산 예산 초과는 `CANCELLED`/`PLANNING_BUDGET`이며 도달 불가 금지 간선과 구분합니다.
후보 선정, 부분집합 탐색, 우선순위 공동 경로, 전체 반복 탐사의 최적성을 의미하지 않습니다.
