# moon_mapping_demo — long 버전

네 대의 LIMO가 달 지형의 이동하는 그림자를 피하면서 탐사하는 ROS 2 패키지입니다. 기본 실행은 ROS 2 Jazzy와 Webots R2025a를 포함한 Docker 환경을 사용합니다.

## 실행

저장소 루트에서 Docker 이미지를 빌드한 뒤 실행합니다. 다운로드와 이미지 빌드는 [루트 README](../../README.md)를 따릅니다.

```bash
LUNAR_GPU=mesa \
LUNAR_RVIZ=true \
LUNAR_DEPARTURE_POLICY=simultaneous \
LUNAR_RISK_STAGE=candidates \
bash docker/lunar/run_demo.sh explore-wide
```

`explore`는 400m² 영역, `explore-wide`는 3364m² 영역을 사용합니다. `bt-fixed`는 네 로봇의 고정 목표 주행입니다. 종료는 Ctrl+C입니다.

다른 터미널에서 탐사율을 확인합니다.

```bash
watch -n 1 bash docker/lunar/run_demo.sh coverage
```

## 알고리즘 흐름

라이다 관측 누적 → 탐사 경계와 목표 후보 생성 → 시간 경로와 비용 계산 → 헝가리안 목표 배정 → 공동 경로 예약 → 행동 트리 실행 → 새 관측과 다음 목표 선정 순서입니다.

- **관측 격자:** 취득 시각의 센서 TF로 광선을 투영합니다. 관측한 셀의 합집합 면적을 고정 평가 영역의 면적으로 나눠 탐사율을 구합니다.
- **긴 경로 후보:** 경계 방향을 사전 지형 위에서 연장합니다. 넓은 설정은 12m급, 작은 설정은 10m급 후보를 사용하며 짧은 대안도 함께 고려합니다.
- **공간·시간 A\*:** 시각별 위치와 이동·회전·대기 시간을 탐색합니다. 긴 후보는 직선 연결을 먼저 검색하고 필요한 경우 격자 탐색을 사용합니다.
- **헝가리안 알고리즘:** 로봇–목표 시간 비용 행렬로 목표를 배정합니다. 동료의 예약 경로와 몸체 간 충돌 조건으로 공동 경로를 조정합니다.
- **그림자와 지형:** 원본 삼각형 지형과 시각별 그림자 투영을 원형 로봇 몸체와 비교합니다. 현재 위치와 남은 경로의 위험을 확인하고 필요한 경우 대피 경로를 계산합니다.
- **계산 예산:** 관측한 시뮬레이션 시계 속도와 출발 준비 기한을 사용합니다. 시각이 지난 계획 결과는 취소합니다.
- **TeamBT·RobotBT:** 팀 배정, 출발 준비, 이동, 목표 관측과 재계획을 관리합니다.

## 주요 파일

| 파일 | 역할 |
| --- | --- |
| `launch/moon_exploration.launch.py` | 탐사 실행 |
| `config/exploration_wide.yaml` | 넓은 영역과 긴 후보 설정 |
| `config/exploration_short.yaml` | 작은 영역과 긴 후보 설정 |
| `moon_mapping_demo/exploration.py` | 탐사 단계와 계산 기한 관리 |
| `moon_mapping_demo/exploration_planning.py` | 후보 경로와 목표 배정 |
| `moon_mapping_demo/space_time.py` | 공간·시간 경로 탐색 |
| `moon_mapping_demo/work_regions.py` | 작업 구역과 긴 목표 후보 |
| `moon_mapping_demo/observation.py` | 센서 관측 격자 |
| `moon_mapping_demo/risk.py` | 위험 예측과 대피 판단 |

`long_routes`, `long_route_distance_m`, `max_path_seconds`로 긴 후보와 경로 시간 범위를 설정합니다. `short` 버전 태그와 `exploration_short.yaml`의 작은 영역 설정은 서로 다른 구분입니다.
