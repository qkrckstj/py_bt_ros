# 달 탐사 Docker 실행

Ubuntu 24.04, ROS 2 Jazzy와 Webots R2025a를 포함하는 실행 환경입니다. 현재 체크아웃을 컨테이너의 `/workspace/py_bt_ros`에 연결합니다.

## 이미지 빌드

저장소 루트에서 실행합니다.

```bash
export LUNAR_REPO="$(pwd -P)"
export LUNAR_UID="$(id -u)"
export LUNAR_GID="$(id -g)"
docker compose -f docker/lunar/compose.yaml build lunar
```

## Webots와 RViz 실행

호스트 데스크톱 터미널에서 실행합니다.

```bash
export LUNAR_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

LUNAR_GPU=mesa \
LUNAR_RVIZ=true \
LUNAR_DEPARTURE_POLICY=simultaneous \
LUNAR_RISK_STAGE=candidates \
bash docker/lunar/run_demo.sh explore-wide
```

X 인증 파일은 현재 데스크톱 세션의 실제 경로를 사용합니다. 기존 실행은 Ctrl+C로 종료한 뒤 새 실행을 시작합니다.

| 환경 변수 | 값 |
| --- | --- |
| `LUNAR_GPU` | `auto`, `mesa`, `nvidia`, `software` |
| `LUNAR_RVIZ` | `true`, `false` |
| `LUNAR_DEPARTURE_POLICY` | `simultaneous`, `staggered_pairs` |
| `LUNAR_RISK_STAGE` | 기본 실행 명령은 `candidates` 사용 |
| `LUNAR_ROS_DOMAIN_ID` | 기본 `63` |

Mesa 모드는 Intel·AMD·nouveau GPU의 `/dev/dri`를 사용합니다. NVIDIA 독점 드라이버 모드는 NVIDIA Container Toolkit이 필요합니다. 기본 `auto`는 커널 드라이버에 따라 GPU 모드를 선택합니다.

## 실행 모드와 제어

| 마지막 인자 | 내용 |
| --- | --- |
| `explore-wide` | 넓은 영역 탐사 |
| `explore` | 작은 영역 탐사 |
| `bt-fixed` | 네 로봇의 고정 목표 주행 |
| `coverage` | 탐사율과 현재 단계 표시 |
| `stop` | 창을 유지하고 임무 정지 |
| `start` | 현재 위치에서 임무 다시 시작 |

다른 터미널의 프로젝트 루트에서 탐사율을 확인합니다.

```bash
watch -n 1 bash docker/lunar/run_demo.sh coverage
```

실행 결과는 `results/gui_<모드>_<시각>_<PID>/`에 저장됩니다. RViz의 `/exploration/observations` 지도에서 흰색은 관측한 빈 공간, 검은색은 장애물, 회색은 미관측 영역입니다.

버전 선택과 사용 알고리즘은 [루트 README](../../README.md)를 참고하세요.
