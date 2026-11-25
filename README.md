# LIMO 4대 달 탐사 — long 버전

ROS 2 Jazzy와 Webots R2025a에서 네 대의 LIMO가 지형과 이동하는 그림자를 피해 탐사합니다. Webots와 RViz는 Docker로 실행합니다.

## 버전

같은 `dev_gayeon` 브랜치의 작업을 다음 태그로 구분합니다.

| 태그 | 내용 |
| --- | --- |
| `short` | 이전에 업로드한 짧은 경로 버전 |
| `long` | 긴 직선 목표 후보와 시계 속도에 맞춘 계산 예산을 사용하는 현재 버전 |

`short`와 `long`은 코드 버전이고, 아래 `explore`와 `explore-wide`는 탐사 영역 설정입니다.

## 준비

- Ubuntu 데스크톱 터미널과 X11 또는 Xwayland 화면
- 현재 사용자로 사용할 수 있는 Docker Engine과 Docker Compose
- 읽을 수 있는 현재 데스크톱 세션의 X 인증 파일
- Mesa GPU는 `/dev/dri`, NVIDIA 독점 드라이버는 NVIDIA Container Toolkit

## 다운로드와 이미지 빌드

long 버전을 새 폴더에 받습니다.

```bash
git clone --branch long --single-branch \
  https://github.com/qkrckstj/py_bt_ros.git lunar_hungarian_long
cd lunar_hungarian_long

export LUNAR_REPO="$(pwd -P)"
export LUNAR_UID="$(id -u)"
export LUNAR_GID="$(id -g)"
docker compose -f docker/lunar/compose.yaml build lunar
```

short 버전은 별도 폴더에 받을 수 있습니다.

```bash
git clone --branch short --single-branch \
  https://github.com/qkrckstj/py_bt_ros.git lunar_hungarian_short
```

이미 현재 프로젝트 폴더와 Docker 이미지가 있으면 아래 실행 명령을 사용합니다.

## 실행

프로젝트 루트에서 실행합니다. 기존 실행이 열려 있으면 해당 터미널에서 **Ctrl+C**로 종료합니다.

```bash
export LUNAR_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

LUNAR_GPU=mesa \
LUNAR_RVIZ=true \
LUNAR_DEPARTURE_POLICY=simultaneous \
LUNAR_RISK_STAGE=candidates \
bash docker/lunar/run_demo.sh explore-wide
```

`LUNAR_XAUTHORITY`에는 현재 세션의 실제 인증 파일을 지정합니다. NVIDIA 독점 드라이버 환경에서는 `LUNAR_GPU=nvidia`를 사용합니다. `LUNAR_RVIZ=false`로 Webots만 표시할 수도 있습니다.

| 명령의 마지막 인자 | 실행 내용 |
| --- | --- |
| `explore-wide` | 3364m² 영역 탐사, 최대 시뮬레이션 1800초 |
| `explore` | 400m² 영역 탐사, 최대 시뮬레이션 180초 |
| `bt-fixed` | 네 로봇의 고정 목표 주행 |

처음에는 제자리에서 회전하며 관측한 뒤 이동합니다. 대기와 이동 시각은 시뮬레이션 시간 기준입니다. 종료는 **Ctrl+C**입니다.

## 탐사율과 지도

다른 터미널에서 같은 프로젝트 루트로 이동해 실행합니다.

```bash
watch -n 1 bash docker/lunar/run_demo.sh coverage
```

탐사율, 관측 면적, 현재 단계와 계산 진행 상황을 표시합니다. RViz에서는 관측한 빈 공간이 흰색, 장애물이 검은색, 미관측 영역이 회색입니다. 지도 토픽은 `/exploration/observations`입니다.

창을 유지하면서 임무를 정지하거나 다시 시작할 수 있습니다.

```bash
bash docker/lunar/run_demo.sh stop
bash docker/lunar/run_demo.sh start
```

실행 기록은 `results/gui_<모드>_<시각>_<PID>/`에 저장됩니다.

## 사용한 알고리즘

1. **관측 지도와 탐사 경계:** 라이다 광선과 취득 시각의 TF로 관측 격자를 누적하고, 관측 영역과 미관측 영역의 경계에서 목표 후보를 만듭니다.
2. **긴 목표 후보:** 경계 방향을 사전 지형 위에서 연장해 넓은 영역은 12m급, 작은 영역은 10m급 직선 목표 후보를 만듭니다. 실제 선택 경로는 지형과 충돌 조건에 따라 정해집니다.
3. **공간·시간 A\*:** 이동, 회전, 대기를 시간에 포함합니다. 직선 후보를 먼저 탐색하고, 필요한 경우 격자 경로를 탐색합니다.
4. **헝가리안 배정:** 로봇과 목표 사이의 이동 시간 비용 행렬로 목표를 배정하고, 동료 로봇의 경로 예약을 반영해 공동 경로를 조정합니다.
5. **지형·그림자 회피:** 원본 지형 삼각형과 시각별 그림자 투영을 원형 로봇 몸체와 대조합니다. 관측한 시계 속도와 출발 기한에 맞춰 계산 예산을 정합니다.
6. **행동 트리:** TeamBT와 RobotBT로 배정, 출발 준비, 이동, 관측과 재계획을 관리합니다. 탐사율은 실제 센서로 관측한 셀의 면적으로 계산합니다.

[Docker 사용법](docker/lunar/README.md) · [탐사 패키지와 설정](ros_packages/moon_mapping_demo/README.md)
