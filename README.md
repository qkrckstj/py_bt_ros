# LIMO 4대 달 탐사 실행

`dev_gayeon` 브랜치의 ROS 2 Jazzy · Webots R2025a 실행 안내입니다. Docker 이미지에 ROS 2와 Webots가 포함됩니다.

## 준비

- Ubuntu 데스크톱의 터미널에서 실행합니다. X11 또는 Xwayland 화면과 읽을 수 있는 X 인증 파일이 필요합니다.
- Docker Engine과 Docker Compose가 설치되어 있고, 현재 사용자로 `docker info`와 `docker compose version`이 실행되어야 합니다.
- 아래 명령은 Intel/AMD 또는 nouveau의 Mesa GPU와 `/dev/dri`를 사용합니다. NVIDIA 독점 드라이버 환경은 `LUNAR_GPU=nvidia`와 NVIDIA Container Toolkit이 필요합니다.

## 처음 다운로드하고 이미지 빌드

```bash
git clone --branch dev_gayeon --single-branch \
  https://github.com/qkrckstj/py_bt_ros.git lunar_hungarian_project
cd lunar_hungarian_project

export LUNAR_REPO="$(pwd -P)"
export LUNAR_UID="$(id -u)"
export LUNAR_GID="$(id -g)"
docker compose -f docker/lunar/compose.yaml build lunar
```

이미 작업 폴더가 있으면 그 폴더에서 실행합니다. 이미지를 빌드한 뒤에는 아래 실행 명령만 사용하면 됩니다.

## Webots와 RViz 실행

프로젝트 루트에서 실행합니다. 기존 실행이 열려 있으면 해당 터미널에서 **Ctrl+C로 종료한 뒤** 다시 실행합니다.

```bash
export LUNAR_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
test -r "$LUNAR_XAUTHORITY"

LUNAR_GPU=mesa \
LUNAR_DEPARTURE_POLICY=simultaneous \
LUNAR_RISK_STAGE=candidates \
bash docker/lunar/run_demo.sh explore-wide
```

`test -r`이 실패하면 현재 데스크톱 세션의 실제 인증 파일을 `LUNAR_XAUTHORITY`에 지정합니다. 예: `export LUNAR_XAUTHORITY=/실제/인증파일/경로`.

스크립트는 현재 소스를 빌드하고 자동 검사를 통과한 뒤 Webots와 RViz를 실행합니다. 종료는 **Ctrl+C**입니다.

| 마지막 인자 | 실행 내용 |
| --- | --- |
| `bt-fixed` | 4대 고정 목표 주행 |
| `explore` | 400 m² 영역 단기 탐사, 최대 시뮬레이션 180초 |
| `explore-wide` | 3364 m² 영역 탐사, 최대 시뮬레이션 1800초 |
| `gpu-check` | GPU 연결과 렌더러 확인 |

먼저 `bt-fixed`, 다음 `explore`, 이후 `explore-wide` 순서로 확인할 수 있습니다. 설정된 최대 시간 전에 완료 조건이나 오류로 종료될 수 있습니다.

## 실행 중 상태 확인

다른 터미널의 같은 프로젝트 루트에서 실행합니다.

```bash
bash docker/lunar/run_demo.sh status
bash docker/lunar/run_demo.sh diagnose
```

`diagnose`는 12초간 시계와 로봇 위치 입력을 확인합니다. 결과와 ROS 로그는 실행 터미널에 출력된 `results/gui_<모드>_<날짜와 시각>_<PID>/`에 저장됩니다.

Docker 접근 오류는 Docker를 사용할 수 있는 호스트 터미널에서 확인합니다. `DISPLAY` 또는 X 인증 오류는 로그인한 데스크톱 세션의 터미널에서 확인합니다. GPU 오류는 같은 환경에서 `LUNAR_GPU=mesa bash docker/lunar/run_demo.sh gpu-check`로 확인합니다.

현재 장시간 탐사는 모의 299.4초에서 대피 계산 예산 초과가 남아 있으며, 최신 mesh 코드의 실제 Webots 주행은 아직 재검증하지 못했습니다.

상세 설정은 [Docker 안내](docker/lunar/README.md), 작업 상태는 [다음 세션 안내](NEXT_SESSION.md)를 참고하세요.
