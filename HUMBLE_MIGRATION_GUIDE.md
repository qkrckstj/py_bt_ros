# Humble 호환 수정 요약 및 다음 Git clone 실행 가이드

작성일: 2026-10-19

## 1. 작업한 폴더와 현재 상태

- 동료의 과제 소스에서 시작한 폴더: `/home/seoyeong/new_bt_ws/src/py_bt_ros`
- 작업 브랜치: `humble`
- `new_bt_ws`는 이번 작업에서 새로 만든 폴더가 아니다.
- 아래 수정사항과 이 가이드는 `humble` 브랜치에 함께 보관한다. 수정본 저장소는 `https://github.com/qkrckstj/py_bt_ros.git`이다.
- 과제 실행으로 켜진 Webots, RViz, ROS 노드와 카메라 구독은 사용자 요청에 따라 종료했다.

> 다음에 Git clone할 때는 수정사항이 포함된 `humble` 브랜치를 지정한다. 이후 추가로 수정한 내용은 commit/push해야 다른 clone에도 반영된다.

## 2. 주요 문제와 수정

### 핵심 원인: Humble tf2 0.25.23의 교착 버그

목표를 지정해도 로봇이 움직이지 않거나, 이동하지 않았는데 Nav2가 성공을 반환했다. 외부 `/tf`와 `/odom`은 계속 갱신됐지만 Nav2 내부 TF는 시작 직후 시각에 멈춰 있었다.

설치된 `tf2 0.25.23`의 잠금 순서 역전 문제와 일치하는 증상이었고, 공식 수정본 `0.25.24`를 과제 실행에 적용한 뒤 실제 목표 이동과 촬영·복귀가 성공했다.

- `scripts/prepare_humble_tf2.sh` 추가: apt에서 수정된 공식 Humble tf2 패키지를 내려받아 저장소의 `output/compat/tf2/`에 추출한다.
- 시스템 패키지는 교체하지 않으며, 준비 스크립트에는 sudo가 필요 없다.
- `compat.py`는 설치 버전이 `0.25.21~0.25.23`일 때만 로컬 수정 라이브러리를 선택한다.
- launch는 자식 프로세스의 `LD_LIBRARY_PATH`에 선택된 라이브러리 경로를 추가한다.
- 시스템 tf2가 `0.25.24` 이상이면 로컬 수정본을 사용하지 않는다.
- 영향을 받는 버전인데 로컬 수정본이 없으면 준비 명령을 안내하고 실행을 중단한다.

**앞서 시도한 UDP 통신 변경은 해결책이 아니었다. UDP 기본 설정과 관련 파일은 최종 코드에서 제거했다.**

근거: [공식 tf2 0.25.24 변경 내역](https://github.com/ros2/geometry2/blob/0.25.24/tf2/CHANGELOG.rst), [관련 교착 보고 #992](https://github.com/ros2/geometry2/issues/992).

### 함께 수정한 호환 문제

| 문제 | 최종 변경 |
| --- | --- |
| 설치된 launch와 소스 helper의 인자 개수가 다름 | `prepare_resources()`의 배포판 인자와 호출부를 일치시킴 |
| launch가 생성되지 않는 설정 파일을 참조함 | Nav2·ros2_control·RViz·URDF·월드·BT 설정을 실행 시 생성 |
| 제어기가 시뮬레이션 시계 준비 전에 시작해 활성화 실패 | 진행 중인 `/clock` 확인 → 두 제어기 그룹 활성화 → Nav2 시작 |
| Humble과 Jazzy의 속도 명령 타입 차이 | Humble은 Twist, Jazzy는 TwistStamped에 맞춰 설정과 remapping 선택 |
| MPPI를 가정한 설정이 설치된 Humble DWB와 맞지 않음 | 실제 FollowPath 플러그인을 확인하고 DWB/MPPI별 속도 파라미터 적용 |
| Humble에 없는 RViz 패널 사용 | 설치된 Nav2의 기본 RViz 설정을 바탕으로 과제용 설정 생성 |
| `~/ros2_ws`로 고정된 경로가 실제 `~/new_bt_ws`와 다름 | 소스 직접 실행 시 저장소 위치 자동 인식; 설치 진입점에는 설치 당시 경로 기록 |
| 복사된 시스템 launch와 소스가 달라짐 | 설치 스크립트가 저장소 소스를 매번 읽는 진입점을 설치하도록 변경 |
| 장애물 안 목표를 최대 0.5m 떨어진 위치로 대체하고 성공함 | planner의 목표 대체 허용 오차를 0.05m로 축소 |
| Snap VS Code에서 Webots 소켓 경로가 달라 연결 실패 | Webots와 외부 제어기의 임시 경로 정렬 |
| Snap의 GNOME schema 때문에 시스템 RViz 시작 실패 | 해당 환경에서 RViz에 호스트 schema 경로 전달 |

`BallJoint`를 URDF로 내보낼 수 없다는 캐스터 경고는 그대로 있는 상태에서도 이동과 과제 완료를 확인했다. 이번 이동 실패의 직접 원인으로 판단하지 않았다.

## 3. 변경·추가한 파일

| 파일 | 내용 |
| --- | --- |
| `launch/robot_launch.py` | 배포판·경로 인식, tf2 수정본 선택, 시작 순서, GUI 선택, Snap 환경 처리 |
| `scenarios/turtlebot3_photo/compat.py` **추가** | Humble/Jazzy, DWB/MPPI 및 tf2 버전별 호환 처리 |
| `scenarios/turtlebot3_photo/launch_support.py` | 실행용 설정·월드·URDF 생성 |
| `scenarios/turtlebot3_photo/nav2_overrides.yaml` | 공통 크기·속도·도착 오차, planner 허용 오차 |
| `scripts/wait_for_sim_clock.py` **추가** | 0보다 크고 실제로 진행 중인 `/clock` 확인 |
| `scripts/prepare_humble_tf2.sh` **추가** | 공식 수정 tf2 패키지를 프로젝트 내부에 준비 |
| `scripts/install_webots_launch.sh` | humble/jazzy 선택, 원본 백업, 소스 기반 진입점 설치 |
| `README.md` | Humble 설정 및 실행 방법 반영 |
| `requirements.txt` | ROS 의존성 설명을 Humble/Jazzy에 맞게 수정 |
| `docs/PHOTO_MISSION.md` | 실행 구조와 환경 설명 갱신 |
| `docs/HUMBLE_COMPATIBILITY.md` **추가** | 원인 분석·수정·실행 검증 상세 기록 |
| `HUMBLE_MIGRATION_GUIDE.md` **추가** | 현재 문서: 수정 요약과 새 clone 재현 절차 |

기존 `.bak`, `.before_humble`, `.before_humble_fix` 파일은 삭제하거나 덮어쓰지 않았다. 이들은 실행에 필요한 수정 소스가 아니라 이전 설정 백업이다.

## 4. 다음 clone 전에 수정본 보관하기

현재 폴더에서 변경 파일과 새 파일을 함께 commit해야 한다. 특히 새로 추가한 `compat.py`, `prepare_humble_tf2.sh`, `wait_for_sim_clock.py`가 빠지면 실행이 안 된다.

아래는 이후 추가 수정사항을 저장할 때 참고할 명령이다.

```bash
cd /home/seoyeong/new_bt_ws/src/py_bt_ros
git status --short

git add README.md requirements.txt HUMBLE_MIGRATION_GUIDE.md \
  docs/PHOTO_MISSION.md docs/HUMBLE_COMPATIBILITY.md \
  launch/robot_launch.py \
  scenarios/turtlebot3_photo/compat.py \
  scenarios/turtlebot3_photo/launch_support.py \
  scenarios/turtlebot3_photo/nav2_overrides.yaml \
  scripts/install_webots_launch.sh \
  scripts/prepare_humble_tf2.sh \
  scripts/wait_for_sim_clock.py

git diff --cached --stat
git commit -m "Fix Humble compatibility and tf2 deadlock for photo mission"
git push origin humble
```

`origin`에 쓰기 권한이 없다면 본인 저장소나 fork로 push하고, 다음에는 그 수정본 URL과 브랜치를 clone한다.

### Git으로 복사되지 않는 것

`output/`은 `.gitignore` 대상이다. 따라서 다음 항목은 clone만으로 생기지 않는다.

- `output/compat/tf2/`: 새 clone에서 필요하면 준비 스크립트를 다시 실행한다.
- `output/captures/`: 기존 사진이 필요하면 별도로 복사한다.
- `/tmp/` 아래 실행 로그·임시 월드·임시 설정: 소스 launch가 다시 생성하는 실행 자료다.
- `/opt/ros/...`에 설치한 진입점: 패키지 명령을 쓰려면 새 저장소 경로에서 설치 스크립트를 다시 실행한다.

## 5. 새로 clone한 뒤 실행하기

### 전제

ROS 2 Humble, Webots, `webots_ros2_turtlebot`, Nav2, RViz 및 README의 ROS/Python 의존성이 준비된 환경을 기준으로 한다. Git clone이나 `requirements.txt` 설치만으로 시뮬레이터와 ROS 환경 전체가 설치되지는 않는다.

### ① 수정본이 push된 브랜치 clone

아래는 현재 수정본 저장소와 `humble` 브랜치를 받는 명령이다. 다른 저장소나 브랜치에 수정본을 보관했다면 URL과 브랜치 이름을 바꾼다. 예시는 기존 `new_bt_ws`와 별도 경로를 사용한다.

```bash
mkdir -p ~/humble_photo_ws/src
cd ~/humble_photo_ws/src
git clone --branch humble https://github.com/qkrckstj/py_bt_ros.git py_bt_ros
cd py_bt_ros
source /opt/ros/humble/setup.bash
```

### ② 설치된 tf2 버전 확인

```bash
dpkg-query -W -f='${Version}\n' ros-humble-tf2
```

- `0.25.21~0.25.23`이면 다음 준비 명령을 실행한다.
- `0.25.24` 이상이면 준비를 생략할 수 있다.

```bash
bash scripts/prepare_humble_tf2.sh
```

이 스크립트에는 ROS apt 저장소의 패키지 목록과 네트워크 연결이 필요하다. 수정된 패키지를 찾을 수 없다면 apt 저장소 설정과 패키지 목록 갱신 상태를 확인한다.

### ③ 소스 launch로 실행

저장소 루트에서 실행한다. Python 소스 직접 실행 방식이므로 이 과제 저장소 자체를 위한 별도 `colcon build`는 필요 없다.

```bash
source /opt/ros/humble/setup.bash
ros2 launch ./launch/robot_launch.py nav:=true
```

이 방식은 `/opt/ros`의 launch 파일을 교체할 필요가 없다. 영향을 받는 tf2 버전이면 시작 로그에 `Using fixed Humble tf2 from ...`가 출력된다.

기존 `PY_BT_ROS_ROOT` 환경 변수가 다른 저장소를 가리키면 그 값이 우선한다. 새 경로로 갱신하거나 해당 변수를 해제한 뒤 실행한다.

### ④ 카메라 구독 유지

별도 터미널에서 실행하고 촬영이 끝날 때까지 유지한다.

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /TurtleBot3Burger/front_camera/image_color \
  sensor_msgs/msg/Image --qos-reliability best_effort --no-arr
```

### ⑤ RViz에서 목표 지정

1. Webots가 재생 중이고 Nav2 준비가 끝났는지 확인한다.
2. RViz Fixed Frame은 `map`을 사용한다.
3. 위치 추정이 지도와 다르면 `2D Pose Estimate`로 위치와 방향을 맞춘다.
4. 지도상 이동 가능한 빈 공간에 목표를 지정한다.

| 도구 | 동작 |
| --- | --- |
| `Nav2 Goal` | Nav2에 이동만 요청 |
| `2D Goal Pose` | `/bt/goal_pose`로 과제 시작: 이동 → 검사 → 촬영 → 복귀 → 검사 |

과제 완료 상태는 다음 명령으로 확인한다.

```bash
ros2 topic echo /bt/mission_status --qos-durability transient_local
```

최종 `stage`가 `COMPLETE`인지 확인한다. 외부 카메라 구독이 없으면 촬영에 실패하며, 과제 설계상 자동 복귀하지 않는다.

### 선택: 기존 패키지 launch 명령 사용

새로 clone한 저장소 루트에서 한 번 설치한다. 관리자 권한이 필요한 명령이다.

```bash
sudo bash scripts/install_webots_launch.sh humble
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
```

설치 시 기존 파일의 백업 이름은 `robot_launch.py.before_py_bt_ros`다. 저장소를 옮겼다면 설치를 다시 하거나 `PY_BT_ROS_ROOT`를 새 위치로 지정한다. Jazzy에서는 배포판 인자를 `jazzy`로 바꾼다.

## 6. 확인한 결과와 사진 위치

최종 수정본의 전체 launch에서 다음을 확인했다.

- Nav2 액션 목표 `(0.5, 0)`에 실제 이동 후 성공했다.
- 과제 토픽 목표 `(0, -0.5)`에 이동·도착 검사·PNG 저장·복귀·복귀 검사를 수행하고 `COMPLETE`를 받았다.
- 최종 도착 오차: 거리 약 `0.127m`, 헤딩 약 `0.139rad`.
- 최종 복귀 오차: 거리 약 `0.113m`, 헤딩 약 `0.139rad`.
- PNG는 `320×240`이며 실제 장면이 저장된 것을 확인했다.
- 실제 Jazzy 실행은 검증하지 않았다. Jazzy 설정 분기는 정적·가상 설정 검사 범위다.

현재 컴퓨터 사진 폴더:

```text
/home/seoyeong/new_bt_ws/src/py_bt_ros/output/captures/
```

확인한 사진:

- `20261019_163109_380052_420b390f.png`
- `20261019_163537_805191_a5f8c7d3.png`
- `20261019_163719_139966_6875c341.png`

새 clone에서 찍은 사진은 **그 새 저장소의** `output/captures/`에 저장된다. launch의 `output_dir` 인자로 다른 위치를 지정할 수도 있다.

## 7. 종료

launch 터미널에서 `Ctrl+C`를 누른다. 별도 카메라 구독 터미널도 `Ctrl+C`로 종료한다.

상세 원인·검증 기록: [docs/HUMBLE_COMPATIBILITY.md](docs/HUMBLE_COMPATIBILITY.md).
