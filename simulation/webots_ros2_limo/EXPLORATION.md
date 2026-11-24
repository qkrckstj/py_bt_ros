# 4대 LIMO 미지 지도 탐사 (시뮬레이션)

이 패키지는 Webots의 4대 LIMO가 같은 기지에서 출발해 각자 SLAM으로 지도를 만들고, 팀 BT가 합쳐진 지도의 탐사 목표를 배정하는 데모다. 실제 LIMO 연결·실측 지도·하드웨어 주행 코드는 포함하지 않는다.

- **팀 BT** (`exploration_runtime.py:TeamBT`): 4대 상태와 공유 지도 입력 확인 → 기지까지의 다중 홉 통신 확인 → 탐사 완료 판정 → 목표 할당 또는 레이더식 스윕 → 지도 갱신 중단 → 기지 복귀. 할당은 관측된 자유 공간에서 미지 영역과 맞닿은 frontier를 찾고, 경로 거리·목표 간격·작업 구역 예약·통신 연결 조건을 고려한다. `moon_radar`에서는 기지에서 바깥으로 펼친 편대로 원형 영역을 훑는 전략을 사용한다.
- **개별 BT** (`exploration_runtime.py:RobotBT`): 임무 수신 → 통신/명령 유효성 확인 → Nav2 목표 주행 → 도착 후 관측 → 결과 보고. 팀 BT가 각 로봇에 다음 목표를 보내며, Nav2가 주행과 근거리 장애물 회피를 맡는다.
- **지도/통신**: 로봇별 `slam_toolbox` 지도 4장을 공유 `/map`으로 합친다. 통신 모형은 로봇 또는 기지와 **중심 간 거리**가 `radio_range` 이내이면 연결되며, 다른 로봇을 통한 기지 연결도 허용한다. 원과 연결선을 Webots에 표시한다. 이는 거리 기반 시뮬레이션이며 실제 무선 품질 측정이 아니다.

## 실행

ROS 2 Jazzy 및 Webots R2025a 환경에서, 이 저장소를 다른 `webots_ros2_limo` 체크아웃과 겹치지 않는 별도 ROS 작업 공간에 두고 빌드한다.

```bash
cd <workspace>
colcon build --packages-select webots_ros2_limo --symlink-install
source install/setup.bash
ros2 launch webots_ros2_limo exploration_flat_launch.py world:=moon_radar radio_enabled:=true radio_range:=11.0
```

기본 `branched` 맵도 실행할 수 있다. 이미 4대의 `/robotN/map`, Nav2, TF가 동작 중이라면 `exploration_allocation_launch.py`로 BT만 연결할 수 있다. RViz와 원래 `py_bt_ros/modules/bt_visualiser.py` 형식의 Pygame BT 창은 기본적으로 함께 열린다. 시각화만 다시 열 때는 `exploration_visualization_launch.py bt_repo:=<py_bt_ros 절대경로>`를 사용한다. `bt_viewer:=false`로 Pygame을 끌 수 있다.

이 구현은 그림자 위험 회피나 실제 RF 전파 모델을 포함하지 않는다. 달 지형·텍스처 출처와 라이선스는 `resource/moon_map_release/`의 고지를 참조한다.
