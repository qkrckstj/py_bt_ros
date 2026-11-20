# 원본과 출처

## Webots
달 지면 텍스처는 Cyberbotics Webots R2025a 공식 예제에서 가져왔습니다.
https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/demos/worlds/textures/moon/moon_ground.jpg

Webots 저장소: https://github.com/cyberbotics/webots
Apache-2.0 라이선스 전문을 LICENSES/Apache-2.0.txt에 포함했습니다. 개별 자산에 별도 고지가 있는 경우 해당 원본 고지가 우선합니다.

## LIMO 모델
LimoFourDiff.proto와 limo_base.dae, limo_wheel.dae는 로컬 py_bt_ros_multi/ros_packages/multirobot_sim/protos에서 복사했습니다. PROTO의 Apache-2.0 헤더와 메시의 원본 메타데이터를 유지했습니다. 이 데모에서는 카메라 하향 각도와 해상도를 변경했습니다.
상위 저장소: https://github.com/qkrckstj/py_bt_ros
상위 저장소의 GPL-3.0 라이선스 전문도 LICENSES/upstream-repository-GPL-3.0.txt에 보존했습니다. 이 고지는 모든 파일의 라이선스를 하나로 재지정한다는 의미가 아닙니다.

## 참고한 개념
https://mrsdprojects.ri.cmu.edu/2018teami/implementation/software-system/
현재 태양 제어는 일정 고도의 가상 회전 조명이며, 해당 연구의 원본 코드나 SPICE 데이터를 포함하지 않습니다.
