# MRTA - Greedy Task Allocation

## 구현 기능
- TF(map 좌표계)로 LIMO 3대의 실제 위치 수신
- Greedy 알고리즘으로 로봇별 순찰 목표 할당
- /multi_bt/patrol_tasks 토픽으로 BT에 할당 결과 전달

## 실행 방법
Webots + Nav2가 실행 중인 상태에서:

source /opt/ros/jazzy/setup.bash
cd /root/py_bt_ros/scenarios/limo_factory_search_mrta
python3 patrol_publisher.py --ros-args -p use_sim_time:=true

이후 별도로 multirobot_bt를 실행하고
/multi_bt/start 서비스를 호출하면 이동을 시작합니다.

## 테스트 결과
- LIMO 3대 실제 좌표 수신 성공
- Greedy 업무 할당 및 BT 메시지 전달 성공
- LIMO 3대 이동 확인
- Nav2 status=4(SUCCEEDED) 확인

## 참고 사항
- 현재 작업은 한 번 할당하며, 동적 재할당은 미구현
- 장애물을 고려한 실제 경로 비용이 아닌 직선거리 기반 Greedy
- Webots 실행 속도 저하 현상 존재
- 같은 토픽의 중복 MRTA 발행자 실행 금지
