# 탐사 코드 출처

로컬 `chanseo_review_20261124`의 dev_chanseo 커밋
`02cda62197e30858f295de25567003f02242ad64`에서 복사했습니다.

- exploration_bt.py, exploration_policy.py, exploration_sweep.py: 원본 그대로.
- exploration_runtime.py: 기본 greedy 할당은 유지하고, 실제/가상 담당 구역을 검사하기 위한 allocator 주입 지점만 추가했습니다.
- mission.py는 이 패키지만 사용하며 서영 BT와 CP-SAT 모듈은 불러오지 않습니다.

원격 브랜치 최신 상태를 새로 조회한 결과가 아닌, 기존 로컬 스냅샷 기준입니다.
