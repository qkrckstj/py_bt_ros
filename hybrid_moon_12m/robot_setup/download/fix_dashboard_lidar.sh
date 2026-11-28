#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source "${LIMO_ROS_SETUP:-$HOME/wego_ws/install/setup.bash}"
config="$(ros2 pkg prefix --share ydlidar_ros2_driver)/params/ydlidar.yaml"
if pgrep -x -f '^.*[/]ydlidar_ros2_driver_node( .*)?$' >/dev/null; then
    echo '대시보드의 센서 실행을 종료한 뒤 다시 실행하세요.' >&2
    exit 1
fi
/usr/bin/python3 - "$config" <<'PY'
import re
import shutil
import sys
from pathlib import Path

path = Path(sys.argv[1])
original = path.read_text()
updated, count = re.subn(r'(?m)^([ \t]*fixed_resolution:[ \t]*)false(?=[ \t]*(?:#.*)?$)', r'\g<1>true', original)
if count != 1:
    if not count and re.search(r'(?m)^\s*fixed_resolution:\s*true\b', original):
        print('고정 해상도가 이미 적용되어 있습니다.')
        sys.exit(0)
    raise SystemExit(f'fixed_resolution 설정을 확인하세요: {path}')
backup = path.with_name(path.name + '.before_fixed_resolution')
if not backup.exists():
    shutil.copy2(path, backup)
path.write_text(updated)
print(f'fixed_resolution: true 적용 완료. 백업: {backup}')
PY
