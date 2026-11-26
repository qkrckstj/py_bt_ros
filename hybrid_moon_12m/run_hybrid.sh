#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [ ! -s pairing.key ]; then
  echo '페어링 키가 없습니다. 실제 리모의 pairing.key를 먼저 가져오세요.' >&2
  exit 1
fi
export HYBRID_PHYSICAL_MODE=1 HYBRID_KEY_FILE="$PWD/pairing.key" LIMO_USER_APPROVED=YES
exec bash run_mission.sh
