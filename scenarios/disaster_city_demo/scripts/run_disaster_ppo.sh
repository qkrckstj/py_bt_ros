#!/usr/bin/env bash
set -eo pipefail
CITY_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ "$#" -lt 1 ]; then
  echo 'Usage: bash scripts/run_disaster_ppo.sh MODEL.zip [MEDIA_DIRECTORY]'
  exit 2
fi
export CITY_PPO_MODEL="$(realpath "$1")"
export CITY_SCENARIO_DIR="${CITY_SCENARIO_DIR:-$CITY_REPO/disaster_city/profiles/junyong_10_damage}"
export CITY_RUNTIME_PYTHON="${CITY_RL_PYTHON:-$CITY_REPO/.venv/bin/python}"
export PYTHONPATH="${CITY_EXTRA_PYTHONPATH:-}:$CITY_REPO:${PYTHONPATH:-}"
if [ "$#" -ge 2 ]; then
  mkdir -p "$2"
  export CITY_MEDIA_DIR="$(realpath "$2")"
fi
exec bash "$CITY_REPO/scripts/run_disaster_city.sh"
