#!/usr/bin/env bash
set -eo pipefail
CITY_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$CITY_REPO"
# Override CITY_RL_PYTHON when using your own environment with requirements installed.
CITY_RL_PYTHON="${CITY_RL_PYTHON:-$CITY_REPO/.venv/bin/python}"
export PYTHONPATH="${CITY_EXTRA_PYTHONPATH:-}:$CITY_REPO:${PYTHONPATH:-}"
exec "$CITY_RL_PYTHON" -m disaster_city.rl.train "$@"
