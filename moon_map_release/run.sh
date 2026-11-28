#!/usr/bin/env bash
set -euo pipefail
MAP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if command -v webots >/dev/null 2>&1; then
    WEBOTS_BIN="$(command -v webots)"
elif [[ -n "${WEBOTS_HOME:-}" && -x "$WEBOTS_HOME/webots" ]]; then
    WEBOTS_BIN="$WEBOTS_HOME/webots"
elif [[ -x /usr/local/webots/webots ]]; then
    WEBOTS_BIN=/usr/local/webots/webots
else
    echo 'Webots not found. Add webots to PATH or set WEBOTS_HOME.' >&2
    exit 1
fi
# Minimal project-local preferences; never copy personal preferences or change them.
export XDG_CONFIG_HOME="$MAP_ROOT/.runtime/config"
mkdir -p "$XDG_CONFIG_HOME/Cyberbotics"
cat > "$XDG_CONFIG_HOME/Cyberbotics/Webots-R2025a.conf" <<'CONFIG'
[OpenGL]
disableShadows=false
CONFIG
exec "$WEBOTS_BIN" --mode=realtime "$MAP_ROOT/worlds/moon_terrain.wbt" "$@"
