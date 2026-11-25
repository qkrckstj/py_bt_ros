#!/usr/bin/env bash
# 컨테이너 안에서 실제 OpenGL 공급자와 렌더러를 확인한다.
set -euo pipefail
LUNAR_GRAPHICS_MODE=${1:-nvidia}
LUNAR_GRAPHICS_OUTPUT=${2:-results/graphics_check}
case "$LUNAR_GRAPHICS_MODE" in
  nvidia|mesa|software) ;;
  *) printf '%s\n' '렌더링 모드는 nvidia, mesa 또는 software여야 합니다.' >&2; exit 2 ;;
esac
mkdir -p -- "$LUNAR_GRAPHICS_OUTPUT"
dpkg-query -W libgl1-mesa-dri mesa-vulkan-drivers libvulkan1 \
  > "$LUNAR_GRAPHICS_OUTPUT/graphics_packages.txt" 2>&1 || true
if [[ $LUNAR_GRAPHICS_MODE == mesa ]] && command -v vulkaninfo >/dev/null; then
  timeout 20s vulkaninfo --summary > "$LUNAR_GRAPHICS_OUTPUT/vulkan_summary.txt" \
    2> "$LUNAR_GRAPHICS_OUTPUT/vulkan_error.log" || true
fi
if [[ $LUNAR_GRAPHICS_MODE == nvidia ]]; then
  timeout 15s nvidia-smi --query-gpu=name,driver_version --format=csv,noheader \
    | tee "$LUNAR_GRAPHICS_OUTPUT/nvidia.txt"
fi
if ! LC_ALL=C timeout 20s glxinfo -B > "$LUNAR_GRAPHICS_OUTPUT/renderer.txt" \
    2> "$LUNAR_GRAPHICS_OUTPUT/renderer_error.log"; then
  cat "$LUNAR_GRAPHICS_OUTPUT/renderer_error.log" >&2
  printf '%s\n' 'OpenGL 초기화에 실패했습니다. renderer_error.log를 확인하세요.' >&2
  exit 1
fi
cat "$LUNAR_GRAPHICS_OUTPUT/renderer.txt"
python3 - "$LUNAR_GRAPHICS_MODE" "$LUNAR_GRAPHICS_OUTPUT" <<'PY'
import json
import os
from pathlib import Path
import sys

mode, output = sys.argv[1], Path(sys.argv[2])
lines = (output / 'renderer.txt').read_text().splitlines()
def value(prefix):
    return next((line.strip()[len(prefix):].strip() for line in lines
                 if line.strip().startswith(prefix)), '')
vendor, renderer = value('OpenGL vendor string:'), value('OpenGL renderer string:')
software = any(token in renderer.lower() for token in ('llvmpipe', 'softpipe', 'swrast', 'software rasterizer'))
passed = bool(renderer) and (software if mode == 'software' else not software)
if mode == 'nvidia':
    passed = passed and vendor.lower().startswith('nvidia')
result = {'requested': mode, 'passed': passed, 'vendor': vendor,
          'renderer': renderer, 'software_renderer': software,
          'dri_prime': os.environ.get('DRI_PRIME')}
(output / 'graphics.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(result, ensure_ascii=False, indent=2))
if not passed:
    print('요청한 렌더러와 실제 OpenGL 렌더러가 다릅니다. 실행을 중단합니다.', file=sys.stderr)
    raise SystemExit(2)
PY
