#!/usr/bin/env python3
"""Docker 연결 없이 가장 최근 탐사 실행의 저장된 관측률을 표시한다."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import time


RUN_NAME = re.compile(r'^gui_(?:explore|explore-wide|bt-fixed)_(\d{8}_\d{6})_(\d+)$')


def latest_run(results):
    runs = []
    if results.is_dir():
        for path in results.iterdir():
            match = RUN_NAME.fullmatch(path.name)
            if path.is_dir() and match:
                runs.append((match.group(1), int(match.group(2)), path))
    if not runs:
        raise ValueError('탐사 실행 기록이 없습니다. 먼저 bt-fixed, explore 또는 explore-wide를 실행하세요.')
    return max(runs, key=lambda item: item[:2])[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='특정 실행 폴더 또는 세션 폴더를 확인합니다.')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    try:
        run = args.output if args.output is not None else latest_run(root / 'results')
        runtime = run / 'runtime.json'
        if not runtime.is_file():
            metadata_path = run / 'latest.json'
            if not metadata_path.is_file():
                runtime=run/'startup_status.json'
                if not runtime.is_file():
                    raise ValueError(f'최신 실행의 임무 기록이 아직 없습니다: {run.name}\n빌드·초기화가 끝난 뒤 다시 확인하세요.')
            else:
                metadata = json.loads(metadata_path.read_text())
                # 컨테이너의 /workspace 경로를 같은 실행 폴더의 세션 이름으로 연결한다.
                session = Path(metadata['directory']).name
                if session in ('', '.', '..'):
                    raise ValueError('실행 기록의 세션 경로가 올바르지 않습니다.')
                runtime = run / session / 'runtime.json'
        state = json.loads(runtime.read_text())
        observed = state['observation']
        age = max(0., time.time() - runtime.stat().st_mtime)
        updated = datetime.fromtimestamp(runtime.stat().st_mtime).strftime('%H:%M:%S')
        print(f"탐사율: {100 * observed['coverage_fraction']:.2f}%")
        print(f"관측 면적: {observed['observed_area_m2']:.2f} / {observed['denominator_m2']:.2f} m²")
        print(f"상태: {state['phase']} | 시뮬레이션: {state['sim_time']:.2f}초")
        if state['phase'] == 'ERROR':
            print(f"오류 사유: {state.get('termination_reason') or state.get('reason', '기록 없음')}")
        elif state['phase'] in ('WAITING','STOPPED'):
            print(f"사유: {state.get('reason', '기록 없음')}")
        startup=state.get('runtime_inputs',{}).get('startup')
        if startup and state['phase']=='WAITING':
            print(f"입력 안정화: 실제 {startup['wall_elapsed_seconds']:.1f} / {startup['wall_required_seconds']:.1f}초"
                  f" | 시계 진행 {startup['sim_elapsed_seconds']:.2f} / {startup['sim_required_seconds']:.2f}초")
        initial=state.get('initial_observation')
        if initial and state['phase']=='BOOTSTRAP_OBSERVING':
            counts=', '.join(f'{name} {value}' for name,value in initial['frames_by_robot'].items())
            print(f"초기 관측: 시뮬레이션 {initial['sim_elapsed_seconds']:.2f} / {initial['sim_required_seconds']:.2f}초"
                  f" | 유효 스캔: {counts} (각 {initial['minimum_frames']}개 이상)")
        planning=state.get('planning')
        if planning and state['phase']=='PLANNING':
            print(f"계산: 실제 {planning['wall_elapsed_seconds']:.1f} / {planning['wall_budget_seconds']:.1f}초"
                  f" | 출발 예정: 시뮬레이션 {planning['t0']:.2f}초")
        print(f'마지막 저장: {updated} ({age:.0f}초 전) | 실행: {run.name}')
        if age > 5.:
            print('저장된 값이 갱신되지 않고 있습니다. 실행 중인 임무인지 확인하세요.')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f'탐사율 확인: {exc}')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
