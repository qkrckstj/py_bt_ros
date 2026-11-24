#!/usr/bin/env python3
"""Start, inspect or stop this checkout's lunar demonstration."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
STATUS = REPO/'artifacts/lunar_chanseo_cp_sat/live_status.json'


def processes():
    found = []
    scripts = {HERE/'bridge.py', HERE/'map_visualizer.py', HERE/'lunar.launch.py'}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            args = entry.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
            cwd = entry.joinpath('cwd').resolve()
            if not args or not args[0]:
                continue
            kind = None
            for arg in args[1:]:
                if arg.endswith(('/bridge.py', '/map_visualizer.py', '/lunar.launch.py')):
                    path = Path(arg) if arg.startswith('/') else cwd/arg
                    if path.resolve() in scripts:
                        kind = path.name
            if Path(args[0]).name == 'rviz2' and '-d' in args:
                config = Path(args[args.index('-d')+1])
                if config.is_relative_to(REPO) or str(config).startswith('/tmp/lunar_exploration_rviz_'):
                    kind = 'rviz2'
            if kind:
                found.append((int(entry.name), kind))
        except (OSError, ValueError, IndexError):
            continue
    return found


def status():
    running = processes()
    print('실행 프로세스:', ', '.join(f'{name} PID={pid}' for pid,name in running) or '없음')
    if not STATUS.exists():
        print('아직 상태 기록이 없습니다.')
        return
    try:
        data = json.loads(STATUS.read_text())
    except (OSError, ValueError):
        print('상태 파일 갱신 중입니다. 잠시 뒤 다시 확인하세요.')
        return
    coverage = data.get('coverage', {})
    age = time.time()-STATUS.stat().st_mtime
    print(f"관측률: {coverage.get('observed_percent', 0.):.2f}% / 60×60m")
    print(f"상태: {data.get('phase')} | 입력 준비: {data.get('inputs_ready')} | 기록 나이: {age:.1f}초")
    print(f"방식: {data.get('allocator')} / {data.get('path_planner')} | 시뮬레이션 시간: {data.get('sim_time')}초")
    print('로봇별 누적 이동(m):', data.get('travel_m', {}))
    print('이동 허가:', data.get('motion_permits', []), '| 차단:', data.get('physical_blocked', []))
    if not running or age > 15.:
        print('주의: 과거 실행의 기록일 수 있습니다. 위 실행 프로세스와 기록 나이를 함께 확인하세요.')


def stop():
    current = processes()
    if not current:
        print('이 작업 폴더의 달 탐사 프로세스가 없습니다.')
        return
    # Ask task owners to cancel navigation before stopping the simulator.
    current.sort(key=lambda pair: pair[1] != 'bridge.py')
    for pid,name in current:
        try:
            os.kill(pid, signal.SIGINT)
            print('종료 요청:', name, pid, flush=True)
        except ProcessLookupError:
            pass
    deadline = time.monotonic()+30.
    while time.monotonic() < deadline:
        if not processes():
            print('달 탐사와 RViz를 종료했습니다.')
            return
        time.sleep(.5)
    raise RuntimeError('아직 종료 중인 프로세스가 있습니다. status로 확인하세요. 강제 종료하지 않았습니다.')


def start(method):
    if processes():
        status()
        print('기존 실행이 있어 중복 시작하지 않았습니다. 새 실험은 먼저 stop을 실행하세요.')
        return
    planner = 'astar'
    process = subprocess.Popen([str(HERE/'run.sh'), f'method:={method}',
                                f'path_planner:={planner}', 'rviz:=true'],
                               cwd=REPO, start_new_session=True)
    try:
        code = process.wait()
        if code:
            raise RuntimeError(f'실행이 종료 코드 {code}로 끝났습니다. 위 터미널 로그를 확인하세요.')
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=30.)
        except subprocess.TimeoutExpired:
            print('종료 중입니다. 다른 터미널에서 manage.py status로 확인하세요.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('start', 'status', 'stop'))
    parser.add_argument('--method', choices=('cp_sat', 'space_astar'), default='cp_sat')
    parser.add_argument('--watch', action='store_true', help='Refresh status every three seconds')
    args = parser.parse_args()
    if args.command == 'start':
        start(args.method)
    elif args.command == 'stop':
        stop()
    else:
        while True:
            status()
            if not args.watch:
                break
            print()
            time.sleep(3.)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(exc)
        raise SystemExit(1)
