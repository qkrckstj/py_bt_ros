#!/usr/bin/env python3
"""Launch live demonstrations with Webots/RViz, or compare two fresh runs."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def display_ready():
    display = os.environ.get('DISPLAY', '')
    if not display:
        raise RuntimeError('DISPLAY is unset. Start this launcher from the desktop terminal.')
    if display.startswith(':'):
        path = '/tmp/.X11-unix/X' + display[1:].split('.')[0]
        with socket.socket(socket.AF_UNIX) as connection:
            try:
                connection.connect(path)
            except OSError as exc:
                raise RuntimeError(f'Cannot access desktop display {display}: {exc}. '
                                   'Open the desktop launcher from your logged-in desktop session.') from exc


def stop(process):
    if process.poll() is not None:
        return
    # Signal only this launch's process group. Never terminate unrelated ROS jobs.
    for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 8), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass


def run(method, directory, seconds=None):
    log = directory / f'{method}.log'
    print(f'Launching {method}: Webots + RViz. Log: {log}', flush=True)
    started, ready_at, latest, last_report = time.time(), None, {}, 0.
    report = REPO / f'artifacts/lunar_chanseo_cp_sat/live_status_{method}.json'
    with log.open('w') as stream:
        process = subprocess.Popen([str(HERE/'run.sh'), f'method:={method}',
                                    'path_planner:=auto', 'rviz:=true'],
                                   cwd=REPO, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            while process.poll() is None:
                try:
                    if report.stat().st_mtime >= started:
                        latest = json.loads(report.read_text())
                        if latest.get('inputs_ready') and ready_at is None:
                            ready_at = time.monotonic()
                        if time.monotonic()-last_report > 5.:
                            last_report = time.monotonic()
                            observed = latest.get('coverage', {}).get('observed_percent', 0.)
                            print(f"{method}: {latest.get('phase')} | observed {observed:.2f}% | "
                                  f"travel {latest.get('travel_m', {})}", flush=True)
                except (OSError, ValueError):
                    pass
                if seconds is not None and ready_at is not None and time.monotonic()-ready_at >= seconds:
                    return latest
                if seconds is not None and ready_at is None and time.time()-started > 180.:
                    raise RuntimeError(f'{method} did not become ready in 180 seconds. Check {log}')
                time.sleep(.5)
            if process.returncode:
                raise RuntimeError(f'{method} exited with {process.returncode}. Check {log}')
            if seconds is not None:
                raise RuntimeError(f'{method} stopped before the comparison interval ended. Check {log}')
            return latest
        finally:
            stop(process)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('cp_sat', 'space_astar', 'compare'))
    parser.add_argument('--seconds', type=float, default=120., help='Ready runtime per comparison round')
    args = parser.parse_args()
    if not args.seconds > 0:
        parser.error('--seconds must be positive')
    display_ready()
    directory = REPO/'artifacts'/('lunar_live_demo_'+time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir(parents=True)
    if args.mode != 'compare':
        run(args.mode, directory)
        return
    results = {'scope': 'Two fresh live simulations, same initial world; observation is not full coverage',
               'seconds_per_ready_round': args.seconds, 'runs': {}}
    for method in ('cp_sat', 'space_astar'):
        results['runs'][method] = run(method, directory, args.seconds)
        (directory/'comparison.json').write_text(json.dumps(results, indent=2)+'\n')
    print(f'Both live demonstrations finished. Results: {directory / "comparison.json"}')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nDemonstration stopped.')
    except (RuntimeError, OSError) as exc:
        print(f'Could not start demonstration: {exc}', flush=True)
        raise SystemExit(1)
