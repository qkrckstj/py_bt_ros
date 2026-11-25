#!/usr/bin/env python3
"""새 체크아웃의 실제 Webots launch를 제한 시간 실행하고 ROS 증거를 저장한다.

Xvfb 또는 X11과 빌드한 Jazzy overlay가 필요하다. pytest 자동 수집 대상 아님.
시각화 모드도 화면을 사람이 확인한 것으로 취급하지 않는다.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as PathMsg
from std_msgs.msg import String, Bool
from visualization_msgs.msg import MarkerArray
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['visualization', 'single', 'multi'], default='visualization')
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    share = Path(__file__).resolve().parents[1]
    config = share / 'config' / ('shadow_single.yaml' if args.mode == 'single' else 'shadow_experiment.yaml')
    settings = yaml.safe_load(config.read_text())
    selected = settings['robots']
    env = yaml.safe_load((share / 'config/environment.yaml').read_text())
    cmd = ['ros2', 'launch', 'moon_mapping_demo', 'moon_shadow.launch.py',
           'rviz:=' + ('true' if args.mode == 'visualization' else 'false'),
           'execute:=' + ('false' if args.mode == 'visualization' else 'true'),
           'config:=' + str(config), 'output:=' + str(output)]
    log_path = output / 'launch.log'
    rclpy.init()
    probe = Node('webots_shadow_validation')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    statuses, paths, odoms, shadows, markers = [], {}, {}, [], []
    odom_received, shadow_counts = {}, {}

    def odom(robot, msg):
        odoms[robot] = msg
        odom_received[robot] = time.monotonic()

    def shadow(robot, msg):
        shadows.append((robot, msg.data))
        shadow_counts[robot] = shadow_counts.get(robot, 0) + 1
    probe.create_subscription(String, '/shadow_mission/status', lambda msg: statuses.append(json.loads(msg.data)), qos)
    for r in env['starts']:
        probe.create_subscription(Odometry, f'/{r}/odom', lambda msg, r=r: odom(r, msg), qos_profile_sensor_data)
        probe.create_subscription(Bool, f'/{r}/in_terrain_shadow', lambda msg, r=r: shadow(r, msg), 10)
    for r in selected:
        probe.create_subscription(PathMsg, f'/{r}/shadow_path', lambda msg, r=r: paths.update({r: msg}), qos)
    probe.create_subscription(MarkerArray, '/shadow_mission/markers', lambda msg: markers.append(len(msg.markers)), qos)
    success, error = False, 'timeout'
    start = time.monotonic()
    positions = {}
    goal_errors = {}
    minimum_distance = math.inf
    with log_path.open('w') as log:
        process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while time.monotonic() - start < args.timeout:
                rclpy.spin_once(probe, timeout_sec=.05)
                if process.poll() is not None:
                    error = f'launch exited {process.returncode}'
                    break
                if odoms:
                    for r, msg in odoms.items():
                        p, s = msg.pose.pose.position, env['starts'][r]
                        positions[r] = (s['x'] + math.cos(s['yaw']) * p.x - math.sin(s['yaw']) * p.y,
                                        s['y'] + math.sin(s['yaw']) * p.x + math.cos(s['yaw']) * p.y)
                    names = list(positions)
                    for i, a in enumerate(names):
                        for b in names[i + 1:]:
                            minimum_distance = min(minimum_distance, math.dist(positions[a], positions[b]))
                phase = statuses[-1]['phase'] if statuses else ''
                if args.mode != 'visualization':
                    if any(value for robot, value in shadows if robot in selected):
                        error = '선택 로봇의 지형 그림자 진입 관측'
                        break
                    if minimum_distance <= 2 * settings['planner']['robot_radius']:
                        error = '대기 로봇을 포함한 몸체 간격 위반 관측'
                        break
                if phase in ('ERROR', 'STOPPED'):
                    error = statuses[-1]['reason']
                    break
                if args.mode == 'visualization':
                    ready = phase == 'VISUALIZED' and set(paths) == set(selected) and markers and len(odoms) == len(env['starts'])
                    log_text = log_path.read_text()
                    if ready and 'rviz2' in log_text and 'Stereo is NOT SUPPORTED' in log_text:
                        success, error = True, None
                        break
                elif phase == 'COMPLETE':
                    if set(paths) != set(selected) or set(positions) != set(env['starts']):
                        error = '완료 상태지만 경로 또는 전체 로봇 odom 누락'
                        break
                    wall_timeout = settings['execution'].get('input_wall_timeout', settings['execution']['odom_timeout'])
                    if any(time.monotonic() - odom_received[r] > wall_timeout
                           for r in env['starts']):
                        error = '완료 상태지만 odom 관측이 오래됨'
                        break
                    if any(not shadow_counts.get(r) for r in selected):
                        error = '선택 로봇의 그림자 관측 누락'
                        break
                    try:
                        plan = json.loads((output / 'execution_plan.json').read_text())
                        goals = plan['assigned_goals']
                        goal_errors = {r: math.dist(positions[r], goals[r]) for r in selected}
                    except (OSError, ValueError, KeyError, TypeError) as exc:
                        error = '완료 상태지만 배정 목표 기록이 유효하지 않음: ' + str(exc)
                        break
                    success = all(d <= settings['execution']['goal_tolerance'] for d in goal_errors.values())
                    error = None if success else '완료 상태와 실제 odom 목표 도착 오차 불일치'
                    break
        finally:
            if process.poll() is None:
                # launch가 자식에게 종료 신호를 전달한다. 프로세스 그룹에 같은
                # SIGINT를 중복 전달해 정상 종료 중 KeyboardInterrupt를 만들지 않는다.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=12.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
    report = {'mode': args.mode, 'passed': success, 'error': error,
              'wall_seconds': time.monotonic() - start, 'selected_robots': selected,
              'status_history': statuses, 'odom_robot_ids': list(odoms),
              'path_robot_ids': list(paths), 'marker_counts': markers,
              'last_positions_from_odom': positions,
              'goal_errors_from_odom_m': goal_errors,
              'shadow_message_counts': shadow_counts,
              'minimum_interrobot_distance_from_odom': minimum_distance if math.isfinite(minimum_distance) else None,
              'any_reported_terrain_shadow': any(v for r, v in shadows if r in selected),
              'screen_visually_inspected': False, 'command': cmd}
    (output / 'validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'status_history'}, ensure_ascii=False, indent=2))
    probe.destroy_node()
    rclpy.shutdown()
    return 0 if success else 2


if __name__ == '__main__':
    raise SystemExit(main())
