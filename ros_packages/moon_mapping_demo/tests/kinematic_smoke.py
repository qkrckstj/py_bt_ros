#!/usr/bin/env python3
"""실제 WBT와 ROS 실행기의 제어 판단을 이상적인 차동 구동 모델로 검사한다.

ROS/DDS, Webots 물리, 바퀴 미끄러짐과 오도메트리 오차를 검증하지 않는다.
계획·충돌 검사·시간 추종·지연 재계획을 화면 없이 재현하는 보조 검사다.
"""
import argparse
import json
import math
from pathlib import Path
import time

from moon_mapping_demo.coordination import coordinate, moving_discs_collide
from moon_mapping_demo.execution import control_step
from moon_mapping_demo.experiment import load_experiment, make_field, path_data, plan_experiment


def integrate(pose, control, dt):
    x, y, yaw = pose
    turn = control.angular * dt
    if abs(control.angular) > 1e-12:
        x += control.linear / control.angular * (math.sin(yaw + turn) - math.sin(yaw))
        y += control.linear / control.angular * (math.cos(yaw) - math.cos(yaw + turn))
    else:
        x += control.linear * math.cos(yaw) * dt
        y += control.linear * math.sin(yaw) * dt
    return x, y, yaw + turn


def run(config, output, timeout=180., start_sim=0.):
    before = time.monotonic()
    cfg, env, pcfg, share = load_experiment(config)
    field = make_field(cfg, env, pcfg, share)
    settings = cfg['execution']
    poses = {r: tuple(s[k] for k in ('x', 'y', 'yaw')) for r, s in env['starts'].items()}
    result = plan_experiment(cfg, env, pcfg, field, start_sim + settings['lead_seconds'], output=output)
    paths, goals = result['paths'], result.get('assigned_goals', {})
    events, replans, now = [], 0, 0.
    error = result['error']
    success = False
    minimum = min(math.dist(poses[a][:2], poses[b][:2])
                  for i, a in enumerate(poses) for b in list(poses)[i + 1:])
    trace = []
    dt = settings['control_period']
    for step in range(math.ceil(timeout / dt) + 1):
        now = start_sim + step * dt
        if not paths or error:
            break
        controls, reason = control_step(field, paths, poses, now, settings,
                                        min(.22, pcfg.speed * 1.2), pcfg.robot_radius)
        if reason:
            events.append({'sim_time': now, 'reason': reason})
            if replans >= settings['max_replans']:
                error = '재계획 횟수 초과: ' + reason
                break
            replans += 1
            starts = {r: p[:2] for r, p in poses.items()}
            standby = [(p[0], p[1], pcfg.robot_radius) for r, p in poses.items() if r not in goals]
            try:
                paths = coordinate(field, pcfg, starts, goals, now + settings['lead_seconds'],
                                   standby, settings['goal_hold_seconds'])
            except RuntimeError as exc:
                error = str(exc)
                break
            # 계산 중에는 정지. 계산 wall time을 시뮬레이션에 더하지 않는 이상적 모델이다.
            continue
        end = max(p.points[-1].t for p in paths.values()) + settings['goal_hold_seconds']
        if all(c.arrived for c in controls.values()) and now >= end:
            success = True
            break
        next_poses = {r: integrate(p, controls[r], dt) if r in controls else p
                      for r, p in poses.items()}
        for robot, pose in poses.items():
            # 접선과 원호의 차이를 몸체 반경에 더해 한 제어 구간 전체를 검사한다.
            c = controls.get(robot)
            extra = abs(c.linear * c.angular) * dt * dt / 2 if c else 0.
            if field.blocked_segment(pose[:2], next_poses[robot][:2], now, now + dt,
                                     pcfg.robot_radius + extra):
                error = robot + ': 적분 경로가 지형·그림자 금지 영역에 진입'
                break
        for i, a in enumerate(poses):
            for b in list(poses)[i + 1:]:
                minimum = min(minimum, math.dist(next_poses[a][:2], next_poses[b][:2]))
                if moving_discs_collide(poses[a][:2], next_poses[a][:2],
                                        poses[b][:2], next_poses[b][:2], 2 * pcfg.robot_radius):
                    error = f'{a}/{b}: 적분 경로의 로봇 충돌'
        if error:
            break
        poses = next_poses
        if step % max(1, round(1. / dt)) == 0:
            trace.append({'sim_time': now + dt, 'poses': poses})
    goal_errors = {r: math.dist(poses[r][:2], goal) for r, goal in goals.items()}
    success = success and all(v <= settings['goal_tolerance'] for v in goal_errors.values())
    report = {
        'validation_kind': 'ideal_differential_drive_kinematics',
        'physical_webots_run': False, 'ros_transport_tested': False,
        'passed': success, 'error': error or (None if success else '시뮬레이션 제한 시간 초과'),
        'config': str(Path(config).resolve()), 'sim_seconds': now,
        'start_sim_seconds': start_sim,
        'wall_seconds': time.monotonic() - before, 'step_seconds': dt,
        'selected_robots': cfg['robots'], 'replans': replans, 'replan_events': events,
        'goal_errors_m': goal_errors, 'minimum_sampled_interrobot_distance_m': minimum,
        'original_assignment_total_ticks': result['assignment'].total_cost if result['assignment'] else None,
        'initial_coordinated_total_ticks': sum(p.cost_ticks for p in result['paths'].values()),
        'last_poses': poses, 'last_paths': path_data(paths),
    }
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'kinematic_validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    (output / 'kinematic_trace.json').write_text(json.dumps(trace, ensure_ascii=False, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('single', 'multi'), default='single')
    parser.add_argument('--timeout-sim', type=float, default=180.)
    parser.add_argument('--start-sim', type=float, default=0., help='초기 관측 시뮬레이션 시각')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_sim) or args.timeout_sim <= 0:
        parser.error('--timeout-sim은 양의 유한 초여야 합니다.')
    if not math.isfinite(args.start_sim) or args.start_sim < 0:
        parser.error('--start-sim은 음수가 아닌 유한 초여야 합니다.')
    share = Path(__file__).resolve().parents[1]
    config = share / 'config' / ('shadow_single.yaml' if args.mode == 'single' else 'shadow_experiment.yaml')
    report = run(config, args.output, args.timeout_sim, args.start_sim)
    print(json.dumps({k: v for k, v in report.items() if k not in ('last_paths', 'last_poses')},
                     ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
