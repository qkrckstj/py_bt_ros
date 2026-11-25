"""ROS 없이 같은 t0의 비용 행렬·Hungarian·공동 경로를 재현하는 CLI."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time

import yaml

from .coordination import coordinate
from .exchange import write_problem
from .hungarian import solve, InfeasibleAssignment
from .space_time import PlannerConfig, SpaceTimePlanner
from .terrain import TerrainModel


def load_experiment(config_path, share=None):
    config_path = Path(config_path).resolve()
    share = Path(share) if share else config_path.parent.parent
    cfg = yaml.safe_load(config_path.read_text())
    env = yaml.safe_load((share / 'config/environment.yaml').read_text())
    robots, goals = cfg['robots'], cfg['goals']
    if (not isinstance(robots, list) or not robots or len(set(robots)) != len(robots)
            or any(r not in env['starts'] for r in robots)):
        raise ValueError('robots는 environment.yaml에 있는 중복 없는 로봇 ID 목록이어야 합니다.')
    if len(goals) != len(robots) or len({g['id'] for g in goals}) != len(goals):
        raise ValueError('로봇 수와 목표 수가 같아야 하며 목표 ID는 고유해야 합니다.')
    for g in goals:
        if not isinstance(g['id'], str) or not g['id'] or not all(math.isfinite(g[k]) for k in ('x', 'y')):
            raise ValueError('목표 ID와 유한한 x,y 좌표가 필요합니다.')
    for start in env['starts'].values():
        if not all(math.isfinite(start[k]) for k in ('x', 'y', 'yaw')):
            raise ValueError('environment 시작 위치는 유한해야 합니다.')
    pcfg = PlannerConfig(**cfg['planner'])
    if pcfg.speed > .22:
        raise ValueError('계획 속도는 기존 LIMO 드라이버 상한 0.22m/s 이하여야 합니다.')
    for key, value in cfg['execution'].items():
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'execution.{key}는 양의 유한 수여야 합니다.')
    return cfg, env, pcfg, share


def make_field(cfg, env, pcfg, share):
    field = TerrainModel.from_world(
        share / 'worlds' / env['world'],
        share / 'controllers/lunar_sun/sun_config.json',
        safety_time_step=pcfg.safety_time_step,
        terrain_height_threshold=cfg.get('terrain_height_threshold', .001),
        column_resolution=pcfg.resolution,
        geometry_mode=cfg.get('terrain_geometry','mesh'),
        shadow_plane_height=cfg.get('shadow_plane_height',.02))
    actual = getattr(field, 'robot_starts', {})
    if actual:
        if set(actual) != set(env['starts']):
            raise ValueError('월드와 environment.yaml의 전체 로봇 ID가 다릅니다.')
        for robot, value in env['starts'].items():
            if math.dist((actual[robot]['x'], actual[robot]['y']), (value['x'], value['y'])) > 1e-5:
                raise ValueError(f'{robot}: 월드와 environment.yaml 시작 좌표가 다릅니다.')
            if abs(math.atan2(math.sin(actual[robot]['yaw'] - value['yaw']),
                              math.cos(actual[robot]['yaw'] - value['yaw']))) > 1e-5:
                raise ValueError(f'{robot}: 월드와 environment.yaml 시작 방위가 다릅니다.')
    return field


def path_data(paths):
    return {r: {'cost_ticks': p.cost_ticks, 'points': [asdict(v) for v in p.points]}
            for r, p in paths.items()}


def plan_experiment(cfg, env, pcfg, field, t0, starts=None, output=None):
    if not math.isfinite(t0) or t0 < 0:
        raise ValueError('t0는 음수가 아닌 유한 시뮬레이션 시각이어야 합니다.')
    starts = starts or {r: (v['x'], v['y']) for r, v in env['starts'].items()}
    if set(starts) != set(env['starts']):
        raise ValueError('대기 로봇을 포함한 전체 로봇 위치가 필요합니다.')
    robot_ids = cfg['robots']
    goals = cfg['goals']
    standby = [(p[0], p[1], pcfg.robot_radius) for r, p in starts.items() if r not in robot_ids]
    independent = field.with_static_robots(standby)
    planner = SpaceTimePlanner(independent, pcfg)
    before = time.perf_counter()
    candidates, pair_statuses = [], {}
    for r in robot_ids:
        row, statuses = [], {}
        for g in goals:
            row.append(planner.plan(starts[r], (g['x'], g['y']), t0))
            statuses[g['id']] = {'status': planner.last_status, 'reason': planner.last_reason}
        candidates.append(row)
        pair_statuses[r] = statuses
    matrix_seconds = time.perf_counter() - before
    costs = [[p.cost_ticks if p else None for p in row] for row in candidates]
    before = time.perf_counter()
    assignment = None
    error = None
    try:
        assignment = solve(costs)
    except InfeasibleAssignment as exc:
        error = str(exc)
    assignment_seconds = time.perf_counter() - before
    metadata = {
        'map_id': field.identifier,
        'config_id': hashlib.sha256(json.dumps(cfg, sort_keys=True).encode()).hexdigest(),
        'experiment': '사전 지형 기반 평지 통로 실험',
        'all_robot_positions': starts,
        'standby_robot_ids': [r for r in starts if r not in robot_ids],
        'planner': asdict(pcfg),
        'goals': goals,
        'pair_statuses': pair_statuses,
        'time_origin': 'Webots getTime() = ROS /clock; 태양 위상 기준 0초',
        'infeasibility_scope': '설정된 격자·시간 해상도·horizon과 보수적 지형 모델 안에서의 불가',
    }
    if output:
        write_problem(output, robot_ids, [g['id'] for g in goals], costs, t0, pcfg.time_step,
                      metadata, matrix_seconds, assignment_seconds, assignment)
    result = {'t0': t0, 'costs': costs, 'matrix_generation_seconds': matrix_seconds,
              'assignment_seconds': assignment_seconds, 'assignment': assignment,
              'error': error, 'paths': {}, 'independent_paths': {}, 'metadata': metadata}
    if assignment is not None:
        selected = {r: (goals[j]['x'], goals[j]['y']) for r, j in zip(robot_ids, assignment.columns)}
        result['assigned_goals'] = selected
        result['independent_paths'] = {r: candidates[i][assignment.columns[i]] for i, r in enumerate(robot_ids)}
        before = time.perf_counter()
        try:
            result['paths'] = coordinate(field, pcfg, starts, selected, t0, standby,
                                         cfg['execution']['goal_hold_seconds'])
        except RuntimeError as exc:
            result['error'] = str(exc)
        result['coordination_seconds'] = time.perf_counter() - before
    if output:
        save_execution_plan(Path(output) / 'execution_plan.json', result)
    return result


def save_execution_plan(path, result):
    assignment = result['assignment']
    payload = {k: v for k, v in result.items() if k not in ('assignment', 'paths', 'independent_paths')}
    payload['assignment'] = asdict(assignment) if assignment else None
    payload['paths'] = path_data(result['paths'])
    payload['independent_paths'] = path_data(result['independent_paths'])
    payload['original_assignment_total_ticks'] = assignment.total_cost if assignment else None
    payload['coordinated_total_ticks'] = sum(p.cost_ticks for p in result['paths'].values()) if result['paths'] else None
    if assignment and result['paths']:
        payload['coordination_extra_ticks'] = payload['coordinated_total_ticks'] - assignment.total_cost
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--share', type=Path)
    parser.add_argument('--t0', type=float, default=0.)
    parser.add_argument('--output', type=Path, default=Path('results/shadow_hungarian'))
    args = parser.parse_args(argv)
    if args.config is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            args.config = Path(get_package_share_directory('moon_mapping_demo')) / 'config/shadow_experiment.yaml'
        except (ImportError, LookupError):
            args.config = Path(__file__).resolve().parents[1] / 'config/shadow_experiment.yaml'
    cfg, env, pcfg, share = load_experiment(args.config, args.share)
    field = make_field(cfg, env, pcfg, share)
    result = plan_experiment(cfg, env, pcfg, field, args.t0, output=args.output)
    print(json.dumps({'costs_ticks': result['costs'],
                      'assignment': asdict(result['assignment']) if result['assignment'] else None,
                      'matrix_generation_seconds': result['matrix_generation_seconds'],
                      'assignment_seconds': result['assignment_seconds'],
                      'safe_joint_plan': bool(result['paths']), 'error': result['error']}, ensure_ascii=False, indent=2))
    return 0 if result['paths'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
