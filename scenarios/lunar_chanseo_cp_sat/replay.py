#!/usr/bin/env python3
"""Offline allocation replay; no ROS, motor commands, or completion claims."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
from webots_ros2_limo.exploration_methods import allocate_exploration
from webots_ros2_limo.exploration_policy import FrontierGrid
from webots_ros2_limo.exploration_radio import MobileRadioNetwork
from webots_ros2_limo.exploration_coverage import coverage_snapshot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True, type=Path)
    parser.add_argument('--checkpoint', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--path-planner', choices=('auto', 'astar', 'distance_field'), default='auto')
    args = parser.parse_args()
    saved = np.load(args.map)
    state = json.loads(args.checkpoint.read_text())
    positions = state['positions']
    grid = FrontierGrid(saved['cells'], float(saved['resolution']), (*saved['origin'][:2], 0.),
                        bounds=(-30., -30., 30., 30.), clearance=.4, cache_distances=True)
    radio = MobileRadioNetwork(10., .3)
    def route_to(name, goal, distances=None):
        if planner == 'astar':
            return grid.astar_path(positions[name], goal)
        return grid.path(positions[name], goal,
                         grid.distances(positions[name]) if distances is None else distances)
    def transform(name, goal, distances):
        from webots_ros2_limo.exploration_policy import Goal
        route = route_to(name, goal.xy, distances)
        if not route:
            return None
        point = radio.advance(positions, name, route)
        return Goal(*point, goal.yaw, goal.gain)
    def valid(name, goal, chosen):
        projected = {**positions, **{n: g.xy for n, g in chosen.items()}}
        return radio.allows(projected, name, goal.xy)
    result = {'source_map': str(args.map), 'source_checkpoint': str(args.checkpoint),
              'path_planner': args.path_planner, 'coverage': coverage_snapshot(grid)[1],
              'scope': 'Saved-map allocation only; no live movement, shadow avoidance or full coverage certification',
              'input_connected': radio.graph(positions)['all_connected'],
              'candidate_count': len(grid.candidates), 'planners': {}}
    for name in ('cp_sat', 'space_astar'):
        planner = args.path_planner
        if planner == 'auto':
            planner = 'distance_field' if name == 'cp_sat' else 'astar'
        if name == 'space_astar':
            planner = 'astar'
        diagnostics = {}
        started = time.monotonic()
        kwargs = dict(transform_goal=transform, validate_goal=valid)
        kwargs.update(solver_seconds=1., diagnostics=diagnostics, network=radio, team_positions=positions)
        goals = allocate_exploration(name, grid, positions, **kwargs)
        projected = {**positions, **{n: g.xy for n, g in goals.items()}}
        result['planners'][name] = {
            'path_planner': planner,
            'elapsed_s': time.monotonic()-started, 'assigned': len(goals),
            'goals': {n: [g.x, g.y, g.yaw] for n, g in goals.items()},
            'endpoint_connected': radio.graph(projected, safe=True)['all_connected'],
            'observed_reachable': all(math.isfinite(grid.distances(positions[n])[grid.index(g.xy)])
                                      for n, g in goals.items()),
            'selected_routes_found': all(bool(route_to(n, g.xy)) for n, g in goals.items()),
            'route_points': {n: len(route_to(n, g.xy)) for n, g in goals.items()},
            'solver': diagnostics}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
