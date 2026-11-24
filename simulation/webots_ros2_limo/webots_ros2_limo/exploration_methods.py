"""Selectable exploration: joint CP-SAT assignment or greedy goals with spatial A*."""
import math
import time

from .exploration_policy import allocate as greedy


def allocate_exploration(method, grid, positions, *, network=None, team_positions=None,
                         active_goals=None, diagnostics=None, solver_seconds=.3, **constraints):
    if method == 'cp_sat':
        from .exploration_cp_sat import allocate
        return allocate(grid, positions, network=network, team_positions=team_positions,
                        active_goals=active_goals, diagnostics=diagnostics,
                        solver_seconds=solver_seconds, **constraints)
    if method != 'space_astar':
        raise ValueError('Unknown exploration method')
    started = time.monotonic()
    external_validate = constraints.pop('validate_goal', None)
    transform = constraints.pop('transform_goal', None)
    endpoints = {**(team_positions or positions),
                 **{n: tuple(p[:2]) for n, p in (active_goals or {}).items()}}

    def observed_goal(name, goal, distances):
        # A* is the route search. Frontier assignment is greedy shortest-grid
        # distance minus bounded information gain, not a joint optimizer.
        goal = transform(name, goal, distances) if transform else goal
        if goal is None:
            return None
        row, col = grid.index(goal.xy)
        if not (0 <= row < grid.cells.shape[0] and 0 <= col < grid.cells.shape[1]
                and grid.safe[row, col] and math.isfinite(distances[row, col])
                and grid.astar_path(positions[name], goal.xy)):
            return None
        return goal

    def valid(name, goal, chosen):
        projected = {**endpoints, **{n: g.xy for n, g in chosen.items()}, name: goal.xy}
        return ((network is None or network.graph(projected, safe=True)['all_connected'])
                and (external_validate is None or external_validate(name, goal, chosen)))

    result = greedy(grid, positions, transform_goal=observed_goal,
                    validate_goal=valid, **constraints)
    if not all(valid(n, g, {k: v for k, v in result.items() if k != n})
               for n, g in result.items()):
        result = {}
    if diagnostics is not None:
        diagnostics.update(status='SPATIAL_ASTAR_GREEDY', assigned=len(result),
                           elapsed_s=time.monotonic()-started)
    return result
