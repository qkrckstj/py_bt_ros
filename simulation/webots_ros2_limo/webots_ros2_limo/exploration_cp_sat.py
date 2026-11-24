"""Bounded CP-SAT frontier matching with the production BT constraint hooks.

Maximize assigned robots first, then minimize observed-grid distance minus
bounded information gain. No straight-line shortcut through unknown cells.
A rejected full assignment is excluded and solved again within one time budget.
No unvalidated assignment or implicit Greedy fallback is returned on timeout.
"""
import math
import time

from ortools.sat.python import cp_model


def allocate(grid, positions, *, reserved=(), excluded=(), separation=2., min_distance=.5,
             transform_goal=None, validate_goal=None, solver_seconds=.15,
             candidates_per_robot=32, diagnostics=None, network=None,
             team_positions=None, active_goals=None):
    if not math.isfinite(solver_seconds) or solver_seconds <= 0 or candidates_per_robot < 1:
        raise ValueError('Positive solver budget and candidate limit required')
    started = time.monotonic()
    choices = {}
    for robot, xy in sorted(positions.items()):
        distance = grid.distances(xy)
        unique = {}
        for original in grid.candidates:
            frontier_cost = distance[grid.index(original.xy)]
            if not math.isfinite(frontier_cost):
                continue
            goal = transform_goal(robot, original, distance) if transform_goal else original
            if goal is None or math.dist(xy[:2], goal.xy) < min_distance:
                continue
            row, col = grid.index(goal.xy)
            if not (0 <= row < grid.cells.shape[0] and 0 <= col < grid.cells.shape[1]):
                continue
            cost = distance[row, col]
            if (not grid.safe[row, col] or not math.isfinite(cost)
                    or any(math.dist(goal.xy, p) < separation for p in reserved)
                    or any(math.dist(goal.xy, p) < radius for p, radius in excluded)):
                continue
            # A radio/shadow transform may return only the next short leg.
            # Rank the original frontier, not the arbitrarily truncated leg;
            # otherwise a distant frontier with a tiny permitted step wins.
            score = round(1000 * (frontier_cost - .12 * min(original.gain, 8.)))
            key = goal.xy
            if key not in unique or score < unique[key][0]:
                unique[key] = (score, goal)
        choices[robot] = sorted(unique.values(), key=lambda pair: (pair[0], pair[1].xy))[:candidates_per_robot]
    model = cp_model.CpModel()
    variables, holds = {}, {}
    for robot, options in choices.items():
        variables[robot] = [model.new_bool_var(f'{robot}_{i}') for i in range(len(options))]
        holds[robot] = model.new_bool_var(f'{robot}_hold')
        model.add_exactly_one([*variables[robot], holds[robot]])
        model.add_hint(holds[robot], 1)
        for v in variables[robot]:
            model.add_hint(v, 0)
    names = list(choices)
    for index, a in enumerate(names):
        for b in names[index+1:]:
            for i, (_, ga) in enumerate(choices[a]):
                for j, (_, gb) in enumerate(choices[b]):
                    if math.dist(ga.xy, gb.xy) < separation:
                        model.add(variables[a][i] + variables[b][j] <= 1)
    if network is not None:
        # Single-commodity flow makes the selected joint endpoints connected.
        # Unassigned robots retain their actual pose; active goals remain fixed.
        endpoints = {n: [tuple(p[:2])] for n, p in (team_positions or positions).items()}
        endpoints.update({n: [tuple(p[:2])] for n, p in (active_goals or {}).items()})
        selectors = {}
        for n in names:
            endpoints[n] = [positions[n][:2], *[g.xy for _, g in choices[n]]]
            selectors[n] = [holds[n], *variables[n]]
        from .exploration_radio import MobileRadioNetwork
        if not isinstance(network, MobileRadioNetwork):
            endpoints['__base__'] = [network.base]
        vertices = sorted(endpoints)
        if '__base__' in vertices:
            vertices.remove('__base__')
            vertices.insert(0, '__base__')
        for n in vertices:
            if n not in selectors:
                selectors[n] = [model.new_constant(1)]
        flows = {}
        capacity = len(vertices)-1
        for i, a in enumerate(vertices):
            for b in vertices[i+1:]:
                linked = model.new_bool_var(f'link_{a}_{b}')
                for j, pa in enumerate(endpoints[a]):
                    for k, pb in enumerate(endpoints[b]):
                        if math.dist(pa, pb) > network.radius-network.margin:
                            model.add(selectors[a][j]+selectors[b][k]+linked <= 2)
                for source, target in ((a, b), (b, a)):
                    flow = model.new_int_var(0, capacity, f'flow_{source}_{target}')
                    model.add(flow <= capacity*linked)
                    flows[source, target] = flow
        for i, n in enumerate(vertices):
            incoming = sum(v for (a, b), v in flows.items() if b == n)
            outgoing = sum(v for (a, b), v in flows.items() if a == n)
            model.add(outgoing-incoming == (capacity if i == 0 else -1))
    # This bound makes one additional assignment dominate every cost difference.
    bound = sum(max([abs(c) for c, _ in opts] or [0]) for opts in choices.values())
    reward = 2 * bound + 1
    model.maximize(sum((reward - cost) * v for robot, opts in choices.items()
                       for v, (cost, _) in zip(variables[robot], opts)))
    status, attempts, result = 'NO_VALID_ASSIGNMENT', 0, {}
    deadline = time.monotonic() + solver_seconds
    while names and time.monotonic() < deadline:
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = max(.001, deadline - time.monotonic())
        solver.parameters.num_search_workers = 1
        solver.parameters.random_seed = 0
        solver.parameters.cp_model_presolve = False
        code = solver.solve(model)
        attempts += 1
        status = solver.status_name(code)
        if code not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            break
        candidate, decisions = {}, []
        for robot in names:
            decisions.append(holds[robot] if solver.value(holds[robot]) else next(
                v for v in variables[robot] if solver.value(v)))
            for v, (_, goal) in zip(variables[robot], choices[robot]):
                if solver.value(v):
                    candidate[robot] = goal
        # Validate the complete joint endpoint set (including active goals held
        # by the caller), not only a Greedy prefix that may miss radio coupling.
        if validate_goal is None or all(validate_goal(n, g, {k: v for k, v in candidate.items() if k != n})
                                        for n, g in candidate.items()):
            result = candidate
            break
        model.add(sum(decisions) <= len(decisions) - 1)
        status = 'CONSTRAINT_RETRY'
    if diagnostics is not None:
        diagnostics.update(status=status, attempts=attempts, assigned=len(result),
                           candidates=sum(map(len, choices.values())),
                           elapsed_s=time.monotonic()-started)
    return result
