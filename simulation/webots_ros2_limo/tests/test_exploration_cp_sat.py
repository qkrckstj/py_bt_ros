"""CP-SAT allocation contracts, joint vetoes and actual TeamBT feedback."""
import math
from functools import partial

import numpy as np

from webots_ros2_limo.exploration_policy import FrontierGrid, Goal, allocate as greedy
from webots_ros2_limo.exploration_cp_sat import allocate
from webots_ros2_limo.exploration_runtime import TeamBT
from webots_ros2_limo.exploration_radio import MobileRadioNetwork


def grid():
    cells = np.full((80, 80), -1, dtype=np.int8)
    cells[8:72, 8:72] = 0
    return FrontierGrid(cells, .2, cache_distances=True)


def test_cp_sat_avoids_greedy_assignment_trap():
    g = grid()
    g.candidates = [Goal(5., 5., 0., 1.), Goal(9., 5., 0., 1.)]
    positions = {'a': (5., 4.), 'b': (5., 3.)}
    def feasible(name, goal, chosen):
        return name != 'b' or goal.x == 5.
    assert len(greedy(g, positions, validate_goal=feasible)) == 1
    result = allocate(g, positions, validate_goal=feasible, solver_seconds=1.)
    assert result['a'].x == 9. and result['b'].x == 5.


def test_walls_unknown_reserved_and_excluded():
    g = grid()
    cells = g.cells.copy()
    cells[:, 40] = 100
    g = FrontierGrid(cells, .2)
    result = allocate(g, {'left': (3., 6.)}, solver_seconds=1.)
    assert result['left'].x < 8.
    reserved = [result['left'].xy]
    result = allocate(g, {'left': (3., 6.)}, reserved=reserved,
                      excluded=[((3., 3.), 1.)], solver_seconds=1.)
    assert not result or (math.dist(result['left'].xy, reserved[0]) >= 2.
                          and math.dist(result['left'].xy, (3., 3.)) >= 1.)
    assert allocate(g, {'unknown': (0., 0.)}) == {}


def test_full_joint_validator_never_leaks_rejected_solution():
    g = grid()
    g.candidates = [Goal(5., 5., 0., 1.), Goal(9., 5., 0., 1.)]
    def joint(name, goal, chosen):
        return not chosen
    result = allocate(g, {'a': (5., 4.), 'b': (9., 4.)},
                      validate_goal=joint, solver_seconds=1.)
    assert len(result) == 1
    assert allocate(g, {'a': (5., 4.)}, validate_goal=lambda *args: False,
                    solver_seconds=.05) == {}


def test_new_bt_dispatches_cp_sat_and_preserves_active_goal():
    g = grid()
    team = TeamBT(['a', 'b'], lambda *a: None, autostart=True,
                  allocator=partial(allocate, solver_seconds=1.))
    team.update_map(g, 0.)
    for name, pos in [('a', [3., 3.]), ('b', [12., 12.])]:
        team.update_robot(name, {'ready': True, 'position': pos, 'status': 'IDLE'}, 0.)
    team.tick(0.)
    assert len(team.active) == 2
    task = dict(team.active['a'])
    team.tick(.1)
    assert team.active['a'] == task
    team.tick(10.)
    assert all(t['cancel_requested'] for t in team.active.values())


def test_mobile_radio_has_no_fixed_base():
    radio = MobileRadioNetwork(10., .3)
    positions = {'robot1': (100., 0.), 'robot2': (109., 0.),
                 'robot3': (118., 0.), 'robot4': (127., 0.)}
    assert radio.graph(positions)['all_connected']
    assert radio.allows(positions, 'robot1', (101., 0.))
    assert not radio.allows(positions, 'robot2', (150., 0.))


def test_flow_keeps_idle_relays_and_active_goals_in_joint_network():
    g = grid()
    g.candidates = [Goal(5., 5., 0., 1.), Goal(9., 5., 0., 1.)]
    radio = MobileRadioNetwork(4.2, .1)
    positions = {'a': (5., 4.), 'b': (9., 4.), 'relay': (12., 4.)}
    result = allocate(g, {'a': positions['a'], 'b': positions['b']}, network=radio,
                      team_positions=positions, active_goals={'relay': (12., 5.)}, solver_seconds=1.)
    assert len(result) == 2
    projected = {**positions, 'relay': (12., 5.), **{n: goal.xy for n, goal in result.items()}}
    assert radio.graph(projected, safe=True)['all_connected']


def test_transformed_goal_cannot_jump_to_unknown_or_wall():
    g = grid()
    for point in [(0., 0.), (1000., 1000.)]:
        assert allocate(g, {'a': (5., 5.)},
                        transform_goal=lambda *args: Goal(*point, 0., 1.)) == {}


def test_finished_robot_replenishes_without_waiting_for_other_three():
    g = grid()
    team = TeamBT(['a', 'b', 'c', 'd'], lambda *args: None, autostart=True,
                  allocator=partial(allocate, solver_seconds=1.))
    positions = {'a': [3., 3.], 'b': [12., 3.], 'c': [3., 12.], 'd': [12., 12.]}
    team.update_map(g, 0.)
    for n, pos in positions.items():
        team.update_robot(n, {'ready': True, 'position': pos, 'status': 'IDLE'}, 0.)
    team.tick(0.)
    assert len(team.active) == 4
    first = {n: t['task_id'] for n, t in team.active.items()}
    team.update_robot('a', {'ready': True, 'position': team.active['a']['goal'][:2],
                           'status': 'VISITED', 'task_id': first['a'], 'session': team.session}, .1)
    team.tick(.2)
    assert team.active['a']['task_id'] != first['a']
    assert all(team.active[n]['task_id'] == first[n] for n in ('b', 'c', 'd'))
