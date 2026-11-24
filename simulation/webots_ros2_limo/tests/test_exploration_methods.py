import math

import numpy as np
import pytest

from webots_ros2_limo.exploration_methods import allocate_exploration
from webots_ros2_limo.exploration_policy import FrontierGrid, Goal
from webots_ros2_limo.exploration_radio import MobileRadioNetwork


def test_methods_offer_distinct_joint_and_greedy_assignment():
    grid = FrontierGrid(np.zeros((80, 80), dtype=np.int8), .2, clearance=.2)
    grid.candidates = [Goal(5., 5., 0., 1.), Goal(9., 5., 0., 1.)]
    positions = {'a': (5., 4.), 'b': (5., 3.)}
    def valid(name, goal, chosen):
        return name != 'b' or goal.x == 5.
    joint = allocate_exploration('cp_sat', grid, positions, validate_goal=valid, solver_seconds=1.)
    spatial = allocate_exploration('space_astar', grid, positions, validate_goal=valid)
    assert len(joint) == 2
    assert len(spatial) == 1
    assert all(grid.astar_path(positions[n], g.xy) for n, g in spatial.items())


@pytest.mark.parametrize('method', ['cp_sat', 'space_astar'])
def test_both_methods_keep_network_and_reject_unknown_transformed_goals(method):
    cells = np.zeros((80, 80), dtype=np.int8)
    cells[0, 0] = -1
    grid = FrontierGrid(cells, .2, clearance=.2)
    grid.candidates = [Goal(5., 5., 0., 1.), Goal(9., 5., 0., 1.)]
    positions = {'a': (5., 4.), 'b': (9., 4.), 'relay': (12., 4.)}
    radio = MobileRadioNetwork(4.2, .1)
    result = allocate_exploration(method, grid, {'a': positions['a'], 'b': positions['b']},
        network=radio, team_positions=positions, active_goals={'relay': (12., 5.)}, solver_seconds=1.)
    assert result
    projected = {**positions, 'relay': (12., 5.), **{n: g.xy for n, g in result.items()}}
    assert radio.graph(projected, safe=True)['all_connected']
    assert all(math.isfinite(grid.distances(positions[n])[grid.index(g.xy)]) for n, g in result.items())
    for point in ((.1, .1), (-10., -10.)):
        assert allocate_exploration(method, grid, {'a': positions['a']},
            transform_goal=lambda *args: Goal(*point, 0., 1.)) == {}


def test_invalid_method_rejected():
    with pytest.raises(ValueError, match='Unknown exploration method'):
        allocate_exploration('invalid', None, {})


@pytest.mark.parametrize('method', ['cp_sat', 'space_astar'])
def test_short_leg_does_not_discard_original_frontier_distance(method):
    grid = FrontierGrid(np.zeros((80, 80), dtype=np.int8), .2, clearance=.2)
    start = grid.world(25, 25)
    near, far = Goal(*grid.world(35, 25), 0., 1.), Goal(*grid.world(25, 60), 0., 1.)
    grid.candidates = [far, near]
    # The farther target has only a tiny radio-safe advance. Scoring that
    # advance would choose it forever over the useful route to a near frontier.
    near_leg, far_leg = grid.world(32, 25), grid.world(25, 28)
    def leg(name, goal, distances):
        return Goal(*(near_leg if goal == near else far_leg), goal.yaw, goal.gain)
    result = allocate_exploration(method, grid, {'a': start}, transform_goal=leg, solver_seconds=1.)
    assert result['a'].xy == near_leg
