"""Spatial A* must preserve the CP-SAT grid's traversability rules."""
import math

import numpy as np
import pytest

from webots_ros2_limo.exploration_policy import FrontierGrid, Goal
from webots_ros2_limo.exploration_cp_sat import allocate


def test_astar_detours_and_matches_shortest_observed_grid_distance():
    cells = np.zeros((30, 30), dtype=np.int8)
    cells[2:24, 15] = 100
    cells[24:26, 15] = -1  # Unknown cells cannot provide a shortcut.
    grid = FrontierGrid(cells, .2, clearance=.2)
    start, goal = grid.world(10, 8), grid.world(10, 22)
    route = grid.astar_path(start, goal)
    assert route and route[-1] == goal
    assert all(grid.safe[grid.index(p)] for p in route)
    assert max(grid.index(p)[0] for p in route) >= 26 or min(grid.index(p)[0] for p in route) < 2
    length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
    assert length == pytest.approx(grid.distances(start)[grid.index(goal)])


def test_astar_disconnected_unknown_and_outside_goals():
    cells = np.zeros((20, 20), dtype=np.int8)
    cells[:, 10] = 100
    cells[5, 5] = -1
    grid = FrontierGrid(cells, .2, clearance=.2)
    start = grid.world(4, 4)
    for goal in (grid.world(4, 14), grid.world(5, 5), (-1., -1.), (100., 100.)):
        assert grid.astar_path(start, goal) == []
    assert grid.astar_path((-1., -1.), start) == []


def test_astar_robot_footprint_exception_never_bridges_measured_obstacles():
    cells = np.zeros((30, 30), dtype=np.int8)
    cells[15, 15] = -1
    grid = FrontierGrid(cells, .1, clearance=.2)
    start, goal = grid.world(15, 15), grid.world(15, 23)
    assert grid.astar_path(start, goal)
    cells[15, 16] = 100
    grid = FrontierGrid(cells, .1, clearance=.2)
    assert not grid.astar_path(start, goal)


def test_cp_sat_assignments_use_astar_transform_without_crossing_wall():
    cells = np.zeros((30, 30), dtype=np.int8)
    cells[:, 15] = 100
    grid = FrontierGrid(cells, .2, clearance=.2)
    grid.candidates = [Goal(*grid.world(20, 8), 0., 1.), Goal(*grid.world(20, 22), 0., 1.)]
    positions = {'left': grid.world(5, 8), 'right': grid.world(5, 22)}
    routes = {}
    def transform(name, goal, distances):
        routes[name, goal.xy] = grid.astar_path(positions[name], goal.xy)
        return goal if routes[name, goal.xy] else None
    result = allocate(grid, positions, transform_goal=transform, solver_seconds=1.)
    assert set(result) == set(positions)
    for name, goal in result.items():
        assert routes[name, goal.xy]
        assert (goal.x < 3.) == (name == 'left')
