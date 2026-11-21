import numpy as np
import pytest

from webots_ros2_limo.exploration_policy import FrontierGrid


def test_same_start_cell_reuses_exact_distance_field():
    cells = np.zeros((30, 30), dtype=np.int8)
    grid = FrontierGrid(cells, .1, cache_distances=True)
    uncached = FrontierGrid(cells, .1)
    first = grid.distances((1.01, 1.01))
    assert grid.distances((1.09, 1.09)) is first
    np.testing.assert_array_equal(first, uncached.distances((1.09, 1.09)))
    adjacent = grid.distances((1.11, 1.01))
    assert adjacent is not first
    assert adjacent[grid.index((1.11, 1.01))] == 0.
    assert first[grid.index((1.11, 1.01))] > 0.


def test_new_map_recomputes_reachability_after_a_wall_appears():
    cells = np.zeros((30, 30), dtype=np.int8)
    old = FrontierGrid(cells, .1, cache_distances=True)
    start, target = (.75, 1.5), (2.25, 1.5)
    old_field = old.distances(start)
    assert np.isfinite(old_field[old.index(target)])
    cells[:, 15] = 100
    new = FrontierGrid(cells, .1, cache_distances=True)
    assert np.isinf(new.distances(start)[new.index(target)])
    # Input arrays and caches are not shared between immutable snapshots.
    assert old.distances(start) is old_field
    assert np.isfinite(old_field[old.index(target)])


def test_cached_inputs_and_results_reject_accidental_mutation():
    grid = FrontierGrid(np.zeros((30, 30)), .1, cache_distances=True)
    field = grid.distances((1.5, 1.5))
    for array in (grid.cells, grid.safe, field):
        with pytest.raises(ValueError, match='read-only'):
            array[15, 15] = 0


def test_cache_evicts_old_cells_and_bounds_retained_distance_memory():
    grid = FrontierGrid(np.zeros((30, 30)), .1, cache_distances=True)
    starts = [(float(x), 1.5) for x in np.arange(.55, 1.46, .1)]
    first = grid.distances(starts[0])
    recently_used = grid.distances(starts[1])
    for xy in starts[2:8]:
        grid.distances(xy)
    assert grid.distances(starts[1]) is recently_used
    grid.distances(starts[8])
    assert len(grid._distance_cache) == 8
    assert grid.distances(starts[1]) is recently_used
    replacement = grid.distances(starts[0])
    assert replacement is not first
    np.testing.assert_array_equal(replacement, first)
    assert len(grid._distance_cache) == 8


def test_default_grid_retains_mutable_uncached_behavior():
    grid = FrontierGrid(np.zeros((30, 30)), .1)
    first = grid.distances((1.5, 1.5))
    assert grid.distances((1.5, 1.5)) is not first
    grid.cells[15, 15] = 100
    rebuilt = FrontierGrid(grid.cells, .1)
    assert np.isinf(rebuilt.distances((1.5, 1.5))).all()
    assert first.flags.writeable
