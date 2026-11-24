import math

import numpy as np
import pytest

from webots_ros2_limo.exploration_coverage import coverage_snapshot
from webots_ros2_limo.exploration_policy import FrontierGrid


def test_unmapped_exterior_is_unknown_in_fixed_mission_denominator():
    cells = np.zeros((10, 10), dtype=np.int8)
    cells[0, 0] = 100  # Observed obstacle also counts as mapped.
    grid = FrontierGrid(cells, 1., bounds=(0., 0., 20., 20.))
    overlay, stats = coverage_snapshot(grid)
    assert np.all(overlay == 100)
    assert stats['total_cells'] == 400
    assert stats['observed_cells'] == 100
    assert stats['observed_free_cells'] == 99
    assert stats['unknown_cells'] == 300
    assert stats['observed_percent'] == 25.
    assert not stats['bounds_fully_known']


def test_rotated_map_roi_and_unknown_overlay():
    cells = np.zeros((10, 10), dtype=np.int8)
    cells[:5] = -1
    grid = FrontierGrid(cells, 1., (10., 0., math.pi/2), bounds=(0., 0., 10., 10.))
    overlay, stats = coverage_snapshot(grid)
    assert np.all(overlay[:5] == -1)
    assert np.all(overlay[5:] == 100)
    assert stats['observed_percent'] == pytest.approx(50.)
    assert not stats['bounds_fully_known']


def test_fully_observed_bounds_and_cells_outside_roi():
    grid = FrontierGrid(np.zeros((20, 20), dtype=np.int8), 1., bounds=(5., 5., 15., 15.))
    overlay, stats = coverage_snapshot(grid)
    assert np.all(overlay[:5] == -1)
    assert stats['total_cells'] == stats['observed_cells'] == 100
    assert stats['observed_percent'] == 100.
    assert stats['bounds_fully_known']
