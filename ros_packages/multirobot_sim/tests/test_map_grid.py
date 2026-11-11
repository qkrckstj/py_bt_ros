import math
import unittest

import numpy as np

from multirobot_sim.map_grid import Grid, align_grid


class TestMapGrid(unittest.TestCase):
    def test_preserves_translated_origin_and_unknown_cells(self):
        cells = np.array([[0, 100, -1], [0, -1, 0]], dtype=np.int8)
        aligned = align_grid(Grid(cells, 1.0, -1.0, 2.0), resolution=1.0)

        self.assertEqual((aligned.x, aligned.y), (-1.0, 2.0))
        np.testing.assert_array_equal(aligned.cells, cells)

    def test_rotates_a_grid_about_its_origin(self):
        grid = Grid(np.array([[0, 100], [-1, 100]], dtype=np.int8), 1.0, 2.0, 0.0, math.pi / 2)
        aligned = align_grid(grid, resolution=1.0)

        self.assertEqual((aligned.x, aligned.y, aligned.yaw), (0.0, 0.0, 0.0))
        np.testing.assert_array_equal(aligned.cells, [[-1, 0], [100, 100]])

    def test_resamples_an_input_at_a_different_resolution(self):
        grid = Grid(np.array([[0, 100]], dtype=np.int8), 0.1, 0.0, 0.0)
        aligned = align_grid(grid)

        np.testing.assert_array_equal(aligned.cells, [[0, 0, 100, 100], [0, 0, 100, 100]])

    def test_a_revised_scan_map_can_clear_an_old_obstacle(self):
        before = Grid(np.array([[100]], dtype=np.int8), 1.0, 0.0, 0.0)
        after = Grid(np.array([[0]], dtype=np.int8), 1.0, 0.0, 0.0)
        self.assertEqual(align_grid(before, resolution=1.0).cells[0, 0], 100)
        self.assertEqual(align_grid(after, resolution=1.0).cells[0, 0], 0)
