"""실제 센서 광선, 고정 분모, 중복과 관측 시각의 회귀 검사."""
import math
import unittest

import numpy as np

from moon_mapping_demo.observation import EvaluationArea, ObservationGrid, ray_cells


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.area = EvaluationArea(0., 0., 6., 4., .5)
        self.grid = ObservationGrid(self.area, ('a', 'b'), sensor_range=4.)

    def scan(self, robot='a', stamp=1., pose=(.25, .25, 0.), ranges=(2.,)):
        return self.grid.integrate(robot, stamp, pose, ranges, 0., .1, .05, 5.)

    def test_fixed_denominator_and_no_observation_from_construction(self):
        self.assertEqual(self.grid.area.denominator_m2, 24.)
        self.assertEqual(self.grid.observed_area_m2, 0.)
        self.assertTrue(np.all(self.grid.cells == -1))
        with self.assertRaises(ValueError):
            EvaluationArea(0, 0, 1, 1, .3)

    def test_first_obstacle_occludes_cells_behind_it(self):
        self.scan(ranges=(1.,))
        self.assertEqual(self.grid.cells[0, 2], 100)
        self.assertTrue(np.all(self.grid.cells[0, 3:] == -1))
        self.assertEqual(self.grid.observed_area_m2, .75)

    def test_infinite_no_return_uses_sensor_cap_but_zero_nan_do_not(self):
        self.scan(ranges=(0., math.nan, -math.inf))
        self.assertEqual(self.grid.observed_area_m2, 0.)
        self.scan(stamp=2., ranges=(math.inf,))
        self.assertTrue(np.all(self.grid.cells[0, :9] == 0))
        self.assertTrue(np.all(self.grid.cells[0, 9:] == -1))

    def test_team_overlap_counts_once_and_robot_credits_sum_to_union(self):
        first = self.scan()
        self.assertGreater(first, 0.)
        self.assertEqual(self.scan('b', 2.), 0.)
        state = self.grid.snapshot()
        self.assertAlmostEqual(sum(state['new_area_m2_by_robot'].values()), state['observed_area_m2'])
        self.assertEqual(self.grid.area.denominator_m2, 24.)

    def test_pose_heading_changes_the_observed_direction(self):
        self.scan(pose=(2.25, 1.25, math.pi / 2), ranges=(1.,))
        self.assertEqual(self.grid.cells[4, 4], 100)
        self.assertEqual(self.grid.cells[2, 6], -1)

    def test_duplicate_or_backward_scan_does_not_add_area(self):
        self.scan(stamp=2.)
        before = self.grid.observed_count
        self.scan(stamp=2., ranges=(4.,))
        self.scan(stamp=1., ranges=(4.,))
        self.assertEqual(self.grid.observed_count, before)
        self.assertEqual(self.grid.rejected_frames, 2)

    def test_older_other_robot_scan_does_not_clear_newer_obstacle(self):
        self.scan(stamp=5., ranges=(1.,))
        self.scan('b', stamp=4., ranges=(math.inf,))
        self.assertEqual(self.grid.cells[0, 2], 100)
        self.assertEqual(self.grid.last_seen[0, 2], 5.)
        self.assertEqual(self.grid.first_seen[0, 2], 4.)

    def test_observation_completion_requires_frames_after_arrival(self):
        for t in (1., 2., 3.):
            self.scan(stamp=t)
        self.assertFalse(self.grid.observed_since('a', 2.5, 2))
        self.scan(stamp=4.)
        self.assertTrue(self.grid.observed_since('a', 2.5, 2))

    def test_potential_gain_never_marks_cells_observed(self):
        self.assertGreater(self.grid.potential_gain((2., 2.)), 0.)
        self.assertEqual(self.grid.observed_count, 0)

    def test_roi_clips_outside_sensor_rays(self):
        cells = list(ray_cells(self.area, (-2., .25), (8., .25)))
        self.assertEqual(cells, [(0, col) for col in range(12)])
        self.assertEqual(list(ray_cells(self.area, (-1., 5.), (7., 5.))), [])


if __name__ == '__main__':
    unittest.main()
