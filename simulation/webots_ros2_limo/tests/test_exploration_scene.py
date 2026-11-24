import math
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest

from webots_ros2_limo.exploration_scan import mapping_ranges
from webots_ros2_limo.slam_scan_adapter import real_no_return_ranges
from webots_ros2_limo.exploration_world import layout
from webots_ros2_limo.exploration_policy import FrontierGrid


def test_no_return_does_not_turn_invalid_or_near_measurements_into_free_rays():
    values, limit = mapping_ranges([math.inf, math.nan, -math.inf, 0., .05, .1, 2., 12., 13.])
    assert 12. < values[0] < limit
    assert all(math.isnan(values[i]) for i in (1, 2, 3, 4, 5, 8))
    assert values[6] == 2.
    assert values[7] == values[0]


def test_real_no_return_clears_only_distant_bounded_zero_sectors():
    values, limit = real_no_return_ranges([0., 2., 0., 2.1, 5., 0., 0., 6., 13.], 64.)
    assert all(math.isnan(values[i]) for i in (0, 2, 8))
    assert values[1] == 2. and values[3] == 2.1
    assert 12. < values[5] < limit and values[5] == values[6]


def test_shared_base_has_clear_spawns_and_routes_to_all_four_room_areas():
    starts, walls = layout('branched')
    resolution = .1
    yy, xx = np.indices((240, 240))
    x, y = -12+(xx+.5)*resolution, -12+(yy+.5)*resolution
    cells = np.zeros(x.shape, dtype=np.int8)
    for wx, wy, width, height in walls:
        cells[(abs(x-wx) <= width/2) & (abs(y-wy) <= height/2)] = 100
    grid = FrontierGrid(cells, resolution, (-12., -12., 0.))
    for name, spawn in starts.items():
        assert grid.safe[grid.index(spawn)]
        assert abs(spawn[0]) <= 1.8 and spawn[1] == -9.5
        assert all(math.dist(spawn[:2], other[:2]) >= 1.19 for n, other in starts.items() if n != name)
    distance = grid.distances(starts['robot2'])
    assert all(math.isfinite(distance[grid.index(p)]) for p in ((-8., -7.), (9., -8.), (-8., 8.), (6., 9.)))


def test_installed_karto_clears_no_return_without_fake_endpoints(tmp_path):
    """Use the real SDK, so a sentinel regression cannot create a fake lidar ring."""
    root = Path('/opt/ros/jazzy')
    if not shutil.which('g++') or not (root/'include/karto_sdk/Karto.h').exists():
        pytest.skip('Installed Karto SDK and compiler required')
    source = tmp_path/'scan.cpp'
    source.write_text(r'''
#include <karto_sdk/Karto.h>
#include <cassert>
#include <cmath>
#include <iostream>
class TestGrid : public karto::OccupancyGrid {
 public:
  using karto::OccupancyGrid::OccupancyGrid;
  void Build(const karto::LocalizedRangeScanVector & scans) { CreateFromScans(scans); }
};
int main() {
  using namespace karto;
  auto laser = LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom, Name("clearing_test"));
  laser->SetMinimumRange(.1); laser->SetMaximumRange(12.02); laser->SetRangeThreshold(12.);
  laser->SetMinimumAngle(-M_PI); laser->SetMaximumAngle(M_PI/2); laser->SetAngularResolution(M_PI/2);
  SensorManager::GetInstance()->RegisterSensor(laser);
  std::vector<double> readings{12.01, 12.01, 5., 12.01};
  LocalizedRangeScan scan(laser->GetName(), readings);
  scan.SetCorrectedPose(Pose2(0., 0., 0.));
  assert(scan.GetPointReadings(true).size() == 1);
  TestGrid grid(300, 300, Vector2<double>(-15., -15.), .1);
  grid.SetMinPassThrough(2); grid.SetOccupancyThreshold(.1);
  LocalizedRangeScanVector scans{&scan, &scan, &scan};
  grid.Build(scans);
  auto cell = [&](double x, double y) { return grid.GetValue(grid.WorldToGrid(Vector2<double>(x,y))); };
  assert(cell(-11.,0.) == GridStates_Free);
  assert(cell(0.,11.) == GridStates_Free);
  assert(cell(5.,0.) == GridStates_Occupied);
  assert(cell(6.,0.) == GridStates_Unknown);
  assert(cell(-13.,0.) == GridStates_Unknown);
  TestGrid first_scan(300, 300, Vector2<double>(-15., -15.), .1);
  first_scan.SetMinPassThrough(0); first_scan.SetOccupancyThreshold(.1);
  LocalizedRangeScanVector one_scan{&scan};
  first_scan.Build(one_scan);
  assert(first_scan.GetValue(first_scan.WorldToGrid(Vector2<double>(-11.,0.))) == GridStates_Free);
  std::cout << "Only real hits occupied; no-return rays clear to the physical range.\n";
}
''')
    binary = tmp_path/'scan'
    compiled = subprocess.run(['g++', '-std=c++17', str(source), '-I'+str(root/'include'),
                    '-L'+str(root/'lib'), '-Wl,-rpath,'+str(root/'lib'),
                    '-lkartoSlamToolbox', '-lboost_serialization', '-o', str(binary)], capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr
    executed = subprocess.run([str(binary)], capture_output=True, text=True)
    assert executed.returncode == 0, executed.stderr
