"""Shared map fusion tests."""
import numpy as np
import pytest
from types import SimpleNamespace

from webots_ros2_limo.exploration_map_grid import Grid
from webots_ros2_limo.exploration_flat_map import FlatMap, include_localized_footprints, merge_native_grids
from webots_ros2_limo.exploration_policy import FrontierGrid


def test_native_lattice_retains_obstacles_and_world_coordinates_when_map_grows():
    cells = np.zeros((20, 20), dtype=np.int8)
    cells[10, 10] = 100
    original = Grid(cells, .1, 0., 0.)
    first = merge_native_grids([original])
    expanded = np.full((30, 30), -1, dtype=np.int8)
    expanded[20, 20] = 100
    # Same measured wall in a source whose extent grew west/south by 1 m.
    second = merge_native_grids([Grid(expanded, .1, -1., -1.), original])
    assert first.resolution == second.resolution == .1
    for grid in (first, second):
        row = round((1.05-grid.y)/grid.resolution-.5)
        col = round((1.05-grid.x)/grid.resolution-.5)
        assert grid.cells[row, col] == 100
    assert np.count_nonzero(second.cells == 100) == 1


def test_robot_just_outside_slam_bounds_can_seed_reachable_observed_space():
    source = Grid(np.zeros((30, 50), dtype=np.int8), .1, 1.6, -1.5)
    pose = (1.5, .7)
    original = FrontierGrid(source.cells, source.resolution, (source.x, source.y, 0.), clearance=.32)
    assert not np.isfinite(original.distances(pose)).any()
    expanded = include_localized_footprints(source, [pose])
    planning = FrontierGrid(expanded.cells, expanded.resolution, (expanded.x, expanded.y, 0.), clearance=.32)
    assert np.isfinite(planning.distances(pose)[planning.index((3., .7))])
    # Padding preserves the original cell lattice and obstacle coordinates.
    assert (source.x-expanded.x)/source.resolution == pytest.approx(4.)
    assert expanded.y == source.y


def test_padding_is_unknown_outside_robot_footprints_and_does_not_invent_corridor():
    source = Grid(np.zeros((20, 20), dtype=np.int8), .1, 1.6, -1.)
    source.cells[5, 5] = 100
    original = source.cells.copy()
    pose = (.5, 0.)
    expanded = include_localized_footprints(source, [pose])
    yy, xx = np.ogrid[:expanded.cells.shape[0], :expanded.cells.shape[1]]
    x, y = expanded.x+(xx+.5)*expanded.resolution, expanded.y+(yy+.5)*expanded.resolution
    padding = np.broadcast_to(x < source.x, expanded.cells.shape)
    footprint = (x-pose[0])**2+(y-pose[1])**2 <= .30**2
    assert np.all(expanded.cells[padding & ~footprint] == -1)
    assert np.all(expanded.cells[footprint] == 0)
    assert np.array_equal(source.cells, original)
    planning = FrontierGrid(expanded.cells, expanded.resolution, (expanded.x, expanded.y, 0.), clearance=.32)
    assert not np.isfinite(planning.distances(pose)[planning.index((2.5, 0.))])
    assert planning.cells[planning.index((2.15, -.45))] == 100


def test_footprint_extent_keeps_cell_limit_and_frozen_map_unchanged():
    source = Grid(np.zeros((2, 2), dtype=np.int8), .1, 0., 0.)
    with pytest.raises(ValueError, match='exceeds limit'):
        include_localized_footprints(source, [(1000., 1000.)])
    message = SimpleNamespace(header=SimpleNamespace(stamp=None), data=[0, -1, 100],
                              info=SimpleNamespace(width=3, height=1, origin=(1.6, -1.)))
    published = []
    fake = SimpleNamespace(publish_status=lambda: None, mapping=SimpleNamespace(frozen_map=message),
                           publisher=SimpleNamespace(publish=published.append),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'fresh')))
    # No grid/TF fields are supplied: frozen publication must never re-fuse or
    # extend its geometry when the returning robots change their positions.
    FlatMap.publish(fake)
    assert published == [message]
    assert message.header.stamp == 'fresh'
    assert message.data == [0, -1, 100] and message.info.origin == (1.6, -1.)
    assert (message.info.width, message.info.height) == (3, 1)
