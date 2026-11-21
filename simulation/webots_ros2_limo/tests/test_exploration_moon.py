"""Validate physical geometry, not just the terrain metadata."""
import math
from pathlib import Path

import pytest

from webots_ros2_limo.exploration_moon import mesh_points, solid_blocks, wide_moon, radar_moon, circular_radar_moon
from webots_ros2_limo.exploration_world import layout


SOURCE = Path(__file__).resolve().parents[1]/'resource/moon_map_release/worlds/moon_terrain.wbt'


def test_circular_floor_is_physical_disk_and_terrain_stays_inside_connected_perimeter():
    world, meta = circular_radar_moon(SOURCE.read_text())
    assert meta['shape'] == 'disk' and meta['radius'] == 30.
    assert 'boundingObject USE CIRCULAR_GROUND_MESH' in world
    assert 'size 60 60 0.1' not in world
    assert 'DEF BOUNDARY_WEST' not in world
    walls = list(solid_blocks(world, r'DEF BOUNDARY_RING_(\d+) Solid\s*\{'))
    assert len(walls) == 96
    for _, _, block in walls:
        assert 'boundingObject Box' in block
    for f in meta['features']:
        assert math.hypot(*f['position'][:2])+f['radius'] < 29.
    _, _, floor = next(solid_blocks(world, r'DEF LUNAR_GROUND Solid\s*\{'))
    _, points = mesh_points(floor)
    assert len(points) == 194
    assert max(math.hypot(x, y) for x, y, z in points) == pytest.approx(30., abs=1e-5)


def test_radar_scene_nine_times_original_area_unique_collision_meshes_and_central_staging():
    world, meta = radar_moon(SOURCE.read_text())
    assert meta['size'] == [60., 60.] and meta['base'] == [0., 0.]
    assert world.count('size 60 60 0.1') == 2
    assert world.count('size 0.2 60.2 0.5') == 4
    assert world.count('size 60.2 0.2 0.5') == 4
    assert 'translation -30 0 0.25' in world and 'translation 0 30 0.25' in world
    blocks = list(solid_blocks(world, r'DEF (DEMO_(HILL|CRATER)_\d+) Solid\s*\{'))
    assert len(blocks) == len(meta['features']) == 36
    assert len({m[1] for m, _, _ in blocks}) == 36
    starts, _ = layout('moon_radar')
    for (match, _, block), f in zip(blocks, meta['features']):
        assert f'boundingObject USE {match[1]}_MESH' in block
        assert all(math.dist(f['position'][:2], p[:2]) > f['radius']+1 for p in starts.values())
        assert max(abs(v) for v in f['position'][:2])+f['radius'] < 29.
        assert all(math.dist(f['position'][:2], peer['position'][:2]) > f['radius']+peer['radius']+1.5
                   for peer in meta['features'] if peer is not f)


def test_wide_scene_reproducible_clear_and_terrain_collision_matches_rendering():
    original = SOURCE.read_text()
    world, meta = wide_moon(original)
    assert wide_moon(original) == (world, meta)
    assert wide_moon(original, 43)[1] != meta
    assert world.count('size 40 20 0.1') == 2
    assert 'material1 "wheel" material2 "default"' in world
    assert 'forceDependentSlip 0.01 bounce 0' in world
    assert 'translation -20 0 0.25' in world and 'translation 20 0 0.25' in world
    assert world.count('size 40.2 0.2 0.5') == 4
    starts, _ = layout('moon')
    for feature in meta['features']:
        x, y, _ = feature['position']
        radius = feature['radius']
        assert abs(x)+radius < 19 and abs(y)+radius < 9
        assert all(math.dist((x, y), p[:2]) > radius+1 for p in starts.values())
        assert all(math.dist((x, y), f['position'][:2]) >= radius+f['radius']+1.49
                   for f in meta['features'] if f is not feature)
    for match, _, block in solid_blocks(world, r'DEF (DEMO_(HILL|CRATER)_\d+) Solid\s*\{'):
        feature = next(f for f in meta['features'] if f['def'] == match[1])
        assert f'boundingObject USE {match[1]}_MESH' in block
        _, points = mesh_points(block)
        assert max(p[2] for p in points)+.02 == pytest.approx(feature['height'], abs=1e-5)
        if match[2] == 'CRATER':
            # Every radial section of the bowl has a rim above the actual
            # horizontal lidar plane and a floor below it: real ray intersection.
            assert points[0][2]+.02 < .31
            for angular_index in range(96):
                radial = [p[2]+.02 for p in points[1+angular_index::96]]
                assert max(radial) > .6
                assert min(radial) < .31
