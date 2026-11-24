import math
from pathlib import Path
import re

import cv2
import numpy as np
import pytest
import yaml

from webots_ros2_limo.map_to_webots import (
    OccupancyMap, generate_world, load_map, main, merge_rectangles,
    render_world, validate_spawn,
)


def saved_map(tmp_path, image_bytes, **metadata):
    image_path = tmp_path / 'input.pgm'
    image_path.write_bytes(image_bytes)
    content = {'image': image_path.name, 'resolution': 0.5, 'origin': [2.0, -3.0, 0.0],
               'occupied_thresh': 0.65, 'free_thresh': 0.2, 'negate': 0, 'mode': 'trinary'}
    content.update(metadata)
    path = tmp_path / 'input.yaml'
    path.write_text(yaml.safe_dump(content))
    return path


def free_map():
    return OccupancyMap(np.zeros((20, 20), dtype=np.int8), 0.1, (0.0, 0.0, 0.0), Path('test.yaml'))


def test_map_pixel_rows_origin_rotation_and_negate(tmp_path):
    path = saved_map(tmp_path, b'P2\n# map image\n3 2\n100\n0 100 50\n100 0 100\n',
                     origin=[2.0, -3.0, math.pi / 2])
    occupancy = load_map(path)
    assert occupancy.cells.tolist() == [[0, 100, 0], [100, 0, -1]]
    # Upper-left image pixel is (x=0.25,y=0.75) relative to map origin;
    # after 90deg origin rotation it is (-0.75,+0.25).
    assert occupancy.grid_to_world(0.5, 1.5) == pytest.approx((1.25, -2.75))
    assert occupancy.world_to_grid(1.25, -2.75) == pytest.approx((0.5, 1.5))
    meta = yaml.safe_load(path.read_text())
    meta['negate'] = 1
    path.write_text(yaml.safe_dump(meta))
    assert load_map(path).cells.tolist() == [[100, 0, 100], [0, 100, -1]]


def test_binary_pgm_preserves_whitespace_pixel_and_16bit_range(tmp_path):
    path = saved_map(tmp_path, b'P5\n3 1\n255\n' + bytes([10, 255, 128]))
    assert load_map(path).cells.tolist() == [[100, 0, -1]]
    # PGM 16-bit raster is big endian, and its declared range need not be 65535.
    path = saved_map(tmp_path, b'P5\r\n3 1\r\n1000\r\n' + np.array([0, 1000, 500], dtype='>u2').tobytes())
    assert load_map(path).cells.tolist() == [[100, 0, -1]]


def test_png_transparent_pixels_are_unknown(tmp_path):
    image = np.array([[[0, 0, 0, 255], [255, 255, 255, 255],
                       [0, 0, 0, 0], [255, 255, 255, 100]]], dtype=np.uint8)
    cv2.imwrite(str(tmp_path / 'input.png'), image)
    path = saved_map(tmp_path, b'P2\n1 1\n255\n0\n', image='input.png')
    assert load_map(path).cells.tolist() == [[100, 0, -1, -1]]


@pytest.mark.parametrize('changes', [
    {'mode': 'raw'}, {'resolution': 0}, {'resolution': float('nan')},
    {'origin': [0, 0]}, {'origin': [0, 0, float('inf')]},
    {'free_thresh': 0.8, 'occupied_thresh': 0.5}, {'negate': 2},
])
def test_invalid_metadata_is_rejected(tmp_path, changes):
    path = saved_map(tmp_path, b'P2\n1 1\n255\n0\n', **changes)
    with pytest.raises(ValueError):
        load_map(path)


def test_binary_pgm_truncation_rejected(tmp_path):
    path = saved_map(tmp_path, b'P5\n2 2\n255\n\x00')
    with pytest.raises(ValueError, match='Truncated'):
        load_map(path)


def test_rectangle_merging_has_exact_coverage_without_bridging_holes():
    generator = np.random.default_rng(42)
    for mask in (np.ones((7, 9), dtype=bool), generator.random((40, 50)) < 0.3):
        rectangles = merge_rectangles(mask)
        reconstructed = np.zeros(mask.shape, dtype=np.uint8)
        for rectangle in rectangles:
            reconstructed[rectangle.y:rectangle.y + rectangle.height,
                          rectangle.x:rectangle.x + rectangle.width] += 1
        np.testing.assert_array_equal(reconstructed, mask.astype(np.uint8))
    assert len(merge_rectangles(np.ones((100, 100)))) == 1
    assert not merge_rectangles(np.zeros((5, 5)))


def test_spawn_checks_occupied_unknown_boundary_and_footprint():
    occupancy = free_map()
    validate_spawn(occupancy, 1.0, 1.0, 0.0)
    with pytest.raises(ValueError, match='outside'):
        validate_spawn(occupancy, 0.1, 1.0, 0.0)
    occupancy.cells[10, 12] = 100
    with pytest.raises(ValueError, match='clearance overlaps'):
        validate_spawn(occupancy, 1.0, 1.0, 0.0)
    # Explicit clearance zero still rejects unknown/occupied robot center.
    occupancy.cells[10, 10] = -1
    with pytest.raises(ValueError, match='known free'):
        validate_spawn(occupancy, 1.05, 1.05, 0.0, clearance=0)


def test_render_geometry_matches_map_rotation_and_physical_robot():
    cells = np.zeros((4, 5), dtype=np.int8)
    cells[2:4, 0:2] = 100
    cells[0, 4] = -1
    occupancy = OccupancyMap(cells, 1.0, (4.0, 8.0, math.pi / 2), Path('rotated.yaml'))
    sx, sy = occupancy.grid_to_world(3, 1)
    world, report = render_world(occupancy, robot_x=sx, robot_y=sy, robot_yaw=0.3)
    assert report['occupied_rectangles'] == 1
    assert report['occupied_cells'] == 4
    assert report['unknown_cells'] == 1
    # Wall center grid=(1,3), world=(1,9), length=2x2, half-height=1.
    assert 'DEF MAP_WALL_0 Solid {\n  translation 1 9 1' in world
    assert 'size 2 2 2' in world
    assert 'boundingObject USE WALL_GEOMETRY_0' in world
    assert 'DEF LIMO LimoFourDiff' in world
    assert 'controller "<extern>"' in world
    assert 'supervisor TRUE' in world
    assert 'rotation 0 0 1 0.3' in world
    assert 'coordinateSystem "ENU"' in world
    assert 'orientation 0 1 0 1.5707963267948966' in world
    # Unknown was not extruded as a second wall.
    assert len(re.findall(r'DEF MAP_WALL_\d+ Solid', world)) == 1


def test_api_and_cli_write_new_world_without_overwriting_existing_map(tmp_path, capsys):
    path = saved_map(tmp_path, b'P2\n4 4\n255\n' + b'255 ' * 16, origin=[0, 0, 0])
    before = path.read_bytes()
    output = tmp_path / 'scene.wbt'
    report = generate_world(path, output, robot_x=1.0, robot_y=1.0, robot_yaw=0.2)
    assert output.is_file()
    assert report['initial_pose'] == {'x': 1.0, 'y': 1.0, 'yaw': 0.2}
    assert path.read_bytes() == before
    original = output.read_bytes()
    with pytest.raises(ValueError, match='already exists'):
        generate_world(path, output, robot_x=0.8, robot_y=0.8)
    assert output.read_bytes() == original
    main([str(path), '--output', str(output), '--force', '--robot-x', '0.8', '--robot-y', '0.8', '--robot-yaw', '0'])
    assert '"initial_pose": {"x": 0.8' in capsys.readouterr().out
    assert not list(tmp_path.glob('*.tmp'))


def test_rectangle_budget_prevents_accidental_huge_world():
    occupancy = free_map()
    occupancy.cells[0, 0] = 100
    with pytest.raises(ValueError, match='exceeding'):
        render_world(occupancy, robot_x=1, robot_y=1, max_rectangles=1)
