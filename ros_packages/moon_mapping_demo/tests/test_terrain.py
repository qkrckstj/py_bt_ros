"""실제 60m 월드 파싱과 연속 구간의 보수적 몸체/그림자 판정."""
from pathlib import Path
from collections import OrderedDict
import math

import numpy as np
import pytest

from moon_mapping_demo.terrain import TerrainModel, _segment_hits_boxes, _trig_range, sun_direction


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def terrain():
    return TerrainModel.from_world(PACKAGE/'worlds/moon_terrain.wbt',
                                  PACKAGE/'controllers/lunar_sun/sun_config.json')


def test_current_world_geometry_and_starts_are_read(terrain):
    assert terrain.bounds == (-30, 30, -30, 30)
    assert terrain.triangle_count > 60000
    assert set(terrain.robot_starts) == {'robot1', 'robot2', 'robot3', 'robot4'}
    assert terrain.robot_starts['robot1'] == {'x': -26., 'y': -26., 'yaw': 0.}
    assert len(terrain.obstacle_boxes) > 1000
    assert len(terrain.identifier) == 64


def test_body_and_swept_segment_not_just_center_endpoints():
    boxes = np.array([[1., 1., 1.1, 1.1]])
    assert _segment_hits_boxes((0, 1.2), (2, 1.2), boxes, .11)
    assert not _segment_hits_boxes((0, 1.3), (2, 1.3), boxes, .1)
    assert _segment_hits_boxes((0, 0), (2, 2), boxes, 0)


def test_static_waiting_robot_remains_an_obstacle(terrain):
    field = terrain.with_static_robots([(-26, -24.5, .24)])
    assert field.blocked_segment((-27, -24.5), (-25, -24.5), 0, 10, .24)
    assert field.blocked_segment((-26, -24.1), (-26, -24.1), 0, 0, .24)
    assert not terrain.blocked_segment((-26, -26), (-26, -26), 0, 0, .24)


def test_boundaries_and_nonflat_terrain_forbidden(terrain):
    assert terrain.blocked_segment((-29.9, -26), (-29.9, -26), 0, 0, .24)
    assert terrain.blocked_segment((-18, -25), (-18, -25), 0, 0, .24)


def test_shadow_interval_contains_interior_extremum_and_all_snapshots(terrain):
    boxes = terrain.shadow_boxes(0, 120)
    for t in np.linspace(0, 120, 31):
        instant = terrain.shadow_boxes(float(t), float(t))
        assert np.all(boxes[:, :2] <= instant[:, :2]+1e-9)
        assert np.all(boxes[:, 2:] >= instant[:, 2:]-1e-9)
    assert _trig_range(0, math.pi) == (0., 1.)


def test_same_sun_enu_absolute_time_equation():
    assert np.allclose(sun_direction(0, 15, 270, 240), [-math.cos(math.pi/12), 0, math.sin(math.pi/12)])
    assert np.allclose(sun_direction(240, 15, 270, 240), sun_direction(0, 15, 270, 240))


def test_shadow_crosses_waiting_body_between_safe_endpoints():
    field = TerrainModel()
    field.bounds = (-5, 5, -5, 5)
    field.obstacle_boxes = np.array([[0., 0., .1, .1]])
    field.boundary_boxes = np.empty((0, 4))
    field._caster_boxes = field.obstacle_boxes.copy()
    field._caster_heights = np.array([1.])
    field._shadow_cache = OrderedDict()
    field._static_cache = OrderedDict()
    field.static_robots = ()
    field.elevation, field.initial_azimuth, field.period = 45., 0., 240.
    field.safety_time_step = .25
    p = (-.8, .05)
    assert not field.blocked_segment(p, p, 0, 0, .05)
    assert not field.blocked_segment(p, p, 120, 120, .05)
    assert field.blocked_segment(p, p, 60, 60, .05)
    assert field.blocked_segment(p, p, 0, 120, .05)


def test_legacy_shadow_region_filter_matches_full_sweep():
    terrain=TerrainModel.from_world(PACKAGE/'worlds/moon_terrain.wbt',
        PACKAGE/'controllers/lunar_sun/sun_config.json',geometry_mode='legacy_columns')
    # 격자·시간 경계 양옆, 반경 변경, 정지·장거리 이동과 겹치는 시간 구간.
    rng = np.random.default_rng(732)
    for index in range(800):
        a = tuple(rng.uniform(-29, 29, 2))
        b = a if index % 3 == 0 else tuple(np.add(a, rng.uniform(-3, 3, 2)))
        t = (index % 17) * 16 + (0., .00001, -.00001)[index % 3] + 16
        end = t + (0., .25, 3., 22., 90.)[index % 5]
        for radius in (.26, .31):
            expected = terrain.static_blocked_segment(a, b, radius) or _segment_hits_boxes(
                a, b, terrain.shadow_boxes(t, end), radius)
            assert terrain.blocked_segment(a, b, t, end, radius) == expected
