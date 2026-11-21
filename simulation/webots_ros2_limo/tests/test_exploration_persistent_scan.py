import math

import pytest

from webots_ros2_limo.exploration_scan import PersistentMappingHits


def test_stable_wall_is_confirmed_after_history_and_ego_translation_is_compensated():
    f = PersistentMappingHits()
    for i in range(22):
        now = i*.1
        x = .4*now
        result = f.filter(now, [5.-x, math.inf], 0., .1, (x, 0.), 0.)
        assert result[1] == math.inf
        if i < 15:
            assert math.isnan(result[0])
    assert result[0] == pytest.approx(5.-x)


def test_moving_endpoint_does_not_paint_a_static_trail_or_clear_behind_it():
    f = PersistentMappingHits()
    for i in range(40):
        result = f.filter(i*.1, [2.+.4*i*.1], 0., .1, (0., 0.), 0.)
        assert math.isnan(result[0])  # unknown, never a free no-return ray


def test_rotation_compensation_preserves_a_static_endpoint():
    f = PersistentMappingHits()
    for i in range(25):
        yaw = i*.01
        result = f.filter(i*.1, [3.], -yaw, .1, (0., 0.), yaw)
    assert result[0] == 3.
