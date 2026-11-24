import math

import pytest
from webots_ros2_limo.exploration_localization import MappingScanGate, integrate, scan_geometry, slam_prior
from webots_ros2_limo.exploration_viewer import SnapshotNode
from webots_ros2_limo.exploration_bt import Status


def test_webots_fixed_lidar_pixel_centers_match_ros_rays():
    fov, count = 3.839724, 248
    low, high, step = scan_geometry(fov, count)
    for i in (0, 1, 123, 247):
        webots_angle = fov/2-(count-1-i+.5)*fov/count
        assert low+i*step == pytest.approx(webots_angle)
    assert low+(count-1)*step == pytest.approx(high)
    assert high < fov/2


def test_midpoint_turn_integrates_on_correct_side_of_yaw_wrap():
    x, y, change = integrate(0, 0, 1, math.radians(179), math.radians(-179))
    assert x == pytest.approx(-1)
    assert y == pytest.approx(0, abs=1e-12)
    assert change == pytest.approx(math.radians(2))


def test_imu_profile_obeys_karto_positive_window_and_step_contract():
    prior = slam_prior('imu_prior')
    # Karto GridIndexLookup requires nonzero angular window AND resolution;
    # the misleadingly named fine offset is a step, not a zero-lock switch.
    assert prior['coarse_search_angle_offset'] > 0
    assert prior['coarse_angle_resolution'] > 0
    assert prior['fine_search_angle_offset'] > 0
    assert not prior['use_response_expansion']


def test_mapping_rejects_floor_intersecting_tilt_and_waits_for_settling():
    gate = MappingScanGate()
    assert gate.accept(0., 0., 0., 0.)
    assert not gate.accept(.1, 0., math.radians(2.), 0.)
    assert gate.reason == 'tilted_scan_plane'
    assert not gate.accept(.2, 0., 0., 0.)
    assert gate.accept(.4, 0., 0., 0.)
    assert not gate.accept(.5, 0., 0., .6)
    assert not gate.accept(.6, 0., 0., 0.)
    assert gate.accept(1., 0., 0., 0.)


def test_original_renderer_gets_real_status_and_condition_shape():
    node = SnapshotNode({'name': 'RobotBT', 'status': 'RUNNING', 'children': [
        {'name': 'UnavailableOrStopped', 'status': 'FAILURE'},
        {'name': 'CancelAndWait', 'status': None}]})
    assert node.status is Status.RUNNING
    assert node.children[0].type == 'Condition'
    assert node.children[0].status is Status.FAILURE
    assert node.children[1].status is None
