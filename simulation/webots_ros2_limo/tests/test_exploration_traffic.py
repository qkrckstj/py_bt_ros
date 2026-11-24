import math
import numpy as np

from webots_ros2_limo.exploration_radio import RadioNetwork
from webots_ros2_limo.exploration_traffic import (
    constrain_motion, reserve_routes, route_window, routes_conflict,
)


def test_distant_crossing_does_not_serialize_both_robots():
    a = np.array([[-4., 0.], [4., 0.]])
    b = np.array([[0., -4.], [0., 4.]])
    assert routes_conflict(np.linspace(a[0], a[-1], 100), np.linspace(b[0], b[-1], 100), .85)
    accepted, _ = reserve_routes({'a': a[0], 'b': b[0]}, {'a': a, 'b': b}, ['a', 'b'])
    assert set(accepted) == {'a', 'b'}
    assert all(np.linalg.norm(p[-1]-p[0]) <= 1.201 for p in accepted.values())


def test_following_uses_same_time_positions_and_waiting_peer_still_blocks():
    positions = {'leader': [1.4, 0.], 'follower': [0., 0.]}
    routes = {n: np.array([p, [10., 0.]]) for n, p in positions.items()}
    accepted, _ = reserve_routes(positions, routes, ['leader', 'follower'])
    assert set(accepted) == set(positions)
    accepted, _ = reserve_routes(positions, {'follower': routes['follower']}, ['follower'])
    assert not accepted


def test_imminent_crossing_is_reserved_for_one_robot():
    positions = {'a': [0., -1.], 'b': [-1., 0.]}
    routes = {'a': np.array([[0., -1.], [0., 3.]]),
              'b': np.array([[-1., 0.], [3., 0.]])}
    accepted, _ = reserve_routes(positions, routes, ['a', 'b'])
    assert set(accepted) == {'a'}


def test_stationary_leader_blocks_actual_command_even_when_paths_predict_motion():
    p = {'leader': [1.1, 0.], 'follower': [0., 0.]}
    accepted, blocked = constrain_motion(p, dict.fromkeys(p, 0.),
        {'leader': (0., 0.), 'follower': (.4, 0.)}, ['leader', 'follower'])
    assert accepted['follower'][0] == 0 and blocked['follower'] == 'collision'


def test_combined_radio_and_collision_guard_preserves_graph_at_every_sample():
    p = {'a': [1., 0.], 'b': [7., 0.], 'c': [13., 0.]}
    radio = RadioNetwork([0., 0.], 7., .5)
    accepted, blocked = constrain_motion(p, dict.fromkeys(p, 0.),
        {'b': (-.6, 0.), 'c': (.4, 0.)}, ['b', 'a', 'c'], radio=radio)
    assert blocked['b'] == 'radio'
    for t in np.arange(.1, 1.201, .1):
        future = {n: radio.project(xy, 0., *accepted[n], float(t)) for n, xy in p.items()}
        assert radio.graph(future, safe=True)['all_connected']
        assert all(math.dist(future[a], future[b]) >= .85 for a in p for b in p if a != b)


def test_radio_margin_band_allows_inward_recovery_through_a_relay():
    radio = RadioNetwork([0., 0.], 11., .5)
    positions = {'relay': [7., 0.], 'outer': [17.7, 0.]}
    assert radio.graph(positions)['all_connected']
    assert not radio.graph(positions, safe=True)['all_connected']
    accepted, blocked = constrain_motion(positions, dict.fromkeys(positions, 0.),
        {'outer': (-.12, 0.)}, ['relay', 'outer'], radio=radio)
    assert accepted['outer'] == (-.12, 0.)
    assert not blocked
    for t in np.arange(.1, 1.201, .1):
        future = {n: radio.project(p, 0., *accepted[n], float(t)) for n, p in positions.items()}
        assert radio.graph(future)['all_connected']


def test_radio_margin_recovery_never_allows_crossing_actual_range():
    radio = RadioNetwork([0., 0.], 11., .5)
    positions = {'outer': [10.7, 0.]}
    accepted, blocked = constrain_motion(positions, {'outer': 0.},
        {'outer': (.4, 0.)}, ['outer'], radio=radio)
    assert accepted['outer'][0] == 0.
    assert blocked == {'outer': 'radio'}
    # A graph already within the margin must continue preserving it.
    accepted, blocked = constrain_motion({'outer': [10.4, 0.]}, {'outer': 0.},
        {'outer': (.4, 0.)}, ['outer'], radio=radio)
    assert accepted['outer'][0] == 0.
    assert blocked == {'outer': 'radio'}


def test_actual_radio_disconnect_holds_even_a_command_that_would_reconnect():
    radio = RadioNetwork([0., 0.], 11., .5)
    positions = {'outer': [11.01, 0.]}
    accepted, blocked = constrain_motion(positions, {'outer': 0.},
        {'outer': (-.4, .2)}, ['outer'], radio=radio)
    assert accepted == {'outer': (0., 0.)}
    assert blocked == {'outer': 'radio'}


def test_window_interpolates_corner_without_skipping_path_distance():
    route = np.array([[0., 0.], [1., 0.], [1., 2.]])
    assert np.allclose(route_window(route, 1.2), [[0, 0], [1, 0], [1, .2]])
