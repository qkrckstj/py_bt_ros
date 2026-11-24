import math
import numpy as np

from webots_ros2_limo.exploration_radio import RadioNetwork, physical_radio_permit, select_physical_commands
from webots_ros2_limo.exploration_policy import FrontierGrid
from webots_ros2_limo.exploration_runtime import TeamBT
from test_exploration_allocation import state


def test_multihop_graph_keeps_all_robots_reachable_without_direct_base_links():
    radio = RadioNetwork([0., 0.], 7., .5)
    positions = {f'r{i}': [6.*i, 0.] for i in range(1, 5)}
    graph = radio.graph(positions)
    assert graph['all_connected'] and graph['hops']['r4'] == 4
    assert ('base', 'r4') not in graph['links']
    assert not radio.allows(positions, 'r2', [6., 6.])


def test_overlapping_discs_are_not_a_radio_link_and_neighbors_alone_do_not_suffice():
    radio = RadioNetwork([0., 0.], 7.)
    graph = radio.graph({'a': [1., 0.], 'b': [11., 0.], 'c': [12., 0.]})
    assert ('b', 'c') in graph['links'] and not graph['connected']['b']
    assert ('a', 'b') not in graph['links']
    assert not graph['all_connected']


def test_motion_shield_preserves_downstream_robots_and_checks_combined_commands():
    radio = RadioNetwork([0., 0.], 7., .5)
    positions = {'a': [6., 0.], 'b': [12., 0.], 'c': [18., 0.], 'd': [24., 0.]}
    granted, blocked = radio.constrain(positions, {n: 0. for n in positions},
                                      {'b': (-2., 0.)}, ['b', 'a', 'c', 'd'])
    assert 'b' in blocked and granted['b'][0] == 0.
    for i in range(1, 9):
        future = {n: radio.project(p, 0., *granted[n], .8*i/8) for n, p in positions.items()}
        assert radio.graph(future, safe=True)['all_connected']


def test_advance_stops_before_cutting_another_robots_only_path():
    radio = RadioNetwork([0., 0.], 7., .5)
    positions = {'a': [6., 0.], 'b': [12., 0.]}
    route = [(6., i*.1) for i in range(81)]
    goal = radio.advance(positions, 'a', route)
    assert goal[1] < 3.
    assert radio.allows(positions, 'a', goal)


def test_team_disconnect_branch_stops_every_robot_instead_of_marking_complete():
    radio = RadioNetwork([0., 0.], 7.)
    sent = []
    team = TeamBT(['a', 'b'], lambda n, c: sent.append(c), autostart=True, radio=radio)
    grid = FrontierGrid(np.zeros((80, 80)), .5)
    team.update_map(grid, 0.)
    team.update_robot('a', state([2., 2.]), 0.)
    team.update_robot('b', state([20., 20.]), 0.)
    team.tick(0.)
    assert team.phase == 'COMMUNICATION_LOST' and not team.finished
    assert all(c['mode'] == 'stop' for c in sent)


def test_return_sends_outer_leaf_first_so_it_does_not_abandon_a_relay():
    radio = RadioNetwork([0., 0.], 7., .5)
    team = TeamBT(['a', 'b', 'c'], lambda *a: None, autostart=True, radio=radio,
                  homes={'a': [1., 1., 0.], 'b': [2., 1., 0.], 'c': [3., 1., 0.]})
    team.update_map(FrontierGrid(np.zeros((80, 80)), .5), 0.)
    for name, p in {'a': [6., 0.], 'b': [12., 0.], 'c': [18., 0.]}.items():
        team.update_robot(name, state(p), 0.)
    team.exploration_done, team.exploration_result = True, 'COMPLETE'
    team.update_mapping({'session': team.session, 'mode': 'LOCALIZATION', 'ready': True, 'frozen': True}, 0.)
    team.tick(0.)
    assert list(team.active) == ['c']


def test_simulated_radio_interlock_accepts_multihop_but_holds_a_departing_relay():
    radio = RadioNetwork([0., 0.], 7., .5)
    positions = {f'r{i}': [6.*i, 0.] for i in range(1, 5)}
    yaws = {n: 0. for n in positions}
    assert physical_radio_permit(radio, positions, yaws, {}, positions)
    assert physical_radio_permit(radio, positions, yaws, {'r4': (.3, 0.)}, positions)
    assert not physical_radio_permit(radio, positions, yaws, {'r2': (-.8, 0.)}, positions)
    # Estimated coordinates may claim a perfect chain; the physical channel
    # still rejects the actual disconnected arrangement.
    disconnected = {**positions, 'r4': [27., 0.]}
    assert not physical_radio_permit(radio, disconnected, yaws, {}, positions)
    assert not physical_radio_permit(radio, {'r1': [1., 0.]}, yaws, {}, positions)


def test_measured_link_guard_overrides_connected_but_wrong_localization_and_expires():
    radio = RadioNetwork([0., 0.], 7.)
    team = TeamBT(['a', 'b'], lambda *a: None, autostart=True, radio=radio,
                  require_radio_measurement=True)
    team.update_map(FrontierGrid(np.zeros((80, 80)), .5), 0.)
    team.update_robot('a', state([2., 2.]), 0.)
    team.update_robot('b', state([3., 2.]), 0.)
    assert radio.graph(team.positions())['all_connected']
    team.radio_measurement = {'ready': True, 'connected': {'a': True, 'b': False}}
    team.radio_received = 0.
    team.tick(0.)
    assert team.phase == 'COMMUNICATION_LOST'
    team.radio_measurement['connected']['b'] = True
    assert team.communication_available()
    team.now = 1.1
    assert not team.communication_available()


def test_physical_interlock_holds_departing_relay_but_allows_unrelated_safe_motion():
    radio = RadioNetwork([0., 0.], 7., .5)
    positions = {f'r{i}': [6.*i, 0.] for i in range(1, 5)}
    yaws = dict.fromkeys(positions, 0.)
    commands = {'r2': (-.8, 0.), 'r4': (.3, 0.)}
    assert not physical_radio_permit(radio, positions, yaws, commands, positions)
    granted, blocked = select_physical_commands(radio, positions, yaws, commands, positions)
    assert granted['r2'] == (0., 0.) and granted['r4'] == commands['r4']
    assert blocked == ['r2']
    for t in np.arange(0., 1.201, .1):
        future = {n: radio.project(p, yaws[n], *granted[n], float(t)) for n, p in positions.items()}
        assert radio.graph(future, safe=True)['all_connected']
    assert all(granted[n] in ((0., 0.), commands.get(n, (0., 0.))) for n in positions)


def test_physical_interlock_rechecks_follower_collision_after_holding_leader():
    radio = RadioNetwork([0., 0.], 7., .5)
    positions = {'leader': [6.4, 0.], 'follower': [5.3, 0.], 'remote': [6.4, -6.49]}
    yaws = dict.fromkeys(positions, 0.)
    commands = {'leader': (.4, 0.), 'follower': (.4, 0.)}
    # Following together preserves their separation but breaks the remote link.
    assert not physical_radio_permit(radio, positions, yaws, commands, positions)
    # Merely stopping the leader fixes radio while creating a rear-end collision.
    follower_only = {'follower': commands['follower']}
    assert physical_radio_permit(radio, positions, yaws, follower_only, positions)
    assert not physical_radio_permit(radio, positions, yaws, follower_only, positions, clearance=.85)
    granted, blocked = select_physical_commands(radio, positions, yaws, commands, positions)
    assert granted == dict.fromkeys(positions, (0., 0.))
    assert blocked == ['leader', 'follower']


def test_physical_interlock_accepts_joint_motion_even_if_outer_cannot_go_alone():
    radio = RadioNetwork([0., 0.], 7., .5)
    positions = {'relay': [6., 0.], 'outer': [12.4, 0.]}
    yaws = dict.fromkeys(positions, 0.)
    commands = dict.fromkeys(positions, (.4, 0.))
    assert not physical_radio_permit(radio, positions, yaws, {'outer': (.4, 0.)}, positions)
    granted, blocked = select_physical_commands(radio, positions, yaws, commands, positions)
    assert granted == commands and not blocked


def test_physical_interlock_preserves_stale_missing_and_disconnected_holds():
    radio = RadioNetwork([0., 0.], 11., .5)
    positions, yaws, commands = {'r': [10.7, 0.]}, {'r': 0.}, {'r': (-.12, 0.)}
    granted, blocked = select_physical_commands(radio, positions, yaws, commands, positions)
    assert granted == commands and not blocked  # Connected margin-band recovery.
    for p, y, fresh in ((positions, yaws, False), ({}, yaws, True),
                        ({'r': [11.01, 0.]}, yaws, True)):
        granted, blocked = select_physical_commands(radio, p, y, commands, ['r'], inputs_fresh=fresh)
        assert granted == {'r': (0., 0.)} and blocked == ['r']
    granted, blocked = select_physical_commands(radio, positions, yaws, {'r': (.4, 0.)}, ['r'])
    assert granted == {'r': (0., 0.)} and blocked == ['r']  # Never cross actual 11 m.


def test_soft_clearance_recovery_allows_separating_and_independent_motion():
    from webots_ros2_limo.exploration_radio import MobileRadioNetwork
    radio = MobileRadioNetwork(10., .3)
    positions = {'a': (0., 0.), 'b': (.83, 0.), 'c': (3., 3.), 'd': (5., 3.)}
    yaws = dict.fromkeys(positions, 0.)
    commands = {'a': (-.1, 0.), 'c': (0., .2), 'd': (.1, 0.)}
    # Reproduce the observed all-stop latch at 83 cm.
    strict, _ = select_physical_commands(radio, positions, yaws, commands, positions)
    assert not any(any(c) for c in strict.values())
    granted, _ = select_physical_commands(radio, positions, yaws, commands, positions,
                                           recovery_floor=.65)
    assert granted['a'] == commands['a']
    assert granted['d'] == commands['d']
    assert physical_radio_permit(radio, positions, yaws, granted, positions,
                                  clearance=.85, recovery_floor=.65)


def test_recovery_still_blocks_closer_approach_and_hard_overlap():
    from webots_ros2_limo.exploration_radio import MobileRadioNetwork
    radio = MobileRadioNetwork(10., .3)
    positions = {'a': (0., 0.), 'b': (.83, 0.), 'c': (3., 3.), 'd': (5., 3.)}
    yaws = dict.fromkeys(positions, 0.)
    granted, blocked = select_physical_commands(radio, positions, yaws,
        {'a': (.1, 0.), 'd': (.1, 0.)}, positions, recovery_floor=.65)
    assert 'a' in blocked and granted['a'] == (0., 0.)
    assert granted['d'] == (.1, 0.)
    overlap = {**positions, 'b': (.6, 0.)}
    stopped, _ = select_physical_commands(radio, overlap, yaws,
        {'a': (-.1, 0.), 'd': (.1, 0.)}, overlap, recovery_floor=.65)
    assert not any(any(c) for c in stopped.values())


def test_recovery_cannot_break_radio_or_approach_another_pair():
    from webots_ros2_limo.exploration_radio import MobileRadioNetwork
    radio = MobileRadioNetwork(10., .3)
    positions = {'a': (0., 0.), 'b': (.83, 0.), 'c': (9., 0.), 'd': (18.5, 0.)}
    yaws = dict.fromkeys(positions, 0.)
    granted, _ = select_physical_commands(radio, positions, yaws,
        {'a': (-.1, 0.), 'd': (1., 0.)}, positions, recovery_floor=.65)
    assert granted['a'] == (-.1, 0.)
    assert granted['d'] == (0., 0.)
