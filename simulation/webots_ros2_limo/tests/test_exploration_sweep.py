import math

import numpy as np

from webots_ros2_limo.exploration_policy import FrontierGrid
from webots_ros2_limo.exploration_radio import RadioNetwork
from webots_ros2_limo.exploration_runtime import TeamBT
from webots_ros2_limo.exploration_sweep import RadarSweep


ROBOTS = ('a', 'b', 'c', 'd')


def grid():
    cells = np.full((120, 120), -1, dtype=np.int8)
    cells[4:-4, 4:-4] = 0
    return FrontierGrid(cells, .5, (-30., -30., 0.))


def test_line_fits_15m_multihop_through_cardinal_and_diagonal_extents():
    sweep = RadarSweep(ROBOTS)
    radio = RadioNetwork((0., 0.), 15.)
    sweep.stage = 'ROTATE'
    for angle in np.linspace(0., math.tau, 73):
        sweep.angle = angle
        slots = sweep.slots()
        assert radio.graph(slots, safe=True)['all_connected']
        for slot in slots.values():
            assert max(abs(v) for v in slot) <= 29.+1e-8
        assert abs(slots['a'][0]*slots['d'][1]-slots['a'][1]*slots['d'][0]) < 1e-8


def test_circular_sweep_retains_four_hop_base_connection_at_11m():
    sweep = RadarSweep(ROBOTS, radius=28.)
    sweep.stage = 'ROTATE'
    radio = RadioNetwork((0., 0.), 11.)
    for angle in np.linspace(0., math.tau, 73):
        sweep.angle = angle
        graph = radio.graph(sweep.slots(), safe=True)
        assert graph['all_connected'] and graph['hops']['d'] == 4
        assert all(math.hypot(*p) <= 28.+1e-8 for p in sweep.slots().values())


def test_compact_formation_leaves_relay_and_boundary_detour_room():
    sweep = RadarSweep(ROBOTS, radius=20., slot_slack=.75)
    radio = RadioNetwork((0., 0.), 11.)
    for angle in np.linspace(0., math.tau, 73):
        sweep.angle = angle
        slots = sweep.slots()
        # Opposite 2 m lateral detours retain the planning margin, even without
        # relying on the extra skip-a-peer links of the compact nominal line.
        detours = {n: (p[0]-math.sin(angle)*2*(-1)**i,
                        p[1]+math.cos(angle)*2*(-1)**i)
                   for i, (n, p) in enumerate(slots.items())}
        assert radio.graph(detours, safe=True)['all_connected']
        assert max(math.hypot(*p) for p in detours.values()) < 21.


def test_soft_slots_accept_small_detours_but_do_not_complete_exploration():
    team = TeamBT(ROBOTS, lambda *a: None, strategy='radar',
                  sweep_radius=20., sweep_slot_slack=.75)
    sweep = team.sweep
    positions = {n: (p[0], 1.5*(-1)**i) for i, (n, p) in enumerate(sweep.slots().items())}
    observed = grid()
    before = observed.cells.copy()
    sweep.update(positions, ROBOTS, 0., observed)
    assert sweep.stage == 'ROTATE' and math.degrees(sweep.angle) == 5.
    assert not team.exploration_done and not team.finished
    assert np.array_equal(before, observed.cells)
    # Being far from the preferred sector still requires actual movement.
    for now in range(1, 20):
        sweep.update(positions, ROBOTS, now, observed)
    assert math.degrees(sweep.angle) < 20.


def test_soft_target_prefers_shorter_off_axis_approach_without_weakening_radio():
    positions = dict(zip(ROBOTS, [(5., 0.), (10., 0.), (15., 0.), (18., 1.)]))
    sweep = RadarSweep(ROBOTS, radius=20., slot_slack=.75)
    radio = RadioNetwork((0., 0.), 11.)
    observed = grid()
    goal = sweep.choose(observed, positions, ('d',), {}, radio)['d']
    assert goal.y > 0.  # No forced return to the exact zero-y line.
    assert math.dist(goal.xy, (20., 0.)) < 1.
    assert radio.allows(positions, 'd', goal.xy)
    assert observed.safe[observed.index(goal.xy)]
    assert math.isfinite(observed.distances(positions['d'])[observed.index(goal.xy)])


def test_arrived_formation_advances_without_waiting_for_all_action_results():
    sweep = RadarSweep(ROBOTS)
    sweep.radius = 28.
    sweep.update(sweep.slots(), (), 0., grid())
    assert sweep.stage == 'ROTATE' and math.degrees(sweep.angle) == 5.


def test_progress_requires_real_arrival_not_dispatch_or_elapsed_time():
    sweep = RadarSweep(ROBOTS)
    positions = {n: (i+1., 0.) for i, n in enumerate(ROBOTS)}
    sweep.update(positions, ROBOTS, 0., grid())
    sweep.update(positions, (), 30., grid())
    assert sweep.radius == 28. and sweep.angle == 0.
    sweep.update(sweep.slots(), ROBOTS, 31., grid())
    assert sweep.stage == 'ROTATE' and math.degrees(sweep.angle) == 5.


def test_final_deploy_has_no_submetre_waypoint_arrival_dead_zone():
    sweep = RadarSweep(ROBOTS)
    sweep.radius = 29.
    positions = sweep.slots()
    positions['d'] = (28.09, 0.)
    sweep.update(positions, ROBOTS, 0., grid())
    assert sweep.stage == 'ROTATE' and math.degrees(sweep.angle) == 5.


def test_goals_are_known_reachable_separated_and_do_not_jump_wall_or_unknown():
    cells = np.full((100, 100), -1, dtype=np.int8)
    cells[40:60, 45:65] = 0
    cells[40:60, 60] = 100  # No route through this wall to the free far side.
    observed = FrontierGrid(cells, .2, (-10., -10., 0.), clearance=.2)
    sweep = RadarSweep(ROBOTS)
    positions = dict(zip(ROBOTS, [(-.7, -.6), (-.7, .6), (.7, -.6), (.7, .6)]))
    goals = sweep.choose(observed, positions, positions, {})
    assert goals
    for name, goal in goals.items():
        assert goal.x < 2.
        assert observed.cells[observed.index(goal.xy)] == 0
        assert math.isfinite(observed.distances(positions[name])[observed.index(goal.xy)])
    for i, a in enumerate(goals.values()):
        for b in list(goals.values())[i+1:]:
            assert math.dist(a.xy, b.xy) >= 1.1


def test_full_revolution_or_stall_only_enters_frontier_cleanup():
    sweep = RadarSweep(ROBOTS)
    observed = grid()
    for now in range(100):
        sweep.update(sweep.slots(), ROBOTS, float(now), observed)
        if sweep.stage == 'CLEANUP':
            break
    assert sweep.stage == 'CLEANUP' and sweep.angle == math.tau
    stuck = RadarSweep(ROBOTS)
    p = {n: (i+1., 0.) for i, n in enumerate(ROBOTS)}
    stuck.update(p, ROBOTS, 0., observed)
    stuck.update(p, ROBOTS, 91., observed)
    assert stuck.stage == 'CLEANUP' and stuck.reason
    team = TeamBT(ROBOTS, lambda *a: None, autostart=True, strategy='radar')
    team.sweep = stuck
    team.update_map(observed, 0.)
    for name, pos in p.items():
        team.update_robot(name, {'ready': True, 'status': 'IDLE', 'position': pos}, 0.)
    team.tick(0.)
    assert not team.exploration_done and not team.finished
    assert team.active  # Existing frontier allocator takes over.


def test_team_starts_outer_leader_first_and_preserves_radio_and_execution_protocol():
    team = TeamBT(ROBOTS, lambda *a: None, autostart=True, strategy='radar',
                  radio=RadioNetwork((0., 0.), 15.))
    positions = dict(zip(ROBOTS, [(1.5, -.7), (1.5, .7), (2.9, -.7), (2.9, .7)]))
    team.update_map(grid(), 0.)
    for name, pos in positions.items():
        team.update_robot(name, {'ready': True, 'status': 'IDLE', 'position': pos}, 0.)
    team.tick(0.)
    assert set(team.active) == {'d'}
    assert all(t['formation'] and t['purpose'] == 'explore' for t in team.active.values())
    assert team.snapshot()['sweep']['stage'] == 'DEPLOY'
    assert team.radio.graph({**positions, **{n: t['goal'][:2] for n, t in team.active.items()}})['all_connected']


def test_followers_wait_for_real_predecessor_progress_and_never_chase_future_goal():
    sweep = RadarSweep(ROBOTS)
    positions = dict(zip(ROBOTS, [(1.5, -.7), (1.5, .7), (2.9, -.7), (2.9, .7)]))
    future = {'d': {'goal': [20., 0., 0.]}}
    assert sweep.choose(grid(), positions, ('a', 'b', 'c'), future) == {}
    positions['d'] = (12., 0.)
    targets = sweep.deployment_targets(positions)
    assert targets['c'] == (5., 0.)
    assert targets['b'] == positions['b'] and targets['a'] == positions['a']
    goals = sweep.choose(grid(), positions, ('a', 'b', 'c'), {}, RadioNetwork((0., 0.), 11.))
    assert set(goals) == {'c'} and goals['c'].x > positions['c'][0]
    positions['c'] = (11., 0.)
    goals = sweep.choose(grid(), positions, ('a', 'b'), {}, RadioNetwork((0., 0.), 11.))
    assert set(goals) == {'b'}


def test_followers_do_not_reverse_when_predecessor_slows_or_localization_shifts():
    sweep = RadarSweep(ROBOTS)
    positions = dict(zip(ROBOTS, [(7., 0.), (14., 0.), (21., 0.), (26., 0.)]))
    targets = sweep.deployment_targets(positions)
    assert targets['c'] == positions['c']
    assert not sweep.choose(grid(), positions, ('a', 'b', 'c'), {})


def test_follower_finishes_lateral_alignment_at_final_radius_after_a_detour():
    sweep = RadarSweep(ROBOTS)
    positions = dict(zip(ROBOTS, [(7., 0.), (14., 2.), (21., 0.), (28., 0.)]))
    radio = RadioNetwork((0., 0.), 11.)
    sweep.update(positions, ROBOTS, 0., grid())
    assert sweep.stage == 'DEPLOY'
    assert sweep.deployment_targets(positions)['b'] == (14., 0.)
    goals = sweep.choose(grid(), positions, ROBOTS, {}, radio)
    assert set(goals) == {'b'}
    assert abs(goals['b'].y) < abs(positions['b'][1])
    assert radio.allows(positions, 'b', goals['b'].xy)
    positions['b'] = goals['b'].xy
    sweep.update(positions, ROBOTS, 1., grid())
    assert sweep.stage == 'ROTATE'


def test_lateral_alignment_uses_actual_predecessor_and_never_moves_target_backwards():
    sweep = RadarSweep(ROBOTS)
    positions = dict(zip(ROBOTS, [(7., 0.), (14.2, 2.), (21., 0.), (28., 0.)]))
    assert sweep.deployment_targets(positions)['b'] == (14.2, 0.)
    positions['c'] = (18., 0.)
    assert sweep.deployment_targets(positions)['b'] == positions['b']
    # A future predecessor goal does not open the alignment gate.
    assert not sweep.choose(grid(), positions, ('b',), {'c': {'goal': [21., 0., 0.]}})


def test_deployment_uses_longer_observed_legs_without_exceeding_six_metres():
    sweep = RadarSweep(ROBOTS)
    positions = dict(zip(ROBOTS, [(1.5, -.7), (1.5, .7), (2.9, -.7), (2.9, .7)]))
    goal = sweep.choose(grid(), positions, ROBOTS, {})['d']
    assert 4. < math.dist(positions['d'], goal.xy) <= 6.1


def test_unknown_crater_slot_skips_after_two_idle_seconds_without_claiming_it_explored():
    cells = np.full((80, 80), -1, dtype=np.int8)
    cells[1:-1, 1:-1] = 0
    xs, ys = np.meshgrid(-5.+(np.arange(80)+.5)*.5, -10.+(np.arange(80)+.5)*.5)
    cells[(xs >= 5.) & (xs < 9.) & (ys >= -2.) & (ys < 2.)] = -1
    observed = FrontierGrid(cells, .5, (-5., -10., 0.))
    radio = RadioNetwork((0., 0.), 11.)
    team = TeamBT(ROBOTS, lambda *a: None, autostart=True, strategy='radar', radio=radio)
    team.sweep.stage = 'ROTATE'
    positions = team.sweep.slots()
    positions['a'] = (9.4, 0.)  # Closest safe approach to the unknown slot at (7, 0).
    assert observed.cells[observed.index(team.sweep.slots()['a'])] == -1
    assert not team.sweep.choose(observed, positions, ROBOTS, {}, radio)
    team.update_map(observed, 0.)
    for name, pos in positions.items():
        team.update_robot(name, {'ready': True, 'status': 'IDLE', 'position': pos}, 0.)
    team.tick(0.)
    team.tick(1.99)
    assert not team.active and team.sweep.angle == 0.
    team.tick(2.)
    assert set(team.active) == set(ROBOTS)
    assert math.isclose(math.degrees(team.sweep.angle), 5.)
    assert team.sweep.snapshot()['skipped_sectors'][0]['angle_degrees'] == 0.
    assert not team.exploration_done and not team.finished and not team.history
    assert team.mapping_mode == 'MAPPING'
    np.testing.assert_array_equal(observed.cells, cells)
    projected = {name: task['goal'][:2] for name, task in team.active.items()}
    assert radio.graph(projected, safe=True)['all_connected']
    for name, point in projected.items():
        assert observed.cells[observed.index(point)] == 0
        assert math.isfinite(observed.distances(positions[name])[observed.index(point)])


def test_sector_skip_requires_continuous_all_idle_rotation_and_stops_at_cleanup():
    sweep = RadarSweep(ROBOTS)
    assert not sweep.skip_unavailable_sector(ROBOTS, {}, {}, 0.)
    assert not sweep.skip_unavailable_sector(ROBOTS, {}, {}, 3.)
    assert sweep.stage == 'DEPLOY' and sweep.angle == 0.
    sweep.stage = 'ROTATE'
    assert not sweep.skip_unavailable_sector(ROBOTS, {}, {}, 4.)
    assert not sweep.skip_unavailable_sector(ROBOTS[:-1], {'d': {}}, {}, 6.)
    assert not sweep.skip_unavailable_sector(ROBOTS, {}, {}, 7.)
    assert not sweep.skip_unavailable_sector(ROBOTS, {}, {'d': object()}, 9.)
    assert not sweep.skip_unavailable_sector(ROBOTS, {}, {}, 10.)
    assert sweep.skip_unavailable_sector(ROBOTS, {}, {}, 12.)
    assert math.isclose(math.degrees(sweep.angle), 5.)
    assert not sweep.skip_unavailable_sector(ROBOTS, {}, {}, 12.)
    assert len(sweep.skipped_sectors) == 1
    sweep.angle = math.tau-sweep.step
    sweep.unavailable_since = None
    assert not sweep.skip_unavailable_sector(ROBOTS, {}, {}, 13.)
    assert sweep.skip_unavailable_sector(ROBOTS, {}, {}, 15.)
    assert sweep.stage == 'CLEANUP' and sweep.angle == math.tau
    assert not sweep.skip_unavailable_sector(ROBOTS, {}, {}, 18.)
