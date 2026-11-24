"""Mission exhaustion, freeze acknowledgement and return are distinct stages."""
import copy

import numpy as np

from webots_ros2_limo.exploration_mapping import MappingMode
from webots_ros2_limo.exploration_policy import FrontierGrid
from webots_ros2_limo.exploration_runtime import TeamBT, RobotBT
from test_exploration_allocation import Navigation, make_grid, state


def enclosed_grid():
    cells = np.full((60, 60), 100, dtype=np.int8)
    cells[2:-2, 2:-2] = 0
    # Unknown interiors of solid objects need not be observed for exhaustion.
    cells[25:35, 25:35] = 100
    cells[27:33, 27:33] = -1
    return FrontierGrid(cells, .2)


def inputs(team, grid, wall, sim, positions=None):
    team.update_map(grid, wall)
    for name in team.robots:
        previous = team.states.get(name, ({},))[0]
        team.update_robot(name, state((positions or {}).get(name, previous.get('position', [2., 2.])),
                                     **{k: v for k, v in previous.items() if k not in ('position', 'ready')}), wall)
    team.tick(wall, sim)


def frozen(team, now):
    team.update_mapping({'session': team.session, 'mode': 'LOCALIZATION',
                         'frozen': True, 'ready': True}, now)


def test_five_simulation_seconds_then_freeze_before_any_home_goal():
    sent = []
    team = TeamBT(['a'], lambda n, c: sent.append(c), autostart=True)
    grid = enclosed_grid()
    inputs(team, grid, 0., 0.)
    inputs(team, grid, 4., 0.)  # Paused simulator: wall time must not finish it.
    assert not team.exploration_done
    inputs(team, grid, 5., 4.99)
    assert not team.exploration_done
    inputs(team, grid, 6., 5.)
    assert team.phase == 'FREEZING_MAP' and team.exploration_done
    assert not team.active and not team.finished
    assert all(c['mode'] == 'stop' for c in sent)
    frozen(team, 6.5)
    inputs(team, grid, 6.5, 5.5, {'a': [8., 8.]})
    assert team.phase == 'RETURNING'
    assert team.active['a']['goal'] == [2., 2., 0.]
    assert team.active['a']['purpose'] == 'return'


def test_frontier_reappearance_or_stale_input_resets_five_second_check():
    team = TeamBT(['a'], lambda *a: None, autostart=True)
    inputs(team, enclosed_grid(), 0., 0.)
    inputs(team, make_grid(), 4., 4.)
    assert team.complete_since is None and team.active
    # A temporary stale message stream cannot certify completion either.
    team.active.clear()
    inputs(team, enclosed_grid(), 5., 5.)
    team.tick(10., 10.)
    assert team.phase == 'WAITING' and team.complete_since is None
    inputs(team, enclosed_grid(), 11., 11.)
    assert not team.exploration_done


def test_assignment_cooldown_does_not_mean_no_exploration_space():
    team = TeamBT(['a'], lambda *a: None, autostart=True)
    team.excluded = [((5., 5.), 100., 1000.)]
    for t in (0., 5., 10.):
        inputs(team, make_grid(), t, t, {'a': [5., 5.]})
    assert team.phase == 'WAITING_FOR_RETRY' and not team.exploration_done


def test_unreachable_frontiers_return_as_partial_not_successful_full_coverage():
    team = TeamBT(['a'], lambda *a: None, autostart=True)
    cells = enclosed_grid().cells.copy()
    cells[5:10, 5:10] = -1
    grid = FrontierGrid(cells, .2)
    # Put the robot in a different enclosed free island.
    cells[:, 20:23] = 100
    grid = FrontierGrid(cells, .2)
    for t in (0., 5.):
        inputs(team, grid, t, t, {'a': [9., 9.]})
    assert team.exploration_result == 'PARTIAL' and team.phase == 'FREEZING_MAP'


def test_four_serial_returns_and_all_arrivals_required():
    robots = {n: RobotBT(Navigation()) for n in ('a', 'b', 'c', 'd')}
    now = 0.
    homes = {n: [2.+i, 2., 1.57] for i, n in enumerate(robots)}
    team = TeamBT(robots, lambda n, c: robots[n].receive(c, now),
                  autostart=True, homes=homes)
    grid = enclosed_grid()
    for now in (0., 5.):
        inputs(team, grid, now, now, {n: [8., 8.] for n in robots})
    for name in robots:
        now += .5
        frozen(team, now)
        inputs(team, grid, now, now)
        assert list(team.active) == [name]
        for robot in robots.values():
            robot.tick(now)
        robots[name].navigation.state = 'SUCCEEDED'
        robots[name].tick(now+.1)
        assert robots[name].status == 'RETURNED'  # No mapping observation delay.
        team.update_robot(name, state(homes[name][:2], **robots[name].snapshot()), now+.1)
        assert name in team.returned
        assert not team.finished
    now += .5
    frozen(team, now)
    inputs(team, grid, now, now)
    assert team.finished and team.phase == 'COMPLETE' and len(team.returned) == 4


def test_failed_return_retries_but_never_marks_mission_complete():
    team = TeamBT(['a'], lambda *a: None, autostart=True, return_retries=1)
    grid = enclosed_grid()
    for t in (0., 5.):
        inputs(team, grid, t, t)
    for t in (6., 12.):
        frozen(team, t)
        inputs(team, grid, t, t)
        task = team.active['a']
        # Even a success message at the wrong position is rejected.
        team.update_robot('a', state([9., 9.], task_id=task['task_id'], session=team.session,
                                     status='RETURNED'), t+.1)
    frozen(team, 13.)
    inputs(team, grid, 13., 13.)
    assert team.phase == 'RETURN_FAILED' and not team.finished
    assert team.return_failed == {'a'} and not team.returned


def test_freeze_ack_requires_every_robot_and_preserves_snapshot():
    mode = MappingMode(['a', 'b', 'c', 'd'])
    grid = {'cells': np.zeros((5, 5)), 'origin': [1., 2.]}
    assert mode.request('s', 'MAPPING', grid)
    for n in mode.robots:
        mode.acknowledge(n, 's', 'MAPPING', True)
    assert mode.snapshot(True)['ready']
    assert mode.request('s', 'LOCALIZATION', grid)
    expected = copy.deepcopy(mode.frozen_map)
    grid['cells'][:] = 100
    grid['origin'][0] = 100
    for n in ('a', 'b', 'c'):
        mode.acknowledge(n, 's', 'LOCALIZATION', True)
    assert mode.snapshot(False)['frozen'] and not mode.snapshot(False)['ready']
    mode.acknowledge('d', 's', 'MAPPING', True)  # Delayed acknowledgement.
    assert not mode.snapshot(False)['ready']
    mode.acknowledge('d', 's', 'LOCALIZATION', True)
    assert mode.snapshot(False)['ready']
    assert np.array_equal(expected['cells'], mode.frozen_map['cells'])
    assert expected['origin'] == mode.frozen_map['origin']
    assert not mode.request('s', 'MAPPING', grid)
    assert mode.request('new', 'MAPPING', grid)
    assert not mode.request('s', 'LOCALIZATION', grid)


def test_no_freeze_without_map_and_no_stale_handshake_after_team_restart():
    mode = MappingMode(['a'])
    assert not mode.request('s', 'LOCALIZATION', None)
    assert mode.request('s', 'MAPPING', None)
    mode.acknowledge('a', 's', 'MAPPING', False)
    assert not mode.snapshot(True)['ready']
    team = TeamBT(['a'], lambda *a: None, autostart=True, require_mapping=True)
    team.update_mapping({'session': 'old', 'mode': 'MAPPING', 'ready': True}, 0.)
    inputs(team, enclosed_grid(), 0., 0.)
    assert team.phase == 'WAITING'
