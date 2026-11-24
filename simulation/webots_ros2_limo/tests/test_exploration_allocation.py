"""Allocation and cancellation regression tests using the production BTs."""
import math

import numpy as np
import pytest

from webots_ros2_limo.exploration_bt import Leaf, Status, fallback, sequence
from webots_ros2_limo.exploration_policy import FrontierGrid, Goal, allocate
from webots_ros2_limo.exploration_runtime import RobotBT, TeamBT


class Navigation:
    def __init__(self):
        self.state, self.started, self.cancels = 'IDLE', [], 0

    def ready(self):
        return True

    def start(self, goal):
        assert self.state not in ('NAVIGATING', 'SENDING', 'CANCELING')
        self.started.append(goal)
        self.state = 'NAVIGATING'

    def poll(self):
        return self.state

    def cancel(self):
        if self.state in ('NAVIGATING', 'SENDING'):
            self.cancels += 1
            self.state = 'CANCELING'


def make_grid():
    cells = np.full((80, 80), -1, dtype=np.int8)
    cells[8:72, 8:72] = 0
    return FrontierGrid(cells, .2)


def state(position, **extra):
    return {'ready': True, 'position': position, 'status': 'IDLE', **extra}


def test_four_distinct_reachable_observed_goals():
    grid = make_grid()
    positions = {'robot1': (3., 3.), 'robot2': (12., 3.),
                 'robot3': (3., 12.), 'robot4': (12., 12.)}
    goals = allocate(grid, positions)
    assert len(goals) == 4
    assert len(set(g.xy for g in goals.values())) == 4
    for robot, goal in goals.items():
        assert grid.cells[grid.index(goal.xy)] == 0
        assert math.isfinite(grid.distances(positions[robot])[grid.index(goal.xy)])
    for a in goals.values():
        for b in goals.values():
            if a is not b:
                assert math.dist(a.xy, b.xy) >= 2.


def test_longer_leg_preference_does_not_strand_a_short_remaining_frontier():
    grid = make_grid()
    grid.candidates = [Goal(5.7, 5., 0., 1.)]
    team = TeamBT(['a'], lambda *a: None, autostart=True, min_goal_distance=1.2)
    team.update_map(grid, 0.)
    team.update_robot('a', state([5., 5.]), 0.)
    team.tick(0.)
    assert team.active['a']['goal'][:2] == [5.7, 5.]


def test_working_area_excludes_peer_target_beyond_simple_goal_separation():
    grid = make_grid()
    grid.candidates = [Goal(10.2, 8., 0., 1.), Goal(12., 12., 0., 1.)]
    team = TeamBT(['a', 'b'], lambda *a: None, autostart=True, claim_radius=2.5)
    team.update_map(grid, 0.)
    team.update_robot('a', state([6., 8.]), 0.)
    team.update_robot('b', state([10., 6.]), 0.)
    team.new_task('a', [8., 8., 0.], 'explore')
    # 2.2 m is outside the old 2 m target-only reservation, but inside A's work area.
    team.tick(0.)
    assert team.active['b']['goal'][:2] == [12., 12.]
    assert team.snapshot()['claims']['a']['radius'] == 2.5


def test_short_recent_ownership_releases_after_expiry_and_never_finishes_mission():
    grid = make_grid()
    grid.candidates = [Goal(10.2, 8., 0., 1.)]
    team = TeamBT(['a', 'b'], lambda *a: None, autostart=True, claim_radius=2.5)
    team.update_map(grid, 0.)
    team.update_robot('a', state([6., 8.]), 0.)
    team.update_robot('b', state([10., 6.]), 0.)
    team.new_task('a', [8., 8., 0.], 'explore')
    task = team.active['a']
    team.update_robot('a', state([8., 8.], status='VISITED', task_id=task['task_id'], session=team.session), .1)
    team.update_robot('a', state([8., 8.], status='OBSERVING'), .2)
    team.tick(.2)
    assert 'b' not in team.active and not team.finished
    team.update_map(grid, 16.)
    team.update_robot('a', state([8., 8.], status='OBSERVING'), 16.)
    team.update_robot('b', state([10., 6.]), 16.)
    team.tick(16.)
    assert 'b' in team.active


def test_no_crossing_wall_or_unknown_and_active_goal_reserved():
    grid = make_grid()
    grid.cells[:, 40] = 100
    grid = FrontierGrid(grid.cells, .2)
    position = (3., 6.)
    goals = allocate(grid, {'left': position})
    assert goals['left'].x < 8.
    goal = goals['left']
    alternate = allocate(grid, {'left': position}, reserved=[goal.xy])
    assert 'left' not in alternate or math.dist(alternate['left'].xy, goal.xy) >= 2.
    # A robot in unknown space cannot be silently projected across a wall.
    assert allocate(grid, {'unknown': (0., 0.)}) == {}


def test_rotated_grid_coordinates_and_completion_bounds():
    grid = FrontierGrid(np.zeros((20, 30)), .2, (3., -2., math.pi/2))
    for index in ((0, 0), (9, 12), (19, 29)):
        assert grid.index(grid.world(*index)) == index
    assert not grid.bounds_fully_known()
    assert FrontierGrid(np.zeros((20, 20)), .2, bounds=[0., 0., 4., 4.]).bounds_fully_known()
    assert not FrontierGrid(np.zeros((20, 20)), .2, bounds=[0., 0., 5., 4.]).bounds_fully_known()
    with pytest.raises(ValueError):
        FrontierGrid(np.array([[0, 101]]), .2)


def test_local_footprint_seed_connects_to_observed_space_without_crossing_wall():
    cells = np.full((60, 60), -1, dtype=np.int8)
    cells[10:50, 10:50] = 0
    cells[:, 30] = 100
    grid = FrontierGrid(cells, .1)
    # Free but within clearance distance of a wall: no jump to the other side.
    distance = grid.distances(grid.world(25, 29))
    assert math.isfinite(distance[25, 20])
    assert not math.isfinite(distance[25, 31])


def test_unknown_under_robot_body_can_reach_adjacent_observed_free_space():
    cells = np.full((30, 30), -1, dtype=np.int8)
    cells[10:25, 10:25] = 0
    cells[15, 15] = -1
    grid = FrontierGrid(cells, .1, clearance=.24)
    distances = grid.distances(grid.world(15, 15))
    assert math.isfinite(distances[15, 20])
    cells[15, 16] = 100
    blocked = FrontierGrid(cells, .1, clearance=.24)
    assert not math.isfinite(blocked.distances(blocked.world(15, 15))[15, 20])


def test_missing_fourth_robot_does_not_start_and_stale_input_cancels():
    sent = []
    team = TeamBT(['a', 'b', 'c', 'd'], lambda *args: sent.append(args), autostart=True)
    team.update_map(make_grid(), 0.)
    for name in ('a', 'b', 'c'):
        team.update_robot(name, state([5., 5.]), 0.)
    team.tick(0.)
    assert not team.active and team.phase == 'WAITING'
    team.update_robot('d', state([10., 10.]), 0.)
    team.tick(.1)
    assert len(team.active) == 4
    sent.clear()
    team.tick(5.)
    assert team.phase == 'WAITING'
    assert all(command['mode'] == 'stop' for _, command in sent)
    assert all(t['cancel_requested'] for t in team.active.values())


def test_cooldown_never_certifies_exploration_complete():
    team = TeamBT(['a'], lambda *args: None, autostart=True)
    grid = make_grid()
    team.update_map(grid, 0.)
    team.update_robot('a', state([5., 5.]), 0.)
    team.excluded = [((8., 8.), 100., 100.)]
    team.tick(0.)
    assert not team.finished and team.phase == 'WAITING_FOR_RETRY'
    # A fully observed auto-sized map still has no declared mission boundary.
    team.update_map(FrontierGrid(np.zeros((20, 20)), .2), 1.)
    team.update_robot('a', state([2., 2.]), 1.)
    team.tick(1.)
    assert not team.finished


def test_completion_requires_fresh_inputs_explicit_bounds_and_stability():
    team = TeamBT(['a'], lambda *args: None, autostart=True, complete_hold=2.,
                  completion_mode='bounds', return_home=False)
    grid = FrontierGrid(np.zeros((20, 20)), .2, bounds=[0., 0., 4., 4.])
    for now in (0., 1., 2.):
        team.update_map(grid, now)
        team.update_robot('a', state([2., 2.]), now)
        team.tick(now)
    assert team.finished and team.phase == 'COMPLETE'


def test_same_goal_is_idempotent_and_old_session_is_rejected():
    nav = Navigation()
    robot = RobotBT(nav)
    command = {'session': 'one', 'mode': 'goal', 'task_id': 'one:1', 'goal': [1., 2., 0.]}
    assert robot.receive(command, 0.)
    robot.tick(0.)
    for t in (.5, 1., 1.5):
        robot.receive(command, t)
        robot.tick(t)
    assert len(nav.started) == 1
    assert robot.receive({'session': 'two', 'mode': 'stop'}, 2.)
    assert not robot.receive(command, 2.1)


def test_disconnect_lease_cancels_and_new_goal_waits_for_cancel_result():
    nav = Navigation()
    robot = RobotBT(nav, lease=1.)
    command = {'session': 's', 'mode': 'goal', 'task_id': 's:1', 'goal': [1., 2., 0.]}
    robot.receive(command, 0.)
    robot.tick(0.)
    robot.tick(2.)
    assert nav.state == 'CANCELING'
    command = {**command, 'task_id': 's:2'}
    robot.receive(command, 2.1)
    robot.tick(2.1)
    assert len(nav.started) == 1
    nav.state = 'CANCELED'
    robot.tick(2.2)
    assert len(nav.started) == 2


def test_localization_dropout_cancels_then_reassigns_after_pose_recovers():
    nav = Navigation()
    available = [True]
    nav.ready = lambda: available[0]
    robot = RobotBT(nav)
    now = 0.
    team = TeamBT(['a'], lambda name, command: robot.receive(command, now), autostart=True)
    grid = make_grid()
    team.update_map(grid, now)
    team.update_robot('a', state([5., 5.], **robot.snapshot()), now)
    team.tick(now)
    robot.tick(now)
    first = robot.task['task_id']
    assert len(nav.started) == 1

    available[0], now = False, .2
    robot.tick(now)
    assert nav.state == 'CANCELING'
    team.update_robot('a', state(None, ready=False, **robot.snapshot()), now)
    team.tick(now)
    assert team.phase == 'WAITING' and not team.finished
    nav.state, now = 'CANCELED', .3
    robot.tick(now)
    team.update_robot('a', state(None, ready=False, **robot.snapshot()), now)
    team.tick(now)
    assert not team.active and len(nav.started) == 1

    available[0], now = True, .4
    team.update_map(grid, now)
    team.update_robot('a', state([5., 5.], **robot.snapshot()), now)
    team.tick(now)
    robot.tick(now)
    assert robot.task['task_id'] != first
    assert len(nav.started) == 2 and nav.state == 'NAVIGATING'


def test_four_robot_feedback_drives_next_allocation_and_stale_result_is_ignored():
    robots = {name: RobotBT(Navigation(), observe_seconds=.2)
              for name in ('a', 'b', 'c', 'd')}
    now = 0.
    team = TeamBT(robots, lambda name, cmd: robots[name].receive(cmd, now), autostart=True)
    positions = dict(zip(robots, ([3., 3.], [12., 3.], [3., 12.], [12., 12.])))
    grid = make_grid()
    for name, robot in robots.items():
        team.update_robot(name, state(positions[name], **robot.snapshot()), now)
    team.update_map(grid, now)
    team.tick(now)
    first = dict(team.active)
    for robot in robots.values():
        robot.tick(now)
        robot.navigation.state = 'SUCCEEDED'
    for now in (.1, .4):
        for name, robot in robots.items():
            robot.tick(now)
            team.update_robot(name, state(positions[name], **robot.snapshot()), now)
    assert len(team.history) == 4
    assert not team.active
    now = .5
    team.tick(now)
    assert team.active
    for name in team.active:
        assert team.active[name]['task_id'] != first[name]['task_id']
        before = team.active[name]['task_id']
        team.update_robot(name, state(positions[name], session=team.session,
                                     task_id=first[name]['task_id'], status='VISITED'), now)
        assert team.active[name]['task_id'] == before


def test_bt_guard_halts_running_action_on_preemption():
    allowed, halted = [True], []
    tree = fallback('Root', sequence('Work',
        Leaf('Allowed', lambda: Status.SUCCESS if allowed[0] else Status.FAILURE),
        Leaf('Navigate', lambda: Status.RUNNING, lambda: halted.append(True))),
        Leaf('Wait', lambda: Status.RUNNING))
    tree.tick()
    allowed[0] = False
    tree.tick()
    assert halted == [True]
