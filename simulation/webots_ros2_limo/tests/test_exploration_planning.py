"""A deliberately blocked solver must not delay command/lease callbacks."""
from threading import Event
import time

from webots_ros2_limo.exploration_planning import PlanningWorker
from webots_ros2_limo.exploration_planning import recent_visit_exclusions


def test_recent_visits_exclude_only_endpoints_and_expire():
    history = [
        {'result': 'VISITED', 'purpose': 'explore', 'goal': [1., 2., 0.], 'finished_at': 100.},
        {'result': 'CANCELED', 'purpose': 'explore', 'goal': [3., 4., 0.], 'finished_at': 100.},
        {'result': 'VISITED', 'purpose': 'return', 'goal': [5., 6., 0.], 'finished_at': 100.},
    ]
    assert recent_visit_exclusions(history, 150.) == [((1., 2.), 1.25)]
    assert recent_visit_exclusions(history, 280.) == []
    assert recent_visit_exclusions(history, 99.) == []


def test_busy_planner_does_not_block_control_ticks_or_start_another_job():
    worker, release, started = PlanningWorker(), Event(), Event()
    def compute():
        started.set()
        release.wait(2.)
        return {'robot1': 'goal'}
    try:
        assert worker.submit('task-session', compute)
        assert started.wait(1.)
        callbacks = []
        for i in range(100):
            assert worker.poll('task-session') == (False, None)
            callbacks.append(i)  # command lease and motion-permit renewal
        assert len(callbacks) == 100
        assert not worker.submit('duplicate', lambda: None)
        release.set()
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            done, value = worker.poll('task-session')
            if done:
                assert value == {'robot1': 'goal'}
                break
        else:
            raise AssertionError('Solver result never became available')
    finally:
        release.set()
        worker.close()


def test_changed_active_tasks_discard_old_planning_result():
    worker = PlanningWorker()
    try:
        assert worker.submit(('old-active-goal',), lambda: {'robot1': 'stale goal'})
        worker.future.result(timeout=2.)
        assert worker.poll(('new-active-goal',)) == (True, None)
        assert worker.future is None
    finally:
        worker.close()


def test_reposition_candidates_stay_in_observed_safe_space():
    import math
    import numpy as np
    from webots_ros2_limo.exploration_policy import FrontierGrid
    from webots_ros2_limo.exploration_planning import observed_reposition_goals
    cells = np.full((80, 80), -1, dtype=np.int8)
    cells[10:70, 10:70] = 0
    cells[:, 40] = 100
    grid = FrontierGrid(cells, .1)
    candidates = observed_reposition_goals(grid, {'a': (3., 3.)})
    assert candidates
    for g in candidates:
        assert grid.safe[grid.index(g.xy)]
        assert grid.cells[grid.index(g.xy)] == 0
    # Some safe endpoints can be behind the wall; they still require a finite
    # observed path in the allocator. This helper never invents a passage.
    from webots_ros2_limo.exploration_cp_sat import allocate
    grid.candidates = candidates
    result = allocate(grid, {'a': (3., 3.)}, solver_seconds=1.)
    assert result['a'].x < 4.
