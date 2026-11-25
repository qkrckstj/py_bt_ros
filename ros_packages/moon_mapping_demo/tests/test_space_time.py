"""실제 이동/대기 전이의 정수 비용과 금지 구간을 검증한다."""
import math

import pytest

from moon_mapping_demo.space_time import PlannerConfig, SpaceTimePlanner


class Empty:
    bounds = (0., 5., 0., 5.)

    def blocked_segment(self, a, b, ta, tb, radius):
        return False


def config(**kwargs):
    return PlannerConfig(resolution=1, time_step=1, speed=1, robot_radius=.2,
                         horizon=kwargs.pop('horizon', 12), **kwargs)


def test_exact_grid_start_goal_no_free_or_extra_snapping():
    path = SpaceTimePlanner(Empty(), config()).plan((1, 1), (3, 1), 17)
    assert path.cost_ticks == 2
    assert (path.points[0].x, path.points[0].y, path.points[0].t) == (1, 1, 17)
    assert (path.points[-1].x, path.points[-1].y, path.points[-1].t) == (3, 1, 19)


def test_offgrid_endpoints_pay_for_actual_motion():
    cfg = config()
    path = SpaceTimePlanner(Empty(), cfg).plan((1.1, 1.2), (3.2, 1.1), 4)
    assert (path.points[0].x, path.points[0].y) == (1.1, 1.2)
    assert (path.points[-1].x, path.points[-1].y) == (3.2, 1.1)
    assert all(math.dist((a.x, a.y), (b.x, b.y)) <= (b.t-a.t)*cfg.speed+1e-9
               for a, b in zip(path.points, path.points[1:]))
    assert path.cost_ticks == path.points[-1].t-path.points[0].t


def test_direct_goal_connection_avoids_grid_zigzag_and_prices_actual_distance():
    cfg=config(direct_goal_links=True)
    planner=SpaceTimePlanner(Empty(),cfg)
    path=planner.plan((1.1,1.2),(3.2,2.7),4.)
    assert len(path.points)==2
    assert path.cost_ticks==math.ceil(math.dist((1.1,1.2),(3.2,2.7)))
    assert path.points[-1].t==4.+path.cost_ticks


def test_direct_goal_connection_still_checks_obstacles_and_rotation_time():
    from moon_mapping_demo.coordination import moving_discs_collide
    class Pillar(Empty):
        def blocked_segment(self,a,b,ta,tb,radius):
            return moving_discs_collide(a,b,(2.,2.),(2.,2.),radius+.3)
    cfg=config(direct_goal_links=True,horizon=30.)
    field=Pillar()
    path=SpaceTimePlanner(field,cfg).plan((1.,2.),(3.,2.),0.,initial_yaw=math.pi)
    assert path is not None and len(path.points)>2
    for a,b in zip(path.points,path.points[1:]):
        assert not field.blocked_segment((a.x,a.y),(b.x,b.y),a.t,b.t,cfg.robot_radius)
    straight=SpaceTimePlanner(Empty(),cfg).plan((1.,1.),(2.,1.),0.,initial_yaw=math.pi)
    assert straight.cost_ticks>=1.+math.pi/.5


def test_wait_until_door_opens_at_absolute_sim_time():
    class Door(Empty):
        def blocked_segment(self, a, b, ta, tb, radius):
            return a != b and ta < 12
    path = SpaceTimePlanner(Door(), config()).plan((1, 1), (2, 1), 10)
    assert path.cost_ticks == 3
    assert [(p.x, p.y, p.t) for p in path.points] == [(1, 1, 10), (1, 1, 11), (1, 1, 12), (2, 1, 13)]


def test_shadow_during_wait_is_forbidden():
    class Trap(Empty):
        def blocked_segment(self, a, b, ta, tb, radius):
            if a != b:
                return True
            return ta < .5 < tb
    planner = SpaceTimePlanner(Trap(), config())
    assert planner.plan((1, 1), (2, 1), 0) is None
    assert '시간' in planner.last_reason


def test_every_transition_passes_full_body_and_interval_to_oracle():
    class Recorder(Empty):
        def __init__(self):
            self.calls = []
        def blocked_segment(self, a, b, ta, tb, radius):
            self.calls.append((a, b, ta, tb, radius))
            return False
    field = Recorder()
    path = SpaceTimePlanner(field, config()).plan((1, 1), (3, 1), 5)
    for a, b in zip(path.points, path.points[1:]):
        assert ((a.x, a.y), (b.x, b.y), a.t, b.t, .2) in field.calls


def test_unreachable_horizon_and_bad_configuration():
    planner = SpaceTimePlanner(Empty(), config(horizon=1))
    assert planner.plan((1, 1), (4, 1), 0) is None
    assert '유한' in planner.last_reason
    with pytest.raises(ValueError):
        PlannerConfig(speed=0)
    with pytest.raises(ValueError):
        planner.plan((math.nan, 1), (4, 1), 0)


def test_initial_forbidden_position_is_not_success():
    class Blocked(Empty):
        def blocked_segment(self, *args):
            return True
    planner = SpaceTimePlanner(Blocked(), config())
    assert planner.plan((1, 1), (1, 1), 0) is None
    assert '출발점' in planner.last_reason


def test_offgrid_goal_is_checked_exactly_for_body_collision():
    class GoalCollision(Empty):
        def static_blocked_segment(self, a, b, radius):
            return a[0] > 3 and a[1] > 3
    planner = SpaceTimePlanner(GoalCollision(), config())
    assert planner.plan((1, 1), (3.1, 3.1), 4.3) is None
    assert planner.last_status == 'goal_static_forbidden'


def test_initial_sun_phase_and_fractional_t0_are_never_reset():
    class Phase(Empty):
        def blocked_segment(self, a, b, ta, tb, radius):
            return a != b and ta < 240.3
    planner = SpaceTimePlanner(Phase(), config())
    path = planner.plan((1, 1), (2, 1), 239.3)
    assert [p.t for p in path.points] == [239.3, 240.3, 241.3]
    assert path.cost_ticks == 2
    assert planner.last_status == 'found'


def test_horizon_exhaustion_does_not_claim_global_unreachability():
    planner = SpaceTimePlanner(Empty(), config(horizon=1))
    assert planner.plan((1, 1), (4, 1), 0) is None
    assert planner.last_status == 'no_path_within_horizon'


def test_invalid_numeric_configuration_and_outside_start():
    with pytest.raises(ValueError):
        PlannerConfig(speed='fast')
    planner = SpaceTimePlanner(Empty(), config())
    assert planner.plan((-1, 1), (2, 1), 0) is None
    assert planner.last_status == 'start_outside_world'
