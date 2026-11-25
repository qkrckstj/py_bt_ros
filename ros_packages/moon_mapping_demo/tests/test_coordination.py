"""연속 선분 충돌·도착 후 점유·우선순위 예약의 의미 검증."""
import math
import unittest

from moon_mapping_demo.coordination import (
    JointPlanningFailure, coordinate, joint_conflicts, moving_discs_collide,
    position_at, segment_conflicts, with_turn_holds,
)
from moon_mapping_demo.space_time import PlannerConfig, SpaceTimePlanner, TimedPath, TimedPoint


def path(*points):
    return TimedPath(tuple(TimedPoint(*p) for p in points), int(points[-1][2] - points[0][2]))


class ClearField:
    bounds = (-3., 3., -3., 3.)

    def __init__(self, static=()):
        self.static = tuple(static)

    def with_static_robots(self, static):
        return ClearField(static)

    def blocked_segment(self, a, b, ta, tb, radius):
        if any(not (-3 + radius <= p[0] <= 3 - radius
                    and -3 + radius <= p[1] <= 3 - radius) for p in (a, b)):
            return True
        return any(moving_discs_collide(a, b, (x, y), (x, y), radius + r)
                   for x, y, r in self.static)


class CoordinationTests(unittest.TestCase):
    def test_mid_segment_cross_and_swap_are_collisions(self):
        self.assertTrue(moving_discs_collide((-1, 0), (1, 0), (0, -1), (0, 1), .2))
        self.assertTrue(moving_discs_collide((-1, 0), (1, 0), (1, 0), (-1, 0), .2))
        self.assertFalse(moving_discs_collide((-1, 0), (1, 0), (-1, 1), (1, 1), .2))
        self.assertTrue(moving_discs_collide((-1, 0), (1, 0), (-1, .2), (1, .2), .2))

    def test_interpolation_includes_wait_before_start_and_after_arrival(self):
        planned = path((0, 0, 5), (1, 0, 7), (1, 0, 10))
        self.assertEqual(position_at(planned, 0), (0, 0))
        self.assertEqual(position_at(planned, 6), (.5, 0))
        self.assertEqual(position_at(planned, 8), (1, 0))
        self.assertEqual(position_at(planned, 100), (1, 0))

    def test_conflict_checks_other_robot_turns_inside_own_segment(self):
        turning = path((0, -1, 0), (0, 0, 1), (1, 0, 2))
        self.assertTrue(segment_conflicts((-1, 0), (1, 0), 0, 2, turning, .1))

    def test_standby_and_goal_occupancy_are_not_ignored(self):
        moving = path((-1, 0, 0), (1, 0, 2))
        standby = path((0, 0, 0))
        self.assertEqual(joint_conflicts({'moving': moving, 'standby': standby}, .1),
                         [('moving', 'standby')])
        arrived = path((0, 1, 0), (0, 0, 1))
        later = path((-1, 0, 1), (1, 0, 3))
        self.assertTrue(joint_conflicts({'arrived': arrived, 'later': later}, .1))

    def test_simultaneous_single_point_collision(self):
        self.assertEqual(joint_conflicts({'a': path((0, 0, 3)), 'b': path((0, 0, 3))}, .1),
                         [('a', 'b')])

    def test_priority_reservation_resolves_crossing(self):
        field = ClearField()
        config = PlannerConfig(resolution=.5, time_step=.5, speed=1.,
                               robot_radius=.15, horizon=10., safety_time_step=.1)
        starts = {'a': (-1., 0.), 'b': (0., -1.)}
        goals = {'a': (1., 0.), 'b': (0., 1.)}
        independent = {r: SpaceTimePlanner(field, config).plan(starts[r], goals[r], 3.)
                       for r in starts}
        self.assertTrue(joint_conflicts(independent, config.robot_radius))
        paths = coordinate(field, config, starts, goals, 3., goal_hold=1.)
        self.assertFalse(joint_conflicts(paths, config.robot_radius, 10.))
        self.assertGreater(sum(p.cost_ticks for p in paths.values()),
                           sum(p.cost_ticks for p in independent.values()))
        self.assertTrue(all(p.points[0].t == 3. for p in paths.values()))

    def test_unplanned_robot_start_is_conservatively_reserved(self):
        # 현재 방식은 후순위 출발점과 전순위 목표가 겹치는 교환을 실패로 보고한다.
        config = PlannerConfig(resolution=.5, time_step=.5, speed=1.,
                               robot_radius=.15, horizon=3., safety_time_step=.1)
        with self.assertRaises(JointPlanningFailure):
            coordinate(ClearField(), config,
                       {'a': (-1., 0.), 'b': (1., 0.)},
                       {'a': (1., 0.), 'b': (-1., 0.)}, 0.)

    def test_standby_body_changes_route(self):
        config = PlannerConfig(resolution=.5, time_step=.5, speed=1.,
                               robot_radius=.15, horizon=10., safety_time_step=.1)
        paths = coordinate(ClearField(), config, {'a': (-1., 0.)},
                           {'a': (1., 0.)}, 0., standby=[(0., 0., .15)], goal_hold=1.)
        for a, b in zip(paths['a'].points, paths['a'].points[1:]):
            self.assertFalse(moving_discs_collide((a.x, a.y), (b.x, b.y), (0, 0), (0, 0), .3))

    def test_turn_hold_is_timed_and_rechecked_against_dynamic_shadow(self):
        config = PlannerConfig(resolution=.5,time_step=1.,speed=.18,robot_radius=.15,horizon=30.)
        planned = path((0,0,10),(.5,0,13),(.5,.5,16))
        turned = with_turn_holds(planned,math.pi,config)
        self.assertGreater(turned.cost_ticks,planned.cost_ticks)
        self.assertEqual(turned.points[0].t,10.)
        self.assertEqual(turned.points[1].x,turned.points[0].x)
        class Shadow(ClearField):
            def with_static_robots(self, robots): return self
            def blocked_segment(self,a,b,ta,tb,radius):
                return super().blocked_segment(a,b,ta,tb,radius) or tb>15.
        with self.assertRaises(JointPlanningFailure):
            coordinate(Shadow(),config,{'a':(0.,0.)},{'a':(.5,0.)},10.,goal_hold=.1,headings={'a':math.pi})


if __name__ == '__main__':
    unittest.main()
