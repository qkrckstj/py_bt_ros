"""추종 명령이 대기·출발 시각을 보존하고 이탈을 안전 정지로 바꾸는지 검증."""
import math
import unittest

from moon_mapping_demo.coordination import joint_conflicts, moving_discs_collide
from moon_mapping_demo.execution import InputStability, command, control_step, inputs_fresh, remaining_safe
from moon_mapping_demo.space_time import TimedPath, TimedPoint


def path(*points):
    return TimedPath(tuple(TimedPoint(*p) for p in points), int(points[-1][2] - points[0][2]))


class OpenField:
    def __init__(self, robots=()):
        self.robots = robots

    def with_static_robots(self, robots):
        return OpenField(robots)

    def blocked_segment(self, a, b, ta, tb, radius):
        return any(moving_discs_collide(a, b, (x, y), (x, y), radius + r)
                   for x, y, r in self.robots)


class ExecutionTests(unittest.TestCase):
    settings = {'position_tolerance': .3, 'goal_tolerance': .05,
                'delay_tolerance': 1., 'control_period': .1}

    def test_startup_requires_stable_simulation_and_wall_time(self):
        gate = InputStability(3.)
        self.assertFalse(gate.ready(1., 10., True))
        self.assertFalse(gate.ready(5., 11., True))
        self.assertTrue(gate.ready(6., 13., True))

    def test_startup_gap_resets_settling_without_starting_a_mission(self):
        gate = InputStability(3.)
        self.assertFalse(gate.ready(1., 10., True))
        self.assertFalse(gate.ready(3., 12., False))
        self.assertFalse(gate.ready(4., 13., True))
        self.assertFalse(gate.ready(6., 15., True))
        self.assertTrue(gate.ready(7., 16., True))

    def test_paused_clock_cannot_finish_startup(self):
        gate = InputStability(3.)
        self.assertFalse(gate.ready(1., 10., True))
        self.assertFalse(gate.ready(1., 20., True))

    def test_startup_clock_rewind_resets_observation_window(self):
        gate = InputStability(3.)
        self.assertFalse(gate.ready(10., 10., True))
        self.assertFalse(gate.ready(1., 14., True))
        self.assertFalse(gate.ready(3., 16., True))
        self.assertTrue(gate.ready(4., 17., True))

    def test_slow_simulation_uses_separate_wall_timeout(self):
        observed = {'a': (10., 20.)}
        self.assertTrue(inputs_fresh(observed, ('a',), 10.02, 20.9, .6, 3.))
        self.assertFalse(inputs_fresh(observed, ('a',), 10.02, 20.9, .6, .6))

    def test_wall_allowance_does_not_allow_stale_simulation_observation(self):
        self.assertFalse(inputs_fresh({'a': (10., 20.)}, ('a',), 10.8, 20.1, .6, 3.))

    def test_wall_timeout_stops_even_if_simulation_clock_is_frozen(self):
        self.assertFalse(inputs_fresh({'a': (10., 20.)}, ('a',), 10., 23.1, .6, 3.))

    def test_standby_observation_is_required_with_separate_timeouts(self):
        self.assertFalse(inputs_fresh({'a': (10., 20.)}, ('a', 'standby'), 10., 20.1, .6, 3.))

    def test_future_observation_is_rejected_with_separate_timeouts(self):
        self.assertFalse(inputs_fresh({'a': (10., 20.)}, ('a',), 9.9, 20.1, .6, 3.))

    def test_wait_transition_preserved(self):
        planned = path((0, 0, 5), (0, 0, 7), (1, 0, 12))
        waiting = command(planned, (0, 0, 0), 6., self.settings, .2)
        self.assertEqual((waiting.linear, waiting.angular), (0., 0.))
        self.assertFalse(waiting.replan_reason)
        self.assertFalse(waiting.arrived)

    def test_does_not_move_before_common_start_time(self):
        planned = path((0, 0, 5), (1, 0, 10))
        control = command(planned, (0, 0, 0), 3., self.settings, .2)
        self.assertEqual(control.linear, 0.)
        self.assertFalse(control.replan_reason)

    def test_goal_early_arrival_waits_for_scheduled_time(self):
        planned = path((0, 0, 5), (1, 0, 10))
        early = command(planned, (1, 0, 0), 9., self.settings, .2)
        self.assertEqual(early.linear, 0.)
        self.assertFalse(early.arrived)
        self.assertFalse(early.replan_reason)
        arrived = command(planned, (1, 0, 0), 10., self.settings, .2)
        self.assertTrue(arrived.arrived)

    def test_expected_progress_on_later_segment_does_not_revisit_old_waypoint(self):
        planned = path((0, 0, 0), (1, 0, 5), (2, 0, 10))
        control = command(planned, (1.5, 0, 0), 7.5, self.settings, .2)
        self.assertFalse(control.replan_reason)
        self.assertGreater(control.linear, 0.)
        self.assertLessEqual(control.linear, .2)

    def test_location_deviation_requests_replan_and_zero_speed(self):
        planned = path((0, 0, 0), (1, 0, 5))
        control = command(planned, (0, 1, 0), 1., self.settings, .2)
        self.assertTrue(control.replan_reason)
        self.assertEqual((control.linear, control.angular), (0., 0.))

    def test_late_goal_requests_replan_even_within_position_tolerance(self):
        planned = path((0, 0, 0), (1, 0, 5))
        control = command(planned, (.8, 0, 0), 7., self.settings, .2)
        self.assertTrue(control.replan_reason)
        self.assertEqual((control.linear, control.angular), (0., 0.))

    def test_turn_before_translating(self):
        planned = path((0, 0, 0), (1, 0, 5))
        control = command(planned, (0, 0, 1.), 0., self.settings, .2)
        self.assertEqual(control.linear, 0.)
        self.assertLess(control.angular, 0.)

    def test_terminal_hold_keeps_zero_command(self):
        planned = path((0, 0, 0), (1, 0, 5))
        control = command(planned, (1, 0, 0), 20., self.settings, .2)
        self.assertTrue(control.arrived)
        self.assertEqual((control.linear, control.angular), (0., 0.))

    def test_wait_prepares_next_heading_without_starting_early(self):
        planned = path((0, 0, 5), (0, 0, 7), (0, 1, 12))
        control = command(planned, (0, 0, 0), 6., self.settings, .2)
        self.assertEqual(control.linear, 0.)
        self.assertGreater(control.angular, 0.)
        self.assertFalse(control.replan_reason)

    def test_early_waypoint_prepares_corner_without_skipping_departure(self):
        planned = path((0, 0, 0), (1, 0, 5), (1, 1, 10))
        control = command(planned, (.96, 0, 0), 4.9, self.settings, .2)
        self.assertEqual(control.linear, 0.)
        self.assertGreater(control.angular, 0.)
        self.assertFalse(control.replan_reason)

    def test_preturn_skips_duplicate_wait_points(self):
        planned = path((0, 0, 0), (1, 0, 5), (1, 0, 7), (1, -1, 12))
        control = command(planned, (.96, 0, 0), 4.9, self.settings, .2)
        self.assertEqual(control.linear, 0.)
        self.assertLess(control.angular, 0.)

    def test_already_at_goal_waits_for_common_departure_time(self):
        planned = path((0, 0, 5))
        before = command(planned, (0, 0, 0), 3., self.settings, .2)
        self.assertFalse(before.arrived)
        self.assertFalse(before.replan_reason)
        self.assertEqual((before.linear, before.angular), (0., 0.))
        self.assertTrue(command(planned, (0, 0, 0), 5., self.settings, .2).arrived)

    def test_future_collision_uses_actual_positions_instead_of_old_paths(self):
        planned = {'a': path((-1, -.3, 0), (1, -.3, 10)),
                   'b': path((1, .3, 0), (-1, .3, 10))}
        actual = {'a': (-.4, -.15, 0), 'b': (.4, .15, 0)}
        self.assertFalse(joint_conflicts(planned, .26))
        # 각 로봇은 원래 경로에서 0.15m만 이탈했지만 복귀 선분은 미래에 충돌한다.
        self.assertGreater(math.dist(actual['a'][:2], actual['b'][:2]), .52)
        self.assertFalse(remaining_safe(OpenField(), planned, actual, 3., .26, 3.))

    def test_remaining_path_includes_moved_standby_robot(self):
        planned = {'a': path((-1, 0, 0), (1, 0, 10))}
        actual = {'a': (-1, 0, 0), 'standby': (0, 0, 0)}
        self.assertFalse(remaining_safe(OpenField(), planned, actual, 0., .26, 3.))

    def test_team_command_is_withheld_when_one_robot_requires_replan(self):
        planned = {'a': path((-1, -1, 0), (1, -1, 10)),
                   'b': path((-1, 1, 0), (1, 1, 10))}
        actual = {'a': (-.6, -1, 0), 'b': (-1, 1, 0)}
        settings = {**self.settings, 'goal_hold_seconds': 3., 'lookahead_seconds': .3}
        controls, reason = control_step(OpenField(), planned, actual, 2., settings, .2, .26)
        self.assertEqual(controls, {})
        self.assertIn('b:', reason)


if __name__ == '__main__':
    unittest.main()
