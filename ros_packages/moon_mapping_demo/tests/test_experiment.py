"""같은 t0·정수 비용·대기 로봇 포함 문제 생성의 통합 검증."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from moon_mapping_demo.coordination import moving_discs_collide
from moon_mapping_demo.exchange import read_problem
from moon_mapping_demo.experiment import load_experiment, make_field, plan_experiment
from moon_mapping_demo.space_time import PlannerConfig, SpaceTimePlanner


SHARE = Path(__file__).resolve().parents[1]


class ClearField:
    bounds = (-3., 3., -3., 3.)
    identifier = 'synthetic-flat-map-v1'

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


class ExperimentTests(unittest.TestCase):
    def setUp(self):
        self.cfg = {
            'robots': ['robot1', 'robot2', 'robot3'],
            'goals': [{'id': 'a', 'x': 2., 'y': -2.},
                      {'id': 'b', 'x': 2., 'y': 0.},
                      {'id': 'c', 'x': 2., 'y': 2.}],
            'execution': {'goal_hold_seconds': 1.},
        }
        self.env = {'starts': {
            'robot1': {'x': -2., 'y': -2., 'yaw': 0.},
            'robot2': {'x': -2., 'y': 0., 'yaw': 0.},
            'robot3': {'x': -2., 'y': 2., 'yaw': 0.},
            'robot4': {'x': 0., 'y': 0., 'yaw': 0.},
        }}
        self.pcfg = PlannerConfig(resolution=1., time_step=.5, speed=.2,
                                  robot_radius=.15, horizon=60., safety_time_step=.1)

    def test_same_t0_integer_costs_standby_avoidance_and_export(self):
        calls = []
        original_plan = SpaceTimePlanner.plan

        def recording_plan(planner, start, goal, t0, reservations=()):
            calls.append((tuple(start), tuple(goal), t0))
            return original_plan(planner, start, goal, t0, reservations)

        with tempfile.TemporaryDirectory() as output:
            with patch.object(SpaceTimePlanner, 'plan', recording_plan):
                result = plan_experiment(self.cfg, self.env, self.pcfg, ClearField(),
                                         7.5, output=output)
            self.assertEqual(len(calls[:9]), 9)
            self.assertTrue(all(call[2] == 7.5 for call in calls[:9]))
            self.assertTrue(all(value is None or type(value) is int
                                for row in result['costs'] for value in row))
            self.assertIsNone(result['error'])
            self.assertEqual(set(result['paths']), set(self.cfg['robots']))
            self.assertEqual(result['metadata']['standby_robot_ids'], ['robot4'])
            # 중앙 대기 로봇 때문에 robot2->중앙 목표는 직선 시간 40tick보다 길다.
            self.assertGreater(result['costs'][1][1], 40)
            for planned in result['paths'].values():
                self.assertEqual(planned.points[0].t, 7.5)
                for a, b in zip(planned.points, planned.points[1:]):
                    self.assertFalse(moving_discs_collide((a.x, a.y), (b.x, b.y),
                                                         (0, 0), (0, 0), .3))
            problem = read_problem(Path(output) / 'problem.json')
            self.assertEqual(problem['costs'], result['costs'])
            self.assertEqual(problem['t0_sim_seconds'], 7.5)
            self.assertEqual(problem['time_step_seconds'], .5)
            execution = json.loads((Path(output) / 'execution_plan.json').read_text())
            self.assertEqual(execution['original_assignment_total_ticks'], result['assignment'].total_cost)
            self.assertEqual(execution['coordinated_total_ticks'],
                             sum(p.cost_ticks for p in result['paths'].values()))
            self.assertEqual(execution['coordination_extra_ticks'],
                             execution['coordinated_total_ticks'] - execution['original_assignment_total_ticks'])
            self.assertGreaterEqual(execution['matrix_generation_seconds'], 0.)
            self.assertGreaterEqual(execution['assignment_seconds'], 0.)

    def test_all_actual_robot_positions_are_required(self):
        with self.assertRaises(ValueError):
            plan_experiment(self.cfg, self.env, self.pcfg, ClearField(), 0.,
                            starts={'robot1': (-2, -2), 'robot2': (-2, 0), 'robot3': (-2, 2)})

    def test_infeasible_goal_exports_no_fake_assignment(self):
        self.cfg['goals'][0]['x'] = 20.
        with tempfile.TemporaryDirectory() as output:
            result = plan_experiment(self.cfg, self.env, self.pcfg, ClearField(), 0., output=output)
            self.assertIsNone(result['assignment'])
            self.assertFalse(result['paths'])
            self.assertTrue(result['error'])
            self.assertTrue(all(row[0] is None for row in result['costs']))
            self.assertIsNone(read_problem(Path(output) / 'problem.json')['assignment'])

    def test_actual_default_world_environment_agree(self):
        cfg, env, pcfg, share = load_experiment(SHARE / 'config/shadow_experiment.yaml')
        field = make_field(cfg, env, pcfg, share)
        self.assertEqual(set(field.robot_starts), set(env['starts']))
        self.assertEqual(field.bounds, (-30., 30., -30., 30.))


if __name__ == '__main__':
    unittest.main()
