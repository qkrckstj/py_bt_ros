"""정수 비용·금지 조합·공통 시각의 JSON/CSV 교환 검증."""

import copy
import csv
import json
from pathlib import Path
import tempfile
import unittest

from moon_mapping_demo.exchange import read_problem, validate_problem, write_problem
from moon_mapping_demo.hungarian import Assignment, solve


class ExchangeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.costs = [[15, None, 21], [8, 16, 7], [None, 4, 12]]
        self.arguments = {
            'directory': self.temporary.name,
            'robot_ids': ['robot1', 'robot2', 'robot3'],
            'goal_ids': ['관측A', '관측B', '관측C'],
            'costs': self.costs, 't0': 31.5, 'time_step': 0.5,
            'metadata': {'map_id': 'sha256:world', 'config_id': 'sha256:config',
                         'sun_id': 'sha256:sun', 'mode': '사전 지형 기반 실험'},
            'matrix_generation_seconds': 1.25, 'assignment_seconds': 0.0001,
            'assignment': solve(self.costs),
        }

    def test_json_and_csv_roundtrip_preserves_exact_costs_and_forbidden(self):
        paths = write_problem(**self.arguments)
        problem = read_problem(paths['json'])
        self.assertEqual(problem['costs'], self.costs)
        self.assertEqual(problem['cost_unit'], 'tick')
        self.assertEqual(problem['time_step_seconds'], 0.5)
        self.assertEqual(problem['t0_sim_seconds'], 31.5)
        self.assertEqual(problem['assignment']['total_cost_ticks'], 26)
        self.assertEqual(problem['forbidden_pairs'], [
            {'robot_id': 'robot1', 'goal_id': '관측B'},
            {'robot_id': 'robot3', 'goal_id': '관측A'}])
        self.assertEqual(problem['timings'], {
            'matrix_generation_seconds': 1.25, 'assignment_seconds': 0.0001})
        reconstructed = [[None] * 3 for _ in range(3)]
        with paths['csv'].open(newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 9)
        for row in rows:
            self.assertEqual(row['t0_sim_seconds'], '31.5')
            self.assertEqual(row['map_id'], 'sha256:world')
            self.assertEqual(row['config_id'], 'sha256:config')
            i, j = int(row['robot_index']), int(row['goal_index'])
            self.assertEqual(row['robot_id'], problem['robot_ids'][i])
            self.assertEqual(row['goal_id'], problem['goal_ids'][j])
            if row['allowed'] == '1':
                reconstructed[i][j] = int(row['cost_ticks'])
            else:
                self.assertEqual(row['allowed'], '0')
                self.assertEqual(row['cost_ticks'], '')
        self.assertEqual(reconstructed, self.costs)

    def test_infeasible_problem_is_exported_without_fake_assignment(self):
        arguments = dict(self.arguments, costs=[[None] * 3 for _ in range(3)], assignment=None)
        problem = read_problem(write_problem(**arguments)['json'])
        self.assertIsNone(problem['assignment'])
        self.assertEqual(len(problem['forbidden_pairs']), 9)

    def test_mismatched_or_forbidden_assignment_rejected(self):
        for assignment in (Assignment((0, 1, 2), 0), Assignment((1, 2, 0), 100),
                           Assignment((0, 0, 1), 27), Assignment((0,), 15)):
            with self.subTest(assignment=assignment), self.assertRaises(ValueError):
                write_problem(**dict(self.arguments, assignment=assignment))

    def test_invalid_inputs_do_not_create_artifacts(self):
        modifications = [
            {'robot_ids': ['robot1', 'robot1', 'robot3']},
            {'goal_ids': ['a', 'b']}, {'goal_ids': ['a', 'b', '']},
            {'time_step': 0}, {'t0': float('nan')}, {'t0': -1},
            {'matrix_generation_seconds': float('inf')},
            {'assignment_seconds': -1}, {'metadata': {}},
            {'metadata': dict(self.arguments['metadata'], invalid=float('nan'))},
            {'costs': [[float('inf')] * 3 for _ in range(3)]},
        ]
        for change in modifications:
            with self.subTest(change=change), self.assertRaises(ValueError):
                write_problem(**dict(self.arguments, **change))
        self.assertEqual(list(Path(self.temporary.name).iterdir()), [])

    def test_reader_rejects_corrupted_contract(self):
        path = write_problem(**self.arguments)['json']
        original = read_problem(path)
        modifications = [
            {'schema_version': 'unknown'}, {'cost_unit': 'seconds'},
            {'objective': 'minimize_makespan'}, {'forbidden_pairs': []},
            {'constraints': {'complete_assignment': False}},
            {'costs': [[True] * 3 for _ in range(3)]},
        ]
        for change in modifications:
            corrupted = copy.deepcopy(original)
            corrupted.update(change)
            path.write_text(json.dumps(corrupted), encoding='utf-8')
            with self.subTest(change=change), self.assertRaises(ValueError):
                read_problem(path)
        path.write_text('{"schema_version": NaN}', encoding='utf-8')
        with self.assertRaises(ValueError):
            read_problem(path)

    def test_integer_cost_is_not_rounded_to_seconds(self):
        huge = 10 ** 100
        problem = read_problem(write_problem(**dict(
            self.arguments, robot_ids=['robot1'], goal_ids=['goal1'],
            costs=[[huge]], assignment=Assignment((0,), huge)))['json'])
        self.assertEqual(problem['costs'], [[huge]])
        self.assertEqual(problem['assignment']['total_cost_ticks'], huge)
        self.assertIs(type(problem['costs'][0][0]), int)


if __name__ == '__main__':
    unittest.main()
