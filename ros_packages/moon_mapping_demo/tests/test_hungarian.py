"""작은 완전탐색 정답으로 직접 구현 Hungarian을 검증한다."""

from itertools import permutations
import random
import unittest

from moon_mapping_demo.hungarian import InfeasibleAssignment, solve


def exhaustive(costs):
    totals = [sum(costs[i][column] for i, column in enumerate(columns))
              for columns in permutations(range(len(costs)))
              if all(costs[i][column] is not None
                     for i, column in enumerate(columns))]
    return min(totals) if totals else None


class HungarianTests(unittest.TestCase):
    def test_known_optimum(self):
        result = solve([[4, 1, 3], [2, 0, 5], [3, 2, 2]])
        self.assertEqual(result.columns, (1, 0, 2))
        self.assertEqual(result.total_cost, 5)

    def test_tie_is_deterministic_and_optimal(self):
        costs = [[2] * 4 for _ in range(4)]
        self.assertEqual(solve(costs).total_cost, 8)
        self.assertEqual(solve(costs), solve(costs))
        self.assertEqual(set(solve(costs).columns), set(range(4)))

    def test_forbidden_combinations(self):
        result = solve([[None, 3, None], [None, None, 1], [7, None, None]])
        self.assertEqual(result.columns, (1, 2, 0))
        self.assertEqual(result.total_cost, 11)

    def test_missing_row_and_hall_failure_are_infeasible(self):
        for costs in ([[None]], [[None, None], [1, 2]],
                      [[1, None, None], [1, None, None], [None, 1, 1]]):
            with self.subTest(costs=costs), self.assertRaises(InfeasibleAssignment):
                solve(costs)

    def test_integer_precision_does_not_use_big_m(self):
        huge = 10 ** 100
        costs = [[huge + 1, huge], [huge, None]]
        result = solve(costs)
        self.assertEqual(result.columns, (1, 0))
        self.assertEqual(result.total_cost, 2 * huge)

    def test_zero_and_empty(self):
        self.assertEqual(solve([]).columns, ())
        self.assertEqual(solve([]).total_cost, 0)
        self.assertEqual(solve([[0]]).total_cost, 0)

    def test_invalid_matrix_rejected(self):
        for costs in (None, [[1, 2]], [[1], [2]], [[-1]], [[True]], [[1.0]],
                      [[float('inf')]], [[float('nan')]], [['3']]):
            with self.subTest(costs=costs), self.assertRaises(ValueError):
                solve(costs)

    def test_seeded_dense_sparse_and_tied_matrices_against_exhaustive(self):
        generator = random.Random(20261121)
        # n<=6의 1,500개 문제. 작은 정수 범위로 동점 최적해도 자주 생성한다.
        for size in range(1, 7):
            for trial in range(250):
                forbidden_probability = (trial % 5) / 5
                costs = [[None if generator.random() < forbidden_probability
                          else generator.randrange(8)
                          for _ in range(size)] for _ in range(size)]
                expected = exhaustive(costs)
                with self.subTest(size=size, trial=trial):
                    if expected is None:
                        with self.assertRaises(InfeasibleAssignment):
                            solve(costs)
                    else:
                        actual = solve(costs)
                        self.assertEqual(actual.total_cost, expected)
                        self.assertEqual(set(actual.columns), set(range(size)))
                        self.assertTrue(all(costs[i][column] is not None
                                            for i, column in enumerate(actual.columns)))


if __name__ == '__main__':
    unittest.main()
