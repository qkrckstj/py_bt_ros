"""Hungarian/CP-SAT이 같은 문제를 읽기 위한 JSON/CSV 교환 규격.

JSON의 costs는 정수 tick 또는 null이다. CSV는 모든 조합을 행으로 기록하고
금지 조합은 allowed=0, 빈 cost_ticks로 표시한다. CP-SAT에서는 허용 간선의
이진 변수에 행/열 합=1, 목적함수 Σ cost_ticks*x를 그대로 적용한다.
"""

import csv
import json
import math
from pathlib import Path

from .hungarian import Assignment, checked_matrix


SCHEMA_VERSION = 'limo-shadow-assignment/v1'
OBJECTIVE = 'minimize_sum_cost_ticks'


def _number(value, name, *, positive=False):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or value < 0 or (positive and value == 0)):
        raise ValueError(f'{name}은 유한한 {"양수" if positive else "0 이상 수"}여야 합니다.')
    return value


def _ids(values, name, n):
    if (not isinstance(values, (list, tuple)) or len(values) != n
            or any(not isinstance(value, str) or not value.strip() for value in values)
            or len(set(values)) != n):
        raise ValueError(f'{name}은 비용 행렬 크기에 맞는 고유한 비어 있지 않은 ID 목록이어야 합니다.')
    return list(values)


def _forbidden(matrix, robot_ids, goal_ids):
    return [{'robot_id': robot_ids[i], 'goal_id': goal_ids[j]}
            for i, row in enumerate(matrix) for j, value in enumerate(row)
            if value is None]


def _assignment_data(assignment, matrix):
    if assignment is None:
        return None
    if isinstance(assignment, Assignment):
        result = {'columns': list(assignment.columns),
                  'total_cost_ticks': assignment.total_cost}
    elif isinstance(assignment, dict):
        result = dict(assignment)
    else:
        raise ValueError('assignment는 Assignment, dict 또는 None이어야 합니다.')
    columns = result.get('columns')
    n = len(matrix)
    if (not isinstance(columns, (list, tuple)) or len(columns) != n
            or any(type(column) is not int or not 0 <= column < n for column in columns)
            or len(set(columns)) != n
            or any(matrix[i][column] is None for i, column in enumerate(columns))):
        raise ValueError('assignment는 모든 로봇과 목표를 허용 조합으로 한 번씩 배정해야 합니다.')
    total = sum(matrix[i][column] for i, column in enumerate(columns))
    if type(result.get('total_cost_ticks')) is not int or result['total_cost_ticks'] != total:
        raise ValueError('assignment 총비용이 비용 행렬과 일치하지 않습니다.')
    return {'columns': list(columns), 'total_cost_ticks': total}


def validate_problem(problem):
    """입력 문제와 결과의 구조·단위·금지 조합 일관성을 검증한다.

    결과 배정의 실현 가능성과 합계만 검사한다. 최적성 검증은 별도의
    알고리즘 검증에서 수행하며, 공동 주행 충돌은 이 선형 배정 규격 밖이다.
    """
    if not isinstance(problem, dict) or problem.get('schema_version') != SCHEMA_VERSION:
        raise ValueError('지원하지 않는 배정 문제 JSON 규격입니다.')
    matrix = checked_matrix(problem.get('costs'))
    robot_ids = _ids(problem.get('robot_ids'), 'robot_ids', len(matrix))
    goal_ids = _ids(problem.get('goal_ids'), 'goal_ids', len(matrix))
    if (problem.get('cost_unit') != 'tick' or problem.get('objective') != OBJECTIVE
            or problem.get('time_origin') != 'ros_clock_sim_seconds'
            or problem.get('constraints') != {
                'one_goal_per_robot': True, 'one_robot_per_goal': True,
                'complete_assignment': True}):
        raise ValueError('목적함수, 시간 단위 또는 1:1 완전 배정 제약이 다릅니다.')
    _number(problem.get('t0_sim_seconds'), 't0_sim_seconds')
    _number(problem.get('time_step_seconds'), 'time_step_seconds', positive=True)
    if problem.get('forbidden_pairs') != _forbidden(matrix, robot_ids, goal_ids):
        raise ValueError('forbidden_pairs와 costs의 null 항목이 일치하지 않습니다.')
    metadata = problem.get('metadata')
    if (not isinstance(metadata, dict)
            or any(not isinstance(metadata.get(key), str) or not metadata[key].strip()
                   for key in ('map_id', 'config_id'))):
        raise ValueError('metadata에 비어 있지 않은 map_id와 config_id가 필요합니다.')
    timings = problem.get('timings')
    if not isinstance(timings, dict):
        raise ValueError('timings가 필요합니다.')
    for key in ('matrix_generation_seconds', 'assignment_seconds'):
        _number(timings.get(key), key)
    _assignment_data(problem.get('assignment'), matrix)
    # metadata 안의 NaN/Infinity 및 직렬화 불가능한 값도 함께 거부한다.
    try:
        json.dumps(problem, ensure_ascii=False, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError('문제의 모든 값은 유한한 JSON 값이어야 합니다.') from error
    return problem


def write_problem(directory, robot_ids, goal_ids, costs, t0, time_step, metadata,
                  matrix_generation_seconds, assignment_seconds, assignment=None):
    """문제를 ``problem.json``, ``problem.csv``로 쓰고 두 Path를 반환한다.

    ``metadata``에는 map_id/config_id가 필수다. 권장 값은 실제 사용한 월드,
    태양 설정, 실험 설정의 SHA-256이며 추가 메타데이터도 보존한다.
    시간 측정은 wall-clock 초이고 경로 비용의 정수 시뮬레이션 tick과 별개다.
    """
    matrix = checked_matrix(costs)
    robots = _ids(robot_ids, 'robot_ids', len(matrix))
    goals = _ids(goal_ids, 'goal_ids', len(matrix))
    problem = {
        'schema_version': SCHEMA_VERSION,
        'robot_ids': robots,
        'goal_ids': goals,
        'costs': [list(row) for row in matrix],
        'cost_unit': 'tick',
        'time_step_seconds': time_step,
        't0_sim_seconds': t0,
        'time_origin': 'ros_clock_sim_seconds',
        'objective': OBJECTIVE,
        'constraints': {'one_goal_per_robot': True, 'one_robot_per_goal': True,
                        'complete_assignment': True},
        'forbidden_pairs': _forbidden(matrix, robots, goals),
        'metadata': metadata,
        'timings': {'matrix_generation_seconds': matrix_generation_seconds,
                    'assignment_seconds': assignment_seconds},
        'assignment': _assignment_data(assignment, matrix),
    }
    validate_problem(problem)
    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    json_path, csv_path = destination / 'problem.json', destination / 'problem.csv'
    json_path.write_text(json.dumps(problem, ensure_ascii=False, indent=2,
                                    allow_nan=False) + '\n', encoding='utf-8')
    fields = ['schema_version', 'robot_index', 'goal_index', 'robot_id', 'goal_id',
              'allowed', 'cost_ticks', 'cost_unit', 'time_step_seconds',
              't0_sim_seconds', 'time_origin', 'objective', 'map_id', 'config_id']
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for i, row in enumerate(matrix):
            for j, value in enumerate(row):
                writer.writerow({
                    'schema_version': SCHEMA_VERSION,
                    'robot_index': i, 'goal_index': j,
                    'robot_id': robots[i], 'goal_id': goals[j],
                    'allowed': int(value is not None),
                    'cost_ticks': '' if value is None else value,
                    'cost_unit': 'tick', 'time_step_seconds': time_step,
                    't0_sim_seconds': t0, 'time_origin': 'ros_clock_sim_seconds',
                    'objective': OBJECTIVE, 'map_id': metadata['map_id'],
                    'config_id': metadata['config_id'],
                })
    return {'json': json_path, 'csv': csv_path}


def read_problem(path):
    """공통 JSON 문제를 읽고 유효성을 검증한다. 비용을 재계산하지 않는다."""
    def reject_constant(value):
        raise ValueError(f'JSON에 유한하지 않은 상수 {value}를 사용할 수 없습니다.')
    problem = json.loads(Path(path).read_text(encoding='utf-8'),
                         parse_constant=reject_constant)
    return validate_problem(problem)
