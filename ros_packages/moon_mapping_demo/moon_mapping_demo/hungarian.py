"""정수 시간 비용과 금지 간선을 지원하는 직접 구현 Hungarian 알고리즘.

행은 로봇, 열은 목표이며 ``None``은 선택할 수 없는 조합이다. 큰 수를
금지 비용으로 대입하지 않고, 존재하는 간선만으로 증대 경로를 찾는다.
SciPy나 외부 배정 라이브러리를 사용하지 않는다.
"""

from dataclasses import dataclass


class InfeasibleAssignment(ValueError):
    """주어진 허용 간선으로 완전한 1:1 배정을 만들 수 없다."""


@dataclass(frozen=True)
class Assignment:
    """``columns[i]``는 로봇 i에 배정된 목표의 열 번호다."""

    columns: tuple
    total_cost: int


def checked_matrix(costs):
    """비용 행렬을 검증하고 불변 행렬로 복사한다.

    음수, 실수, bool, 무한대는 거부한다. 공통 교환 형식과 두 알고리즘은
    동일한 정수 tick을 그대로 사용해야 한다.
    """
    if not isinstance(costs, (list, tuple)):
        raise ValueError('비용 행렬은 행의 list 또는 tuple이어야 합니다.')
    n = len(costs)
    matrix = []
    for row in costs:
        if not isinstance(row, (list, tuple)) or len(row) != n:
            raise ValueError('비용 행렬은 정사각형이어야 합니다.')
        if any(value is not None and (type(value) is not int or value < 0)
               for value in row):
            raise ValueError('비용은 0 이상의 Python int 또는 금지 조합 None이어야 합니다.')
        matrix.append(tuple(row))
    return tuple(matrix)


def solve(costs):
    """최소 합 비용의 완전 배정을 O(n³) 시간에 구한다.

    행/열 잠재치와 최소 감소 비용을 유지하는 Hungarian 증대 경로 방식이다.
    도달 불가능한 열의 거리는 None으로 표현하므로 큰 정수 비용에서도
    부동소수점 반올림이나 임의의 big-M 값에 의존하지 않는다. 동점에서는
    열을 오름차순으로 조사하여 재현 가능한 최적해 하나를 반환한다.
    """
    matrix = checked_matrix(costs)
    n = len(matrix)
    # 0번 열은 증대 경로의 가상 시작점이며, 실제 행/열은 1부터 센다.
    u, v = [0] * (n + 1), [0] * (n + 1)
    matched_row, predecessor = [0] * (n + 1), [0] * (n + 1)
    for row in range(1, n + 1):
        matched_row[0] = row
        column = 0
        distance = [None] * (n + 1)
        visited = [False] * (n + 1)
        while True:
            visited[column] = True
            active_row = matched_row[column]
            delta, next_column = None, None
            for candidate in range(1, n + 1):
                if visited[candidate]:
                    continue
                edge = matrix[active_row - 1][candidate - 1]
                if edge is not None:
                    reduced = edge - u[active_row] - v[candidate]
                    if distance[candidate] is None or reduced < distance[candidate]:
                        distance[candidate] = reduced
                        predecessor[candidate] = column
                # 이전 방문 행에서 도달한 후보도 다음 증대 단계에 남는다.
                if distance[candidate] is not None and (
                        delta is None or distance[candidate] < delta):
                    delta, next_column = distance[candidate], candidate
            if delta is None:
                raise InfeasibleAssignment(
                    '허용된 로봇–목표 조합으로 완전한 1:1 배정이 불가능합니다.')
            for candidate in range(n + 1):
                if visited[candidate]:
                    u[matched_row[candidate]] += delta
                    v[candidate] -= delta
                elif distance[candidate] is not None:
                    distance[candidate] -= delta
            column = next_column
            if matched_row[column] == 0:
                break
        while column:
            previous = predecessor[column]
            matched_row[column] = matched_row[previous]
            column = previous
    columns = [None] * n
    for column in range(1, n + 1):
        columns[matched_row[column] - 1] = column - 1
    # 방어적 검증: 금지 간선이나 불완전 결과를 성공으로 내보내지 않는다.
    if len(set(columns)) != n or any(
            column is None or matrix[row][column] is None
            for row, column in enumerate(columns)):
        raise InfeasibleAssignment('완전 배정 결과 검증에 실패했습니다.')
    return Assignment(tuple(columns), sum(matrix[row][column]
                                          for row, column in enumerate(columns)))
