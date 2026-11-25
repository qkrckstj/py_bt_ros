"""시간표를 보존하는 차동 구동 추종기. ROS·물리 엔진과 독립적인 제어 계산."""
from dataclasses import dataclass
import math

from .coordination import joint_conflicts, moving_discs_collide, position_at
from .space_time import TimedPath, TimedPoint


@dataclass(frozen=True)
class Control:
    linear: float = 0.
    angular: float = 0.
    replan_reason: str = ''
    arrived: bool = False


class InputStability:
    """초기 시뮬레이터 입력이 두 시계에서 연속으로 안정된 뒤 시작한다."""
    def __init__(self, seconds=3., sim_seconds=None):
        self.seconds = seconds
        self.sim_seconds = seconds if sim_seconds is None else sim_seconds
        if any(not math.isfinite(v) or v<=0 for v in (self.seconds,self.sim_seconds)):
            raise ValueError('입력 안정화 시간은 양의 유한 수여야 합니다.')
        self.since = None
        self.last_sim = None

    def ready(self, sim_time, wall_time, fresh):
        if not fresh or sim_time <= 0 or (self.last_sim is not None and sim_time < self.last_sim):
            self.since = None
        self.last_sim = sim_time
        if not fresh or sim_time <= 0:
            return False
        if self.since is None:
            self.since = sim_time, wall_time
        return (sim_time - self.since[0] >= self.sim_seconds
                and wall_time - self.since[1] >= self.seconds)

    def progress(self, sim_time, wall_time):
        return {'wall_elapsed_seconds':max(0.,wall_time-self.since[1]) if self.since else 0.,
                'wall_required_seconds':self.seconds,
                'sim_elapsed_seconds':max(0.,sim_time-self.since[0]) if self.since else 0.,
                'sim_required_seconds':self.sim_seconds}


def inputs_fresh(observations, robots, sim_time, wall_time, sim_timeout, wall_timeout):
    """시뮬레이션 내 관측 나이와 실제 수신 중단 시간을 별도로 제한한다."""
    return all(robot in observations
               and -.05 <= sim_time - observations[robot][0] <= sim_timeout
               and 0 <= wall_time - observations[robot][1] <= wall_timeout
               for robot in robots)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def align_next(points, index, pose):
    """예정 시각까지 제자리에서 다음 이동 방향을 준비한다."""
    x, y, yaw = pose
    anchor = points[index]
    for target in points[index + 1:]:
        if math.hypot(target.x - anchor.x, target.y - anchor.y) > 1e-9:
            angle = wrap(math.atan2(target.y - y, target.x - x) - yaw)
            return Control(angular=max(-.55, min(.55, 2. * angle)))
    return Control()


def command(path, pose, now, settings, max_speed):
    """각 끝점 시각까지 기다리며 추종한다. 지연/위치 이탈은 재계획 사유다.

    회전과 가감속은 A* 비용에 없으므로 실제 회전 지연은 안전 정지·재계획으로
    처리한다. 도착 전 정지 대기 중에는 다음 선분 방향으로 선회할 수 있다.
    """
    x, y, yaw = pose
    expected = position_at(path, now)
    if math.hypot(x - expected[0], y - expected[1]) > settings['position_tolerance']:
        return Control(replan_reason='시간표 대비 위치 이탈')
    last = path.points[-1]
    if now >= last.t and math.hypot(last.x - x, last.y - y) <= settings['goal_tolerance']:
        return Control(arrived=True)
    points = path.points
    if len(points) == 1 and now < points[0].t:
        return Control()
    for index, (a, b) in enumerate(zip(points, points[1:])):
        if now >= b.t and index < len(points) - 2:
            continue
        distance = math.hypot(b.x - x, b.y - y)
        if now >= b.t and distance <= settings['goal_tolerance']:
            continue
        if now > b.t + settings['delay_tolerance']:
            return Control(replan_reason='예정 끝점 도착 지연')
        if a.x == b.x and a.y == b.y:
            return align_next(points, index + 1, pose)
        angle = wrap(math.atan2(b.y - y, b.x - x) - yaw)
        angular = max(-.55, min(.55, 2. * angle))
        if distance <= settings['goal_tolerance']:
            return align_next(points, index + 1, pose)
        if now < a.t or abs(angle) > .15:
            return Control(angular=angular)
        remaining = max(settings['control_period'], b.t - now)
        return Control(min(max_speed, distance / remaining), angular)
    return Control(replan_reason='도착 시각 이후 목표에 미도달')


def remaining_safe(field, paths, poses, now, radius, goal_hold):
    """현재 관측 위치에서 남은 시간표로 복귀하는 모든 로봇을 함께 검사한다."""
    if not paths:
        return False
    standby = [(p[0], p[1], radius) for r, p in poses.items() if r not in paths]
    local = field.with_static_robots(standby)
    end = max(now, max(p.points[-1].t for p in paths.values()) + goal_hold)
    remaining = {}
    for robot, path in paths.items():
        p = poses[robot]
        points = (TimedPoint(p[0], p[1], now), *(v for v in path.points if v.t > now))
        remaining[robot] = TimedPath(points, 0)
        for a, b in zip(points, points[1:]):
            if local.blocked_segment((a.x, a.y), (b.x, b.y), a.t, b.t, radius):
                return False
        last = points[-1]
        if local.blocked_segment((last.x, last.y), (last.x, last.y), last.t, end, radius):
            return False
    return not joint_conflicts(remaining, radius, end)


def control_step(field, paths, poses, now, settings, max_speed, radius):
    """ROS와 기구학 검사에서 공유하는 정지·재계획·속도 명령 판단."""
    if not remaining_safe(field, paths, poses, now, radius, settings['goal_hold_seconds']):
        return {}, '남은 시간 경로 또는 목표 대기의 그림자·장애물·로봇 충돌 검사 실패'
    controls = {r: command(path, poses[r], now, settings, max_speed) for r, path in paths.items()}
    reasons = [r + ': ' + c.replan_reason for r, c in controls.items() if c.replan_reason]
    if reasons:
        return {}, '; '.join(reasons)
    lookahead = settings['lookahead_seconds']
    projected = {}
    margins = {}
    for robot, control in controls.items():
        p = poses[robot]
        projected[robot] = (p[0] + control.linear * math.cos(p[2]) * lookahead,
                            p[1] + control.linear * math.sin(p[2]) * lookahead)
        # 회전하며 이동할 때 접선 선분과 실제 원호 사이의 최대 오차를 감싼다.
        margins[robot] = abs(control.linear * control.angular) * lookahead * lookahead / 2
        if field.blocked_segment(p[:2], projected[robot], now, now + lookahead,
                                 radius + margins[robot]):
            return {}, robot + ': 실제 속도 명령의 점유 영역 불가'
    moves = {r: (p[:2], projected.get(r, p[:2])) for r, p in poses.items()}
    names = list(moves)
    if any(moving_discs_collide(*moves[a], *moves[b],
                               2 * radius + .02 + margins.get(a, 0.) + margins.get(b, 0.))
           for i, a in enumerate(names) for b in names[i + 1:]):
        return {}, '실제 위치·속도 기준 로봇 충돌 위험(대기 로봇 포함)'
    return controls, ''
