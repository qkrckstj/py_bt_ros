"""금지 영역을 통과하지 않는 유한 시간 격자 Space-Time A*.

최적성은 설정된 격자와 보수적인 ``blocked_segment`` 전이 그래프에 한정한다.
기본은 이동·대기 모델이다. initial_yaw를 주면 방향 상태와 회전 시간을 함께
탐색한다. 가감속·미끄러짐은 모델 밖이므로 실행기가 일정 오차를 감시한다.
"""
from dataclasses import dataclass
import heapq
import itertools
import math
from bisect import bisect_left
from numbers import Real


class PlanningCancelled(RuntimeError):
    """취소 또는 계산 예산 종료. 도달 불가 간선으로 취급하지 않는다."""


@dataclass(frozen=True)
class PlannerConfig:
    resolution: float = .5
    time_step: float = 1.
    speed: float = .18
    robot_radius: float = .24
    horizon: float = 360.
    safety_time_step: float = .25
    direct_goal_links: bool = False
    straight_only: bool = False

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if name in ('direct_goal_links','straight_only'):
                if type(value) is not bool:raise ValueError(name+'는 bool이어야 합니다')
                continue
            if not isinstance(value, Real) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name}: 유한한 양수가 필요합니다')


@dataclass(frozen=True)
class TimedPoint:
    x: float
    y: float
    t: float


@dataclass(frozen=True)
class TimedPath:
    points: tuple
    cost_ticks: int


class SpaceTimePlanner:
    def __init__(self, field, config=None):
        self.field = field
        self.config = config or PlannerConfig()
        self.last_reason = ''
        self.last_status = 'not_started'
        self.expanded = 0
        self.last_turn_ticks = 0
        self._goal_windows = {}

    def plan(self, start, goal, t0, reservations=(), cancelled=None, goal_hold=0., goal_hold_until=None,
             arrive_not_before=None, initial_yaw=None, turn_rate=.5, goal_radius=None):
        """모든 시각은 /clock과 같은 절대 시뮬레이션 초이다.

        시작/끝 좌표를 공짜로 격자에 옮기지 않는다. 실제 좌표와 인접 격자
        연결도 이동 전이로 검사하고 정수 tick 비용을 지불한다.
        ``None``은 유한 horizon 안에서 허용된 경로가 없다는 뜻이다.
        """
        if reservations:
            raise ValueError('예약은 field.blocked_segment 래퍼로 적용해야 합니다')
        start, goal = tuple(start), tuple(goal)
        if (len(start) != 2 or len(goal) != 2 or
                not all(isinstance(v, Real) and not isinstance(v, bool) and math.isfinite(v)
                        for v in (*start, *goal, t0)) or t0 < 0):
            raise ValueError('유한한 2차원 좌표와 음수가 아닌 t0가 필요합니다')
        cfg = self.config
        service_radius = cfg.robot_radius if goal_radius is None else goal_radius
        if (not isinstance(service_radius, Real) or isinstance(service_radius, bool) or not math.isfinite(service_radius)
                or service_radius < cfg.robot_radius):
            raise ValueError('목표 관측 반경은 몸체 반경 이상의 유한 수여야 합니다.')
        if (not math.isfinite(goal_hold) or goal_hold<0 or
            any(v is not None and not math.isfinite(v) for v in (goal_hold_until,arrive_not_before,initial_yaw)) or
            not math.isfinite(turn_rate) or turn_rate<=0):
            raise ValueError('목표 관측/대기·도착 시각·방향·회전 속도 설정이 잘못되었습니다.')
        self.expanded = 0
        self.last_turn_ticks = 0
        self.last_reason = ''
        self.last_status = 'searching'
        xmin, xmax, ymin, ymax = self.field.bounds
        if not (xmin <= start[0] <= xmax and ymin <= start[1] <= ymax):
            self.last_status = 'start_outside_world'
            self.last_reason = '출발점이 월드 경계 밖입니다'
            return None
        if self.field.blocked_segment(start, start, t0, t0, cfg.robot_radius):
            self.last_reason = '출발점이 금지 영역입니다'
            self.last_status = 'start_forbidden'
            return None
        windows=None
        def goal_available(t):
            if windows is not None:
                index=round((t-t0)/cfg.time_step)
                return 0<=index<len(windows) and windows[index]
            return ((arrive_not_before is None or t>=arrive_not_before-1e-9) and
                not self.field.blocked_segment(goal,goal,t,max(t+goal_hold,goal_hold_until or t),service_radius))
        if start == goal and goal_available(t0):
            self.last_status = 'found'
            return TimedPath((TimedPoint(*start, t0),), 0)
        if not (xmin <= goal[0] <= xmax and ymin <= goal[1] <= ymax):
            self.last_status = 'goal_outside_world'
            self.last_reason = '목표가 월드 경계 밖입니다'
            return None
        static_check = getattr(self.field, 'static_blocked_segment', None)
        if static_check and static_check(goal, goal, cfg.robot_radius):
            self.last_status = 'goal_static_forbidden'
            self.last_reason = '목표가 정적 금지 영역입니다'
            return None
        if math.ceil(math.dist(start,goal)/(cfg.speed*cfg.time_step)-1e-10)>math.floor(cfg.horizon/cfg.time_step+1e-10):
            self.last_status='no_path_within_horizon'
            self.last_reason='직선 이동 시간 하한이 설정한 유한 시간 범위를 넘습니다'
            return None
        nx = math.floor((xmax-xmin)/cfg.resolution)
        ny = math.floor((ymax-ymin)/cfg.resolution)
        special_start, special_goal = (-1, -1), (-2, -2)
        start_node, goal_node = special_start, special_goal

        def position(node):
            if node == start_node:
                return start
            if node == goal_node:
                return goal
            return (xmin + node[0]*cfg.resolution, ymin + node[1]*cfg.resolution)

        def nearby(point):
            i = (point[0]-xmin)/cfg.resolution
            j = (point[1]-ymin)/cfg.resolution
            return tuple((a, b) for a in sorted({math.floor(i), math.ceil(i)})
                         for b in sorted({math.floor(j), math.ceil(j)})
                         if 0 <= a <= nx and 0 <= b <= ny)

        start_neighbors = nearby(start)
        goal_neighbors = set(nearby(goal))
        start_node = next((n for n in start_neighbors if position(n) == start), special_start)
        goal_node = next((n for n in goal_neighbors if position(n) == goal), special_goal)
        max_ticks = math.floor(cfg.horizon/cfg.time_step + 1e-10)
        valid_ticks=None
        if not cfg.straight_only and (goal_hold or goal_hold_until is not None or arrive_not_before is not None):
            key=(goal,t0,goal_hold,goal_hold_until,arrive_not_before,service_radius)
            windows=self._goal_windows.get(key)
            if windows is None:
                values=[]
                for k in range(max_ticks+1):
                    if cancelled is not None and cancelled():
                        self.last_status='cancelled';raise PlanningCancelled('목표 안전 시간대 계산 취소')
                    t=t0+k*cfg.time_step
                    values.append((arrive_not_before is None or t>=arrive_not_before-1e-9) and
                        not self.field.blocked_segment(goal,goal,t,max(t+goal_hold,goal_hold_until or t),service_radius))
                windows=tuple(values)
                self._goal_windows[key]=windows
                if len(self._goal_windows)>32:self._goal_windows.pop(next(iter(self._goal_windows)))
            valid_ticks=[k for k,v in enumerate(windows) if v]
            if not valid_ticks:
                self.last_status='goal_no_service_window_within_horizon'
                self.last_reason='설정한 유한 tick 전체에서 관측·후속 대기를 수용할 목표 시간대가 없습니다.'
                return None

        def ticks(a, b):
            return max(1, math.ceil(math.dist(a, b)/(cfg.speed*cfg.time_step)-1e-10))

        def heuristic(point,tick=0):
            distance=math.ceil(math.dist(point, goal)/(cfg.speed*cfg.time_step)-1e-10)
            if valid_ticks is not None:
                index=bisect_left(valid_ticks,tick+distance)
                return valid_ticks[index]-tick if index<len(valid_ticks) else math.inf
            return distance

        counter = itertools.count()
        oriented=initial_yaw is not None
        first = (start_node, 0, initial_yaw) if oriented else (start_node,0)
        queue = [(heuristic(start),0,next(counter),first)]
        parents = {first: None}
        rotations = {first:0}
        closed = set()
        while queue:
            if cancelled is not None and cancelled():
                self.last_status = 'cancelled'
                raise PlanningCancelled('경로 계산이 취소되었거나 계산 예산을 초과했습니다.')
            _, _, _, state = heapq.heappop(queue)
            if state in closed:
                continue
            closed.add(state)
            node, tick = state[:2]
            yaw=state[2] if oriented else None
            self.expanded += 1
            if node == goal_node and goal_available(t0+tick*cfg.time_step):
                self.last_status = 'found'
                states = []
                while state is not None:
                    n, k = state[:2]
                    states.append(TimedPoint(*position(n), t0+k*cfg.time_step))
                    previous=parents[state]
                    rotation=rotations[state]
                    if rotation and previous is not None:
                        states.append(TimedPoint(*position(previous[0]),t0+(previous[1]+rotation)*cfg.time_step))
                        self.last_turn_ticks+=rotation
                    state = previous
                return TimedPath(tuple(reversed(states)), tick)
            point = position(node)
            if cfg.straight_only:
                # 긴 탐사 후보는 출발점·목표·대기만 있는 작은 시간 그래프를
                # 먼저 탐색한다. 회전과 그림자·동료 예약 검사는 그대로 적용한다.
                candidates=[goal_node] if node!=goal_node else []
            elif node == special_goal:
                candidates=list(goal_neighbors)
            elif node == special_start:
                candidates = list(start_neighbors)
                if goal in (position(n) for n in start_neighbors) or math.dist(start, goal) <= cfg.resolution:
                    candidates.append(goal_node)
            else:
                candidates = [(node[0]+dx, node[1]+dy)
                              for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                              if (dx or dy) and 0 <= node[0]+dx <= nx
                              and 0 <= node[1]+dy <= ny]
                if node in goal_neighbors:
                    candidates.append(goal_node)
            # 격자 축으로 꺾어 가야 한다는 인공적인 제한을 줄인다. 현재
            # 위치에서 목표까지의 직선도 회전·이동 시간과 몸체 전체를 검사한다.
            if cfg.direct_goal_links and node!=goal_node and goal_node not in candidates:
                candidates.append(goal_node)
            # 대기도 실제 시간 구간 전체의 그림자/몸체 충돌을 검사한다.
            candidates.append(node)
            for neighbor in candidates:
                target = position(neighbor)
                dt = 1 if neighbor == node else ticks(point, target)
                rotation=0;heading=yaw
                if oriented and neighbor!=node and math.dist(point,target)>1e-9:
                    heading=math.atan2(target[1]-point[1],target[0]-point[0])
                    angle=abs(math.atan2(math.sin(heading-yaw),math.cos(heading-yaw)))
                    rotation=math.ceil((angle/turn_rate+.5)/cfg.time_step) if angle>.15 else 0
                    dt+=rotation
                next_tick = tick + dt
                if next_tick > max_ticks:
                    continue
                next_state = (neighbor,next_tick,heading) if oriented else (neighbor,next_tick)
                if next_state in parents:
                    continue
                lower_bound=heuristic(target,next_tick)
                if not math.isfinite(lower_bound) or next_tick+lower_bound>max_ticks:continue
                ta, tb = t0+tick*cfg.time_step, t0+next_tick*cfg.time_step
                move_start=ta+rotation*cfg.time_step
                if rotation and self.field.blocked_segment(point,point,ta,move_start,cfg.robot_radius):
                    continue
                if self.field.blocked_segment(point, target, move_start, tb, cfg.robot_radius):
                    continue
                parents[next_state] = state
                rotations[next_state]=rotation
                tie=math.dist(target,goal) if windows is not None or oriented else 0
                heapq.heappush(queue, (next_tick+lower_bound,tie,next(counter),next_state))
        self.last_reason = '설정한 유한 시간 범위에서 경로를 찾지 못했습니다'
        self.last_status = 'no_path_within_horizon'
        return None
