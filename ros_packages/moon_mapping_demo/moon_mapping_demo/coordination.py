"""시간 경로의 연속 선분 충돌 검사와 고정 배정의 우선순위 재계획."""
from itertools import permutations
from math import hypot, atan2, sin, cos, ceil

from .space_time import PlanningCancelled, SpaceTimePlanner, TimedPoint, TimedPath


def position_at(path, t):
    """경로 시작 전과 도착 후에는 끝점에 계속 머문다."""
    points = path.points
    if t <= points[0].t:
        return points[0].x, points[0].y
    for a, b in zip(points, points[1:]):
        if t <= b.t:
            ratio = (t - a.t) / (b.t - a.t)
            return a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y)
    return points[-1].x, points[-1].y


def moving_discs_collide(a, b, c, d, clearance):
    """같은 시간 구간에서 등속 이동하는 두 원판의 최소 거리를 정확히 계산."""
    rx, ry = a[0] - c[0], a[1] - c[1]
    vx, vy = b[0] - a[0] - d[0] + c[0], b[1] - a[1] - d[1] + c[1]
    vv = vx * vx + vy * vy
    u = max(0., min(1., -(rx * vx + ry * vy) / vv)) if vv else 0.
    return hypot(rx + u * vx, ry + u * vy) <= clearance


def segment_conflicts(a, b, ta, tb, path, clearance):
    times = sorted({ta, tb, *(p.t for p in path.points if ta < p.t < tb)})
    if ta == tb:
        return hypot(a[0] - position_at(path, ta)[0], a[1] - position_at(path, ta)[1]) <= clearance
    def own(t):
        u = (t - ta) / (tb - ta)
        return a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])
    return any(moving_discs_collide(own(lo), own(hi), position_at(path, lo),
                                  position_at(path, hi), clearance)
               for lo, hi in zip(times, times[1:]))


def joint_conflicts(paths, radius, end_time=None):
    names = list(paths)
    conflicts = []
    for i, name in enumerate(names):
        path = paths[name]
        for other in names[i + 1:]:
            start = min(path.points[0].t, paths[other].points[0].t)
            end = end_time or max(path.points[-1].t, paths[other].points[-1].t)
            if end == start:
                if segment_conflicts(position_at(path, start), position_at(path, start),
                                     start, start, paths[other], 2 * radius):
                    conflicts.append((name, other))
                continue
            times = sorted({start, end, *(p.t for p in path.points if start < p.t < end)})
            if any(segment_conflicts(position_at(path, lo), position_at(path, hi), lo, hi,
                                     paths[other], 2 * radius)
                   for lo, hi in zip(times, times[1:])):
                conflicts.append((name, other))
    return conflicts


class ReservedField:
    def __init__(self, field, reservations, radius):
        self.field, self.reservations, self.radius = field, reservations, radius
        self.bounds = field.bounds

    def static_blocked_segment(self, a, b, radius):
        check = getattr(self.field, 'static_blocked_segment', None)
        return check(a, b, radius) if check else False

    def blocked_segment(self, a, b, ta, tb, radius):
        return self.field.blocked_segment(a, b, ta, tb, radius) or any(
            segment_conflicts(a, b, ta, tb, path, radius + self.radius)
            for path in self.reservations)


class JointPlanningFailure(RuntimeError):
    pass


def with_turn_holds(path, yaw, config, rate=.5):
    """BT 통합용 회전 대기. 변경된 시각은 예약·그림자 검사 전에 확정한다.

    독립 A* 행렬은 기존 이동·대기 모델을 유지한다. 추가 회전 시간은 공동
    실행 비용에만 포함하며 그 결과를 Hungarian 목적함수의 최적값으로 쓰지 않는다.
    """
    points = [path.points[0]]
    extra = 0
    for a, b in zip(path.points, path.points[1:]):
        if hypot(b.x-a.x, b.y-a.y) > 1e-9:
            heading = atan2(b.y-a.y, b.x-a.x)
            angle = abs(atan2(sin(heading-yaw), cos(heading-yaw)))
            ticks = ceil((angle/rate + .5)/config.time_step) if angle > .15 else 0
            if ticks:
                extra += ticks
                points.append(TimedPoint(a.x, a.y, a.t + extra*config.time_step))
            yaw = heading
        points.append(TimedPoint(b.x, b.y, b.t + extra*config.time_step))
    return TimedPath(tuple(points), path.cost_ticks + extra)


def coordinate(field, config, starts, assigned_goals, t0, standby=(), goal_hold=3., cancelled=None, headings=None,
               departure=None, diagnostics=None, safe_goals=False, heading_aware=False, goal_radius=None,
               shared_cache=None):
    """배정은 유지한다. 순서별 예약 재계획의 첫 유효 해를 반환(공동 최적화 아님).

    아직 계획하지 않은 로봇도 시작점의 정적 원판으로 넣는다. 따라서 앞선
    경로가 후순위 로봇의 출발 대기를 침범하지 않는다. 도착 후 점유도 유지한다.
    """
    from .departure import DeparturePolicy, coordinate_pairs, first_translation_time
    service_radius = config.robot_radius if goal_radius is None else goal_radius
    policy=DeparturePolicy.parse(departure)
    diagnostics=diagnostics if diagnostics is not None else {}
    if policy.mode == 'staggered_pairs':
        paths=coordinate_pairs(field,config,starts,assigned_goals,t0,standby,goal_hold,
                               cancelled,headings,policy,diagnostics,safe_goals,heading_aware)
        end=max(p.points[-1].t for p in paths.values())+goal_hold
        if any(field.with_static_robots(standby).blocked_segment(
                (p.points[-1].x,p.points[-1].y),(p.points[-1].x,p.points[-1].y),
                p.points[-1].t,end,service_radius) for p in paths.values()):
            raise JointPlanningFailure('목표 관측·후속 대기의 위치 여유 검사 실패')
        diagnostics['planned_translation_start_times']={r:first_translation_time(p) for r,p in paths.items()}
        return paths
    ids = tuple(assigned_goals)
    # 기본 4대. 설정 확장 시 factorial 폭증을 막고 보수적 실패를 보고한다.
    orders = permutations(ids) if len(ids) <= 6 else [ids]
    diagnostics['reservation_attempts']=[]
    # 순서의 앞부분이 같으면 미계획 원판과 이미 예약한 시간 경로도 같다.
    # 성공·실패를 이 호출 안에서만 재사용한다. 취소된 탐색은 저장하지 않는다.
    prefix_cache = shared_cache if shared_cache is not None else {}
    scope=(id(field),config,t0,goal_hold,service_radius,safe_goals,heading_aware,
           tuple(sorted((r,tuple(p)) for r,p in starts.items())),
           tuple(sorted((headings or {}).items())),tuple(sorted(tuple(p) for p in standby)))
    for order in orders:
        if cancelled is not None and cancelled():
            raise PlanningCancelled('공동 계획 계산 취소 또는 계산 예산 초과')
        paths,turn_ticks = {},{}
        trial={'order':list(order),'failure':None}
        diagnostics['reservation_attempts'].append(trial)
        for index, robot in enumerate(order):
            key=(scope,robot,tuple(assigned_goals[robot]),tuple(sorted(order[index+1:])),tuple(sorted(paths.items())))
            if key in prefix_cache:
                path, turns, failure = prefix_cache[key]
                if path is None:
                    trial['failure'], trial['robot'] = failure, robot
                    break
                paths[robot], turn_ticks[robot] = path, turns
                continue
            unplanned = [(starts[r][0], starts[r][1], config.robot_radius)
                         for r in order[index + 1:]]
            static = list(standby) + unplanned
            local = field.with_static_robots(static)
            reserved = ReservedField(local, tuple(paths.values()), config.robot_radius)
            planner = SpaceTimePlanner(reserved, config)
            options={}
            if safe_goals:options.update(goal_hold=goal_hold, goal_radius=service_radius)
            if heading_aware and headings:options['initial_yaw']=headings[robot]
            path = (planner.plan(starts[robot], assigned_goals[robot], t0, cancelled=cancelled,**options)
                    if cancelled is not None else planner.plan(starts[robot], assigned_goals[robot], t0,**options))
            if path is None:
                trial['failure']=planner.last_status;trial['robot']=robot
                prefix_cache[key]=(None,0,planner.last_status)
                break
            if heading_aware:
                turn_ticks[robot]=planner.last_turn_ticks
            elif headings is not None:
                original_cost=path.cost_ticks
                path = with_turn_holds(path, headings[robot], config)
                turn_ticks[robot]=path.cost_ticks-original_cost
                if (path.points[-1].t > t0 + config.horizon or any(
                    reserved.blocked_segment((a.x, a.y), (b.x, b.y), a.t, b.t, config.robot_radius)
                    for a, b in zip(path.points, path.points[1:]))):
                    trial['failure']='rotation_postprocess_forbidden';trial['robot']=robot
                    prefix_cache[key]=(None,0,trial['failure'])
                    break
            paths[robot] = path
            prefix_cache[key]=(path,turn_ticks.get(robot,0),None)
        if len(paths) != len(ids):
            continue
        end = max(p.points[-1].t for p in paths.values()) + goal_hold
        if safe_goals:
            # 초기 도착의 hold 실패만으로 목표 전체를 포기하지 않는다. 이미
            # 계획한 동료를 예약하고 다른 도착 시각·경로를 유한하게 재탐색한다.
            for _ in range(2*len(ids)):
                bad=next((r for r,p in paths.items() if field.with_static_robots(standby).blocked_segment(
                    (p.points[-1].x,p.points[-1].y),(p.points[-1].x,p.points[-1].y),p.points[-1].t,end,service_radius)),None)
                if bad is None:break
                reserved=ReservedField(field.with_static_robots(standby),tuple(p for r,p in paths.items() if r!=bad),config.robot_radius)
                planner=SpaceTimePlanner(reserved,config)
                candidate=planner.plan(starts[bad],assigned_goals[bad],t0,cancelled=cancelled,
                    goal_hold=goal_hold,goal_hold_until=end,
                    goal_radius=service_radius,
                    arrive_not_before=paths[bad].points[-1].t+config.time_step,
                    initial_yaw=headings[bad] if heading_aware and headings else None)
                if candidate is None:trial['failure']='no_safe_arrival_window';break
                raw_cost=candidate.cost_ticks
                if not heading_aware and headings:
                    candidate=with_turn_holds(candidate,headings[bad],config)
                turn_ticks[bad]=planner.last_turn_ticks if heading_aware else candidate.cost_ticks-raw_cost
                if any(reserved.blocked_segment((a.x,a.y),(b.x,b.y),a.t,b.t,config.robot_radius)
                       for a,b in zip(candidate.points,candidate.points[1:])):
                    trial['failure']='arrival_window_rotation_forbidden';break
                paths[bad]=candidate
                end=max(p.points[-1].t for p in paths.values())+goal_hold
        if any(p.points[-1].t>t0+config.horizon for p in paths.values()):
            trial['failure']='common_horizon_exhausted'
            continue
        if joint_conflicts(paths, config.robot_radius, end):
            trial['failure']='joint_robot_conflict'
            continue
        if any(field.with_static_robots(standby).blocked_segment(
                (p.points[-1].x, p.points[-1].y), (p.points[-1].x, p.points[-1].y),
                p.points[-1].t, end, service_radius) for p in paths.values()):
            trial['failure']=trial['failure'] or 'arrival_observation_hold_forbidden'
            continue
        diagnostics.update(policy='simultaneous',lead_robots=list(ids),follow_robots=[],
            release_times={r:t0 for r in ids},initial_wait_ticks_by_robot=dict.fromkeys(ids,0),
            staging_wait_ticks_by_robot=dict.fromkeys(ids,0),turn_ticks_by_robot=turn_ticks,
            planned_translation_start_times={r:first_translation_time(p) for r,p in paths.items()})
        return paths
    raise JointPlanningFailure('배정은 가능하지만 현재 우선순위 예약 탐색에서 안전한 공동 실행 경로를 찾지 못했습니다.')
