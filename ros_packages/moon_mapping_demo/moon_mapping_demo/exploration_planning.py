"""관측 기반 목표 후보 선정과 공통 t0 Hungarian 배정을 분리한다."""
from dataclasses import asdict, dataclass, replace
import hashlib
from itertools import combinations, product
import json
import math
from pathlib import Path
import time

import numpy as np

from .coordination import coordinate, JointPlanningFailure
from .exchange import write_problem
from .experiment import path_data
from .hungarian import solve, InfeasibleAssignment
from .metrics import atomic_json
from .space_time import PlanningCancelled, SpaceTimePlanner
from .risk import RiskPolicy,safe_until
from .vendor.team_bt.exploration_policy import FrontierGrid


@dataclass(frozen=True)
class Candidate:
    id: str
    x: float
    y: float
    yaw: float = 0.
    gain: float = 0.
    purpose: str = 'explore'
    work_region: str = ''
    preferred_robot: str = ''

    @property
    def xy(self):
        return self.x, self.y


def observed_candidates(observation, poses, field, radius, settings, excluded=()):
    """관측 자유 공간의 frontier 쪽으로 제한 거리만 전진하는 후보를 고른다.

    정보 이득은 미관측 셀 원판의 상한 휴리스틱이다. 가림을 고려한 실제
    면적은 ObservationGrid의 센서 광선 융합으로만 갱신한다.
    """
    area = observation.area
    planning_cells=observation.cells
    if settings.get('spatial_regions',False):
        # 다른 로봇의 스캔에 찍힌 현재 팀 몸체를 정적 지형으로 취급하면
        # 그 로봇의 BFS 시작점이 막힌다. 후보용 복사본에서 현재 몸체 반사만
        # 분리한다. 실제 관측/면적은 그대로 두고 동료 충돌은 공동 예약이 검사한다.
        planning_cells=observation.cells.copy()
        yy,xx=np.ogrid[:area.shape[0],:area.shape[1]]
        x=area.xmin+(xx+.5)*area.resolution
        y=area.ymin+(yy+.5)*area.resolution
        for pose in poses.values():
            if field.static_blocked_segment(pose[:2],pose[:2],radius):
                continue
            body=(x-pose[0])**2+(y-pose[1])**2 <= (radius+area.resolution/math.sqrt(2.))**2
            planning_cells[body & (planning_cells>=25)]=0
    grid = FrontierGrid(planning_cells, area.resolution, (area.xmin, area.ymin, 0.),
                        clearance=radius + .05, spacing=settings.get('candidate_spacing', 1.),
                        gain_radius=min(observation.sensor_range, 2.),
                        bounds=(area.xmin, area.ymin, area.xmax, area.ymax),
                        min_frontier_length=area.resolution, cache_distances=True)
    candidates = {}
    max_step = settings.get('target_step_m', 3.)
    min_step = settings.get('minimum_goal_distance_m', .8)
    frontiers = sorted(grid.candidates, key=lambda g: (-g.gain, g.x, g.y))[:settings.get('max_frontiers', 64)]
    if settings.get('spatial_regions',False):
        from .work_regions import region_candidates
        return region_candidates(observation,poses,field,radius,settings,grid,frontiers,excluded,Candidate),grid
    for robot, pose in sorted(poses.items()):
        distances = grid.distances(pose[:2])
        for goal in frontiers:
            if not math.isfinite(distances[grid.index(goal.xy)]):
                continue
            route = grid.path(pose[:2], goal.xy, distances)
            point, traveled = pose[:2], 0.
            for target in route[1:]:
                length = math.dist(point, target)
                if traveled + length > max_step:
                    break
                point, traveled = target, traveled + length
            if math.dist(point, pose[:2]) < min_step:
                continue
            if field.static_blocked_segment(point, point, radius):
                continue
            if any(math.dist(point, p) < r for p, r in excluded):
                continue
            if any(other != robot and math.dist(point, p[:2]) < 2 * radius + .1
                   for other, p in poses.items()):
                continue
            gain = observation.potential_gain(point)
            if gain < settings.get('minimum_potential_gain_m2', .5):
                continue
            row, col = area.index(point)
            candidate = Candidate(f'frontier_{row}_{col}', float(point[0]), float(point[1]),
                                  math.atan2(goal.y - point[1], goal.x - point[0]), gain)
            candidates[(row, col)] = candidate
    ranked = sorted(candidates.values(), key=lambda g: (-g.gain,
                    min(math.dist(g.xy, p[:2]) for p in poses.values()), g.id))
    risk=RiskPolicy.parse(settings.get('risk'))
    if settings.get('progressive_goals', False):
        # 정보 이득은 후보 순위용 상한이다. 겹치는 관측 원판은 한 번만
        # 우선 배정하며 실제 관측 면적은 센서 융합으로만 갱신한다.
        rows, cols = np.nonzero(observation.cells == -1)
        unknown_x = area.xmin + (cols + .5) * area.resolution
        unknown_y = area.ymin + (rows + .5) * area.resolution
        masks = {g: (unknown_x-g.x)**2 + (unknown_y-g.y)**2 <= observation.sensor_range**2
                 for g in ranked}
        covered = np.zeros(len(rows), dtype=bool)
        followup=risk.followup_seconds(settings['candidate_speed']) if risk.enabled else risk.followup_wait_seconds
        service = settings['observe_seconds'] + 3. + (followup if risk.safe_goal_windows else 0.)
        def reachable_soon(g):
            if not risk.safe_goal_windows:
                return True
            arrival = settings['candidate_t0'] + min(math.dist(g.xy, p[:2]) for p in poses.values()) / settings['candidate_speed']
            return not field.blocked_segment(g.xy, g.xy, arrival, arrival+service, radius)
        soon = {g: reachable_soon(g) for g in ranked}
        selected = []
        limit = settings.get('candidate_limit', len(poses))
        while ranked and len(selected) < limit:
            def score(g):
                gain = np.count_nonzero(masks[g] & ~covered) * area.resolution**2
                travel = min(math.dist(g.xy, p[:2]) for p in poses.values()) / settings['candidate_speed']
                return (not soon[g], -gain / (travel + settings['observe_seconds'] + 3.), -g.gain, g.id)
            candidate = min(ranked, key=score)
            ranked.remove(candidate)
            if any(math.dist(candidate.xy, other.xy) < settings.get('goal_separation_m', 1.) for other in selected):
                continue
            selected.append(candidate)
            covered |= masks[candidate]
        return selected, grid
    if risk.safe_goal_windows:
        t0=settings['candidate_t0']
        def service_rank(g):
            # 후보 순위용 유한 여유 추정. 불리한 최초 도착은 뒤로 정렬하되
            # 전역 도달 불가로 버리지 않는다. 실제 시간대는 A*가 다시 탐색한다.
            travel=min(math.dist(g.xy,p[:2]) for p in poses.values())/settings['candidate_speed']
            arrival=t0+travel
            required=settings['observe_seconds']+3.+risk.followup_wait_seconds
            until=safe_until(field,g.xy,arrival,risk.forecast_seconds,radius)
            slack=until-arrival-required
            return (slack<0,-math.floor(slack/10.),-g.gain,
                    min(math.dist(g.xy,p[:2]) for p in poses.values()),g.id)
        ranked.sort(key=service_rank)
    selected = []
    for candidate in ranked:
        if all(math.dist(candidate.xy, other.xy) >= settings.get('goal_separation_m', 1.) for other in selected):
            selected.append(candidate)
        if len(selected) == len(poses):
            break
    return selected, grid


def safety_candidates(field, poses, robots, radius, now, settings):
    """사전 지형에서 짧은 대피 목표를 고른다. 이 선택은 관측 완료가 아니다."""
    selected = []
    lead = settings.get('safety_lead_seconds', 1.)
    horizon = max(settings.get('stationary_safety_horizon', 20.), settings.get('safety_target_horizon', 60.))
    for robot in robots:
        x, y, yaw = poses[robot]
        chosen = None
        for distance in (.75, 1.5, 2.5, 3.5, 5., 7.):
            for angle in (0., math.pi / 4, -math.pi / 4, math.pi / 2, -math.pi / 2, math.pi):
                target = x + distance * math.cos(yaw + angle), y + distance * math.sin(yaw + angle)
                if any(math.dist(target, g.xy) < 2 * radius + .1 for g in selected):
                    continue
                if field.blocked_segment(target, target, now + lead, now + horizon, radius):
                    continue
                chosen = Candidate('safety_' + robot, *target, yaw + angle, purpose='safety')
                break
            if chosen:
                break
        if chosen:
            selected.append(chosen)
    return selected


def stationary_safe(field, poses, robots, ta, tb, radius):
    return all(not field.blocked_segment(poses[r][:2], poses[r][:2], ta, tb, radius) for r in robots)


def escape_candidate_pools(field,poses,pcfg,t0,hold,settings):
    """로봇별 복수 대피점. 사전 안전 계획용이며 관측 면적을 갱신하지 않는다."""
    risk=RiskPolicy.parse(settings.get('risk'))
    horizon=max(settings.get('safety_target_horizon',60.),hold+risk.required_seconds(pcfg.speed)['total_seconds'])
    pools={}
    for robot,(x,y,yaw) in poses.items():
        choices=[]
        for distance in (.75,1.5,2.5,3.5,5.,7.):
            for angle in (0.,math.pi/4,-math.pi/4,math.pi/2,-math.pi/2,math.pi):
                target=(x+distance*math.cos(yaw+angle),y+distance*math.sin(yaw+angle))
                estimate=distance/pcfg.speed+abs(angle)/risk.turn_rate
                if settings.get('arrival_window_escape',False):
                    # 대피점이 도착 전부터 안전할 필요는 없다. 도착 예상 뒤
                    # 실제 hold를 수용할 시간대가 있는 가까운 점도 후보로 둔다.
                    # 이는 순위용 추정이며 실제 도착·이동·회전·동료 점유는
                    # 동일한 Space-Time A*와 공동 실행 검사에서 다시 확인한다.
                    if field.static_blocked_segment(target,target,pcfg.robot_radius):continue
                    arrival=t0+estimate
                    if not any(arrival+delay<=t0+pcfg.horizon and not field.blocked_segment(
                            target,target,arrival+delay,arrival+delay+hold,pcfg.robot_radius)
                            for delay in (0.,5.,10.,20.)):continue
                elif field.blocked_segment(target,target,t0,t0+horizon,pcfg.robot_radius):continue
                choices.append((estimate,Candidate(f'escape_{robot}_{len(choices)}',*target,yaw+angle,purpose='safety')))
        choices.sort(key=lambda v:(v[0],v[1].id))
        selected=[]
        for _,candidate in choices:
            if all(math.dist(candidate.xy,g.xy)>=.75 for g in selected):selected.append(candidate)
            if len(selected)>=risk.escape_candidates_per_robot:break
        pools[robot]=selected
    return pools


def plan_escape_batch(field,pcfg,poses,candidates,t0,created_sim,hold,robot_order,output,cancel_event,settings,require_all=True):
    risk=RiskPolicy.parse(settings.get('risk'));started=time.monotonic()
    deadline=started+max(0.,settings.get('planning_wall_budget_seconds',8.)-
                        settings.get('planning_result_margin_seconds',0.))
    fast_horizon=settings.get('max_path_seconds',0.)
    if settings.get('allow_partial_escape',False) and not settings.get('_partial_escape_attempted',False):
        # 팀이 분산된 뒤에도 모든 로봇의 대피점을 교차 배정하면, 안전한
        # 동료의 긴 이동 비용 때문에 위험 로봇의 짧은 탈출을 놓칠 수 있다.
        # 유한 회차 동안 정지 불가능한 기체만 먼저 움직인다. 대기 기체는
        # 실제 공동 계획 끝까지 기존 몸체·그림자·동료 충돌 검사를 통과해야 한다.
        # 경로 상한 전체를 기다린다고 가정하면 안전한 동료까지 모두
        # 위험으로 묶인다. 우선 최소 서비스 구간을 기준으로 고르고,
        # 실제 이동을 더한 끝 시각의 대기 가능성은 plan_batch가 판정한다.
        end=t0+hold
        endangered=[r for r in robot_order if not stationary_safe(
            field,poses,[r],created_sim,end,pcfg.robot_radius)]
        if 0<len(endangered)<len(robot_order):
            partial_budget=max(0.,deadline-time.monotonic())*.5
            partial=plan_escape_batch(field,pcfg,poses,candidates,t0,created_sim,hold,endangered,
                Path(output)/'endangered_only',cancel_event,
                {**settings,'planning_wall_budget_seconds':partial_budget,
                 'planning_result_margin_seconds':0.,'_partial_escape_attempted':True},True)
            if partial['status'] in ('OK','CANCELLED'):
                partial['escape_scope']='endangered_only'
                return partial
            remaining=max(0.,deadline-time.monotonic())
            full=plan_escape_batch(field,pcfg,poses,candidates,t0,created_sim,hold,robot_order,
                Path(output)/'full_team_fallback',cancel_event,
                {**settings,'planning_wall_budget_seconds':remaining,
                 'planning_result_margin_seconds':0.,'_partial_escape_attempted':True},require_all)
            full.update(partial_escape_status=partial['status'],escape_scope='full_team_fallback')
            return full
    if (settings.get('progressive_goals',False) and not settings.get('_escape_full_horizon',False)
            and 0 < fast_horizon < pcfg.horizon):
        # 가까운 대피를 먼저 찾는다. 멀리 떨어진 로봇 간 교차 배정 비용을
        # 긴 horizon 전체에서 계산하느라 빠른 탈출 기회를 잃지 않게 한다.
        # 실패하면 남은 동일 예산으로 원래 horizon도 탐색한다.
        fast_budget=min(4.,max(0.,deadline-time.monotonic())*.6)
        fast=plan_escape_batch(field,replace(pcfg,horizon=fast_horizon),poses,candidates,
            t0,created_sim,hold,robot_order,Path(output)/'short_horizon',cancel_event,
            {**settings,'planning_wall_budget_seconds':fast_budget,
             'planning_result_margin_seconds':0.,'_escape_full_horizon':True},require_all)
        atomic_json(Path(output)/'escape_horizon_search.json',{
            'short_horizon_seconds':fast_horizon,'full_horizon_seconds':pcfg.horizon,
            'short_status':fast['status'],'short_budget_seconds':fast_budget,
            'full_horizon_fallback':fast['status'] not in ('OK','CANCELLED')})
        if fast['status'] in ('OK','CANCELLED'):
            fast['escape_horizon_seconds']=fast_horizon
            return fast
    pools=escape_candidate_pools(field,{r:poses[r] for r in robot_order},pcfg,t0,hold,settings)
    empty=[r for r,values in pools.items() if not values]
    if empty:return {'status':'NO_ESCAPE_CANDIDATES','error':'유한 안전 대피 후보 없음: '+','.join(empty),'attempts':[]}
    robots=tuple(robot_order)
    combinations_by_rank=list(product(*(range(len(pools[r])) for r in robots)))
    combinations_by_rank.sort(key=lambda v:(sum(v),v))
    trials=[];last=None;best=None;best_key=None;search_end='CANDIDATE_LIMIT';all_attempts=[]
    for ranks in combinations_by_rank:
        if cancel_event.is_set():return {'status':'CANCELLED','error':'복수 대피 후보 계산 취소','attempts':all_attempts,'candidate_trials':trials}
        remaining=deadline-time.monotonic()
        if remaining<=0:
            search_end='PLANNING_BUDGET'
            break
        goals=[pools[r][rank] for r,rank in zip(robots,ranks)]
        if any(math.dist(a.xy,b.xy)<2*pcfg.robot_radius+.1 for i,a in enumerate(goals) for b in goals[i+1:]):continue
        if len(trials)>=risk.escape_candidate_sets:break
        number=len(trials);directory=Path(output)/f'candidate_set_{number:03d}'
        value=plan_batch(field,pcfg,poses,goals,t0,created_sim,hold,robots,directory,cancel_event,
            {**settings,'planning_wall_budget_seconds':remaining,'planning_result_margin_seconds':0.},True)
        all_attempts.extend(value.get('attempts',[]))
        trials.append({'ranks':list(ranks),'goals':[asdict(g) for g in goals],'status':value['status'],
                       'error':value['error'],'directory':str(directory)})
        if value['status']=='OK':
            end=max(p.points[-1].t for p in value['paths'].values())+hold
            margin=min(safe_until(field,g.xy,value['paths'][r].points[-1].t,risk.forecast_seconds,pcfg.robot_radius)-end
                       for r,g in value['assigned'].items())
            key=(-margin,value['coordinated_total_ticks'],number)
            trials[-1].update(finite_goal_margin_seconds=margin,coordinated_ticks=value['coordinated_total_ticks'])
            if best_key is None or key<best_key:
                best,best_key=value,key
                best['selected_candidate_set']=number
        last=value
        if value['status'] in ('PLANNING_BUDGET','CANCELLED'):
            search_end=value['status'];break
    if cancel_event.is_set():return {'status':'CANCELLED','error':'복수 대피 후보 계산 취소','attempts':all_attempts,'candidate_trials':trials}
    if best is not None:
        best.update(candidate_trials=trials,escape_search_end=search_end,
            attempts=all_attempts,
            escape_candidate_pools={r:[asdict(g) for g in gs] for r,gs in pools.items()})
        atomic_json(Path(output)/'escape_search.json',{'trials':trials,'selected_candidate_set':best['selected_candidate_set'],
            'search_end':search_end,'policy':'검증 완료된 유한 후보 중 목표 대기 여유 우선, 공동 비용 차순; 전역 최적성 없음'})
        return best
    if last is None and search_end=='PLANNING_BUDGET':
        last={'status':'PLANNING_BUDGET','error':'복수 대피 후보 전체 계산 예산 초과','attempts':[]}
    if last is None:last={'status':'NO_SAFE_ASSIGNMENT','error':'서로 떨어진 유효 대피 후보 조합 없음','attempts':[]}
    last['candidate_trials']=trials
    last['attempts']=all_attempts
    atomic_json(Path(output)/'escape_search.json',{'trials':trials,'status':last['status'],'error':last['error']})
    return last


def plan_batch(field, pcfg, poses, candidates, t0, created_sim, hold, robot_order,
               output, cancel_event, settings, require_all=False):
    if not any(g.work_region for g in candidates):
        return _plan_batch(field,pcfg,poses,candidates,t0,created_sim,hold,robot_order,
                           output,cancel_event,settings,require_all)
    if not settings.get('persistent_region_ownership',False):
        # 구역은 서로 다르게 맡되, 실제 그림자 시간 경로 비용으로 로봇을
        # 배정한다. 직전 담당은 다음 구역 목표를 고르는 연속성 비용에 반영한다.
        value=_plan_batch(field,pcfg,poses,candidates,t0,created_sim,hold,robot_order,
            output,cancel_event,{**settings,'region_ownership':False},require_all)
        value.update(allocation_strategy='spatial_work_regions',ownership_reassigned=any(
            g.preferred_robot and r!=g.preferred_robot for r,g in value.get('assigned',{}).items()))
        return value
    started=time.monotonic()
    budget=max(0.,settings.get('planning_wall_budget_seconds',8.)-settings.get('planning_result_margin_seconds',0.))
    value=_plan_batch(field,pcfg,poses,candidates,t0,created_sim,hold,robot_order,output,cancel_event,
        {**settings,'planning_wall_budget_seconds':budget*.55,'planning_result_margin_seconds':0.,
         'region_ownership':True,'region_full_team_first':False},require_all)
    incumbent=value if value['status']=='OK' else None
    reassigned=False
    desired=min(len(robot_order),len({g.work_region for g in candidates if g.work_region}))
    if (value['status']!='CANCELLED' and not cancel_event.is_set()
            and (incumbent is None or len(incumbent['paths'])<desired)):
        remaining=budget-(time.monotonic()-started)
        if remaining>0:
            alternative=_plan_batch(field,pcfg,poses,candidates,t0,created_sim,hold,robot_order,
                Path(output)/'region_reassignment',cancel_event,
                {**settings,'planning_wall_budget_seconds':remaining,'planning_result_margin_seconds':0.,
                 'region_ownership':False,'region_full_team_first':False,
                 'region_minimum_team_size':len(incumbent['paths'])+1 if incumbent else 1},require_all)
            if alternative['status']=='CANCELLED' or cancel_event.is_set():
                return {'status':'CANCELLED','error':'작업 구역 재배정 중 임무 취소','attempts':alternative.get('attempts',[])}
            if alternative['status']=='OK' or incumbent is None:
                value=alternative
                reassigned=True
            else:
                value=incumbent
                value['region_search_end']=alternative['status']
    value.update(allocation_strategy='spatial_work_regions',ownership_reassigned=reassigned)
    return value


def _plan_batch(field, pcfg, poses, candidates, t0, created_sim, hold, robot_order,
               output, cancel_event, settings, require_all=False):
    """4개 배정은 4×4, 부분 배정은 실제 활동 부분집합의 정사각 행렬이다.

    각 부분집합에서는 대기 로봇을 정적 장애물로 넣어 비용을 재계산한다.
    후보/로봇 부분집합 선택은 유한 휴리스틱이며 전체 탐사 최적화가 아니다.
    """
    started = time.monotonic()
    risk=RiskPolicy.parse(settings.get('risk'))
    # 일반 탐사에서는 수십~백 초 뒤의 목표를 기다리는 대신 다음 후보를
    # 시도한다. 고정 목표와 대피 요청에는 원래 horizon을 그대로 적용한다.
    exploratory = bool(candidates) and all(g.purpose in ('explore','support') for g in candidates)
    if exploratory and settings.get('max_path_seconds'):
        pcfg = replace(pcfg, horizon=min(pcfg.horizon, settings['max_path_seconds']))
    # 대피에도 현재 방향과 회전을 탐색에 포함한다. 이동만 찾은 뒤 회전
    # 시간을 더해 탈락한 같은 경로를 우선순위별로 반복하지 않는다.
    joint_heading = risk.heading_planner or settings.get('turn_aware_coordination', False)
    service_radius = pcfg.robot_radius + (risk.warning_position_margin_m
        if exploratory and settings.get('progressive_goals', False) and risk.safe_goal_windows else 0.)
    # worker가 제한 직전에 취소돼도 결과 직렬화·콜백 수신을 끝낼 여유를 둔다.
    # 제어기의 실제 8초 제한과 sim 5초 제한을 늘리지 않는다.
    deadline = started + max(0., settings.get('planning_wall_budget_seconds', 8.) -
                            settings.get('planning_result_margin_seconds', 0.))
    cancelled = lambda: cancel_event.is_set() or time.monotonic() > deadline
    robot_order, candidates = tuple(robot_order), tuple(candidates)
    output = Path(output)
    attempts = []
    max_attempts = settings.get('max_assignment_attempts', 24)
    largest = min(len(robot_order), len(candidates))
    if any(g.work_region for g in candidates):
        largest=min(len(robot_order),len({g.work_region for g in candidates if g.work_region}))
    preserve_four=(settings.get('departure',{}).get('mode')=='staggered_pairs'
                   and len(robot_order)==len(candidates)==4)
    if require_all and (largest != len(robot_order) or len(candidates) != len(robot_order)):
        return {'status': 'NO_COMPLETE_ASSIGNMENT', 'error': '필수 전체 목표 개수가 맞지 않습니다.', 'attempts': []}
    sizes = [largest] if require_all or preserve_four or settings.get('region_full_team_first',False) else range(largest, 0, -1)
    sizes=[size for size in sizes if size>=settings.get('region_minimum_team_size',1)]
    try:
        for size in sizes:
            for robots in combinations(robot_order, size):
                idle = [r for r in poses if r not in robots]
                standby = [(poses[r][0], poses[r][1], pcfg.robot_radius) for r in idle]
                local = field.with_static_robots(standby)
                planner = SpaceTimePlanner(local, pcfg)
                pair_cache = {}
                coordination_cache = {}
                for goals in combinations(candidates, size):
                    regions=[g.work_region for g in goals if g.work_region]
                    if len(regions)!=len(set(regions)):
                        continue
                    if settings.get('region_ownership',False) and any(g.preferred_robot and g.preferred_robot not in robots for g in goals):
                        continue
                    if cancelled():
                        raise PlanningCancelled('임무 계산 취소 또는 실제 시간 계산 예산 초과')
                    # 불가능한 행렬은 공동 경로 시도가 아니다. 이런 행렬로
                    # 횟수를 소진해 뒤의 안전한 부분 배정을 막지 않는다.
                    # 행렬 순회도 동일한 실제 시간 예산 안에서만 수행한다.
                    used_attempts=sum(a['assignment'] is not None for a in attempts)
                    if used_attempts >= max_attempts:
                        return {'status': 'ATTEMPT_LIMIT', 'error': '부분집합 탐색 횟수 제한', 'attempts': attempts}
                    matrix_started = time.perf_counter()
                    costs, statuses, matrix_turns = [], {},[]
                    for robot in robots:
                        row = []
                        turn_row=[]
                        statuses[robot] = {}
                        for goal in goals:
                            if settings.get('region_ownership',False) and goal.preferred_robot and goal.preferred_robot!=robot:
                                row.append(None);turn_row.append(None)
                                statuses[robot][goal.id]='different_work_region'
                                continue
                            key = robot, goal
                            if key not in pair_cache:
                                path = planner.plan(poses[robot][:2], goal.xy, t0, cancelled=cancelled,
                                    goal_hold=hold if risk.safe_goal_windows else 0.,
                                    goal_radius=service_radius,
                                    initial_yaw=poses[robot][2] if joint_heading else None)
                                pair_cache[key] = (path.cost_ticks if path else None,
                                                   planner.last_turn_ticks if path else None,
                                                   planner.last_status)
                            cost, turns, status = pair_cache[key]
                            row.append(cost)
                            turn_row.append(turns)
                            statuses[robot][goal.id] = status
                        costs.append(row)
                        matrix_turns.append(turn_row)
                    matrix_seconds = time.perf_counter() - matrix_started
                    assignment_started = time.perf_counter()
                    assignment, error = None, None
                    try:
                        assignment = solve(costs)
                    except InfeasibleAssignment as exc:
                        error = str(exc)
                    assignment_seconds = time.perf_counter() - assignment_started
                    number = len(attempts)
                    attempt_dir = output / f'attempt_{number:03d}'
                    metadata = {'map_id': getattr(field, 'identifier', 'synthetic'),
                                'planner': asdict(pcfg), 'goals': [asdict(g) for g in goals],
                                'time_model':'space_time_heading_turns' if joint_heading else 'space_time_move_wait_post_rotation',
                                'coordination_time_model': 'space_time_heading_turns' if joint_heading else 'space_time_move_wait_post_rotation',
                                'coordinated_goal_radius_m': service_radius,
                                'work_region_ownership': settings.get('region_ownership',False),
                                'work_regions':{g.id:g.work_region for g in goals},
                                'goal_service_hold_seconds':hold if risk.safe_goal_windows else 0.,
                                'independent_turn_ticks':matrix_turns,
                                'config_id': hashlib.sha256(json.dumps({'planner': asdict(pcfg),
                                    'goals': [asdict(g) for g in goals], 'settings': settings}, sort_keys=True).encode()).hexdigest(),
                                'all_robot_positions': {r: list(p[:2]) for r, p in poses.items()},
                                'all_robot_yaw':{r:p[2] for r,p in poses.items()},
                                'standby_robot_ids': idle, 'pair_statuses': statuses,
                                'candidate_selection': ('observed frontier progress / information gain heuristic'
                                    if all(g.purpose == 'explore' for g in goals) else
                                    'fixed validation goals' if all(g.purpose == 'fixed' for g in goals) else
                                    'prior terrain safety evacuation heuristic'),
                                'optimization_scope': 'fixed selected robot/goal subset integer sum only'}
                    write_problem(attempt_dir, robots, [g.id for g in goals], costs, t0, pcfg.time_step,
                                  metadata, matrix_seconds, assignment_seconds, assignment)
                    attempt = {'robots': list(robots), 'goal_ids': [g.id for g in goals],
                               'costs': costs, 'assignment': asdict(assignment) if assignment else None,
                               'matrix_seconds': matrix_seconds, 'assignment_seconds': assignment_seconds,
                               'error': error, 'directory': str(attempt_dir)}
                    attempt['pair_statuses']=statuses
                    attempts.append(attempt)
                    if assignment is None:
                        continue
                    assigned = {r: goals[j] for r, j in zip(robots, assignment.columns)}
                    coordination_started = time.perf_counter()
                    schedule = {}
                    try:
                        paths = coordinate(field, pcfg, {r: p[:2] for r, p in poses.items()},
                                           {r: g.xy for r, g in assigned.items()}, t0, standby, hold, cancelled,
                                           headings={r: p[2] for r, p in poses.items()},
                                           departure=settings.get('departure'),diagnostics=schedule,
                                           safe_goals=risk.safe_goal_windows,heading_aware=joint_heading,
                                           goal_radius=service_radius,shared_cache=coordination_cache)
                    except JointPlanningFailure as exc:
                        attempt['error'] = str(exc)
                        continue
                    finally:
                        attempt['departure'] = schedule
                    end = max(p.points[-1].t for p in paths.values()) + hold
                    if not stationary_safe(field, poses, poses, created_sim, t0, pcfg.robot_radius):
                        attempt['error'] = '계산·공동 출발 대기 중 그림자 위험'
                        continue
                    unsafe_idle=[r for r in idle if not stationary_safe(
                        field,poses,[r],created_sim,end,pcfg.robot_radius)]
                    if unsafe_idle:
                        attempt['error'] = '미배정 로봇의 임무 전체 대기 안전성 실패'
                        attempt['unsafe_standby_robot_ids']=unsafe_idle
                        continue
                    result = {'status': 'OK', 'error': None, 't0': t0, 'paths': paths,
                              'time_model':metadata['time_model'],
                              'coordination_time_model':metadata['coordination_time_model'],
                              'coordinated_goal_radius_m':service_radius,
                              'execution_hold_seconds':hold,
                              'assigned': assigned, 'standby_robot_ids': idle,
                              'costs': costs, 'assignment': assignment,
                              'original_assignment_total_ticks': assignment.total_cost,
                              'coordinated_total_ticks': sum(p.cost_ticks for p in paths.values()),
                              'departure': schedule,
                              'cost_breakdown': {
                                  'hungarian_total_ticks': assignment.total_cost,
                                  'coordinated_total_ticks': sum(p.cost_ticks for p in paths.values()),
                                  'initial_departure_wait_ticks': sum(schedule.get('initial_wait_ticks_by_robot',{}).values()),
                                  'staging_wait_ticks': sum(schedule.get('staging_wait_ticks_by_robot',{}).values()),
                                  'turn_ticks': sum(schedule.get('turn_ticks_by_robot',{}).values()),
                                  'independent_assignment_turn_ticks':sum(matrix_turns[i][j] for i,j in enumerate(assignment.columns)),
                                  'additional_turn_ticks':sum(schedule.get('turn_ticks_by_robot',{}).values())-sum(matrix_turns[i][j] for i,j in enumerate(assignment.columns)),
                                  'additional_total_ticks': sum(p.cost_ticks for p in paths.values())-assignment.total_cost,
                                  'route_and_other_wait_delta_ticks':sum(p.cost_ticks for p in paths.values())-assignment.total_cost
                                      -sum(schedule.get('initial_wait_ticks_by_robot',{}).values())
                                      -sum(schedule.get('staging_wait_ticks_by_robot',{}).values())
                                      -sum(schedule.get('turn_ticks_by_robot',{}).values())
                                      +sum(matrix_turns[i][j] for i,j in enumerate(assignment.columns))},
                              'coordination_seconds': time.perf_counter() - coordination_started,
                              'attempts': attempts, 'wall_seconds': time.monotonic() - started}
                    atomic_json(output / 'batch_plan.json', {**{k: v for k, v in result.items()
                        if k not in ('paths', 'assigned', 'assignment')}, 'paths': path_data(paths),
                        'assigned': {r: asdict(g) for r, g in assigned.items()}, 'assignment': asdict(assignment)})
                    return result
    except PlanningCancelled as exc:
        return {'status': 'CANCELLED' if cancel_event.is_set() else 'PLANNING_BUDGET',
                'error': str(exc), 'attempts': attempts}
    return {'status': 'NO_SAFE_ASSIGNMENT', 'error': '선정 목표의 유한 부분집합 탐색에서 안전한 공동 배정을 찾지 못했습니다.',
            'attempts': attempts}
