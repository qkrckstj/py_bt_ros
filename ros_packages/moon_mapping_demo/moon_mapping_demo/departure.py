"""Hungarian 배정을 고정한 뒤 선발 조합·출발 시차를 유한 탐색한다.

선발 두 대가 통로의 초기 구간을 비우면 후발도 출발한다. 이후 이동 로봇
수를 두 대로 제한하지 않는다. 이 정책은 공동 실행의 최적화 알고리즘이 아니다.
"""
from dataclasses import asdict, dataclass, replace
from itertools import combinations
import math

from .coordination import (JointPlanningFailure, ReservedField, joint_conflicts,
                           moving_discs_collide, segment_conflicts, with_turn_holds)
from .space_time import PlanningCancelled, SpaceTimePlanner, TimedPath, TimedPoint


@dataclass(frozen=True)
class DeparturePolicy:
    mode: str = 'simultaneous'
    delay_candidates_seconds: tuple = (0., 2., 4., 8.)
    clearance_distance_m: float = 1.
    max_schedule_attempts: int = 24
    staging_distance_m: float = .75
    release_region: tuple | None = None  # xmin, xmax, ymin, ymax; 없으면 이동 거리 기준

    def __post_init__(self):
        if self.mode not in ('simultaneous', 'staggered_pairs'):
            raise ValueError('departure.mode는 simultaneous 또는 staggered_pairs여야 합니다.')
        if (not self.delay_candidates_seconds or any(type(v) not in (int,float) or
            not math.isfinite(v) or v < 0 for v in self.delay_candidates_seconds)):
            raise ValueError('출발 시차 후보는 0 이상의 유한 초 목록이어야 합니다.')
        for key in ('clearance_distance_m','staging_distance_m'):
            if not math.isfinite(getattr(self,key)) or getattr(self,key) <= 0:
                raise ValueError(key + '는 양의 유한 거리여야 합니다.')
        if type(self.max_schedule_attempts) is not int or not 1 <= self.max_schedule_attempts <= 256:
            raise ValueError('max_schedule_attempts는 1~256의 정수여야 합니다.')
        if self.release_region is not None:
            r=self.release_region
            if len(r)!=4 or not all(math.isfinite(x) for x in r) or r[0]>=r[1] or r[2]>=r[3]:
                raise ValueError('release_region은 xmin,xmax,ymin,ymax 순서의 유효 사각형이어야 합니다.')

    @classmethod
    def parse(cls, value=None):
        return value if isinstance(value,cls) else cls(**(value or {}))


def first_translation_time(path):
    return next((a.t for a,b in zip(path.points,path.points[1:])
                 if math.dist((a.x,a.y),(b.x,b.y))>1e-9),None)


def clear_time(path, policy, radius):
    """선발의 목표 도착을 요구하지 않는 초기 통로 해제 시각."""
    origin=(path.points[0].x,path.points[0].y)
    goal=(path.points[-1].x,path.points[-1].y)
    target_distance=min(policy.clearance_distance_m,math.dist(origin,goal)/2.)
    if math.dist(origin,goal)<1e-9:
        return path.points[0].t
    for a,b in zip(path.points,path.points[1:]):
        length=math.dist((a.x,a.y),(b.x,b.y))
        if length<=1e-9:
            continue
        if policy.release_region is None:
            dx,dy=b.x-a.x,b.y-a.y
            ox,oy=a.x-origin[0],a.y-origin[1]
            if math.dist((b.x,b.y),origin)>=target_distance-1e-9:
                aa=dx*dx+dy*dy;bb=2*(ox*dx+oy*dy)
                cc=ox*ox+oy*oy-target_distance*target_distance
                fraction=max(0.,min(1.,(-bb+math.sqrt(max(0.,bb*bb-4*aa*cc)))/(2*aa)))
                return a.t+fraction*(b.t-a.t)
        else:
            xmin,xmax,ymin,ymax=policy.release_region
            # 몸체까지 사각형 밖으로 나가는 선분의 최초 시각을 계산한다.
            for index,(low,high) in enumerate(((xmin-radius,xmax+radius),(ymin-radius,ymax+radius))):
                start=(a.x,a.y)[index];end=(b.x,b.y)[index]
                if end<low or end>high:
                    edge=low if end<low else high
                    fraction=(edge-start)/(end-start) if end!=start else 0.
                    if 0<=fraction<=1:
                        return a.t+fraction*(b.t-a.t)
    return None


def observed_clear(path, xy, policy, radius):
    """후발 해제 때 실제 odom으로 같은 통로 해제 조건을 확인한다."""
    if first_translation_time(path) is None:
        return True
    if policy.release_region is not None:
        xmin,xmax,ymin,ymax=policy.release_region
        return xy[0]<xmin-radius or xy[0]>xmax+radius or xy[1]<ymin-radius or xy[1]>ymax+radius
    start=(path.points[0].x,path.points[0].y);goal=(path.points[-1].x,path.points[-1].y)
    return math.dist(xy,start)>=min(policy.clearance_distance_m,math.dist(start,goal)/2.)


def _blocked_reason(field, reservations, a, b, ta, tb, radius):
    static=getattr(field,'static_blocked_segment',lambda a,b,r:False)
    if static(a,b,radius):
        return 'static_body_or_terrain'
    if field.blocked_segment(a,b,ta,tb,radius):
        return 'shadow_or_time_forbidden'
    if any(segment_conflicts(a,b,ta,tb,p,2*radius) for p in reservations):
        return 'robot_reservation'
    return None


def _turns(path, yaw, field, reservations, config):
    turned=with_turn_holds(path,yaw,config) if yaw is not None else path
    for a,b in zip(turned.points,turned.points[1:]):
        reason=_blocked_reason(field,reservations,(a.x,a.y),(b.x,b.y),a.t,b.t,config.robot_radius)
        if reason:
            return None, reason, 0
    return turned,None,turned.cost_ticks-path.cost_ticks


def _rank_pairs(field, config, starts, goals, t0, policy, cancelled):
    """근처 몸체가 경로를 막는 정도·그림자 접근·현재 목표 방향을 함께 고려."""
    names=tuple(goals)
    urgency={};blocking=dict.fromkeys(names,0)
    for robot in names:
        if cancelled and cancelled():
            raise PlanningCancelled('선발 조합 평가 중 계산 취소')
        p=starts[robot]
        urgency[robot]=next((s for s in (2.,4.,8.,16.,32.) if
            field.blocked_segment(p,p,t0,t0+s,config.robot_radius)),64.)
        target=goals[robot]
        length=math.dist(p,target)
        if length>1e-9:
            tip=(p[0]+(target[0]-p[0])*min(1.,2*policy.clearance_distance_m/length),
                 p[1]+(target[1]-p[1])*min(1.,2*policy.clearance_distance_m/length))
            for other in names:
                if other==robot: continue
                # 목표 방향의 초반 통로를 점유하는 로봇을 먼저 비운다.
                if moving_discs_collide(p,tip,starts[other],starts[other],2*config.robot_radius+.05):
                    blocking[other]+=1
    pairs=list(combinations(names,min(2,len(names))))
    def score(pair):
        tail=[r for r in names if r not in pair]
        return (sum(1/urgency[r] for r in tail),-sum(blocking[r] for r in pair),
                sum(math.dist(starts[r],goals[r]) for r in pair),tuple(sorted(pair)))
    pairs.sort(key=score)
    return pairs,{'shadow_wait_deadline_seconds':urgency,'initial_body_blocking_score':blocking,
                  'ranked_lead_pairs':[list(p) for p in pairs]}


def _last_heading(path, fallback):
    for a,b in reversed(list(zip(path.points,path.points[1:]))):
        if math.dist((a.x,a.y),(b.x,b.y))>1e-9:
            return math.atan2(b.y-a.y,b.x-a.x)
    return fallback


def _movement(field, config, start, goal, release, t0, yaw, reservations, cancelled,
              safe_goals=False,heading_aware=False,hold=0.):
    # 새 출발 시각에서 A*를 다시 실행한다. 이전 경로의 시각을 이동하지 않는다.
    horizon=config.horizon-(release-t0)
    if horizon<config.time_step:
        return None,'common_horizon_exhausted',0
    local_config=replace(config,horizon=horizon)
    planner=SpaceTimePlanner(ReservedField(field,reservations,config.robot_radius),local_config)
    path=planner.plan(start,goal,release,cancelled=cancelled,goal_hold=hold if safe_goals else 0.,
                      initial_yaw=yaw if heading_aware else None)
    if path is None:
        return None,planner.last_status,0
    return (path,None,planner.last_turn_ticks) if heading_aware else _turns(path,yaw,field,reservations,config)


def coordinate_pairs(field, config, starts, goals, t0, standby, hold, cancelled, headings,
                     policy, diagnostics,safe_goals=False,heading_aware=False):
    names=tuple(goals)
    if len(names)<3:
        # 4대 배정은 절대로 2대로 축소하지 않는다. 실제 목표 부족의 부분
        # 배정은 호출자가 별도로 결정하며 여기서는 입력된 배정을 전부 유지한다.
        from .coordination import coordinate
        paths=coordinate(field,config,starts,goals,t0,standby,hold,cancelled,headings,diagnostics=diagnostics,
                         safe_goals=safe_goals,heading_aware=heading_aware)
        diagnostics.update(policy=policy.mode,note='배정된 로봇이 2대 이하여서 별도 후발 없음')
        return paths
    pairs,ranking=_rank_pairs(field,config,starts,goals,t0,policy,cancelled)
    diagnostics.update(policy=policy.mode,parameters=asdict(policy),ranking=ranking,attempts=[])
    attempts=diagnostics['attempts']
    cache={}
    # 조합 순위·순서 변경·추가 시차를 섞어 제한 예산에서도 후보를 분산한다.
    variants=[]
    for delay in sorted(set(policy.delay_candidates_seconds)):
        variants.append((delay,'start'))
        if delay>0:
            variants.append((delay,'forward_staging'))
    schedules=[(rank,vindex,order,pair,delay,location)
               for rank,pair in enumerate(pairs)
               for vindex,(delay,location) in enumerate(variants) for order in range(4)]
    schedules.sort(key=lambda s:(s[0]+s[1]+s[2],s[1],s[0],s[2]))
    for _,_,variant_index,pair,extra,location in schedules:
        if cancelled and cancelled():
            raise PlanningCancelled('출발 시차 탐색 취소 또는 계산 예산 초과')
        if len(attempts)>=policy.max_schedule_attempts:
            raise JointPlanningFailure('출발 시차 탐색 횟수 제한 안에서 안전한 공동 계획을 찾지 못했습니다.')
        tails=tuple(r for r in names if r not in pair)
        lead_order=tuple(pair if variant_index%2==0 else reversed(pair))
        tail_order=tuple(tails if variant_index//2==0 else reversed(tails))
        trial={'lead_robots':list(lead_order),'follow_robots':list(tail_order),
               'extra_wait_seconds':extra,'wait_location':location,'failure':None}
        attempts.append(trial)
        if lead_order not in cache:
            paths={};turns={};error=None
            for index,r in enumerate(lead_order):
                unplanned=lead_order[index+1:]+tails
                local=field.with_static_robots(list(standby)+[(starts[n][0],starts[n][1],config.robot_radius) for n in unplanned])
                path,error,rotation=_movement(local,config,starts[r],goals[r],t0,t0,
                    headings.get(r) if headings else None,tuple(paths.values()),cancelled,safe_goals,heading_aware,hold)
                if error: break
                paths[r]=path;turns[r]=rotation
            cache[lead_order]=(paths,turns,error)
        base,turns,error=cache[lead_order]
        paths,turns=dict(base),dict(turns)
        if error:
            trial['failure']='lead: '+error
            continue
        clears={r:clear_time(paths[r],policy,config.robot_radius) for r in lead_order}
        if any(t is None for t in clears.values()):
            trial['failure']='lead_does_not_clear_initial_region'
            continue
        # 한 tick 뒤에 해제하여 waypoint 도착 허용 오차의 직전 정지를 피한다.
        # 그래도 실제 통과가 확인되지 않으면 실행기는 전체를 재계획한다.
        clear_tick=math.ceil((max(clears.values())-t0)/config.time_step-1e-9)+1
        extra_tick=math.ceil(extra/config.time_step-1e-9)
        release=t0+(clear_tick+(extra_tick if location=='start' else 0))*config.time_step
        trial['release_sim_seconds']=release
        initial_wait={r:0 for r in lead_order}
        staging_wait={r:0 for r in names}
        for index,r in enumerate(tail_order):
            unplanned=tail_order[index+1:]
            local=field.with_static_robots(list(standby)+[(starts[n][0],starts[n][1],config.robot_radius) for n in unplanned])
            reservations=tuple(paths.values())
            error=_blocked_reason(local,reservations,starts[r],starts[r],t0,release,config.robot_radius)
            if error:
                error='initial_wait: '+error
                break
            yaw=headings.get(r) if headings else None
            initial_tick=round((release-t0)/config.time_step)
            if location=='start':
                moved,error,rotation=_movement(local,config,starts[r],goals[r],release,t0,yaw,reservations,cancelled,safe_goals,heading_aware,hold)
            else:
                # 통로가 해제된 뒤 접근 가능한 전방 대기점을 경유한다.
                # 후발이 t0에 먼저 움직이는 방식으로 초기 대기를 우회하지 않는다.
                length=math.dist(starts[r],goals[r])
                ratio=min(.5,policy.staging_distance_m/max(length,1e-9))
                stage=tuple(a+(b-a)*ratio for a,b in zip(starts[r],goals[r]))
                first,error,rotation=_movement(local,config,starts[r],stage,release,t0,yaw,reservations,cancelled,False,heading_aware)
                if error: break
                ta=first.points[-1].t;tb=ta+extra_tick*config.time_step
                error=_blocked_reason(local,reservations,stage,stage,ta,tb,config.robot_radius)
                if error:
                    error='staging_wait: '+error
                    break
                last,error,more_turns=_movement(local,config,stage,goals[r],tb,t0,
                    _last_heading(first,yaw),reservations,cancelled,safe_goals,heading_aware,hold)
                if error: break
                points=first.points+(TimedPoint(*stage,tb),)+last.points[1:]
                moved=TimedPath(points,first.cost_ticks+extra_tick+last.cost_ticks)
                rotation+=more_turns;staging_wait[r]=extra_tick
            if error: break
            prefix=(TimedPoint(*starts[r],t0),) if release>t0 else ()
            paths[r]=TimedPath(prefix+moved.points,initial_tick+moved.cost_ticks)
            initial_wait[r]=initial_tick;turns[r]=rotation
            if paths[r].points[-1].t>t0+config.horizon:
                error='common_horizon_exhausted'
                break
        if error:
            trial['failure']=error
            continue
        if any(p.points[-1].t>t0+config.horizon for p in paths.values()):
            trial['failure']='common_horizon_exhausted'
            continue
        end=max(p.points[-1].t for p in paths.values())+hold
        if joint_conflicts(paths,config.robot_radius,end):
            trial['failure']='joint_robot_conflict'
            continue
        local=field.with_static_robots(standby)
        bad_hold=next((r for r,p in paths.items() if local.blocked_segment((p.points[-1].x,p.points[-1].y),
            (p.points[-1].x,p.points[-1].y),p.points[-1].t,end,config.robot_radius)),None)
        if bad_hold is not None:
            trial['failure']='arrival_observation_hold_forbidden'
            p=paths[bad_hold].points[-1]
            trial['forbidden_hold']={'robot':bad_hold,'xy':[p.x,p.y],'from_sim':p.t,'to_sim':end}
            continue
        diagnostics.update(lead_robots=list(lead_order),follow_robots=list(tail_order),
            lead_clear_times=clears,release_times={r:(release if r in tails else t0) for r in names},
            initial_wait_ticks_by_robot=initial_wait,staging_wait_ticks_by_robot=staging_wait,
            turn_ticks_by_robot=turns,selected_attempt=len(attempts)-1,
            wait_location=location,optimization_scope='유한 선발·시차·대기 위치 탐색의 첫 유효 해; 최적성 보장 없음')
        return paths
    raise JointPlanningFailure('선발 조합·출발 시차·대기 위치 후보에서 안전한 공동 계획을 찾지 못했습니다.')
