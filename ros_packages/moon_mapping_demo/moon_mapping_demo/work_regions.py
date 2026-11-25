"""공유 관측 지도의 frontier를 분리해 지속적인 로봇별 작업 구역을 만든다."""
from itertools import combinations
import math

import numpy as np

from .hungarian import solve, InfeasibleAssignment
from .risk import RiskPolicy


def directed_path(grid, start, goal, distances):
    """같은 BFS 거리의 경로 중 목표 방향에 가까운 경로를 고른다.

    이웃의 y/x 나열 순서가 네 로봇의 전진 방향을 결정하지 않게 한다.
    원래 거리 격자의 관측 자유 셀과 몸체 아래 시작 연결만 사용한다.
    """
    cell = grid.index(goal)
    if not math.isfinite(distances[cell]):
        return []
    route = [cell]
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    while distances[cell] > 1e-6:
        y, x = cell
        neighbors = [(a,b) for a,b in ((y-1,x),(y+1,x),(y,x-1),(y,x+1))
                     if 0 <= a < distances.shape[0] and 0 <= b < distances.shape[1]
                     and distances[a,b] < distances[cell]-1e-6]
        if not neighbors:
            return []
        def rank(p):
            px, py = grid.world(*p)
            return (distances[p], abs((px-start[0])*dy-(py-start[1])*dx), p)
        cell = min(neighbors, key=rank)
        route.append(cell)
    return [tuple(start), *[grid.world(*p) for p in reversed(route)]]


def region_candidates(observation, poses, field, radius, settings, grid, frontiers, excluded, candidate_type):
    robots = tuple(sorted(poses))
    # 몸체 아래 미관측 틈을 별도 작업 구역으로 세면 가까운 로봇이 그
    # 구역을 맡고 이동 후보는 하나도 만들지 못한다. 실제 전진 가능한
    # 경계만 공간 군집에 포함한다.
    frontiers=[g for g in frontiers if min(math.dist(g.xy,p[:2]) for p in poses.values())
               >= settings.get('minimum_goal_distance_m',.8)]
    if not frontiers:
        grid.work_regions = {}
        return []
    # 멀리 떨어진 경계부터 대표점을 뽑은 뒤 공간 군집의 medoid로 보정한다.
    # 정보 이득이 큰 한 구역에 네 작업을 배정하지 않는다.
    centers = [max(frontiers, key=lambda g:(g.gain,g.x,g.y))]
    for _ in range(min(len(robots),len(frontiers))-1):
        target = max(frontiers, key=lambda g:(min(math.dist(g.xy,c.xy) for c in centers),g.gain,g.x,g.y))
        if min(math.dist(target.xy,c.xy) for c in centers) < settings.get('region_spacing_m',6.)/3.:
            break
        centers.append(target)
    points = np.asarray([g.xy for g in frontiers])
    weights = np.asarray([max(1., min(8.,g.gain)) for g in frontiers])
    for _ in range(5):
        labels = np.argmin(np.sum((points[:,None,:]-np.asarray([c.xy for c in centers])[None,:,:])**2,axis=2),axis=1)
        updated = []
        for i,c in enumerate(centers):
            members = np.flatnonzero(labels==i)
            if not len(members):
                updated.append(c)
                continue
            mean = np.average(points[members],axis=0,weights=weights[members])
            chosen = min(members,key=lambda j:(float(np.sum((points[j]-mean)**2)),-frontiers[j].gain,j))
            updated.append(frontiers[chosen])
        if updated == centers:
            break
        centers = updated
    distances = {r:grid.distances(poses[r][:2]) for r in robots}
    previous = settings.get('previous_work_regions',{})
    best = None
    # 부족하거나 서로 단절된 경계에서는 실제 가능한 작업 수만 배정한다.
    for size in range(len(centers),0,-1):
        for indices in combinations(range(len(centers)),size):
            costs = []
            for r in robots:
                row = []
                for i in indices:
                    distance = distances[r][grid.index(centers[i].xy)]
                    old = previous.get(r,{}).get('frontier_xy')
                    continuity = math.dist(old,centers[i].xy) if old is not None else 0.
                    row.append(math.ceil((distance+settings.get('region_continuity_weight',1.5)*continuity)
                                        /settings['candidate_speed']) if math.isfinite(distance) else None)
                costs.append(row+[0]*(len(robots)-size))
            try:
                assignment = solve(costs)
            except InfeasibleAssignment:
                continue
            if best is None or assignment.total_cost < best[0]:
                best = assignment.total_cost, indices, assignment
        if best is not None:
            break
    if best is None:
        grid.work_regions = {}
        return []
    _, indices, assignment = best
    labels = np.argmin(np.sum((points[:,None,:]-np.asarray([c.xy for c in centers])[None,:,:])**2,axis=2),axis=1)
    regions, choices = {}, {}
    max_step = settings.get('target_step_m',4.)
    risk=RiskPolicy.parse(settings.get('risk'))
    followup=(risk.followup_seconds(settings['candidate_speed'])
              if risk.enabled and settings.get('progressive_goals',False) else risk.followup_wait_seconds)
    for row,r in enumerate(robots):
        column = assignment.columns[row]
        if column >= len(indices):
            continue
        i = indices[column]
        center = centers[i]
        region_id = 'work_'+r
        members = [g for j,g in enumerate(frontiers) if labels[j]==i]
        options = {}
        for g in members:
            route = directed_path(grid,poses[r][:2],g.xy,distances[r])
            point, traveled = poses[r][:2], 0.
            for target in route[1:]:
                length = math.dist(point,target)
                if traveled+length > max_step:
                    break
                point,traveled = target,traveled+length
            if math.dist(point,poses[r][:2]) < settings.get('minimum_goal_distance_m',.8):
                continue
            if field.static_blocked_segment(point,point,radius):
                continue
            if any(math.dist(point,p)<clearance for p,clearance in excluded):
                continue
            if any(other!=r and math.dist(point,p[:2])<2*radius+.1 for other,p in poses.items()):
                continue
            gain = observation.potential_gain(point)
            if gain < settings.get('minimum_potential_gain_m2',.5):
                continue
            y,x = observation.area.index(point)
            candidate = candidate_type(f'frontier_{y}_{x}_{r}',*point,
                math.atan2(g.y-point[1],g.x-point[0]),gain,'explore',region_id,r)
            options[(y,x)] = candidate
        def rank_goal(g):
            travel=math.dist(g.xy,poses[r][:2])/settings['candidate_speed']
            arrival=settings['candidate_t0']+travel
            hold=settings['observe_seconds']+3.+(followup if risk.safe_goal_windows else 0.)
            soon=(not risk.safe_goal_windows or any(not field.blocked_segment(
                g.xy,g.xy,arrival+delay,arrival+delay+hold,radius+risk.warning_position_margin_m)
                for delay in (0.,5.,10.)))
            return (not soon,-g.gain/(travel+settings['observe_seconds']+3.),math.dist(g.xy,center.xy),g.id)
        ordered = sorted(options.values(),key=rank_goal)
        choices[r] = []
        for g in ordered:
            if all(math.dist(g.xy,other.xy)>=.75 for other in choices[r]):
                choices[r].append(g)
            if len(choices[r]) >= 2:
                break
        if choices[r]:
            regions[r] = {'id':region_id,'robot':r,'frontier_xy':list(center.xy),
                          'frontier_cells':len(members),'target_xy':list(choices[r][0].xy),
                          'previous_frontier_xy':previous.get(r,{}).get('frontier_xy')}
    selected = []
    # 먼저 모든 구역의 대표 목표, 그 다음 구역 내 대체 목표를 넣는다.
    for rank in range(2):
        for r in robots:
            if len(choices.get(r,())) <= rank:
                continue
            g = choices[r][rank]
            if any(other.work_region!=g.work_region and math.dist(g.xy,other.xy)<settings.get('goal_separation_m',1.)
                   for other in selected):
                continue
            selected.append(g)
    if settings.get('allow_region_support',False):
        # 일부 로봇의 현재 위치를 임무 끝까지 유지할 수 없을 때 그 로봇의
        # 보호 이동도 같은 구역의 대안으로 둔다. 다른 구역의 탐사를 함께
        # 취소하지 않으며, 실제 이동/대기는 공동 계획이 다시 검사한다.
        hold=settings['observe_seconds']+3.+followup
        horizon=settings.get('max_path_seconds',45.)+hold
        for r,pose in sorted(poses.items()):
            if not field.blocked_segment(pose[:2],pose[:2],settings['candidate_t0'],
                                         settings['candidate_t0']+horizon,radius):
                continue
            point=None
            for distance in (.75,1.5,2.5,3.5):
                for angle in (0.,math.pi/4,-math.pi/4,math.pi/2,-math.pi/2,math.pi):
                    target=(pose[0]+distance*math.cos(pose[2]+angle),pose[1]+distance*math.sin(pose[2]+angle))
                    if any(other!=r and math.dist(target,p[:2])<2*radius+.1 for other,p in poses.items()):
                        continue
                    if any(g.work_region!='work_'+r and math.dist(target,g.xy)<settings.get('goal_separation_m',1.) for g in selected):
                        continue
                    if field.blocked_segment(target,target,settings['candidate_t0'],
                            settings['candidate_t0']+horizon,radius+risk.warning_position_margin_m):
                        continue
                    point=target
                    break
                if point is not None:
                    break
            if point is not None:
                region_id='work_'+r
                selected.append(candidate_type('support_'+r,*point,pose[2],0.,'support',region_id,r))
                regions.setdefault(r,{'id':region_id,'robot':r,'frontier_xy':list(point),
                    'frontier_cells':0,'target_xy':list(point),'previous_frontier_xy':previous.get(r,{}).get('frontier_xy')})
    grid.work_regions = regions
    return selected[:settings.get('candidate_limit',8)]
