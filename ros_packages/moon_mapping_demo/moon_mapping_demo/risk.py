"""그림자 접근의 유한 예측과 계산·회전·탈출을 포함한 대피 준비 예산."""
from dataclasses import asdict,dataclass
import math

from .coordination import position_at


@dataclass(frozen=True)
class RiskPolicy:
    enabled: bool = False
    safe_goal_windows: bool = False
    multiple_escape_candidates: bool = False
    heading_planner: bool = False
    forecast_seconds: float = 90.
    check_period_seconds: float = .5
    planning_sim_budget_seconds: float = 5.
    swap_seconds: float = .5
    preparation_seconds: float = 3.
    escape_distance_estimate_m: float = 3.5
    turn_rate: float = .5
    margin_seconds: float = 5.
    followup_wait_seconds: float = 12.
    warning_position_margin_m: float = .05
    escape_candidates_per_robot: int = 3
    escape_candidate_sets: int = 8

    def __post_init__(self):
        for name in ('enabled','safe_goal_windows','multiple_escape_candidates','heading_planner'):
            if type(getattr(self,name)) is not bool:raise ValueError('risk.'+name+'는 bool이어야 합니다.')
        for name in ('forecast_seconds','check_period_seconds','planning_sim_budget_seconds','swap_seconds',
                     'preparation_seconds','escape_distance_estimate_m','turn_rate','margin_seconds','followup_wait_seconds'):
            if not math.isfinite(getattr(self,name)) or getattr(self,name)<=0:
                raise ValueError('risk.'+name+'는 양의 유한 수여야 합니다.')
        if not 0<=self.warning_position_margin_m<=.18:
            raise ValueError('예측 위치 여유는 0~0.18m여야 합니다. 실제 몸체 검사 반경을 줄이지 않습니다.')
        for name in ('escape_candidates_per_robot','escape_candidate_sets'):
            if type(getattr(self,name)) is not int or not 1<=getattr(self,name)<=32:
                raise ValueError(name+'는 1~32 정수여야 합니다.')

    @classmethod
    def parse(cls,value=None):
        return value if isinstance(value,cls) else cls(**(value or {}))

    @classmethod
    def stage(cls,value,stage):
        """같은 설정으로 단계별 비교. 수치 예산·센서 조건은 보존한다."""
        if stage not in ('baseline','early','goals','candidates','heading'):
            raise ValueError('알 수 없는 위험 대응 단계: '+str(stage))
        return {**asdict(cls.parse(value)), 'enabled':stage!='baseline',
            'safe_goal_windows':stage in ('goals','candidates','heading'),
            'multiple_escape_candidates':stage in ('candidates','heading'),
            'heading_planner':stage=='heading'}

    def required_seconds(self,speed):
        parts={'planning_sim_seconds':self.planning_sim_budget_seconds,'swap_seconds':self.swap_seconds,
               'preparation_seconds':self.preparation_seconds,
               'rotation_estimate_seconds':math.pi/self.turn_rate+.5,
               'escape_move_estimate_seconds':self.escape_distance_estimate_m/speed,
               'margin_seconds':self.margin_seconds}
        return {**parts,'total_seconds':sum(parts.values())}

    def followup_seconds(self,speed):
        """다음 계산·준비·회전·탈출 예측보다 짧지 않은 후속 안전 구간."""
        return max(self.followup_wait_seconds,self.required_seconds(speed)['total_seconds'])

    def escape_handoff_seconds(self):
        """대피 후 다음 계획·준비·회전을 시작할 여유. 두 번째 탈출 거리는 제외한다."""
        return max(self.followup_wait_seconds,self.planning_sim_budget_seconds+self.swap_seconds+
                   self.preparation_seconds+math.pi/self.turn_rate+.5+self.margin_seconds)


def safe_until(field,xy,now,horizon,radius,precision=.25):
    """구간 전체 금지 검사로 정지 안전 기한의 하한을 찾는다. 무한 안전 판정 아님."""
    if field.blocked_segment(xy,xy,now,now,radius):
        return now
    end=now+horizon
    if not field.blocked_segment(xy,xy,now,end,radius):
        return end
    lo,hi=now,end
    while hi-lo>precision:
        mid=(lo+hi)/2
        if field.blocked_segment(xy,xy,now,mid,radius): hi=mid
        else: lo=mid
    return lo


def path_risk(field,path,pose,now,horizon,radius):
    """남은 이동·회전/대기·도착 후 유한 hold 중 최초 불가 구간."""
    end=now+horizon
    points=[(now,tuple(pose[:2]))]
    points.extend((p.t,(p.x,p.y)) for p in path.points if now<p.t<end)
    points.append((end,position_at(path,end)))
    for (ta,a),(tb,b) in zip(points,points[1:]):
        if field.blocked_segment(a,b,ta,tb,radius):
            lo,hi=ta,tb
            while hi-lo>.25:
                mid=(lo+hi)/2;fraction=(mid-ta)/max(tb-ta,1e-9)
                p=(a[0]+fraction*(b[0]-a[0]),a[1]+fraction*(b[1]-a[1]))
                if field.blocked_segment(a,p,ta,mid,radius): hi=mid
                else: lo=mid
            kind='movement' if math.dist(a,b)>1e-9 else (
                'observation_or_followup_hold' if ta>=path.points[-1].t else 'rotation_or_wait')
            return {'safe_until':lo,'first_forbidden_upper':hi,'kind':kind,'segment_start':ta,'segment_end':tb}
    return None


def prognosis(field,paths,poses,now,config,policy):
    required=policy.required_seconds(config.speed)
    rows={}
    for r,p in poses.items():
        stop=safe_until(field,p[:2],now,policy.forecast_seconds,config.robot_radius,config.safety_time_step)
        danger=path_risk(field,paths[r],p,now,policy.forecast_seconds,
                         config.robot_radius+policy.warning_position_margin_m) if r in paths else None
        # 예측 끝까지 안전했다는 유한 하한은 위험 발견 시각이 아니다.
        # 특히 고정 임무 종료까지 예측을 줄인 경우 조기 대피를 오발하지 않는다.
        # 검증된 이동 경로가 있으면 현재 좌표에 계속 정지한다는 가정으로
        # 대피를 발동하지 않는다. 경로의 이동·회전·도착 후 대기는 danger가 검사한다.
        deadlines=([stop] if r not in paths and stop<now+policy.forecast_seconds-1e-8 else [])
        if danger:deadlines.append(danger['safe_until'])
        deadline=min(deadlines) if deadlines else None
        rows[r]={'stop_safe_until':stop,'planned_warning':danger,'deadline_sim_seconds':deadline,
                 'seconds_to_warning':deadline-now if deadline is not None else None,
                 'prepare_required_seconds':required['total_seconds'],
                 'needs_early_escape':deadline is not None and deadline-now<=required['total_seconds']}
    return {'sim_time':now,'horizon_seconds':policy.forecast_seconds,'required_time_estimate':required,
            'robots':rows,'trigger':any(v['needs_early_escape'] for v in rows.values()),
            'warning_margin_m':policy.warning_position_margin_m,
            'scope':'보수적 유한 예측/준비 휴리스틱. 실제 경로 안전 검사를 대체하지 않음'}
