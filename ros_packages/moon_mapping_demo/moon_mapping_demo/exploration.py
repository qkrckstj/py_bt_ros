"""팀원 BT를 공동 시간 경로 실행기와 센서 관측에 연결하는 ROS 독립 실행부."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import math
from pathlib import Path
import threading
import time

from .execution import Control, control_step
from .departure import DeparturePolicy, observed_clear
from .risk import RiskPolicy, prognosis, safe_until
from .exploration_planning import (Candidate, observed_candidates, plan_batch,
                                   safety_candidates, stationary_safe,plan_escape_batch)
from .metrics import Metrics, atomic_json
from .observation import EvaluationArea, ObservationGrid
from .vendor.team_bt.exploration_bt import Status
from .vendor.team_bt.exploration_runtime import TeamBT, RobotBT


class TimedNavigation:
    """ready/start/poll/cancel 계약. 공간 목표 액션은 발행하지 않는다."""
    def __init__(self, engine, robot):
        self.engine, self.robot = engine, robot
        self.token, self.state = None, 'IDLE'

    def ready(self):
        return self.engine.inputs_ready

    def reset(self):
        self.token, self.state = None, 'IDLE'

    def start(self, goal):
        e = self.engine
        task = e.team.active.get(self.robot)
        if (not task or not e.batch or task.get('plan_id') != e.plan_id
                or goal != task['goal'] or self.robot not in e.paths):
            self.state = 'FAILED'
            return
        self.token = (e.team.session, task['task_id'], e.plan_id)
        self.state = 'NAVIGATING'
        e.started.add(self.robot)

    def poll(self):
        if self.token and self.token[2] != self.engine.plan_id:
            return 'CANCELED'
        return self.state

    def cancel(self):
        # 완료 로봇의 정지 heartbeat는 아직 이동 중인 동료를 취소하지 않는다.
        if self.state == 'NAVIGATING' and self.token and self.token[2] == self.engine.plan_id:
            self.engine.cancel_reason = self.robot + ': RobotBT navigation 취소'
        self.state = 'CANCELED' if self.token else 'IDLE'


class ObservingRobotBT(RobotBT):
    """원본 트리를 재사용하고 도착 이후의 실제 스캔을 완료 조건에 추가한다."""
    def __init__(self, engine, robot):
        self.engine, self.robot = engine, robot
        super().__init__(TimedNavigation(engine, robot), observe_seconds=engine.options['observe_seconds'])

    def execute(self):
        # 원본의 시간 조건과 추가 센서 조건을 모두 통과해야 VISITED가 된다.
        result = super().execute()
        if self.status in ('NAVIGATING','WAITING_DEPARTURE'):
            self.status = 'WAITING_DEPARTURE' if self.engine.waiting_departure(self.robot) else 'NAVIGATING'
        if self.status == 'VISITED' and not self.engine.observation.observed_since(
                self.robot, self.observing_since, self.engine.options['minimum_observation_frames']):
            self.status = 'OBSERVING'
            return Status.RUNNING
        return result


class HungarianTeamBT(TeamBT):
    def __init__(self, engine):
        self.engine = engine
        super().__init__(engine.robots, engine.send, autostart=True, return_home=False,
                         radio=None, complete_hold=engine.options['complete_hold_seconds'],
                         goal_timeout=1.e9)

    def ready(self):
        return self.engine.inputs_ready

    def can_complete(self):
        reason = self.engine.completion_reason()
        self.reason = reason or ''
        self.remaining = 'EXHAUSTED' if reason in ('fixed_goals_observed', 'coverage_target') else 'BLOCKED'
        return bool(reason)

    def plan(self):
        self.engine.allocate()
        if self.engine.batch:
            self.dispatch_tasks()
            self.phase = 'TIMED_PATH_AND_OBSERVATION'
        else:
            self.phase = self.engine.phase
        return Status.RUNNING


DEFAULTS = dict(observe_seconds=7., bootstrap_seconds=7., minimum_observation_frames=3,
                observation_angular_speed=.5, sensor_timeout_seconds=2.,
                coverage_target=.85, max_sim_seconds=180., max_rounds=4,
                stationary_safety_horizon=20., safety_lead_seconds=1.,
                planning_wall_budget_seconds=8., max_assignment_attempts=24,
                retry_seconds=3., max_planning_failures=3, stagnation_seconds=60.,
                complete_hold_seconds=1.)


class Exploration:
    def __init__(self, cfg, field, pcfg, output, mode='explore', pool=None):
        if mode not in ('fixed', 'explore'):
            raise ValueError('mode는 fixed 또는 explore여야 합니다.')
        self.cfg, self.field, self.pcfg, self.mode = cfg, field, pcfg, mode
        self.robots = tuple(cfg['robots'])
        if self.robots != ('robot1', 'robot2', 'robot3', 'robot4'):
            raise ValueError('통합 모드는 robot1~4의 전체 입력과 실행 권한이 필요합니다.')
        self.settings = dict(cfg['execution'])
        self.plan_pcfg = pcfg
        self.departure = DeparturePolicy.parse(cfg.get('departure'))
        self.risk = RiskPolicy.parse(cfg.get('risk'))
        self.options = {**DEFAULTS, **cfg.get('exploration', {})}
        self.base_hold_seconds=max(self.settings['goal_hold_seconds'],self.options['observe_seconds']+3.)
        for key in DEFAULTS:
            value = self.options[key]
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('exploration.' + key + '는 양의 유한 수여야 합니다.')
        if self.options['coverage_target'] > 1. or self.options['observation_angular_speed'] > .65:
            raise ValueError('탐사율 목표는 1 이하, 관측 회전 속도는 드라이버 상한 .65 이하여야 합니다.')
        for key in ('minimum_observation_frames', 'max_rounds', 'max_assignment_attempts', 'max_planning_failures'):
            if int(self.options[key]) != self.options[key]:
                raise ValueError('exploration.' + key + '는 정수여야 합니다.')
        for key in ('target_step_m', 'max_path_seconds'):
            if key in self.options and (type(self.options[key]) not in (int, float)
                    or not math.isfinite(self.options[key]) or self.options[key] <= 0):
                raise ValueError('exploration.' + key + '는 양의 유한 수여야 합니다.')
        if 'candidate_limit' in self.options and (type(self.options['candidate_limit']) is not int
                or not 1 <= self.options['candidate_limit'] <= 12):
            raise ValueError('exploration.candidate_limit은 1~12 정수여야 합니다.')
        if type(self.options.get('progressive_goals', False)) is not bool:
            raise ValueError('exploration.progressive_goals는 bool이어야 합니다.')
        if type(self.options.get('turn_aware_coordination', False)) is not bool:
            raise ValueError('exploration.turn_aware_coordination는 bool이어야 합니다.')
        if type(self.options.get('finish_round_after_observation', False)) is not bool:
            raise ValueError('exploration.finish_round_after_observation는 bool이어야 합니다.')
        if type(self.options.get('allow_partial_escape', False)) is not bool:
            raise ValueError('exploration.allow_partial_escape는 bool이어야 합니다.')
        if type(self.options.get('arrival_window_escape', False)) is not bool:
            raise ValueError('exploration.arrival_window_escape는 bool이어야 합니다.')
        if type(self.options.get('short_escape_hold', False)) is not bool:
            raise ValueError('exploration.short_escape_hold는 bool이어야 합니다.')
        if type(self.options.get('spatial_regions',False)) is not bool:
            raise ValueError('exploration.spatial_regions는 bool이어야 합니다.')
        for key in ('region_spacing_m','region_continuity_weight'):
            if key in self.options and (type(self.options[key]) not in (int,float)
                    or not math.isfinite(self.options[key]) or self.options[key]<=0):
                raise ValueError('exploration.'+key+'는 양의 유한 수여야 합니다.')
        margin = self.options.get('planning_result_margin_seconds', 0.)
        if (type(margin) not in (int, float) or not math.isfinite(margin)
                or not 0 <= margin < self.options['planning_wall_budget_seconds']):
            raise ValueError('계획 결과 수신 여유는 0 이상이며 실제 계산 예산보다 작아야 합니다.')
        self.area = EvaluationArea(**self.options['evaluation'])
        self.root = Path(output)
        self.pool, self.owns_pool = pool or ThreadPoolExecutor(max_workers=1), pool is None
        self.now = self.wall = 0.
        self.inputs_ready = False
        self.poses, self.paths, self.batch = {}, {}, None
        self.phase, self.reason = 'WAITING', '전체 입력 안정화 대기'
        self.plan_id, self.round_id = 0, 0
        self.future, self.cancel_event, self.future_id = None, None, None
        self.started, self.arrivals, self.cancel_reason = set(), {}, None
        self.observation = ObservationGrid(self.area, self.robots, self.options.get('sensor_range_m', 6.))
        self.metrics = None
        self.team = HungarianTeamBT(self)
        self.robot_bt = {r: ObservingRobotBT(self, r) for r in self.robots}
        self.output = self.root / self.team.session
        self.commands = []
        self.work_regions = {}
        self.runtime_inputs = {}
        self.risk_report = None
        self.last_risk_check = -math.inf
        self.planning_job = None
        self.prediction_seen=set()
        self.retry_at, self.failures, self.last_step, self.last_gain = 0., 0, None, 0.
        self.last_count, self.dispatch_count = 0, dict.fromkeys(self.robots, 0)

    @property
    def active(self):
        return self.metrics is not None and self.metrics.termination_reason is None

    def start(self, now, wall, poses):
        if self.active:
            raise ValueError('진행 중입니다. 먼저 정지해야 합니다.')
        # 새 session으로 이전 명령/결과/계산을 확실히 격리한다.
        self.invalidate('새 session')
        self.now, self.wall, self.poses = now, wall, dict(poses)
        self.team = HungarianTeamBT(self)
        self.robot_bt = {r: ObservingRobotBT(self, r) for r in self.robots}
        self.observation = ObservationGrid(self.area, self.robots, self.options.get('sensor_range_m', 6.))
        self.output = self.root / self.team.session
        self.metrics = Metrics(self.robots, self.area, now, wall, {
            'mode': self.mode, 'upstream_commit': '02cda62197e30858f295de25567003f02242ad64',
            'sensor_topic': 'scan_observation', 'pose_source': 'acquisition-time TF from anchored wheel/IMU odometry',
            'optimization_scope': '선정된 로봇·목표 부분집합에서 정수 시간 비용 합의 Hungarian 최적 배정',
            'exploration_options': self.options})
        self.metrics.metadata['body_radius_m'] = self.pcfg.robot_radius
        self.metrics.metadata['planning_radius_m'] = self.plan_pcfg.robot_radius
        self.metrics.metadata['terrain_geometry']=getattr(self.field,'geometry_mode','synthetic')
        self.metrics.metadata['shadow_plane_height_m']=getattr(self.field,'shadow_plane_height',None)
        self.metrics.metadata['departure_policy'] = asdict(self.departure)
        self.metrics.metadata['risk_policy'] = asdict(self.risk)
        self.phase, self.reason = 'BOOTSTRAP_OBSERVING', '시작 위치의 실제 스캔 누적'
        self.bootstrap_since = now
        self.round_id, self.failures = 0, 0
        self.last_step, self.last_gain, self.last_count = now, now, 0
        self.retry_at = now
        self.safety_pending = False
        self.dispatch_count = dict.fromkeys(self.robots, 0)
        self.commands = []
        self.work_regions = {}
        self.last_risk_check,self.risk_report,self.planning_job=-math.inf,None,None
        self.prediction_seen=set()
        self.persist()

    def invalidate(self, reason):
        job=getattr(self,'planning_job',None)
        if job and job['plan_id']==self.plan_id and job['status']=='RUNNING':
            job.update(status='CANCELLED',cancel_reason=reason,
                sim_elapsed_seconds=self.now-job['submitted_sim'],wall_elapsed_seconds=self.wall-job['submitted_monotonic'])
        if getattr(self,'metrics',None):
            self.metrics.retire_departures(self.plan_id,self.now,reason)
        self.plan_id += 1
        if self.cancel_event:
            self.cancel_event.set()
        if self.future:
            self.future.cancel()
        self.future, self.future_id = None, None
        self.paths, self.batch, self.started, self.arrivals = {}, None, set(), {}
        for bt in getattr(self, 'robot_bt', {}).values():
            bt.navigation.reset()
            bt.mode = 'stop'
            if bt.task and bt.status not in TeamBT.TERMINAL:
                bt.status = 'CANCELED'
        if hasattr(self, 'team'):
            self.team.active.clear()
        self.cancel_reason = None

    def terminate(self, reason, phase='STOPPED'):
        self.invalidate(reason)
        self.phase, self.reason = phase, reason
        self.team.enabled = False
        self.team.phase, self.team.reason = phase, reason
        self.team.tree.halt()
        for bt in self.robot_bt.values():
            bt.tree.halt()
        if self.metrics:
            self.metrics.termination_reason = reason
            self.metrics.sample(self.observation, self.now, self.wall, self.phase, force=True)
            self.persist()

    def replan(self, reason):
        goals = list(self.batch['assigned'].values()) if self.batch else None
        eligible = list(self.batch['assigned']) if self.batch else None
        affected = list(self.paths) or list(self.robots)
        if self.metrics.replans >= self.settings['max_replans']:
            self.terminate('replan_limit: ' + reason, 'ERROR')
            return
        self.metrics.replan(self.now, reason, affected)
        self.invalidate(reason)
        if goals:
            safety = all(g.purpose in ('safety','support') for g in goals)
            self.queue(goals, require_all=self.mode == 'fixed' or safety, safety=safety,
                       eligible=eligible if safety else None)
        else:
            self.phase, self.retry_at = 'WAITING_FOR_TARGETS', self.now

    def send(self, robot, command):
        if command.get('session') != self.team.session:
            return False
        if command['mode'] == 'goal' and command.get('plan_id') != self.plan_id:
            return False
        accepted = self.robot_bt[robot].receive(command, self.now)
        if accepted:
            self.commands.append({'robot': robot, 'sim_time': self.now, **command})
            self.commands = self.commands[-100:]
        return accepted

    def accept_result(self, robot, state, plan_id):
        task = self.team.active.get(robot)
        if (plan_id != self.plan_id or not task or state.get('session') != self.team.session
                or state.get('task_id') != task['task_id']):
            return False
        self.team.update_robot(robot, state, self.now)
        return True

    def completion_reason(self):
        if self.batch or self.future or self.phase == 'BOOTSTRAP_OBSERVING':
            return None
        if self.mode == 'fixed' and self.metrics.rounds_completed:
            return 'fixed_goals_observed'
        if self.mode == 'explore' and self.observation.coverage_fraction >= self.options['coverage_target']:
            return 'coverage_target'
        if self.metrics.rounds_completed >= self.options['max_rounds']:
            return 'round_limit'
        if self.now - self.last_gain >= self.options['stagnation_seconds']:
            return 'no_observation_progress'
        if self.failures >= self.options['max_planning_failures']:
            return 'no_safe_plan_within_search_limits'
        return None

    def queue(self, candidates, require_all=False, safety=False, eligible=None):
        lead = self.options['safety_lead_seconds'] if safety else self.settings['lead_seconds']
        if self.risk.enabled:
            lead=max(lead,self.risk.planning_sim_budget_seconds+self.risk.swap_seconds+self.risk.preparation_seconds)
            limits={r:safe_until(self.field,p[:2],self.now,lead,self.pcfg.robot_radius,self.pcfg.safety_time_step)
                    for r,p in self.poses.items()}
            hold_safe=stationary_safe(self.field,self.poses,self.poses,self.now,self.now+lead,self.pcfg.robot_radius)
            self.metrics.events.append({'event':'planning_stop_check','sim_time':self.now,
                'planned_t0':self.now+lead,'stop_safe_until_by_robot':limits,'safe':hold_safe})
            if not hold_safe:
                self.terminate('no_safe_planning_stop: 현재 위치에서 계산·준비를 기다릴 안전 구간 부족','ERROR')
                return
        self.invalidate('계획 교체')
        self.safety_pending = safety
        self.phase, self.reason = 'PLANNING', '공통 출발 시각의 정수 비용·배정·공동 경로 계산'
        self.cancel_event = threading.Event()
        self.future_id = self.team.session, self.plan_id
        t0 = self.now + lead
        followup=(self.risk.followup_seconds(self.pcfg.speed)
                  if self.risk.enabled and self.options.get('progressive_goals',False)
                  else self.risk.followup_wait_seconds)
        if safety and self.risk.enabled and self.options.get('short_escape_hold',False):
            followup=self.risk.escape_handoff_seconds()
        hold=self.base_hold_seconds+(followup if self.risk.safe_goal_windows and self.mode!='fixed' else 0.)
        order = sorted(eligible or self.robots, key=lambda r: (self.dispatch_count[r], r))
        if safety and self.risk.multiple_escape_candidates:
            order=sorted(order,key=lambda r:(safe_until(self.field,self.poses[r][:2],self.now,self.risk.forecast_seconds,
                                                       self.pcfg.robot_radius),self.dispatch_count[r],r))
        directory = self.output / f'round_{self.round_id:04d}' / f'plan_{self.plan_id:04d}'
        self.planning_job={'event':'planning_timing','plan_id':self.plan_id,'session':self.team.session,
            'submitted_sim':self.now,'submitted_monotonic':self.wall,'t0':t0,
            'submitted_goals':[asdict(g) for g in candidates],
            'snapshot_poses':dict(self.poses),'sim_elapsed_seconds':None,'wall_elapsed_seconds':None,'status':'RUNNING'}
        self.metrics.events.append(self.planning_job)
        worker=plan_escape_batch if safety and self.risk.multiple_escape_candidates else plan_batch
        self.future = self.pool.submit(worker, self.field, self.plan_pcfg, dict(self.poses),
                list(candidates), t0, self.now, hold, order, directory, self.cancel_event,
                {**self.options,'departure':asdict(self.departure),'risk':asdict(self.risk),
                 'round_hold_seconds':self.base_hold_seconds}, require_all)

    def finish_plan(self):
        future, token = self.future, self.future_id
        self.future, self.future_id = None, None
        if token != (self.team.session, self.plan_id) or not self.active:
            return
        if self.planning_job:
            self.planning_job.update(sim_elapsed_seconds=self.now-self.planning_job['submitted_sim'],
                wall_elapsed_seconds=self.wall-self.planning_job['submitted_monotonic'])
        try:
            value = future.result()
        except Exception as exc:
            if self.planning_job:self.planning_job['status']='PLANNING_EXCEPTION'
            self.terminate('planning_exception: ' + str(exc), 'ERROR')
            return
        if self.planning_job:
            self.planning_job['status']=value['status']
            statuses=[a.get('pair_statuses',{}) for a in value.get('attempts',[])]
            self.planning_job['start_forbidden_rows']=sum(1 for groups in statuses for row in groups.values()
                if row and all(v=='start_forbidden' for v in row.values()))
            self.planning_job['allocation_failure_attempts']=sum(a.get('assignment') is None for a in value.get('attempts',[]))
            self.planning_job['joint_failure_attempts']=sum(a.get('assignment') is not None and bool(a.get('error')) for a in value.get('attempts',[]))
            self.planning_job['candidate_search_end']=value.get('escape_search_end')
        if value['status'] != 'OK':
            self.failures += 1
            safety_failed = getattr(self, 'safety_pending', False)
            self.safety_pending = False
            self.reason = value['status'] + ': ' + value['error']
            self.phase, self.retry_at = 'WAITING_FOR_TARGETS', self.now + self.options['retry_seconds']
            atomic_json(self.output / f'plan_failure_{self.plan_id:04d}.json', value)
            if safety_failed:
                self.terminate('no_safe_evacuation: ' + self.reason, 'ERROR')
            elif (self.mode=='explore' and self.options.get('progressive_goals',False)
                  and self.risk.enabled and self.risk.multiple_escape_candidates
                  and value['status']!='CANCELLED' and not self.cancel_event.is_set()
                  and any(a.get('unsafe_standby_robot_ids') for a in value.get('attempts',[]))):
                # 탐사 경로가 있어도 미배정 기체를 회차 끝까지 세워 둘 수
                # 없다면 더 늦은 같은 요청을 반복하지 않고 보호 이동을 찾는다.
                self.request_escape('탐사 배정 중 미배정 로봇의 대기 안전 구간 부족')
            return
        self.batch, self.paths = value, value['paths']
        if self.risk.enabled:
            moved=any(math.dist(self.poses[r][:2],self.planning_job['snapshot_poses'][r][:2])>self.settings['goal_tolerance']
                      for r in self.poses)
            turned=any(abs(math.atan2(math.sin(self.poses[r][2]-self.planning_job['snapshot_poses'][r][2]),
                                     math.cos(self.poses[r][2]-self.planning_job['snapshot_poses'][r][2])))>.15 for r in self.poses)
            if moved or turned or not stationary_safe(self.field,self.poses,self.poses,self.now,max(self.now,value['t0']),self.pcfg.robot_radius):
                self.planning_job['status']='STALE_POSE_OR_STOP_WINDOW'
                self.terminate('계획 교체 직전 위치 또는 정지 안전 구간 변경','ERROR')
                return
        if self.now >= value['t0']:
            if self.planning_job:self.planning_job['status']='MISSED_T0'
            self.replan('계산 중 공통 출발 시각 경과')
            return
        self.settings['goal_hold_seconds'] = value.get('execution_hold_seconds',self.base_hold_seconds)
        self.batch['round_hold_seconds']=self.base_hold_seconds
        self.phase, self.reason, self.failures = 'EXECUTING', 'RobotBT 준비 장벽과 공동 시간 경로 실행', 0
        self.team.now = self.now
        if any(g.work_region for g in value['assigned'].values()):
            by_id={v['id']:v for v in self.work_regions.values()}
            self.work_regions={r:{**by_id[g.work_region],'robot':r,'target_xy':list(g.xy),'activity':g.purpose}
                               for r,g in value['assigned'].items() if g.work_region in by_id}
        for robot, goal in value['assigned'].items():
            self.team.new_task(robot, [goal.x, goal.y, goal.yaw], goal.purpose)
            self.team.active[robot]['plan_id'] = self.plan_id
            self.dispatch_count[robot] += 1
        self.metrics.register_departures(self.team.session,self.plan_id,self.round_id,value,
            self.poses,{r:t['task_id'] for r,t in self.team.active.items()})
        self.metrics.events.append({'event': 'dispatch', 'sim_time': self.now,
            'plan_id': self.plan_id, 'round_id': self.round_id, 't0': value['t0'],
            'assigned': {r: asdict(g) for r, g in value['assigned'].items()},
            'standby': value['standby_robot_ids'], 'costs': value['costs'],
            'assignment': asdict(value['assignment']), 'departure':value.get('departure'),
            'allocation_strategy':value.get('allocation_strategy','selected_candidates'),
            'ownership_reassigned':value.get('ownership_reassigned',False),
            'work_regions':self.work_regions,
            'cost_breakdown':value.get('cost_breakdown')})

    def waiting_departure(self, robot):
        if not self.batch or robot not in self.paths:
            return False
        time_to_move=self.batch.get('departure',{}).get('planned_translation_start_times',{}).get(robot)
        return time_to_move is not None and self.now < time_to_move

    def allocate(self):
        if self.future:
            if self.future.done():
                self.finish_plan()
            return
        if self.batch or self.phase == 'BOOTSTRAP_OBSERVING' or self.now < self.retry_at:
            return
        if self.mode == 'fixed':
            candidates = [Candidate(g['id'], g['x'], g['y'], purpose='fixed') for g in self.cfg['goals']]
        else:
            excluded = [(p, r) for p, r, until in self.team.excluded if until > self.now]
            candidates, grid = observed_candidates(self.observation, self.poses, self.field,
                self.pcfg.robot_radius,{**self.options,'risk':asdict(self.risk),
                'previous_work_regions':self.work_regions,
                'candidate_t0':self.now+self.settings['lead_seconds'],'candidate_speed':self.pcfg.speed},excluded)
            if hasattr(grid,'work_regions'):
                self.work_regions=grid.work_regions
            self.team.update_map(grid, self.now)
        if not candidates:
            self.phase, self.reason = 'WAITING_FOR_TARGETS', '도달 가능한 관측 frontier 후보 없음; 전체 탐사 완료로 간주하지 않음'
            self.retry_at = self.now + self.options['retry_seconds']
            return
        self.metrics.events.append({'event':'candidate_selection','sim_time':self.now,
            'service_runway_ranking':self.risk.safe_goal_windows,
            'strategy':('spatial_work_regions' if self.options.get('spatial_regions',False) else
                        'marginal_gain_per_travel_time' if self.options.get('progressive_goals',False) else 'service_runway'),
            'work_regions':self.work_regions,
            'goals':[asdict(g) for g in candidates]})
        self.queue(candidates, require_all=self.mode == 'fixed')

    def sensor_fresh(self):
        # /clock·TF·scan 콜백 도착 순서의 50ms 허용 범위는 입력 guard와 같다.
        return all(-.05 <= self.now - self.observation.last_valid_stamp[r] <= self.options['sensor_timeout_seconds']
                   for r in self.robots)

    def request_escape(self,reason):
        if self.metrics.replans>=self.settings['max_replans']:
            self.terminate('replan_limit: '+reason,'ERROR')
            return
        candidates=[] if self.risk.multiple_escape_candidates else safety_candidates(self.field,self.poses,self.robots,self.plan_pcfg.robot_radius,self.now,self.options)
        if not self.risk.multiple_escape_candidates and len(candidates)!=len(self.robots):
            self.terminate('다가오는 그림자의 안전 대피 목표 부족','ERROR')
            return
        self.metrics.replan(self.now,reason,self.robots)
        self.queue(candidates,require_all=True,safety=True,eligible=list(self.robots))

    def step(self, now, wall, poses, inputs_ready=True):
        self.now, self.wall, self.poses, self.inputs_ready = now, wall, dict(poses), inputs_ready
        zeros = {r: Control() for r in self.robots}
        if not self.active:
            return zeros
        dt = max(0., now - self.last_step)
        self.last_step = now
        if not inputs_ready or set(poses) != set(self.robots):
            self.terminate('전체 odom 또는 clock 갱신 중단', 'ERROR')
            return zeros
        if now - self.metrics.started_sim >= self.options['max_sim_seconds']:
            self.terminate('sim_duration_limit', 'PARTIAL_COMPLETE')
            return zeros
        if self.observation.observed_count > self.last_count:
            self.last_count, self.last_gain = self.observation.observed_count, now
        # 이동·계산·관측·대기 모두에서 현재 원판 점유의 즉시 위험을 확인한다.
        if not stationary_safe(self.field, poses, self.robots, now, now + self.settings['lookahead_seconds'], self.pcfg.robot_radius):
            self.terminate('현재 위치가 그림자·장애물 금지 영역에 진입', 'ERROR')
            return zeros
        if self.risk.enabled and self.future and self.planning_job:
            job=self.planning_job
            sim_over=now-job['submitted_sim']>self.risk.planning_sim_budget_seconds
            wall_over=wall-job['submitted_monotonic']>self.options.get('planning_wall_budget_seconds',8.) and not self.future.done()
            stop_bad=not stationary_safe(self.field,poses,self.robots,now,max(now,job['t0']),self.pcfg.robot_radius)
            if sim_over or wall_over or stop_bad:
                status='STOP_WINDOW_CHANGED' if stop_bad else 'SIM_BUDGET_EXCEEDED' if sim_over else 'WALL_BUDGET_EXCEEDED'
                job.update(status=status,sim_elapsed_seconds=now-job['submitted_sim'],
                           wall_elapsed_seconds=wall-job['submitted_monotonic'])
                self.terminate('planning_'+status.lower(),'ERROR')
                return zeros
        if self.risk.enabled and not self.future and now-self.last_risk_check>=self.risk.check_period_seconds:
            self.last_risk_check=now
            policy=self.risk
            if self.mode=='fixed' and self.paths:
                from dataclasses import replace
                end=max(p.points[-1].t for p in self.paths.values())+self.settings['goal_hold_seconds']+self.options['complete_hold_seconds']
                policy=replace(policy,forecast_seconds=max(.25,min(policy.forecast_seconds,end-now)))
            elif self.paths and self.options.get('progressive_goals', False):
                # 관측을 마친 뒤 새 계획으로 교체할 경로를 목표에서 영원히
                # 정지할 경로로 연장하지 않는다. 다음 계산·준비 시간까지
                # 검증된 hold를 예측하고 매 tick 실제 이동/정지 안전을 검사한다.
                from dataclasses import replace
                end=max(p.points[-1].t for p in self.paths.values())+self.settings['goal_hold_seconds']
                policy=replace(policy,forecast_seconds=max(.25,min(policy.forecast_seconds,end-now)))
            self.risk_report=prognosis(self.field,self.paths,poses,now,self.pcfg,policy)
            self.risk_report.update(session=self.team.session,plan_id=self.plan_id)
            for robot,row in self.risk_report['robots'].items():
                key=(self.plan_id,robot)
                if key not in self.prediction_seen and row['deadline_sim_seconds'] is not None:
                    self.prediction_seen.add(key)
                    self.metrics.events.append({'event':'risk_first_prediction','sim_time':now,'plan_id':self.plan_id,
                        'robot':robot,**row,'required_time_estimate':self.risk_report['required_time_estimate']})
            if self.risk_report['trigger'] and not getattr(self,'safety_pending',False) and not self.completion_reason():
                if (self.mode == 'explore' and self.options.get('progressive_goals', False)
                        and not self.paths and self.phase == 'WAITING_FOR_TARGETS' and now >= self.retry_at):
                    # 그림자를 피할 다음 탐사 경로가 있으면 탐사를 계속한다.
                    # 실제 제출 전 정지 안전·계산 기한·모든 경로 검사는 동일하다.
                    self.allocate()
                    if self.future:
                        return zeros
                self.metrics.events.append({'event':'early_escape_prediction',**self.risk_report})
                self.request_escape('위험 기한과 계산·준비·회전·탈출 시간 비교에 따른 이른 대피')
                return zeros
        held = [r for r in self.robots if r not in self.paths or r in self.arrivals]
        forecast = now + self.options['stationary_safety_horizon']
        if self.batch:
            # 공동 계획의 hold 이후에는 새 임무/대피를 선정한다. 현재 임무의
            # 검증된 hold 이후 위험 때문에 같은 관측을 반복 취소하지 않는다.
            forecast = min(forecast, max(p.points[-1].t for p in self.paths.values()) + self.settings['goal_hold_seconds'])
        elif self.future and self.planning_job:
            # 작업 중에는 제출 시 검증한 출발 시각까지의 정지를 보장한다.
            # 출발 이후에도 제자리에 남는다는 가정으로 계산을 취소하지 않는다.
            forecast = min(forecast, max(now, self.planning_job['t0']))
        if self.completion_reason():
            forecast = min(forecast, now + self.options['complete_hold_seconds'])
        forecast = max(now, forecast)
        if (held and not stationary_safe(self.field, poses, held, now, forecast, self.pcfg.robot_radius)
                and not getattr(self, 'safety_pending', False)):
            if (self.mode == 'explore' and self.options.get('progressive_goals', False)
                    and not self.future and not self.paths and self.phase == 'WAITING_FOR_TARGETS'
                    and now >= self.retry_at):
                self.allocate()
                if self.future:
                    return zeros
            self.request_escape('계산·관측·대기 중 접근하는 그림자 대피')
            if not self.active:return zeros
        if self.phase == 'BOOTSTRAP_OBSERVING':
            if (now - self.bootstrap_since >= self.options['bootstrap_seconds'] and self.sensor_fresh()
                    and all(self.observation.valid_frames[r] >= self.options['minimum_observation_frames'] for r in self.robots)):
                self.phase = 'WAITING_FOR_TARGETS'
            elif now - self.bootstrap_since > self.options['bootstrap_seconds'] + self.options['sensor_timeout_seconds']:
                self.terminate('시작 위치 센서 관측 입력 부족', 'ERROR')
                return zeros
        elif not self.sensor_fresh():
            self.terminate('실제 센서 관측 갱신 중단', 'ERROR')
            return zeros
        for r, bt in self.robot_bt.items():
            state = {**bt.snapshot(), 'position': list(poses[r][:2]), 'yaw': poses[r][2], 'ready': inputs_ready}
            if not self.accept_result(r, state, self.plan_id):
                self.team.states[r] = state, now
        self.team.tick(now)
        if self.team.finished:
            reason = self.completion_reason() or self.team.reason or 'BT 종료'
            self.terminate(reason, self.team.phase)
            return zeros
        for bt in self.robot_bt.values():
            bt.tick(now)
        if self.cancel_reason:
            self.replan(self.cancel_reason)
            return zeros
        controls = dict(zeros)
        if self.batch:
            failures = [r for r in self.paths if self.robot_bt[r].status in ('FAILED', 'CANCELED', 'REJECTED')]
            if failures:
                self.replan('RobotBT 준비·실행 실패: ' + ','.join(failures))
                return zeros
            if self.started != set(self.paths) and now >= self.batch['t0']:
                self.replan('공동 출발 시각까지 전체 RobotBT 준비 미완료')
                return zeros
        if self.batch and self.started == set(self.paths):
            schedule=self.batch.get('departure',{})
            followers=schedule.get('follow_robots',[])
            due=any(now>=schedule['release_times'][r] and
                    self.metrics.active_departures.get(r,{}).get('first_linear_command_sim_seconds') is None for r in followers)
            if due and any(not observed_clear(self.paths[r],poses[r][:2],self.departure,self.pcfg.robot_radius)
                           and not (self.departure.release_region is None and self.robot_bt[r].navigation.poll()=='SUCCEEDED')
                           for r in schedule.get('lead_robots',[])):
                self.replan('후발 출발 시각에 선발의 실제 통로 해제 미확인')
                return zeros
            controls_active, reason = control_step(self.field, self.paths, poses, now, self.settings,
                                                   min(.22, self.pcfg.speed * 1.2), self.pcfg.robot_radius)
            if reason:
                self.replan(reason)
                return zeros
            controls.update(controls_active)
            for r, control in controls_active.items():
                if control.arrived:
                    self.arrivals.setdefault(r, now)
                    self.robot_bt[r].navigation.state = 'SUCCEEDED'
            end = max(p.points[-1].t for p in self.paths.values()) + self.batch.get('round_hold_seconds',self.settings['goal_hold_seconds'])
            # VISITED는 실제 도착·관측 시간·최소 스캔 수를 모두 통과한다.
            # 적극 탐사는 추가 정지 시간을 채우지 않고 다음 계획으로 넘긴다.
            # 다음 제출의 정지 안전 검사와 공동 경로 검사는 그대로 적용한다.
            observed_handoff = (self.mode == 'explore' and
                                self.options.get('finish_round_after_observation', False))
            if all(self.robot_bt[r].status == 'VISITED' for r in self.paths) and (observed_handoff or now >= end):
                safety = all(g.purpose in ('safety','support') for g in self.batch['assigned'].values())
                self.metrics.events.append({'event': 'round_complete', 'sim_time': now,
                    'plan_id': self.plan_id, 'round_id': self.round_id, 'arrivals': dict(self.arrivals), 'safety': safety})
                if not safety:
                    self.metrics.rounds_completed += 1
                    self.round_id += 1
                self.invalidate('회차 종료')
                self.safety_pending = False
                self.phase, self.retry_at = 'WAITING_FOR_TARGETS', now
                controls = zeros
        reasons = {}
        for r in self.robots:
            observing = self.phase == 'BOOTSTRAP_OBSERVING' or self.robot_bt[r].status == 'OBSERVING'
            if observing:
                controls[r] = Control(angular=self.options['observation_angular_speed'])
                reasons[r] = 'observing'
            elif controls[r].linear == 0:
                release=self.batch.get('departure',{}).get('release_times',{}).get(r,now) if self.batch else now
                reasons[r] = ('departure_wait' if now<release and self.batch else
                              'turning' if abs(controls[r].angular) > .01 else self.phase.lower())
            self.metrics.note_command(r,now,controls[r].linear)
        self.metrics.waiting(dt, reasons)
        self.metrics.sample(self.observation, now, wall, self.phase)
        return controls

    def snapshot(self):
        return {'schema_version': 'limo-bt-exploration/v1', 'phase': self.phase, 'reason': self.reason,
                'session': self.team.session, 'plan_id': self.plan_id, 'round_id': self.round_id,
                'sim_time': self.now, 'team_bt': self.team.snapshot(),
                'robots': {r: {**bt.snapshot(), 'navigation': bt.navigation.poll(),
                               'plan_id': self.plan_id, 'position': list(self.poses[r][:2]) if r in self.poses else None,
                               'yaw': self.poses[r][2] if r in self.poses else None,
                               'departure':self.metrics.active_departures.get(r) if self.metrics else None,
                               'ready': self.inputs_ready} for r, bt in self.robot_bt.items()},
                'observation': self.observation.snapshot(),
                'replans': self.metrics.replans if self.metrics else 0,
                'runtime_inputs': self.runtime_inputs,
                'work_regions': self.work_regions,
                'risk_prediction':self.risk_report,
                'departure_policy': self.departure.mode,
                'departure': self.metrics.active_departures if self.metrics else {},
                'termination_reason': self.metrics.termination_reason if self.metrics else None}

    def persist(self, plot=False):
        if self.metrics:
            atomic_json(self.output / 'runtime.json', self.snapshot())
            self.metrics.save(self.output, self.observation, plot=plot)
            self.observation.save(self.output)
            atomic_json(self.root / 'latest.json', {'session': self.team.session, 'directory': str(self.output),
                                                   'phase': self.phase, 'reason': self.reason})

    def close(self):
        if self.active:
            self.terminate('shutdown')
        self.persist(plot=True)
        if self.owns_pool:
            self.pool.shutdown(wait=False, cancel_futures=True)
