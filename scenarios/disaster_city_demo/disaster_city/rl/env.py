"""Known-map, known-task ten-robot allocation environment (Gymnasium).

Five seconds per decision, one-second motion integration along inflated-grid paths.
No Webots/ROS dependency. Excludes local collision/turning dynamics; report separately.
"""
import copy
import json
import math
from pathlib import Path
from collections import deque
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from disaster_city.engine import Mission
from disaster_city.grid import CityGrid
from disaster_city.allocation import TeamActions, apply_proposals

DEFAULT_PROFILE=Path(__file__).resolve().parents[1]/'profiles/junyong_10_known'
ROLES=('idle','survey','work','queue','charge')
KINDS=('fire','power_box','hydrant','small_debris','large_debris','fallen_tree','cordon')


def encode(obs, actions):
    """Only the same public state available to the runtime allocator."""
    cfg=actions.cfg;grid=actions.grid;speed=cfg['speed_mps'];h=cfg['horizon_s'];v=[obs['time_s']/h]
    robots=obs['robots'];charger=obs['stations'][0]['xy']
    def duration(a,b):return min(1.,grid.distance(a,b)/speed/h)
    for r in robots:
        target=r['target'] or r['xy']
        v.extend([r['xy'][0]/20,r['xy'][1]/20,r['battery']/100,ROLES.index(r['role'])/4,
                  target[0]/20,target[1]/20,duration(r['xy'],target),duration(r['xy'],charger),
                  float(r['state']=='working'),max(0.,r['cooldown']-obs['time_s'])/h])
    for i in range(actions.max_tasks):
        t=obs['tasks'].get(str(i))
        if t is None:v.extend([0.]*11);continue
        v.extend([1.,t['xy'][0]/20,t['xy'][1]/20,KINDS.index(t['kind'])/6,
                  t['minimum']/10,t['maximum']/10,t['progress_s']/max(1,t['seconds']),
                  min(2.,(t['seconds']-t['progress_s'])/300),t['assigned']/10,t['arrived']/10,
                  float(t['state']=='done')])
    for r in robots:
        for i in range(actions.max_tasks):
            t=obs['tasks'].get(str(i))
            v.append(min((duration(r['xy'],p) for p in t['work_positions']),default=1.) if t else 1.)
    v.extend([len(obs['stations'][0]['queue'])/len(robots),cfg.get('fire_growth_per_s',0.)])
    if cfg.get('enable_team_release'):
        for i in range(actions.max_tasks):
            j=obs['tasks'].get(str(i),{});since=j.get('waiting_since')
            v.extend([(obs['time_s']-since)/h if since is not None else 0.,
                      min(1.,actions.quorum_eta(obs,str(i))/h) if j else 1.,
                      max(0.,j.get('retry_after_s',0.)-obs['time_s'])/h])
    return np.asarray(v,dtype=np.float32)


class AllocationEnv(gym.Env):
    metadata={'render_modes':[]}
    def __init__(self, profile=DEFAULT_PROFILE, randomize=False, horizon=None):
        self.profile=Path(profile);self.template=json.loads((self.profile/'scenario.json').read_text())
        if not self.template.get('tasks_known'):raise ValueError('This training stage requires known tasks')
        if horizon:self.template['horizon_s']=horizon
        self.grid=CityGrid(self.profile/'city.pgm');self.randomize=randomize
        self.cfg=copy.deepcopy(self.template);self.actions=TeamActions(self.grid,self.cfg)
        self.action_space=spaces.Discrete(self.actions.size)
        m=Mission(self.cfg,self.grid)
        self.observation_space=spaces.Box(-2.,2.,shape=encode(m.observe(),self.actions).shape,dtype=np.float32)
        self.mission=None;self.routes={};self.route_versions={};self.invalid=0
        self._candidate_key=None;self._candidates={}

    def reset(self,seed=None,options=None):
        super().reset(seed=seed);self.cfg=copy.deepcopy(self.template)
        if self.randomize:
            self.cfg['initial_battery']=self.np_random.uniform(50,100,len(self.cfg['starts'])).tolist()
            self.cfg['fire_growth_per_s']=float(self.np_random.uniform(*self.cfg.get('fire_growth_range',[.05,.15])))
            for t in self.cfg['tasks']:t['seconds']*=float(self.np_random.uniform(.8,1.2))
        self.actions=TeamActions(self.grid,self.cfg);self.mission=Mission(self.cfg,self.grid)
        self.mission.phase='RUNNING'
        for r in self.mission.robots:r['ready']=True
        self.routes={};self.route_versions={};self.invalid=0;self._candidate_key=None
        return encode(self.mission.observe(),self.actions),self.info()

    def candidates(self):
        key=(self.mission.time,tuple(r['version'] for r in self.mission.robots))
        if key!=self._candidate_key:
            self._candidates=self.actions.candidates(self.mission.observe());self._candidate_key=key
        return self._candidates

    def action_masks(self):
        mask=np.zeros(self.action_space.n,dtype=bool);mask[list(self.candidates())]=True;return mask

    def move(self,dt):
        for r in self.mission.robots:
            i=r['id'];target=r['target']
            if target is None:r['speed']=0.;continue
            if self.route_versions.get(i)!=r['version']:
                self.routes[i]=deque(self.grid.path(r['xy'],target)[1:]);self.route_versions[i]=r['version']
                if not self.routes[i]:self.mission.failed(i,r['version']);continue
            remain=self.cfg['speed_mps']*dt;route=self.routes[i]
            while route and remain>1e-9:
                p=route[0];d=math.dist(r['xy'],p)
                if d<=remain:r['xy']=list(p);remain-=d;route.popleft()
                else:
                    r['xy']=[a+(b-a)*remain/d for a,b in zip(r['xy'],p)];remain=0.
            r['speed']=self.cfg['speed_mps'] if route else 0.
            r['nav_status']='navigating' if route else 'arrived'

    def step(self, action):
        if self.mission.phase!='RUNNING':raise RuntimeError('reset() required after episode end')
        before=self.mission.time;potential_before=self.potential();options=self.candidates();invalid=int(action) not in options
        if invalid:self.invalid+=1
        apply_proposals(self.mission,options.get(int(action),[]))
        apply_proposals(self.mission,self.actions.safety(self.mission.observe()))
        for _ in range(int(self.cfg.get('decision_period_s',5))):
            self.move(1.);self.mission.advance(1.)
            if self.mission.phase!='RUNNING':break
        phase=self.mission.phase
        # The observed 2400s deadline is part of this finite-horizon mission.
        terminated=phase in ('COMPLETE','BATTERY_FAILURE','TIME_LIMIT');truncated=False
        # Undiscounted successful return is -makespan/100. Failure carries a horizon penalty.
        reward=-(self.mission.time-before)/100.-.1*invalid
        if (terminated or truncated) and phase!='COMPLETE':reward-=self.cfg['horizon_s']/100.
        # Potential-based shaping with the SAME gamma as PPO preserves the discounted objective.
        potential_after=0. if terminated or truncated else self.potential()
        reward+=.999*potential_after-potential_before
        return encode(self.mission.observe(),self.actions),reward,terminated,truncated,self.info()

    def potential(self):
        return 2.*sum(j["progress_s"]/max(1.,j["seconds"]) for j in self.mission.jobs.values())

    def info(self):
        m=self.mission
        return dict(backend='lightweight_grid',phase=m.phase,time_s=m.time,
                    completed=sum(j['state']=='done' for j in m.jobs.values()),total=len(self.cfg['tasks']),
                    fire_growth_s=sum(j.get('growth_s',0.) for j in m.jobs.values()),
                    fire_completed_s=next((e['time_s'] for e in m.events if e['kind']=='completed' and m.jobs[e['task']]['kind']=='fire'),None),
                    team_releases=sum(e['kind']=='team_released' for e in m.events),
                    invalid_actions=self.invalid,robots=len(m.robots),fire_growth_per_s=self.cfg['fire_growth_per_s'])
