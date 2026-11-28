"""Authoritative task/energy simulation; assignment is supplied externally."""
import copy
import math


class Mission:
    def __init__(self, cfg, grid):
        self.cfg,self.grid=cfg,grid
        self.time=0.;self.phase='PREPARING';self.jobs={};self.surveyed=set()
        self.events=[];self.queue=[];self.charge_owner=None
        if cfg.get('tasks_known',False):
            self.jobs={str(i):{**copy.deepcopy(t),'progress_s':0.,'state':'pending',
                'assigned':0,'arrived':0,'working':0,'base_seconds':t['seconds'],
                'growth_s':0.} for i,t in enumerate(cfg['tasks'])}
        self.robots=[dict(id=i,name=f'robot{i+1}',xy=[float(v) for v in p],yaw=0.,speed=0.,
                          battery=cfg['initial_battery'][i],role='idle',state='idle',
                          target=None,task=None,slot=None,version=0,ready=False,nav_status='idle',
                          cooldown=0.) for i,p in enumerate(cfg['starts'])]

    def event(self,kind,**data):
        self.events.append(dict(time_s=round(self.time,2),kind=kind,**data))

    def pose(self,i,xy,yaw,speed):
        r=self.robots[i];r.update(xy=[float(v) for v in xy],yaw=float(yaw),speed=float(abs(speed)),ready=True)

    def assign(self,i,role,target=None,task=None,slot=None):
        r=self.robots[i]
        if r['role']==role and r['target']==target and r['task']==task:return
        old=r['role'];r.update(role=role,target=target,task=task,slot=slot,
                              version=r['version']+1,nav_status='pending' if target else 'idle')
        self.event('assignment',robot=i,previous=old,role=role,task=task)

    def charge(self,i):
        if i not in self.queue:self.queue.append(i)
        if self.charge_owner==i:return
        target=self.cfg.get('queue_positions', [[-16.,-11.+2.*k] for k in range(len(self.robots))])[i]
        self.assign(i,'queue',target)

    def release_team(self,jid):
        """Cancel an unproductive gathering; preserve completed work and fire growth."""
        job=self.jobs.get(jid)
        if not job or job['state'] in ('working','done'):return
        crew=[r for r in self.robots if r['role']=='work' and r['task']==jid]
        if not crew:return
        for r in crew:self.assign(r['id'],'idle')
        job.update(assigned=0,arrived=0,working=0,waiting_since=None,
                   retry_after_s=self.time+self.cfg.get('team_retry_delay_s',30.))
        self.event('team_released',task=jid,robots=[r['id'] for r in crew])

    def failed(self,i,version):
        r=self.robots[i]
        if r['version']!=version:return
        self.event('navigation_failed',robot=i,role=r['role'])
        if r['target'] is not None:
            r.setdefault('blocked_targets',[]).append(dict(xy=list(r['target']),until=self.time+120.))
        if i in self.queue:self.queue.remove(i)
        if self.charge_owner==i:self.charge_owner=None
        r['cooldown']=self.time+10.
        self.assign(i,'idle')

    def advance(self,dt):
        if self.phase!='RUNNING':return
        self.time+=dt
        # Only physical, current poses can discover sites. The planner never sees cfg.tasks.
        for i,xy in enumerate(self.cfg['survey']):
            if i not in self.surveyed and any(r['ready'] and math.dist(r['xy'],xy)<=self.cfg['discovery_radius']
                      and self.grid.visible(r['xy'],xy) for r in self.robots):
                self.surveyed.add(i);self.event('surveyed',site=i)
                for j,t in enumerate(self.cfg['tasks']):
                    if math.dist(t.get('survey_xy',t['xy']),xy)<.01:
                        self.jobs[str(j)]={**copy.deepcopy(t),'progress_s':0.,'state':'pending',
                                          'assigned':0,'arrived':0,'working':0}
                        self.event('discovered',task=str(j),kind_name=t['kind'])
        for r in self.robots:
            if r['role']=='survey' and any(i in self.surveyed and math.dist(r['target'],p)<.01
                                         for i,p in enumerate(self.cfg['survey'])):
                self.assign(r['id'],'idle')
        if self.charge_owner is None and self.queue:
            self.charge_owner=self.queue[0]
            self.assign(self.charge_owner,'charge',self.cfg['charge_xy'])
        for r in self.robots:
            at=r['target'] is not None and math.dist(r['xy'],r['target'])<.32 and r['speed']<.045
            r['state']=('gathering' if r['role']=='work' else 'queued' if r['role']=='queue' else
                        'charging' if r['role']=='charge' and self.charge_owner==r['id'] else r['role']) if at else (
                        'moving' if r['target'] is not None else 'idle')
        for jid,j in self.jobs.items():
            if j['state']=='done':continue
            assigned=[r for r in self.robots if r['task']==jid and r['role']=='work']
            arrived=[r for r in assigned if r['state']=='gathering']
            j.update(assigned=len(assigned),arrived=len(arrived),working=0)
            if len(arrived)>=j['minimum']:
                if j['state']!='working':self.event('started' if j['progress_s']==0 else 'resumed',task=jid)
                j['state']='working';j['working']=len(arrived)
                j['progress_s']=min(j['seconds'],j['progress_s']+dt*(1.+.5*(len(arrived)-j['minimum'])))
                for r in arrived:r['state']='working'
            else:
                if j['state']=='working':
                    j['state']='paused';self.event('paused',task=jid)
                if j['kind']=='fire':
                    growth=self.cfg.get('fire_growth_per_s',0.)*dt
                    j['seconds']+=growth;j['growth_s']=j.get('growth_s',0.)+growth
        if self.cfg.get('enable_team_release',False):
            for j in self.jobs.values():
                if 0<j['arrived']<j['minimum'] and j['state'] not in ('working','done'):
                    if j.get('waiting_since') is None:j['waiting_since']=self.time
                else:j['waiting_since']=None
        rate=self.cfg['battery_rates']
        for r in self.robots:
            if r['state']=='charging':
                r['battery']=min(100.,r['battery']+rate['charge']*dt)
                if r['battery']>=92.:
                    self.queue.remove(r['id']);self.charge_owner=None
                    self.assign(r['id'],'idle');self.event('charged',robot=r['id'])
            else:
                mode='work' if r['state']=='working' else 'move' if r['state']=='moving' else 'idle'
                r['battery']=max(0.,r['battery']-rate[mode]*dt)
                if r['battery']<=0:self.phase='BATTERY_FAILURE'
        for jid,j in self.jobs.items():
            if j['state']!='done' and j['progress_s']>=j['seconds']:
                j.update(state='done',assigned=0,arrived=0,working=0);self.event('completed',task=jid)
                for r in self.robots:
                    if r['task']==jid:self.assign(r['id'],'idle')
        # Completion uses observed coverage, never the private number of hidden jobs.
        if len(self.surveyed)==len(self.cfg['survey']) and all(j['state']=='done' for j in self.jobs.values()):
            self.phase='COMPLETE';self.event('mission_complete')
        elif self.time>=self.cfg['horizon_s']:self.phase='TIME_LIMIT'

    def observe(self):
        return copy.deepcopy(dict(time_s=round(self.time,2),phase=self.phase,policy=getattr(self,'policy_name','baseline'),
            localization='simulator pose + wheel odometry (not SLAM)',robots=self.robots,tasks=self.jobs,
            surveyed=sorted(self.surveyed),survey=self.cfg['survey'],
            stations=[dict(xy=self.cfg['charge_xy'],slots=1,occupants=[] if self.charge_owner is None else [self.charge_owner],queue=self.queue)],
            events=self.events[-15:]))
