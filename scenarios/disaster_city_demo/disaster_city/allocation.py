"""Shared feasible team actions for the known-task baseline and PPO.

PPO selects task, dispatched crew size and nearest/energy-rich recruitment mode.
Team selection is atomic; navigation remains the robot executor's responsibility.
"""
import math
import numpy as np


class TeamActions:
    def __init__(self, grid, cfg):
        self.grid,self.cfg=grid,cfg
        self.n=len(cfg['starts']);self.max_tasks=len(cfg['tasks']);self.max_crew=6
        self.release_offset=1+self.max_tasks*self.max_crew*2+self.n
        self.size=self.release_offset+(self.max_tasks if cfg.get('enable_team_release') else 0)

    def candidates(self, obs):
        result={0:[]};now=obs['time_s'];charger=obs['stations'][0]['xy']
        robots=obs['robots'];speed=self.cfg['speed_mps'];rate=self.cfg['battery_rates']
        available=[r for r in robots if r['role'] in ('idle','survey') and r['ready'] and r['cooldown']<=now]
        def blocked(r,p):
            return any(b['until']>now and math.dist(b['xy'],p)<.4 for b in r.get('blocked_targets',[]))
        for jid,j in obs['tasks'].items():
            if j['state']=='done' or j.get('retry_after_s',0.)>now:continue
            existing=[r for r in robots if r['role']=='work' and r['task']==jid]
            free=[i for i in range(j['maximum']) if i not in {r['slot'] for r in existing}]
            for mode in range(2):
                pool=list(available);slots=list(free);proposals=[]
                for count in range(1,min(len(free),self.max_crew,len(pool))+1):
                    choices=[]
                    for r in pool:
                        for slot in slots:
                            p=j['work_positions'][slot]
                            if blocked(r,p):continue
                            go=self.grid.distance(r['xy'],p)/speed;back=self.grid.distance(p,charger)/speed
                            remain=max(0.,j['seconds']-j['progress_s'])
                            # Conservative allowance for growth during approach and charger queue.
                            work=remain+(go*self.cfg.get('fire_growth_per_s',0.) if j['kind']=='fire' else 0.)
                            need=(go+back)*rate['move']+work*rate['work']+self.cfg['reserve']+5.
                            if math.isfinite(need) and r['battery']>=need:
                                score=go if mode==0 else go-2*r['battery']
                                choices.append((score,r['id'],slot,r,p))
                    if not choices:break
                    _,_,slot,r,p=min(choices,key=lambda c:c[:3]);pool.remove(r);slots.remove(slot)
                    proposals.append((r['id'],'work',p,jid,slot))
                    if len(existing)+count>=j['minimum']:
                        action=1+int(jid)*12+(count-1)*2+mode
                        result[action]=list(proposals)
        for r in robots:
            if r['role'] not in ('idle','survey') or not r['ready'] or r['cooldown']>now:continue
            if r['battery']<self.cfg.get('charge_request_below',80.) and math.isfinite(self.grid.distance(r['xy'],charger)):
                result[1+self.max_tasks*12+r['id']]=[(r['id'],'charge_request',None,None,None)]
        if self.cfg.get('enable_team_release'):
            for jid,j in obs['tasks'].items():
                since=j.get('waiting_since')
                if since is None or now-since<self.cfg.get('team_wait_release_s',30.):continue
                if j['state'] in ('working','done'):continue
                if self.quorum_eta(obs,jid)<=self.cfg.get('team_arrival_grace_s',15.):continue
                if not any(k!=jid and t['state']!='done' for k,t in obs['tasks'].items()):continue
                result[self.release_offset+int(jid)]=[(-1,'release_team',None,jid,None)]
        return result

    def quorum_eta(self,obs,jid):
        """Earliest feasible quorum, including idle and soon-finishing workers.

        Current work finishes at its current measured crew rate. A worker on
        another pending team is not borrowed. Distinct robots occupy distinct
        slots; already assigned slots remain reserved.
        """
        j=obs['tasks'][jid]
        now=obs['time_s'];speed=self.cfg['speed_mps'];rates=self.cfg['battery_rates']
        crew=[r for r in obs['robots'] if r['role']=='work' and r['task']==jid]
        reserved={r['slot'] for r in crew}
        edges={}
        for r in obs['robots']:
            if not r['ready'] or r['cooldown']>now:continue
            finish=0.;own=r['role']=='work' and r['task']==jid
            if own:
                slots=[r['slot']]
            elif r['role'] in ('idle','survey'):
                slots=[s for s in range(j['maximum']) if s not in reserved]
            elif r['role']=='work' and r['state']=='working':
                other=obs['tasks'].get(r['task'])
                if not other or other['state']!='working' or other['working']<other['minimum']:continue
                finish=max(0.,other['seconds']-other['progress_s'])/(1.+.5*(other['working']-other['minimum']))
                slots=[s for s in range(j['maximum']) if s not in reserved]
            else:continue
            options={}
            for slot in slots:
                p=j['work_positions'][slot]
                if any(b['until']>now and math.dist(b['xy'],p)<.4 for b in r.get('blocked_targets',[])):continue
                go=0. if own and r['state'] in ('gathering','working') else self.grid.distance(r['xy'],p)/speed
                eta=finish+go
                back=self.grid.distance(p,obs['stations'][0]['xy'])/speed
                remain=max(0.,j['seconds']-j['progress_s'])
                work=remain+(eta*self.cfg.get('fire_growth_per_s',0.) if j['kind']=='fire' else 0.)
                need=(finish+work)*rates['work']+(go+back)*rates['move']+self.cfg['reserve']+5.
                if math.isfinite(need) and r['battery']>=need:options[slot]=eta
            if options:edges[r['id']]=options
        def enough(limit):
            matched={}
            def place(robot,seen):
                for slot,eta in edges[robot].items():
                    if eta>limit or slot in seen:continue
                    seen.add(slot)
                    if slot not in matched or place(matched[slot],seen):
                        matched[slot]=robot
                        return True
                return False
            return sum(place(robot,set()) for robot in edges)>=j['minimum']
        times=sorted({eta for options in edges.values() for eta in options.values()})
        lo,hi=0,len(times)
        while lo<hi:
            mid=(lo+hi)//2
            if enough(times[mid]):hi=mid
            else:lo=mid+1
        return times[lo] if lo<len(times) else math.inf

    def mask(self, obs):
        m=np.zeros(self.size,dtype=bool);m[list(self.candidates(obs))]=True;return m

    def safety(self, obs):
        """Common low-battery guard, used by both policies."""
        charger=obs['stations'][0]['xy'];proposals=[]
        for r in obs['robots']:
            if r['role'] in ('charge','queue'):continue
            need=self.grid.distance(r['xy'],charger)/self.cfg['speed_mps']*self.cfg['battery_rates']['move']+self.cfg['reserve']
            if r['battery']<max(15.,need):proposals.append((r['id'],'charge_request',None,None,None))
        return proposals


class KnownScheduler:
    """Earliest minimum-team arrival, with no task-type priority."""
    policy_name='nearest_feasible_quorum_v1'
    def __init__(self,grid,cfg):self.actions=TeamActions(grid,cfg)

    def choose(self,obs):
        options=self.actions.candidates(obs)
        ranked=[]
        robots={r['id']:r for r in obs['robots']}
        for jid,j in obs['tasks'].items():
            possible=[a for a in options if 1<=a<1+self.actions.max_tasks*12 and (a-1)//12==int(jid)]
            # Baseline dispatches the minimum required once, without extra crews.
            existing=[r for r in obs['robots'] if r['role']=='work' and r['task']==jid]
            if len(existing)>=j['minimum']:continue
            if not possible:continue
            action=min(possible)  # Minimum feasible crew, nearest recruitment mode.
            arrivals=[0. if r['state'] in ('gathering','working') else
                      self.actions.grid.distance(r['xy'],r['target'])/self.actions.cfg['speed_mps']
                      for r in existing]
            arrivals.extend(self.actions.grid.distance(robots[i]['xy'],target)/self.actions.cfg['speed_mps']
                            for i,_,target,_,_ in options[action])
            ranked.append((sorted(arrivals)[j['minimum']-1],int(jid),action))
        if ranked:return min(ranked)[2]
        releases=[a for a in options if a>=self.actions.release_offset]
        if releases:return min(releases)
        idle=[r for r in obs['robots'] if r['role']=='idle' and r['battery']<40.]
        if idle and any(j['state']!='done' for j in obs['tasks'].values()):
            for r in sorted(idle,key=lambda r:r['battery']):
                a=1+self.actions.max_tasks*12+r['id']
                if a in options:return a
        return 0

    def decide(self,obs):
        safe=self.actions.safety(obs)
        if safe:return safe
        return self.actions.candidates(obs).get(self.choose(obs),[])


def apply_proposals(mission, proposals):
    for i,role,target,task,slot in proposals:
        if role=='release_team':mission.release_team(task)
        elif role=='charge_request':mission.charge(i)
        else:mission.assign(i,role,target,task,slot)
