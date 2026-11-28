"""Feasible nearest-team baseline; accepts PUBLIC state and the known map only.

A future PPO adapter must return the same (robot, role, target, task, slot) proposals.
No trained weights are supplied or silently substituted in this demo.
"""
import math


class Baseline:
    def __init__(self,grid,speed=.2,reserve=8.):
        self.grid,self.speed,self.reserve=grid,speed,reserve

    def decide(self,obs):
        robots=obs['robots']; proposals=[];busy=set();charger=obs['stations'][0]['xy']
        distance=self.grid.distance
        for r in robots:
            if r['role'] in ('charge','queue'):continue
            # Reach charger plus one full-slot wait allowance. Preserve working crew until needed.
            need=self.reserve+distance(r['xy'],charger)/self.speed*.07+3.
            if (math.isfinite(need) and r['battery']<need+3.) or r['battery']<22.:
                proposals.append((r['id'],'charge_request',None,None,None));busy.add(r['id'])
        available=[r for r in robots if r['id'] not in busy and r['role'] in ('idle','survey') and
                   r['ready'] and r['cooldown']<=obs['time_s']]
        for jid,j in sorted(obs['tasks'].items(),key=lambda p:(p[1]['state']!='paused',-p[1]['progress_s'])):
            if j['state']=='done':continue
            existing=[r for r in robots if r['role']=='work' and r['task']==jid and r['id'] not in busy]
            missing=max(0,j['minimum']-len(existing))
            if not missing:continue
            slots=[i for i in range(j['maximum']) if i not in {r['slot'] for r in existing}]
            candidates=[]
            for r in available:
                approach=min((j['work_positions'][i] for i in slots),
                             key=lambda p:distance(r['xy'],p)+distance(p,charger))
                go=distance(r['xy'],approach)/self.speed
                back=distance(approach,charger)/self.speed
                need=(go+back)*.07+(j['seconds']-j['progress_s'])*.14+self.reserve+5.
                if r['battery']>=need:candidates.append((go,r))
            # Don't strand half a crew indefinitely: recruit a feasible minimum together.
            if len(candidates)<missing:continue
            if len(obs['surveyed'])<len(obs['survey']) and len(available)-missing<1:continue
            for _,r in sorted(candidates,key=lambda x:(x[0],x[1]['id']))[:missing]:
                slot=min(slots,key=lambda i:distance(r['xy'],j['work_positions'][i]));slots.remove(slot)
                proposals.append((r['id'],'work',j['work_positions'][slot],jid,slot))
                available.remove(r);busy.add(r['id'])
        reserved=[r['target'] for r in robots if r['role']=='survey']
        sites=[p for i,p in enumerate(obs['survey']) if i not in obs['surveyed'] and p not in reserved]
        for r in available:
            if r['role']!='idle':continue
            feasible=[p for p in sites if r['battery']>=self.reserve+5.+(
                distance(r['xy'],p)+distance(p,charger))/self.speed*.07]
            if feasible:
                target=min(feasible,key=lambda p:distance(r['xy'],p));sites.remove(target)
                proposals.append((r['id'],'survey',target,None,None))
            elif r['battery']<90. and (sites or any(j['state']!='done' for j in obs['tasks'].values())):
                proposals.append((r['id'],'charge_request',None,None,None))
        return proposals
