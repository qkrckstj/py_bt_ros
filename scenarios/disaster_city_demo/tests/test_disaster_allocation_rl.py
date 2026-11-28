import copy,json,math
from pathlib import Path
import pytest
from disaster_city.engine import Mission
from disaster_city.grid import CityGrid
from disaster_city.allocation import TeamActions,KnownScheduler,apply_proposals

PROFILE=Path(__file__).resolve().parents[1]/'disaster_city/profiles/junyong_10_known'
@pytest.fixture
def setup():
    cfg=json.loads((PROFILE/'scenario.json').read_text());g=CityGrid(PROFILE/'city.pgm')
    m=Mission(cfg,g);m.phase='RUNNING'
    for r in m.robots:r['ready']=True
    return cfg,g,m

def test_known_no_exploration_and_ten_ready(setup):
    cfg,g,m=setup
    assert len(m.observe()['tasks'])==7 and len(m.robots)==10 and cfg['survey']==[]
    assert all(math.isfinite(g.distance(p,cfg['charge_xy'])) for p in cfg['starts'])

def test_fire_grows_before_suppression_and_after_interruption(setup):
    cfg,g,m=setup;j=m.jobs['0'];base=j['seconds'];m.advance(60)
    assert j['seconds']==pytest.approx(base+6)
    for i in range(j['minimum']):m.assign(i,'work',j['work_positions'][i],'0',i);m.pose(i,j['work_positions'][i],0,0)
    m.advance(1);assert j['seconds']==pytest.approx(base+6) and j['progress_s']==1
    m.assign(0,'idle');m.advance(10)
    assert j['state']=='paused' and j['seconds']==pytest.approx(base+7) and j['progress_s']==1

def test_teams_atomic_and_slots_unique(setup):
    cfg,g,m=setup;actions=TeamActions(g,cfg)
    for a,ps in actions.candidates(m.observe()).items():
        if not ps or ps[0][1]!='work':continue
        j=m.jobs[ps[0][3]]
        assert j['minimum']<=len(ps)<=j['maximum']
        assert len({p[0] for p in ps})==len(ps)==len({p[4] for p in ps})

def test_low_battery_team_excluded_and_nearest_quorum(setup):
    cfg,g,m=setup;p=KnownScheduler(g,cfg);ps=p.decide(m.observe())
    assert ps and all(x[3]=='3' for x in ps)
    for r in m.robots:r['battery']=1.
    assert not any(ps and ps[0][1]=='work' for ps in p.actions.candidates(m.observe()).values())

def test_lightweight_determinism_and_masks():
    pytest.importorskip('gymnasium')
    from disaster_city.rl.env import AllocationEnv
    a=AllocationEnv(randomize=True);b=AllocationEnv(randomize=True)
    oa,_=a.reset(seed=123);ob,_=b.reset(seed=123)
    import numpy as np
    np.testing.assert_array_equal(oa,ob)
    for _ in range(5):
        action=KnownScheduler(a.grid,a.cfg).choose(a.mission.observe())
        assert a.action_masks()[action]
        oa,ra,_,_,ia=a.step(action);ob,rb,_,_,ib=b.step(action)
        np.testing.assert_array_equal(oa,ob);assert ra==rb and ia==ib
    assert a.observation_space.contains(oa)

def test_completion_time_and_fire_shared_engine(setup):
    cfg,g,m=setup
    for j in m.jobs.values():j['state']='done';j['progress_s']=j['seconds']
    m.advance(1);assert m.phase=='COMPLETE'

def test_deadline_is_terminal_failure():
    pytest.importorskip('gymnasium')
    from disaster_city.rl.env import AllocationEnv
    e=AllocationEnv(horizon=1);e.reset(seed=0)
    _,reward,terminated,truncated,info=e.step(0)
    assert terminated and not truncated and info['phase']=='TIME_LIMIT'
    assert reward<0


def test_charging_does_not_dissolve_a_committed_team(setup):
    cfg,g,m=setup;actions=TeamActions(g,cfg)
    ps=KnownScheduler(g,cfg).decide(m.observe());apply_proposals(m,ps)
    for i,*_ in ps:
        assert 1+actions.max_tasks*12+i not in actions.candidates(m.observe())


def test_waiting_team_can_release_and_do_another_task(setup):
    cfg,g,m=setup
    cfg.update(enable_team_release=True,team_wait_release_s=30,team_arrival_grace_s=15,team_retry_delay_s=30)
    actions=TeamActions(g,cfg);j=m.jobs['0'];j['progress_s']=4.;j['state']='paused'
    for i in range(2):m.assign(i,'work',j['work_positions'][i],'0',i)
    m.pose(0,j['work_positions'][0],0,0)
    m.advance(1);m.advance(31)
    release=actions.release_offset
    assert release in actions.candidates(m.observe())
    versions=[m.robots[i]['version'] for i in (0,1)]
    apply_proposals(m,actions.candidates(m.observe())[release])
    assert j['progress_s']==4 and j['retry_after_s']>m.time
    assert all(m.robots[i]['role']=='idle' and m.robots[i]['version']>versions[i] for i in (0,1))
    options=actions.candidates(m.observe())
    assert not any(p and p[0][1]=='work' and p[0][3]=='0' for p in options.values())
    assert any(p and p[0][1]=='work' and p[0][3]!='0' for p in options.values())
    for r in m.robots[2:]:r['ready']=False
    alternate=next(p for p in actions.candidates(m.observe()).values() if p and p[0][1]=='work' and p[0][3]=='3')
    apply_proposals(m,alternate)
    assert any(m.robots[i]['role']=='work' and m.robots[i]['task']=='3' for i in (0,1))


def test_waiting_release_respects_imminent_arrivals_and_working(setup):
    cfg,g,m=setup;cfg.update(enable_team_release=True,team_wait_release_s=30,team_arrival_grace_s=15)
    actions=TeamActions(g,cfg);j=m.jobs['0']
    for i in range(2):m.assign(i,'work',j['work_positions'][i],'0',i)
    m.pose(0,j['work_positions'][0],0,0)
    m.advance(1);m.advance(31)
    target=j['work_positions'][1];m.pose(1,[target[0]+.5,target[1]],0,.2)
    assert actions.release_offset not in actions.candidates(m.observe())
    m.pose(1,target,0,0);m.advance(.1)
    assert j['state']=='working'
    m.release_team('0')
    assert m.robots[0]['role']=='work'


def test_damage_profile_observation_and_randomization():
    pytest.importorskip('gymnasium')
    from disaster_city.rl.env import AllocationEnv
    e=AllocationEnv(PROFILE.parent/'junyong_10_damage',randomize=True)
    obs,_=e.reset(seed=42)
    assert e.action_space.n==102 and len(obs)==271
    assert .3<=e.cfg['fire_growth_per_s']<=.9
    assert e.observation_space.contains(obs)


def test_release_counts_idle_and_finishing_workers_by_total_time(setup):
    cfg,g,m=setup
    cfg.update(enable_team_release=True,team_wait_release_s=30,team_arrival_grace_s=15)
    class OpenGrid:
        def distance(self,a,b):return math.dist(a,b)
    actions=TeamActions(OpenGrid(),cfg)
    m.time=40
    j=m.jobs['0'];j.update(minimum=2,maximum=3,work_positions=[[0,0],[0,0],[0,0]],waiting_since=1)
    for r in m.robots:r.update(ready=False,battery=100)
    a,b=m.robots[:2]
    a.update(ready=True,role='work',task='0',slot=0,xy=[0,0],target=[0,0],state='gathering')
    b.update(ready=True,role='idle',xy=[3,0],state='idle')
    assert actions.quorum_eta(m.observe(),'0')==pytest.approx(15)
    assert actions.release_offset not in actions.candidates(m.observe())
    b['xy']=[3.02,0]
    assert actions.release_offset in actions.candidates(m.observe())
    # Five seconds of work plus ten seconds of travel is within the limit.
    other=m.jobs['1'];other.update(state='working',working=other['minimum'],seconds=20,progress_s=15)
    b.update(role='work',task='1',state='working',xy=[2,0])
    assert actions.quorum_eta(m.observe(),'0')==pytest.approx(15)
    assert actions.release_offset not in actions.candidates(m.observe())
    other['progress_s']=14.9
    assert actions.release_offset in actions.candidates(m.observe())
    # Additional workers shorten remaining work using the same engine rate.
    other.update(working=other['minimum']+2,progress_s=10)
    assert actions.quorum_eta(m.observe(),'0')==pytest.approx(15)
    b['battery']=1
    assert actions.release_offset in actions.candidates(m.observe())


def test_release_quorum_counts_each_robot_once_and_excludes_pending_teams(setup):
    cfg,g,m=setup
    class OpenGrid:
        def distance(self,a,b):return math.dist(a,b)
    actions=TeamActions(OpenGrid(),cfg)
    j=m.jobs['0'];j.update(minimum=3,maximum=4,work_positions=[[0,0]]*4)
    for r in m.robots:r.update(ready=False,battery=100)
    a,b=m.robots[:2]
    a.update(ready=True,role='work',task='0',slot=0,xy=[0,0],target=[0,0],state='gathering')
    b.update(ready=True,role='idle',xy=[0,0])
    assert math.isinf(actions.quorum_eta(m.observe(),'0'))
    c=m.robots[2];c.update(ready=True,role='work',task='1',state='gathering',xy=[0,0])
    assert math.isinf(actions.quorum_eta(m.observe(),'0'))
    c.update(role='idle',task=None)
    assert actions.quorum_eta(m.observe(),'0')==0


def test_stronger_fire_and_short_wait_boundary():
    cfg=json.loads((PROFILE.parent/'junyong_10_damage'/'scenario.json').read_text())
    class OpenGrid:
        def distance(self,a,b):return math.dist(a,b)
    g=OpenGrid();m=Mission(cfg,g);m.phase='RUNNING'
    j=m.jobs['0']
    assert j['seconds']==30
    m.advance(60)
    assert j['seconds']==pytest.approx(66)  # +36 work seconds per unattended minute.
    for r in m.robots:r.update(ready=False,battery=100)
    m.assign(0,'work',j['work_positions'][0],'0',0)
    m.pose(0,j['work_positions'][0],0,0)
    m.advance(1)
    actions=TeamActions(g,cfg)
    m.advance(4.99)
    assert actions.release_offset not in actions.candidates(m.observe())
    m.advance(.01)
    assert actions.release_offset in actions.candidates(m.observe())


def test_baseline_task_type_does_not_change_priority(setup):
    cfg,g,m=setup
    scheduler=KnownScheduler(g,cfg)
    before=scheduler.choose(m.observe())
    # Disable growth to isolate label priority from legitimate energy feasibility.
    cfg['fire_growth_per_s']=0.
    original=scheduler.choose(m.observe())
    m.jobs['0']['kind'],m.jobs['3']['kind']=m.jobs['3']['kind'],m.jobs['0']['kind']
    assert scheduler.choose(m.observe())==original==before
