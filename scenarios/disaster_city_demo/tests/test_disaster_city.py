import copy
import json
import unittest
from pathlib import Path
from disaster_city.engine import Mission
from disaster_city.policy import Baseline
from disaster_city.grid import CityGrid

ROOT=Path(__file__).resolve().parents[1]/'disaster_city'

class OpenGrid:
    def visible(self,a,b):return True
    def distance(self,a,b):
        import math
        return math.dist(a,b)

class CityTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((ROOT/'scenario.json').read_text())
        self.m=Mission(self.cfg,OpenGrid());self.m.phase='RUNNING'
        for r in self.m.robots:r['ready']=True

    def discover(self):
        t=self.cfg['tasks'][0]
        self.m.pose(0,t['xy'],0,0);self.m.advance(.1)
        return self.m.jobs['0']

    def test_hidden_tasks_not_in_public_state(self):
        o=self.m.observe();self.assertEqual(o['tasks'],{})
        self.assertNotIn('task_sites',o)
        self.assertNotIn('work_positions',str(o))

    def test_actual_arrivals_and_quorum(self):
        j=self.discover()
        for i in range(2):self.m.assign(i,'work',j['work_positions'][i],'0',i)
        self.m.advance(1);self.assertEqual(j['progress_s'],0)
        for i in range(2):self.m.pose(i,j['work_positions'][i],0,0)
        self.m.advance(2);self.assertEqual(j['progress_s'],2)
        self.m.pose(1,[0,-12],0,0);self.m.advance(3)
        self.assertEqual(j['progress_s'],2);self.assertEqual(j['state'],'paused')
        self.m.pose(1,j['work_positions'][1],0,0);self.m.advance(1)
        self.assertEqual(j['progress_s'],3)

    def test_extra_crew_speed_and_release(self):
        j=self.discover()
        for i in range(4):
            self.m.assign(i,'work',j['work_positions'][i],'0',i);self.m.pose(i,j['work_positions'][i],0,0)
        self.m.advance(5);self.assertEqual(j['progress_s'],10)
        self.m.advance(10);self.assertEqual(j['state'],'done')
        self.assertTrue(all(r['task'] is None for r in self.m.robots))

    def test_single_charger_and_arrival_required(self):
        self.m.charge(0);self.m.charge(1);before=self.m.robots[0]['battery']
        self.m.advance(1);self.assertLess(self.m.robots[0]['battery'],before)
        self.m.pose(0,self.cfg['charge_xy'],0,0);self.m.advance(1)
        self.assertEqual(sum(r['state']=='charging' for r in self.m.robots),1)
        self.assertEqual(self.m.charge_owner,0)

    def test_stale_navigation_result_ignored(self):
        self.m.assign(0,'survey',[0,0]);v=self.m.robots[0]['version']
        self.m.assign(0,'survey',[1,1]);self.m.failed(0,v)
        self.assertEqual(self.m.robots[0]['target'],[1,1])

    def test_gathering_consumes_idle_energy(self):
        j=self.discover();self.m.assign(0,'work',j['work_positions'][0],'0',0)
        self.m.pose(0,j['work_positions'][0],0,0);before=self.m.robots[0]['battery']
        self.m.advance(1.)
        self.assertAlmostEqual(before-self.m.robots[0]['battery'],self.cfg['battery_rates']['idle'])

    def test_public_policy_and_map_connected(self):
        grid=CityGrid(ROOT/'city.pgm');p=Baseline(grid)
        proposals=p.decide(self.m.observe())
        self.assertTrue(proposals)
        for t in self.cfg['tasks']:
            for xy in t['work_positions']:
                self.assertLess(grid.distance(self.cfg['starts'][0],xy),float('inf'))

    def test_route_cost_near_inflated_boundary(self):
        grid=CityGrid(ROOT/'city.pgm')
        self.assertLess(grid.distance([-10.5106,-10.948],[-16,-9]),float('inf'))
        self.assertEqual(grid.distance([-12,-8],[-16,-9]),float('inf'))

    def test_discovery_recruits_survey_robots_without_stopping_entire_team(self):
        self.discover()
        for r in self.m.robots:
            r['battery']=100.
            self.m.assign(r['id'],'survey',[0,0])
        proposals=Baseline(OpenGrid()).decide(self.m.observe())
        crew=[p for p in proposals if p[1]=='work']
        self.assertEqual(len(crew),2)
        self.assertLess(len(crew),len(self.m.robots))

if __name__=='__main__':unittest.main()


def test_junyong_all_endpoints_reachable():
    from pathlib import Path
    import json, math
    from disaster_city.grid import CityGrid
    profile=Path(__file__).resolve().parents[1]/'disaster_city/profiles/junyong_30m'
    cfg=json.loads((profile/'scenario.json').read_text());grid=CityGrid(profile/'city.pgm')
    endpoints=cfg['starts']+cfg['survey']+[cfg['charge_xy']]+cfg['queue_positions']
    endpoints += [p for t in cfg['tasks'] for p in t['work_positions']]
    assert all(math.isfinite(grid.distance(cfg['starts'][0],p)) for p in endpoints)
    assert len(cfg['tasks'])==7 and len(cfg['starts'])==6


def test_junyong_discovery_uses_accessible_observation_site():
    from pathlib import Path
    import json
    from disaster_city.grid import CityGrid
    from disaster_city.engine import Mission
    profile=Path(__file__).resolve().parents[1]/'disaster_city/profiles/junyong_30m'
    cfg=json.loads((profile/'scenario.json').read_text());m=Mission(cfg,CityGrid(profile/'city.pgm'))
    m.phase='RUNNING'
    for i,t in enumerate(cfg['tasks']):
        m.pose(0,t['survey_xy'],0,0);m.advance(.1)
        assert str(i) in m.jobs
