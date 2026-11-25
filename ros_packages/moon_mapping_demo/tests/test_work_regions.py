"""공유된 관측 공간의 구역 분산·역할 유지·안전 재배정 계약."""
from dataclasses import replace
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from moon_mapping_demo.exploration_planning import Candidate, observed_candidates, plan_batch
from moon_mapping_demo.observation import EvaluationArea, ObservationGrid
from test_exploration import Field, PCFG, POSES, config, Exploration, ImmediatePool, scans, integrate
from moon_mapping_demo.coordination import coordinate, JointPlanningFailure


class WorkRegionTests(unittest.TestCase):
    def test_shared_reservation_cache_keeps_goals_and_horizon_separate(self):
        field=Field();starts={'a':(-3.,-3.),'b':(3.,3.)};cache={}
        first={'a':(-2.,-3.),'b':(4.,3.)}
        changed={'a':(-3.,-2.),'b':(3.,4.)}
        a=coordinate(field,PCFG,starts,first,10.,shared_cache=cache)
        b=coordinate(field,PCFG,starts,changed,10.,shared_cache=cache)
        self.assertEqual((b['a'].points[-1].x,b['a'].points[-1].y),changed['a'])
        self.assertEqual(coordinate(field,PCFG,starts,first,10.,shared_cache=cache),a)
        with self.assertRaises(JointPlanningFailure):
            coordinate(field,replace(PCFG,horizon=1.),starts,first,10.,shared_cache=cache)

    def test_four_frontier_regions_spread_and_keep_owners_without_area_credit(self):
        poses={'robot1':(-5.,-5.,0.),'robot2':(-5.,-4.,0.),
               'robot3':(-4.,-5.,0.),'robot4':(-4.,-4.,0.)}
        grid=ObservationGrid(EvaluationArea(-10.,-10.,10.,10.,.25),poses,6.)
        grid.cells[8:72,8:72]=0
        settings={'spatial_regions':True,'candidate_t0':10.,'candidate_speed':.18,
                  'target_step_m':4.,'observe_seconds':4.,'region_spacing_m':6.,'candidate_limit':8}
        goals,frontier=observed_candidates(grid,poses,Field(),.26,settings)
        self.assertEqual(len(frontier.work_regions),4)
        self.assertEqual(len({g.work_region for g in goals}),4)
        anchors=[v['frontier_xy'] for v in frontier.work_regions.values()]
        self.assertGreater(min(math.dist(a,b) for i,a in enumerate(anchors) for b in anchors[i+1:]),6.)
        self.assertEqual(grid.observed_count,0)
        self.assertTrue(all(grid.cells[grid.area.index(g.xy)]==0 for g in goals))
        moved={r:(p[0]+.1,p[1]+.1,p[2]) for r,p in poses.items()}
        _,again=observed_candidates(grid,moved,Field(),.26,
            {**settings,'previous_work_regions':frontier.work_regions})
        self.assertEqual({r:v['frontier_xy'] for r,v in frontier.work_regions.items()},
                         {r:v['frontier_xy'] for r,v in again.work_regions.items()})

    def test_unreachable_owned_region_is_reassigned_across_obstacle_components(self):
        class Wall(Field):
            bounds=(-3.,3.,-3.,3.)
            def with_static_robots(self,robots):
                return Wall(robots)
            def static_blocked_segment(self,a,b,radius):
                return super().static_blocked_segment(a,b,radius) or min(a[0],b[0]) <= 0 <= max(a[0],b[0])
        poses={'robot1':(-2.,0.,0.),'robot2':(2.,0.,math.pi)}
        goals=[Candidate('right',1.,0.,work_region='east',preferred_robot='robot1'),
               Candidate('left',-1.,0.,work_region='west',preferred_robot='robot2')]
        with tempfile.TemporaryDirectory() as out:
            value=plan_batch(Wall(),replace(PCFG,horizon=12.),poses,goals,10.,1.,2.,list(poses),
                             Path(out),threading.Event(),{},True)
        self.assertEqual(value['status'],'OK',value)
        self.assertTrue(value['ownership_reassigned'])
        self.assertEqual(value['assigned']['robot1'].work_region,'west')
        self.assertEqual(value['assigned']['robot2'].work_region,'east')

    def test_peer_lidar_body_returns_do_not_disable_region_allocation(self):
        poses={'robot1':(-5.,-5.,0.),'robot2':(-5.,-4.,0.),
               'robot3':(-4.,-5.,0.),'robot4':(-4.,-4.,0.)}
        observation=ObservationGrid(EvaluationArea(-10.,-10.,10.,10.,.25),poses,6.)
        observation.cells[8:72,8:72]=0
        for p in poses.values():
            observation.cells[observation.area.index(p[:2])]=100
        original=observation.cells.copy()
        goals,grid=observed_candidates(observation,poses,Field(),.26,
            {'spatial_regions':True,'candidate_t0':10.,'candidate_speed':.18,
             'target_step_m':4.,'observe_seconds':4.,'candidate_limit':8})
        self.assertEqual(len(grid.work_regions),4)
        self.assertEqual(len({g.preferred_robot for g in goals}),4)
        self.assertTrue((observation.cells==original).all())

    def test_one_available_region_does_not_dispatch_two_robots_to_it(self):
        poses={'robot1':(-2.,-2.,0.),'robot2':(2.,2.,0.)}
        goals=[Candidate('a',-1.,-2.,work_region='one',preferred_robot='robot1'),
               Candidate('b',-1.,-1.,work_region='one',preferred_robot='robot1')]
        with tempfile.TemporaryDirectory() as out:
            value=plan_batch(Field(),PCFG,poses,goals,10.,1.,2.,list(poses),
                             Path(out),threading.Event(),{})
        self.assertEqual(value['status'],'OK',value)
        self.assertEqual(len(value['assigned']),1)

    def test_one_threatened_robot_gets_protection_option_with_other_frontiers(self):
        poses={'robot1':(-5.,-5.,0.),'robot2':(-5.,-4.,0.),
               'robot3':(-4.,-5.,0.),'robot4':(-4.,-4.,0.)}
        observation=ObservationGrid(EvaluationArea(-10.,-10.,10.,10.,.25),poses,6.)
        observation.cells[8:72,8:72]=0
        field=Field(danger=lambda a,b,ta,tb,r:a==b==poses['robot1'][:2] and tb>=20.)
        goals,_=observed_candidates(observation,poses,field,.26,
            {'spatial_regions':True,'allow_region_support':True,'candidate_t0':10.,'candidate_speed':.18,
             'target_step_m':4.,'observe_seconds':4.,'candidate_limit':12})
        support=[g for g in goals if g.purpose=='support']
        self.assertEqual({g.preferred_robot for g in support},{'robot1'})
        self.assertTrue(any(g.purpose=='explore' and g.preferred_robot!='robot1' for g in goals))
        self.assertEqual(observation.observed_count,0)

    def test_protection_only_round_does_not_count_as_completed_exploration(self):
        with tempfile.TemporaryDirectory() as out:
            engine=Exploration(config(),Field(),PCFG,out,pool=ImmediatePool())
            poses=dict(POSES);engine.start(1.,0.,poses)
            engine.queue([Candidate(r,p[0]+1.,p[1],purpose='support') for r,p in poses.items()],True)
            for i in range(700):
                now=1.+i*.1
                scans(engine,now,poses)
                controls=engine.step(now,i*.01,poses)
                poses={r:integrate(p,controls[r],.1) for r,p in poses.items()}
                rounds=[e for e in engine.metrics.events if e['event']=='round_complete']
                if rounds:
                    self.assertTrue(rounds[0]['safety'])
                    self.assertEqual(engine.metrics.rounds_completed,0)
                    break
            else:
                self.fail(engine.reason)

    def test_reassignment_timeout_keeps_verified_partial_but_cancel_discards_it(self):
        poses={'robot1':(-2.,-2.,0.),'robot2':(2.,2.,0.)}
        goals=[Candidate('a',-1.,-2.,work_region='one',preferred_robot='robot1'),
               Candidate('b',1.,2.,work_region='two',preferred_robot='robot2')]
        event=threading.Event()
        incumbent={'status':'OK','error':None,'paths':{'robot1':object()},'attempts':[]}
        with tempfile.TemporaryDirectory() as out, \
                patch('moon_mapping_demo.exploration_planning._plan_batch',
                      side_effect=[dict(incumbent),{'status':'PLANNING_BUDGET','attempts':[]}]):
            value=plan_batch(Field(),PCFG,poses,goals,10.,1.,2.,list(poses),Path(out),event,
                             {'persistent_region_ownership':True})
            self.assertEqual(value['status'],'OK')
            self.assertEqual(set(value['paths']),{'robot1'})
            self.assertEqual(value['region_search_end'],'PLANNING_BUDGET')
        def cancel(*args,**kwargs):
            event.set()
            return {'status':'CANCELLED','attempts':[]}
        count=0
        def dispatcher(*args,**kwargs):
            nonlocal count
            count+=1
            return dict(incumbent) if count==1 else cancel()
        with tempfile.TemporaryDirectory() as out, \
                patch('moon_mapping_demo.exploration_planning._plan_batch',side_effect=dispatcher):
            value=plan_batch(Field(),PCFG,poses,goals,10.,1.,2.,list(poses),Path(out),event,
                             {'persistent_region_ownership':True})
            self.assertEqual(value['status'],'CANCELLED')
            self.assertNotIn('paths',value)



if __name__=='__main__':
    unittest.main()
