"""실제 팀원 BT와 공동 실행기·센서 격자의 통합 계약 검사 (물리 주행 아님)."""
from concurrent.futures import Future
from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest

from moon_mapping_demo.coordination import moving_discs_collide
from moon_mapping_demo.exploration import Exploration
from moon_mapping_demo.exploration_planning import Candidate, observed_candidates, plan_batch
from moon_mapping_demo.metrics import Metrics, plot_metrics
from moon_mapping_demo.observation import EvaluationArea, ObservationGrid
from moon_mapping_demo.space_time import PlannerConfig, PlanningCancelled, SpaceTimePlanner


class ImmediatePool:
    def submit(self, function, *args, **kwargs):
        future = Future()
        try:
            future.set_result(function(*args, **kwargs))
        except Exception as exc:
            future.set_exception(exc)
        return future


class Field:
    bounds = (-12., 12., -12., 12.)
    def __init__(self, static=(), danger=None):
        self.static, self.danger = tuple(static), danger
    def with_static_robots(self, robots):
        return Field(robots, self.danger)
    def static_blocked_segment(self, a, b, radius):
        return any(not (-12+radius <= p[0] <= 12-radius and -12+radius <= p[1] <= 12-radius) for p in (a,b)) or any(
            moving_discs_collide(a,b,(x,y),(x,y),radius+r) for x,y,r in self.static)
    def blocked_segment(self, a, b, ta, tb, radius):
        return self.static_blocked_segment(a,b,radius) or bool(self.danger and self.danger(a,b,ta,tb,radius))


ROBOTS = ['robot1','robot2','robot3','robot4']
POSES = dict(zip(ROBOTS, [(-3.,-3.,0.),(-3.,3.,0.),(3.,-3.,0.),(3.,3.,0.)]))
PCFG = PlannerConfig(resolution=.5,time_step=1.,speed=.18,robot_radius=.26,horizon=120.,safety_time_step=.25)


def config():
    return {'robots': ROBOTS, 'goals': [dict(id=r,x=p[0]+1.,y=p[1]) for r,p in POSES.items()],
            'execution': dict(lead_seconds=3.,goal_hold_seconds=4.,position_tolerance=.18,
                goal_tolerance=.05,delay_tolerance=2.,lookahead_seconds=.3,control_period=.1,max_replans=10),
            'exploration': dict(evaluation=dict(xmin=-10.,ymin=-10.,xmax=10.,ymax=10.,resolution=.25),
                sensor_range_m=2.5,bootstrap_seconds=1.,observe_seconds=1.,minimum_observation_frames=3,
                max_sim_seconds=240.,max_rounds=2,coverage_target=.99,stagnation_seconds=150.,
                planning_wall_budget_seconds=5.,target_step_m=1.5)}


def scans(e, now, poses):
    for r,p in poses.items():
        e.observation.integrate(r,now,p,[math.inf]*72,-math.pi,2*math.pi/72,.01,10.)


def integrate(p,c,dt):
    x,y,yaw=p
    turn=c.angular*dt
    if abs(c.angular)>1e-12:
        x+=c.linear/c.angular*(math.sin(yaw+turn)-math.sin(yaw))
        y+=c.linear/c.angular*(math.cos(yaw)-math.cos(yaw+turn))
    else:
        x+=c.linear*math.cos(yaw)*dt
        y+=c.linear*math.sin(yaw)*dt
    return x,y,yaw+turn


def simulate(e, timeout=240., record=None):
    poses=dict(POSES)
    e.start(1.,0.,poses)
    for i in range(round(timeout/.1)):
        now=1.+i*.1
        if i%2==0:
            scans(e,now,poses)
        for r,p in poses.items():
            e.metrics.pose(r,now,p[:2])
        controls=e.step(now,i*.001,poses)
        nxt={r:integrate(p,controls[r],.1) for r,p in poses.items()}
        assert not any(moving_discs_collide(poses[a][:2],nxt[a][:2],poses[b][:2],nxt[b][:2],2*e.pcfg.robot_radius)
                       for j,a in enumerate(poses) for b in list(poses)[j+1:])
        if record is not None:
            record.append((now,e.phase,e.plan_id))
        poses=nxt
        if not e.active:
            return poses
    raise AssertionError(e.snapshot())


class ExplorationTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
    def engine(self,mode='fixed',pool=None):
        return Exploration(config(),Field(),PCFG,self.tmp.name,mode,pool or ImmediatePool())

    def test_four_fixed_goals_use_upstream_trees_common_time_and_observe(self):
        e=self.engine()
        poses=simulate(e)
        self.assertEqual(e.phase,'COMPLETE',e.reason)
        self.assertEqual(e.metrics.termination_reason,'fixed_goals_observed')
        dispatches=[v for v in e.metrics.events if v['event']=='dispatch']
        self.assertTrue(dispatches)
        self.assertEqual(set(dispatches[0]['assigned']),set(ROBOTS))
        self.assertEqual(len(dispatches[0]['costs']),4)
        self.assertTrue(all(len(row)==4 and all(isinstance(x,int) for x in row) for row in dispatches[0]['costs']))
        self.assertEqual({h['robot'] for h in e.team.history if h['result']=='VISITED'},set(ROBOTS))
        self.assertTrue(all(math.dist(poses[r][:2],(POSES[r][0]+1.,POSES[r][1]))<=.05 for r in ROBOTS))
        self.assertTrue(all(e.dispatch_count[r]>0 for r in ROBOTS))
        self.assertEqual(e.team.tree.name,'TeamBT')
        self.assertTrue(all(bt.tree.name=='RobotBT' for bt in e.robot_bt.values()))

    def test_sensor_candidates_repeat_and_union_metrics_are_actual_rays(self):
        e=self.engine('explore')
        simulate(e)
        self.assertEqual(e.metrics.rounds_completed,2,e.reason)
        self.assertEqual(e.phase,'PARTIAL_COMPLETE',e.reason)
        self.assertEqual(e.metrics.termination_reason,'round_limit')
        events=[v for v in e.metrics.events if v['event']=='round_complete']
        self.assertEqual(len(events),2)
        self.assertGreater(e.observation.observed_area_m2,0.)
        self.assertLess(e.observation.coverage_fraction,.99)
        self.assertAlmostEqual(sum(e.observation.snapshot()['new_area_m2_by_robot'].values()),e.observation.observed_area_m2)
        self.assertTrue(all(e.dispatch_count[r]>=2 for r in ROBOTS),e.dispatch_count)
        e.persist(plot=True)
        self.assertTrue((e.output/'coverage.png').is_file())
        self.assertTrue((e.output/'metrics.csv').is_file())
        data=json.loads((e.output/'metrics.json').read_text())
        self.assertEqual(data['fixed_denominator_m2'],400.)
        self.assertNotEqual(data['samples'][-1]['elapsed_wall_seconds'],data['samples'][-1]['elapsed_sim_seconds'])

    def test_observed_handoff_skips_padding_after_real_scans_but_preserves_fixed_mode(self):
        for mode, enabled in (('explore', False), ('explore', True), ('fixed', True)):
            with self.subTest(mode=mode, enabled=enabled):
                cfg = config()
                cfg['execution']['goal_hold_seconds'] = 12.
                cfg['exploration'].update(finish_round_after_observation=enabled, max_rounds=1)
                e = Exploration(cfg, Field(), PCFG, self.tmp.name, mode, ImmediatePool())
                observations = []
                invalidate = e.invalidate
                def capture(reason):
                    if reason == '회차 종료':
                        observations.extend((r, e.robot_bt[r].status, e.robot_bt[r].observing_since,
                            e.observation.observed_since(r, e.robot_bt[r].observing_since,
                                                         e.options['minimum_observation_frames']))
                            for r in e.paths)
                    return invalidate(reason)
                e.invalidate = capture
                simulate(e)
                event = next(v for v in e.metrics.events if v['event'] == 'round_complete')
                self.assertEqual({v[0] for v in observations}, set(event['arrivals']))
                self.assertTrue(all(v[1] == 'VISITED' and v[3] for v in observations))
                # 실제 마지막 관측이 완료되기 전에는 회차를 끝내지 않는다.
                finished = max(v[2] + e.options['observe_seconds'] for v in observations)
                self.assertGreaterEqual(event['sim_time'], finished)
                if mode == 'explore' and enabled:
                    self.assertLessEqual(event['sim_time'] - finished, .21)
                else:
                    self.assertGreater(event['sim_time'] - finished, 5.)

    def test_observed_handoff_rejects_non_boolean_option(self):
        cfg = config()
        cfg['exploration']['finish_round_after_observation'] = 'true'
        with self.assertRaises(ValueError):
            Exploration(cfg, Field(), PCFG, self.tmp.name, pool=ImmediatePool())

    def test_no_sensor_no_observation_or_arrival_credit(self):
        e=self.engine()
        e.start(1.,0.,POSES)
        e.step(1.,0.,POSES)
        e.step(5.,1.,POSES)
        self.assertEqual(e.phase,'ERROR')
        self.assertEqual(e.observation.observed_count,0)
        self.assertEqual(e.metrics.rounds_completed,0)

    def test_scan_clock_callback_jitter_matches_fusion_tolerance(self):
        e=self.engine()
        e.start(1.,0.,POSES)
        scans(e,1.03,POSES)
        e.now=1.
        self.assertTrue(e.sensor_fresh())
        e.now=.9
        self.assertFalse(e.sensor_fresh())
        e.now=4.
        self.assertFalse(e.sensor_fresh())

    def test_cancel_all_and_reject_stale_session_task_plan(self):
        e=self.engine()
        e.start(1.,0.,POSES)
        for t in (1.,2.,2.1,2.2):
            scans(e,t,POSES)
            e.step(t,t,POSES)
        self.assertTrue(e.batch)
        oldsession,oldplan=e.team.session,e.plan_id
        task=dict(e.team.active['robot1'])
        self.assertFalse(e.accept_result('robot1',dict(session=oldsession,task_id='wrong',status='VISITED'),oldplan))
        e.robot_bt['robot1'].navigation.cancel()
        scans(e,2.3,POSES)
        controls=e.step(2.3,2.3,POSES)
        self.assertGreater(e.plan_id,oldplan)
        self.assertTrue(all(c.linear==0 and c.angular==0 for c in controls.values()))
        self.assertFalse(e.accept_result('robot1',dict(session=oldsession,task_id=task['task_id'],status='VISITED'),oldplan))
        e.terminate('user_stop')
        e.start(3.,3.,POSES)
        self.assertNotEqual(e.team.session,oldsession)
        self.assertFalse(e.send('robot1',dict(mode='goal',session=oldsession,**task)))

    def test_input_guard_stops_all_four(self):
        e=self.engine()
        e.start(1.,0.,POSES)
        controls=e.step(1.,0.,POSES,False)
        self.assertEqual(e.phase,'ERROR')
        self.assertFalse(e.paths)
        self.assertEqual(set(controls),set(ROBOTS))

    def test_robot_prepare_failure_prevents_entire_cohort_departure(self):
        e=self.engine()
        e.start(1.,0.,POSES)
        for t in (1.,2.,2.1,2.2):
            scans(e,t,POSES)
            e.step(t,t,POSES)
        self.assertTrue(e.batch)
        old=e.plan_id
        e.robot_bt['robot4'].navigation.ready=lambda:False
        scans(e,2.3,POSES)
        controls=e.step(2.3,2.3,POSES)
        self.assertGreater(e.plan_id,old)
        self.assertTrue(all(c.linear==0 and c.angular==0 for c in controls.values()))

    def test_cancelled_or_late_worker_cannot_install_paths(self):
        class DelayedPool:
            def __init__(self): self.pending=[]
            def submit(self,*args):
                f=Future(); f.set_running_or_notify_cancel(); self.pending.append(f); return f
        pool=DelayedPool()
        e=self.engine(pool=pool)
        e.start(1.,0.,POSES)
        e.queue([Candidate('g',0.,0.)])
        pending=pool.pending[0]
        e.terminate('user_stop')
        pending.set_result({'status':'OK','paths':{'stale':'bad'}})
        e.step(2.,1.,POSES)
        self.assertFalse(e.paths)
        self.assertEqual(e.phase,'STOPPED')

    def test_fewer_targets_allow_idle_bodies_and_cancel_is_not_forbidden_cost(self):
        event=threading.Event()
        out=Path(self.tmp.name)/'batch'
        result=plan_batch(Field(),PCFG,POSES,[Candidate('one',-2.,-3.)],10.,1.,4.,ROBOTS,out,event,{},False)
        self.assertEqual(result['status'],'OK',result)
        self.assertEqual(len(result['paths']),1)
        self.assertEqual(len(result['standby_robot_ids']),3)
        event.set()
        result=plan_batch(Field(),PCFG,POSES,[Candidate('one',-2.,-3.)],10.,1.,4.,ROBOTS,out,event,{},False)
        self.assertEqual(result['status'],'CANCELLED')
        with self.assertRaises(PlanningCancelled):
            SpaceTimePlanner(Field(),PCFG).plan((-3.,-3.),(2.,2.),10.,cancelled=lambda:True)

    def test_exploration_prefers_another_goal_to_long_shadow_wait(self):
        # 늦은 목표는 60초 이후에만 관측할 수 있다. 일반 탐사는 짧은
        # horizon 안의 다른 목표를 선택하고 고정/대피는 원래 범위를 쓴다.
        delayed = (-2., -3.)
        field = Field(danger=lambda a,b,ta,tb,r: a == b == delayed and ta < 60.)
        poses = {'robot1': (-3., -3., 0.)}
        settings = {'max_path_seconds': 45., 'risk': {'safe_goal_windows': True, 'heading_planner': True}}
        out = Path(self.tmp.name)
        value = plan_batch(field, PCFG, poses,
            [Candidate('late', *delayed), Candidate('now', -3., -2.)],
            10., 1., 4., ['robot1'], out/'explore', threading.Event(), settings)
        self.assertEqual(value['status'], 'OK', value)
        self.assertEqual(value['assigned']['robot1'].id, 'now')
        first = json.loads((out/'explore/attempt_000/problem.json').read_text())
        self.assertEqual(first['metadata']['planner']['horizon'], 45.)
        self.assertIsNone(first['costs'][0][0])
        for purpose in ('fixed', 'safety'):
            value = plan_batch(field, PCFG, poses, [Candidate('late', *delayed, purpose=purpose)],
                10., 1., 4., ['robot1'], out/purpose, threading.Event(), settings, True)
            self.assertEqual(value['status'], 'OK', value)
            self.assertGreaterEqual(value['paths']['robot1'].points[-1].t, 60.)
            problem = json.loads((out/purpose/'attempt_000/problem.json').read_text())
            self.assertEqual(problem['metadata']['planner']['horizon'], PCFG.horizon)

    def test_progressive_candidates_use_observed_free_cells_without_area_credit(self):
        e = self.engine('explore')
        scans(e, 1., POSES)
        before = e.observation.observed_count
        goals, _ = observed_candidates(e.observation, POSES, e.field, PCFG.robot_radius,
            {**e.options, 'progressive_goals': True, 'candidate_limit': 6,
             'target_step_m': 3., 'candidate_t0': 10., 'candidate_speed': PCFG.speed})
        self.assertGreaterEqual(len(goals), 4)
        self.assertEqual(before, e.observation.observed_count)
        for i, g in enumerate(goals):
            self.assertEqual(e.observation.cells[e.area.index(g.xy)], 0)
            self.assertFalse(e.field.static_blocked_segment(g.xy, g.xy, PCFG.robot_radius))
            self.assertTrue(all(math.dist(g.xy, h.xy) >= 1. for h in goals[i+1:]))

    def test_predicted_shadow_while_observing_causes_safety_batch(self):
        e=self.engine('explore')
        e.field=Field(danger=lambda a,b,ta,tb,r: a[0]<-2.9 and b[0]<-2.9 and tb>=15.)
        e.start(1.,0.,POSES)
        scans(e,1.,POSES)
        controls=e.step(1.,0.,POSES)
        self.assertIn(e.phase,('PLANNING','EXECUTING'))
        self.assertEqual(e.metrics.replans,1)
        self.assertTrue(all(c.linear==0 for c in controls.values()))
        if e.batch:
            self.assertTrue(all(g.purpose=='safety' for g in e.batch['assigned'].values()))
        e.terminate('test_end')


if __name__=='__main__':
    unittest.main()
