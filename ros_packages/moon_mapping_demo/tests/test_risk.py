"""이른 대피 예측·정지 기한·계산 지연과 이전 결과 차단 검사."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import threading
from concurrent.futures import Future

from moon_mapping_demo.risk import RiskPolicy,safe_until,prognosis
from moon_mapping_demo.space_time import TimedPath,TimedPoint
from moon_mapping_demo.space_time import SpaceTimePlanner,PlannerConfig
from moon_mapping_demo.exploration_planning import Candidate,plan_escape_batch
from test_exploration import Exploration,ImmediatePool,Field,PCFG,POSES,ROBOTS,config,scans


class RiskTests(unittest.TestCase):
    def test_escape_can_use_goal_after_shadow_clears_without_entering_early(self):
        from moon_mapping_demo.coordination import moving_discs_collide
        from moon_mapping_demo.exploration_planning import escape_candidate_pools,plan_batch
        field=Field(danger=lambda a,b,ta,tb,r: ta<5. and
                    moving_discs_collide(a,b,(.75,0.),(.75,0.),r))
        poses={'robot1':(0.,0.,0.)}
        pcfg=PlannerConfig(speed=.18,robot_radius=.26,horizon=20.)
        settings={'arrival_window_escape':True,'turn_aware_coordination':True,
                  'risk':{'enabled':True,'safe_goal_windows':True}}
        pools=escape_candidate_pools(field,poses,pcfg,1.,2.,settings)
        goal=next(g for g in pools['robot1'] if g.xy==(.75,0.))
        legacy=escape_candidate_pools(field,poses,pcfg,1.,2.,{})
        self.assertNotIn(goal.xy,[g.xy for g in legacy['robot1']])
        with tempfile.TemporaryDirectory() as out:
            value=plan_batch(field,pcfg,poses,[goal],1.,0.,2.,['robot1'],Path(out),
                             threading.Event(),settings,True)
            self.assertEqual(value['status'],'OK',value)
            path=value['paths']['robot1']
            self.assertGreaterEqual(path.points[-1].t,5.)
            for a,b in zip(path.points,path.points[1:]):
                self.assertFalse(field.blocked_segment((a.x,a.y),(b.x,b.y),a.t,b.t,.26))

    def test_partial_escape_moves_endangered_robot_with_safe_stationary_peers(self):
        from moon_mapping_demo.coordination import joint_conflicts
        field=Field(danger=lambda a,b,ta,tb,r: tb>=10. and
                    min(a[0],b[0]) < -2.5 and min(a[1],b[1]) < -2.5)
        pcfg=PlannerConfig(speed=1.,robot_radius=.26,horizon=20.)
        with tempfile.TemporaryDirectory() as out:
            value=plan_escape_batch(field,pcfg,POSES,[],1.,0.,10.,ROBOTS,Path(out),threading.Event(),
                {'allow_partial_escape':True,'progressive_goals':True,'max_path_seconds':10.,
                 'risk':{'enabled':True,'safe_goal_windows':True,'escape_candidate_sets':1}},True)
            self.assertEqual(value['status'],'OK',value)
            self.assertEqual(value['escape_scope'],'endangered_only')
            self.assertEqual(set(value['paths']),{'robot1'})
            self.assertEqual(set(value['standby_robot_ids']),set(ROBOTS)-{'robot1'})
            end=max(p.points[-1].t for p in value['paths'].values())+value['execution_hold_seconds']
            self.assertFalse(joint_conflicts(value['paths'],.26,end))
            for r in value['standby_robot_ids']:
                self.assertFalse(field.blocked_segment(POSES[r][:2],POSES[r][:2],0.,end,.26))

    def test_partial_escape_failure_retains_full_team_fallback(self):
        from moon_mapping_demo.exploration_planning import plan_batch
        field=Field(danger=lambda a,b,ta,tb,r: tb>=10. and
                    min(a[0],b[0]) < -2.5 and min(a[1],b[1]) < -2.5)
        pcfg=PlannerConfig(speed=1.,robot_radius=.26,horizon=20.)
        orders=[]
        def fail_partial(*args):
            orders.append(tuple(args[7]))
            if len(args[7]) < len(ROBOTS):
                return {'status':'NO_SAFE_ASSIGNMENT','error':'부분 대피 불가','attempts':[]}
            return plan_batch(*args)
        with tempfile.TemporaryDirectory() as out, patch(
                'moon_mapping_demo.exploration_planning.plan_batch',side_effect=fail_partial):
            value=plan_escape_batch(field,pcfg,POSES,[],1.,0.,10.,ROBOTS,Path(out),threading.Event(),
                {'allow_partial_escape':True,'progressive_goals':True,'max_path_seconds':10.,
                 'risk':{'enabled':True,'safe_goal_windows':True,'escape_candidate_sets':1}},True)
            self.assertEqual(value['status'],'OK',value)
            self.assertEqual(value['escape_scope'],'full_team_fallback')
            self.assertIn(('robot1',),orders)
            self.assertEqual(set(value['paths']),set(ROBOTS))

    def test_infeasible_matrix_does_not_consume_joint_attempt_budget(self):
        from moon_mapping_demo.exploration_planning import plan_batch
        poses={'robot1':POSES['robot1']}
        goals=[Candidate('outside',100.,100.),Candidate('reachable',-2.,-3.)]
        with tempfile.TemporaryDirectory() as out:
            value=plan_batch(Field(),PCFG,poses,goals,10.,1.,1.,list(poses),Path(out),
                             threading.Event(),{'max_assignment_attempts':1})
            self.assertEqual(value['status'],'OK',value)
            self.assertEqual(len(value['attempts']),2)
            self.assertIsNone(value['attempts'][0]['assignment'])
            self.assertEqual(value['assigned']['robot1'].id,'reachable')

    def test_unsafe_standby_failure_requests_protection_before_retry_limit(self):
        class Pool:
            def submit(self,worker,*args):
                self.worker=worker
                self.future=Future()
                return self.future
        pool=Pool();cfg=config()
        cfg['risk']={'enabled':True,'safe_goal_windows':True,'multiple_escape_candidates':True}
        cfg['exploration']['progressive_goals']=True
        with tempfile.TemporaryDirectory() as out:
            engine=Exploration(cfg,Field(),PCFG,out,pool=pool)
            engine.start(1.,0.,POSES)
            engine.queue([Candidate(r,p[0]+1.,p[1]) for r,p in POSES.items()])
            failed=pool.future
            failed.set_result({'status':'NO_SAFE_ASSIGNMENT','error':'미배정 로봇의 대기 안전성 실패',
                'attempts':[{'assignment':{'columns':[0]},'unsafe_standby_robot_ids':['robot3']}]})
            engine.finish_plan()
            self.assertEqual(engine.phase,'PLANNING')
            self.assertTrue(engine.safety_pending)
            self.assertIs(pool.worker,plan_escape_batch)
            self.assertIsNot(engine.future,failed)
            self.assertEqual(engine.metrics.replans,1)
            self.assertFalse(engine.paths)

    def test_progressive_goal_keeps_full_followup_preparation_runway(self):
        cfg=config()
        cfg['risk']={'enabled':True,'safe_goal_windows':True}
        cfg['exploration']['progressive_goals']=True
        class Pool:
            def submit(self,worker,*args):
                self.hold=args[6]
                self.future=Future()
                return self.future
        pool=Pool()
        with tempfile.TemporaryDirectory() as out:
            engine=Exploration(cfg,Field(),PCFG,out,pool=pool)
            engine.start(1.,0.,POSES)
            engine.queue([Candidate(r,p[0]+1.,p[1]) for r,p in POSES.items()],True)
            self.assertGreater(pool.hold-engine.base_hold_seconds,engine.risk.followup_wait_seconds)
            self.assertGreaterEqual(pool.hold-engine.base_hold_seconds,
                                    engine.risk.required_seconds(PCFG.speed)['total_seconds'])
        with tempfile.TemporaryDirectory() as out:
            fixed=Exploration(cfg,Field(),PCFG,out,mode='fixed',pool=pool)
            fixed.start(1.,0.,POSES)
            fixed.queue([Candidate(r,p[0]+1.,p[1]) for r,p in POSES.items()],True)
            self.assertEqual(pool.hold,fixed.base_hold_seconds)

    def test_short_escape_retains_next_planning_and_turning_time(self):
        import math
        cfg=config()
        cfg['risk']={'enabled':True,'safe_goal_windows':True}
        cfg['exploration'].update(progressive_goals=True,short_escape_hold=True)
        class Pool:
            def submit(self,worker,*args):
                self.hold=args[6]
                return Future()
        pool=Pool()
        with tempfile.TemporaryDirectory() as out:
            engine=Exploration(cfg,Field(),PCFG,out,pool=pool)
            engine.start(1.,0.,POSES)
            goals=[Candidate(r,p[0]+1.,p[1]) for r,p in POSES.items()]
            engine.queue(goals,True)
            normal=pool.hold
            engine.queue(goals,True,safety=True)
            minimum=engine.base_hold_seconds+engine.risk.planning_sim_budget_seconds+engine.risk.swap_seconds+\
                    engine.risk.preparation_seconds+math.pi/engine.risk.turn_rate
            self.assertGreaterEqual(pool.hold,minimum)
            self.assertLess(pool.hold,normal)

    def test_escape_assignment_prices_turns_and_searches_safe_turning_route(self):
        import math
        from moon_mapping_demo.coordination import moving_discs_collide
        from moon_mapping_demo.exploration_planning import plan_batch
        field=Field(danger=lambda a,b,ta,tb,r:tb>=4. and
                    moving_discs_collide(a,b,(1.,1.),(1.,1.),r))
        cfg=PlannerConfig(resolution=.5,time_step=.5,speed=1.,robot_radius=.26,horizon=30.)
        poses={'robot1':(1.,1.,math.pi)}
        goal=Candidate('escape',2.,1.,purpose='safety')
        with tempfile.TemporaryDirectory() as out:
            value=plan_batch(field,cfg,poses,[goal],0.,0.,1.,list(poses),Path(out),threading.Event(),
                             {'turn_aware_coordination':True,'risk':{'enabled':True,'safe_goal_windows':True}},True)
            self.assertEqual(value['status'],'OK',value)
            self.assertEqual(value['time_model'],'space_time_heading_turns')
            self.assertGreater(value['cost_breakdown']['independent_assignment_turn_ticks'],0)
            self.assertEqual(value['costs'][0][0],value['paths']['robot1'].cost_ticks)
            path=value['paths']['robot1']
            self.assertFalse(any(field.blocked_segment((a.x,a.y),(b.x,b.y),a.t,b.t,.26)
                                 for a,b in zip(path.points,path.points[1:])))

    def test_early_arrival_is_replanned_when_peer_wait_crosses_shadow(self):
        from moon_mapping_demo.coordination import coordinate,joint_conflicts
        class Shadow(Field):
            def with_static_robots(self,robots):return Shadow(robots)
            def blocked_segment(self,a,b,ta,tb,r):
                return super().blocked_segment(a,b,ta,tb,r) or (a==b==(-.5,-1.) and ta<4. and tb>=2.)
        field=Shadow();cfg=PlannerConfig(resolution=.5,time_step=.5,speed=1.,robot_radius=.26,horizon=12.)
        starts={'early':(-1.,-1.),'late':(-1.,1.)};goals={'early':(-.5,-1.),'late':(2.,1.)}
        paths=coordinate(field,cfg,starts,goals,0.,goal_hold=1.,safe_goals=True)
        self.assertGreaterEqual(paths['early'].points[-1].t,4.)
        end=max(p.points[-1].t for p in paths.values())+1.
        self.assertFalse(joint_conflicts(paths,.26,end))
        for r,p in paths.items():
            self.assertFalse(field.blocked_segment(goals[r],goals[r],p.points[-1].t,end,.26))

    def test_short_safe_forecast_is_not_a_discovered_danger(self):
        report=prognosis(Field(),{},POSES,1.,PCFG,RiskPolicy(enabled=True,forecast_seconds=10.))
        self.assertFalse(report['trigger'])
        self.assertTrue(all(v['deadline_sim_seconds'] is None for v in report['robots'].values()))

    def test_safe_departure_does_not_evacuate_for_shadow_at_abandoned_position(self):
        field=Field(danger=lambda a,b,ta,tb,r: a[0] < .1 and b[0] < .1 and tb >= 10.)
        poses={'robot1':(0.,0.,0.)}
        path=TimedPath((TimedPoint(0.,0.,1.),TimedPoint(2.,0.,5.)),4)
        policy=RiskPolicy(enabled=True,forecast_seconds=30.)
        stopped=prognosis(field,{},poses,1.,PCFG,policy)
        moving=prognosis(field,{'robot1':path},poses,1.,PCFG,policy)
        self.assertTrue(stopped['trigger'])
        self.assertFalse(moving['trigger'])
        self.assertIsNone(moving['robots']['robot1']['planned_warning'])

    def test_comparison_stage_preserves_numeric_budget(self):
        value=RiskPolicy.stage({'planning_sim_budget_seconds':2.5},'heading')
        self.assertTrue(value['heading_planner'])
        self.assertEqual(value['planning_sim_budget_seconds'],2.5)
        self.assertFalse(RiskPolicy.stage(value,'baseline')['enabled'])

    def test_escape_compares_multiple_complete_candidates_and_cancellation_discards_incumbent(self):
        pools={r:[Candidate(r,p[0]+1.,p[1],purpose='safety')] for r,p in POSES.items()}
        pools['robot1'].append(Candidate('safer',-1.,-3.,purpose='safety'))
        event=threading.Event()
        calls=[]
        def batch(field,pcfg,poses,goals,t0,created,hold,order,out,cancel,settings,require_all):
            calls.append(list(goals))
            return {'status':'OK','error':None,'paths':{r:TimedPath((TimedPoint(*poses[r][:2],t0),
                TimedPoint(*g.xy,t0+5.)),5) for r,g in zip(order,goals)},
                'assigned':dict(zip(order,goals)),'coordinated_total_ticks':20+len(calls),'attempts':[]}
        def margin(field,xy,arrival,horizon,radius):
            return arrival+(100. if xy==(-2.,-3.) else 200.)
        with tempfile.TemporaryDirectory() as out, patch('moon_mapping_demo.exploration_planning.escape_candidate_pools',return_value=pools), \
                patch('moon_mapping_demo.exploration_planning.plan_batch',side_effect=batch), \
                patch('moon_mapping_demo.exploration_planning.safe_until',side_effect=margin):
            value=plan_escape_batch(Field(),PCFG,POSES,[],10.,1.,7.,ROBOTS,Path(out),event,
                {'risk':{'enabled':True,'multiple_escape_candidates':True}},True)
            self.assertEqual(len(calls),2)
            self.assertEqual(value['selected_candidate_set'],1)
            self.assertEqual(value['assigned']['robot1'].id,'safer')
            # 탐색 예산 종료와 명시적 취소는 다르다. 전자는 검증된 완전
            # 계획을 명시적 부분 탐색 결과로 반환할 수 있고 후자는 폐기한다.
            with patch('moon_mapping_demo.exploration_planning.time.monotonic',side_effect=[0.,0.,9.]):
                bounded=plan_escape_batch(Field(),PCFG,POSES,[],10.,1.,7.,ROBOTS,Path(out)/'budget',event,
                    {'risk':{'enabled':True,'multiple_escape_candidates':True}},True)
            self.assertEqual(bounded['status'],'OK')
            self.assertEqual(bounded['escape_search_end'],'PLANNING_BUDGET')
            self.assertEqual(len(bounded['candidate_trials']),1)
            original=batch
            def cancelled(*args):
                result=original(*args);event.set();return result
            with patch('moon_mapping_demo.exploration_planning.plan_batch',side_effect=cancelled):
                value=plan_escape_batch(Field(),PCFG,POSES,[],10.,1.,7.,ROBOTS,Path(out)/'cancel',event,
                    {'risk':{'enabled':True,'multiple_escape_candidates':True}},True)
            self.assertEqual(value['status'],'CANCELLED')
            self.assertNotIn('paths',value)

    def test_short_escape_failure_keeps_full_horizon_fallback(self):
        pools={r:[Candidate(r,p[0]+1.,p[1],purpose='safety')] for r,p in POSES.items()}
        horizons=[]
        def batch(field,pcfg,poses,goals,t0,created,hold,order,out,cancel,settings,require_all):
            horizons.append(pcfg.horizon)
            if pcfg.horizon < PCFG.horizon:
                return {'status':'NO_SAFE_ASSIGNMENT','error':'긴 대피 경로가 필요한 입력','attempts':[]}
            return {'status':'OK','error':None,'attempts':[],
                'paths':{r:TimedPath((TimedPoint(*poses[r][:2],t0),TimedPoint(*g.xy,t0+5.)),5)
                         for r,g in zip(order,goals)},
                'assigned':dict(zip(order,goals)),'coordinated_total_ticks':20}
        with tempfile.TemporaryDirectory() as out, \
                patch('moon_mapping_demo.exploration_planning.escape_candidate_pools',return_value=pools), \
                patch('moon_mapping_demo.exploration_planning.plan_batch',side_effect=batch):
            value=plan_escape_batch(Field(),PCFG,POSES,[],10.,1.,7.,ROBOTS,Path(out),threading.Event(),
                {'progressive_goals':True,'max_path_seconds':45.,'planning_wall_budget_seconds':8.,
                 'planning_result_margin_seconds':.4},True)
            self.assertEqual(value['status'],'OK')
            self.assertEqual(horizons,[45.,PCFG.horizon])
            self.assertEqual(set(value['paths']),set(ROBOTS))

    def test_changed_pose_or_heading_rejects_completed_plan_before_dispatch(self):
        for heading in (False,True):
            cfg=config();cfg['risk']={'enabled':True}
            with self.subTest(heading=heading), tempfile.TemporaryDirectory() as out:
                e=Exploration(cfg,Field(),PCFG,out,pool=ImmediatePool());e.start(1.,0.,POSES)
                e.queue([Candidate(r,p[0]+1.,p[1]) for r,p in POSES.items()],True)
                self.assertEqual(e.future.result()['status'],'OK')
                changed=dict(POSES);x,y,a=changed['robot1']
                changed['robot1']=(x,y,a+.3) if heading else (x+.1,y,a)
                e.poses=changed;e.now=1.1;e.wall=.1;e.finish_plan()
                self.assertEqual(e.phase,'ERROR')
                self.assertEqual(e.planning_job['status'],'STALE_POSE_OR_STOP_WINDOW')
                self.assertFalse(e.paths)
                self.assertFalse(any(e.dispatch_count.values()))

    def test_later_goal_arrival_is_searched_when_first_service_window_is_unsafe(self):
        class GoalShadow(Field):
            bounds=(-3.,3.,-3.,3.)
            def blocked_segment(self,a,b,ta,tb,r):
                return super().blocked_segment(a,b,ta,tb,r) or (a==b==(1.,0.) and ta<4. and tb>=2.)
        cfg=PlannerConfig(resolution=.5,time_step=.5,speed=1.,robot_radius=.26,horizon=12.)
        field=GoalShadow();planner=SpaceTimePlanner(field,cfg)
        earliest=planner.plan((0.,0.),(1.,0.),0.)
        self.assertEqual(earliest.points[-1].t,1.)
        service=planner.plan((0.,0.),(1.,0.),0.,goal_hold=3.)
        self.assertIsNotNone(service)
        self.assertGreaterEqual(service.points[-1].t,4.)
        self.assertFalse(field.blocked_segment((1.,0.),(1.,0.),service.points[-1].t,service.points[-1].t+3.,.26))

    def test_goal_warning_margin_uses_distinct_service_windows(self):
        class Edge(Field):
            def blocked_segment(self,a,b,ta,tb,r):
                return super().blocked_segment(a,b,ta,tb,r) or (a == b == (1.,0.) and r > .3 and ta < 6.)
        cfg=PlannerConfig(resolution=.5,time_step=.5,speed=1.,robot_radius=.26,horizon=12.)
        planner=SpaceTimePlanner(Edge(),cfg)
        body=planner.plan((0.,0.),(1.,0.),0.,goal_hold=1.)
        buffered=planner.plan((0.,0.),(1.,0.),0.,goal_hold=1.,goal_radius=.31)
        again=planner.plan((0.,0.),(1.,0.),0.,goal_hold=1.)
        self.assertEqual(body.points[-1].t,1.)
        self.assertGreaterEqual(buffered.points[-1].t,6.)
        self.assertEqual(again,body)
        with self.assertRaises(ValueError):
            planner.plan((0.,0.),(1.,0.),0.,goal_radius=.25)

    def test_heading_planner_records_rotation_and_avoids_unsafe_postprocessing(self):
        import math
        from moon_mapping_demo.coordination import with_turn_holds,moving_discs_collide
        class Shadow(Field):
            bounds=(0.,3.,0.,3.)
            def blocked_segment(self,a,b,ta,tb,r):
                return super().blocked_segment(a,b,ta,tb,r) or (tb>=4. and moving_discs_collide(a,b,(1.,1.),(1.,1.),r))
        cfg=PlannerConfig(resolution=.5,time_step=.5,speed=1.,robot_radius=.26,horizon=30.)
        field=Shadow();planner=SpaceTimePlanner(field,cfg)
        raw=planner.plan((1.,1.),(2.,1.),0.)
        shifted=with_turn_holds(raw,math.pi,cfg)
        self.assertTrue(any(field.blocked_segment((a.x,a.y),(b.x,b.y),a.t,b.t,.26) for a,b in zip(shifted.points,shifted.points[1:])))
        planned=planner.plan((1.,1.),(2.,1.),0.,initial_yaw=math.pi)
        self.assertIsNotNone(planned)
        self.assertGreater(planner.last_turn_ticks,0)
        self.assertFalse(any(field.blocked_segment((a.x,a.y),(b.x,b.y),a.t,b.t,.26) for a,b in zip(planned.points,planned.points[1:])))

    def test_stop_safe_now_but_not_safe_for_next_departure(self):
        field=Field(danger=lambda a,b,ta,tb,r:a==b and a[0]>2.9 and tb>=8.)
        self.assertFalse(field.blocked_segment((3.,0.),(3.,0.),6.,6.,.26))
        limit=safe_until(field,(3.,0.),6.,10.,.26,.05)
        self.assertLess(limit,8.)
        self.assertGreaterEqual(limit,7.95)

    def test_predicts_future_arrival_hold_with_rotation_and_compute_budget(self):
        field=Field(danger=lambda a,b,ta,tb,r:a[0]>2.9 and b[0]>2.9 and tb>=30.)
        path=TimedPath((TimedPoint(0.,0.,1.),TimedPoint(3.,0.,20.)),19)
        report=prognosis(field,{'r':path},{'r':(0.,0.,0.)},1.,PCFG,RiskPolicy(enabled=True))
        self.assertTrue(report['trigger'])
        self.assertEqual(report['robots']['r']['planned_warning']['kind'],'observation_or_followup_hold')
        parts=report['required_time_estimate']
        self.assertGreater(parts['rotation_estimate_seconds'],6.)
        self.assertGreater(parts['total_seconds'],parts['escape_move_estimate_seconds'])

    def test_refuses_unsafe_planning_stop_before_submitting_worker(self):
        cfg=config();cfg['risk']={'enabled':True}
        field=Field(danger=lambda a,b,ta,tb,r:a==b and a[0]>2.9 and tb>=5.)
        with tempfile.TemporaryDirectory() as out:
            e=Exploration(cfg,field,PCFG,out,pool=ImmediatePool());e.start(1.,0.,POSES)
            e.queue([Candidate(r,p[0]+1.,p[1]) for r,p in POSES.items()],True,True)
            self.assertEqual(e.phase,'ERROR')
            self.assertIn('no_safe_planning_stop',e.reason)
            self.assertIsNone(e.future)
            self.assertFalse(e.metrics.events[-1]['safe'])

    def test_delayed_result_is_cancelled_on_sim_budget_and_cannot_install(self):
        class Pool:
            def submit(self,*args):
                self.f=Future();self.f.set_running_or_notify_cancel();return self.f
        pool=Pool();cfg=config();cfg['risk']={'enabled':True,'planning_sim_budget_seconds':2.}
        with tempfile.TemporaryDirectory() as out:
            e=Exploration(cfg,Field(),PCFG,out,pool=pool);e.start(1.,0.,POSES)
            e.queue([Candidate(r,p[0]+1.,p[1]) for r,p in POSES.items()],True)
            old=e.plan_id
            scans(e,3.1,POSES);commands=e.step(3.1,1.,POSES)
            self.assertEqual(e.phase,'ERROR')
            self.assertGreater(e.plan_id,old)
            self.assertFalse(e.paths)
            self.assertTrue(all(c.linear==0 for c in commands.values()))
            pool.f.set_result({'status':'OK','paths':{'late':'invalid'}})
            e.step(3.2,1.1,POSES)
            self.assertFalse(e.paths)
            job=next(v for v in e.metrics.events if v['event']=='planning_timing')
            self.assertGreater(job['sim_elapsed_seconds'],2.)
            self.assertNotEqual(job['sim_elapsed_seconds'],job['wall_elapsed_seconds'])

    def test_planning_hold_checks_departure_time_instead_of_twenty_second_stop(self):
        class Pool:
            def submit(self,*args):
                self.future=Future();self.future.set_running_or_notify_cancel();return self.future
        pool=Pool();cfg=config();cfg['risk']={'enabled':True}
        cfg['exploration']['progressive_goals']=True
        field=Field(danger=lambda a,b,ta,tb,r:a == b == POSES['robot1'][:2] and tb >= 10.)
        with tempfile.TemporaryDirectory() as out:
            e=Exploration(cfg,field,PCFG,out,pool=pool);e.start(1.,0.,POSES)
            e.queue([Candidate(r,p[0]+1.,p[1]) for r,p in POSES.items()],True)
            pending=e.future
            self.assertEqual(e.planning_job['t0'],9.5)
            scans(e,2.,POSES)
            controls=e.step(2.,1.,POSES)
            self.assertIs(e.future,pending)
            self.assertEqual(e.phase,'PLANNING')
            self.assertEqual(e.metrics.replans,0)
            self.assertTrue(all(c.linear==0 for c in controls.values()))

    def test_early_escape_can_trigger_during_active_motion(self):
        cfg=config();cfg['risk']={'enabled':True}
        field=Field(danger=lambda a,b,ta,tb,r:a[0]>=2.9 and b[0]>=2.9 and tb>=30.)
        with tempfile.TemporaryDirectory() as out:
            e=Exploration(cfg,field,PCFG,out,pool=ImmediatePool());e.start(1.,0.,POSES)
            scans(e,1.,POSES);e.step(1.,0.,POSES)
            self.assertTrue(any(v['event']=='early_escape_prediction' for v in e.metrics.events))
            self.assertTrue(any(v['event']=='planning_stop_check' and v['safe'] for v in e.metrics.events))
            self.assertGreater(e.planning_job['t0']-e.planning_job['submitted_sim'],3.)


if __name__=='__main__':unittest.main()
