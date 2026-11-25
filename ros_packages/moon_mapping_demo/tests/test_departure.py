"""4대 배정을 유지하는 선발·후발 정책의 공간/시간/BT 계약 검사."""
import math
import csv
from pathlib import Path
import tempfile
import threading
import unittest

from moon_mapping_demo.coordination import coordinate, JointPlanningFailure, joint_conflicts, moving_discs_collide, position_at
from moon_mapping_demo.departure import DeparturePolicy, first_translation_time
from moon_mapping_demo.exploration_planning import Candidate, plan_batch
from moon_mapping_demo.space_time import PlannerConfig, PlanningCancelled
from test_exploration import Exploration, ImmediatePool, POSES, ROBOTS, config, scans, simulate


class Corridor:
    """x<0은 폭 0.8m 통로, x>=0은 넓은 출구. 몸체 반경 .26m 유지."""
    bounds=(-6.,6.,-3.,3.)
    def __init__(self,static=(),danger=None): self.static,self.danger=tuple(static),danger
    def with_static_robots(self,static): return Corridor(static,self.danger)
    def static_blocked_segment(self,a,b,radius):
        if any(not(-6+radius<=p[0]<=6-radius and -3+radius<=p[1]<=3-radius) for p in (a,b)): return True
        if any(p[0]<0 and abs(p[1])+radius>.4 for p in (a,b)): return True
        return any(moving_discs_collide(a,b,(x,y),(x,y),radius+r) for x,y,r in self.static)
    def blocked_segment(self,a,b,ta,tb,radius):
        return self.static_blocked_segment(a,b,radius) or bool(self.danger and self.danger(a,b,ta,tb,radius))


STARTS=dict(zip(ROBOTS,[(-4.,0.),(-3.,0.),(-2.,0.),(-1.,0.)]))
GOALS=dict(zip(ROBOTS,[(1.,0.),(2.,0.),(3.,0.),(4.,0.)]))
CFG=PlannerConfig(resolution=.5,time_step=.5,speed=1.,robot_radius=.26,horizon=40.)


class DepartureTests(unittest.TestCase):
    def test_front_pair_clears_corridor_then_rear_moves_before_final_arrival(self):
        d={}
        paths=coordinate(Corridor(),CFG,STARTS,GOALS,2.,goal_hold=1.,headings=dict.fromkeys(ROBOTS,0.),
            departure=dict(mode='staggered_pairs',release_region=(-5.,0.,-.4,.4),max_schedule_attempts=48),diagnostics=d)
        self.assertEqual(set(paths),set(ROBOTS))
        self.assertEqual(set(d['lead_robots']),{'robot3','robot4'})
        self.assertEqual(set(d['follow_robots']),{'robot1','robot2'})
        self.assertTrue(all(p.points[0].t==2. for p in paths.values()))
        release=min(d['release_times'][r] for r in d['follow_robots'])
        self.assertGreater(release,2.)
        self.assertLess(release,min(paths[r].points[-1].t for r in d['lead_robots']))
        for r in d['follow_robots']:
            self.assertEqual(position_at(paths[r],release-.01),STARTS[r])
            self.assertGreaterEqual(first_translation_time(paths[r]),release)
        self.assertFalse(joint_conflicts(paths,.26,max(p.points[-1].t for p in paths.values())+1.))
        # 선발 두 대가 목표 도착하기 전 후발이 움직이는 구간이 존재한다.
        self.assertTrue(any(sum(a.t<=t<b.t and (a.x,a.y)!=(b.x,b.y)
            for p in paths.values() for a,b in zip(p.points,p.points[1:]))>2 for t in [2+i*.1 for i in range(100)]))

    def test_unsafe_wait_selects_shadow_urgent_pair_instead_of_fixed_names(self):
        from test_exploration import Field
        poses={r:p[:2] for r,p in POSES.items()}
        goals={r:(p[0]+2.,p[1]) for r,p in POSES.items()}
        class Shadow(Field):
            def with_static_robots(self,static):
                obj=Shadow();obj.static=tuple(static);return obj
            def blocked_segment(self,a,b,ta,tb,radius):
                return super().blocked_segment(a,b,ta,tb,radius) or (abs(a[0]-3.)<.15 and a==b and tb>4.)
        d={}
        paths=coordinate(Shadow(),CFG,poses,goals,2.,goal_hold=.5,headings=dict.fromkeys(ROBOTS,0.),
                         departure=dict(mode='staggered_pairs',clearance_distance_m=.5),diagnostics=d)
        self.assertEqual(set(d['lead_robots']),{'robot3','robot4'})
        self.assertEqual(set(paths),set(ROBOTS))

    def test_forward_staging_changes_wait_position_when_long_start_hold_is_unsafe(self):
        from test_exploration import Field
        poses={r:p[:2] for r,p in POSES.items()}
        goals={r:(p[0]+2.,p[1]) for r,p in POSES.items()}
        class Shadow(Field):
            def with_static_robots(self,static):
                obj=Shadow();obj.static=tuple(static);return obj
            def blocked_segment(self,a,b,ta,tb,radius):
                return super().blocked_segment(a,b,ta,tb,radius) or (a==b and a in poses.values() and tb>5.)
        d={}
        paths=coordinate(Shadow(),CFG,poses,goals,2.,goal_hold=.5,headings=dict.fromkeys(ROBOTS,0.),
            departure=dict(mode='staggered_pairs',clearance_distance_m=.5,delay_candidates_seconds=[4.]),diagnostics=d)
        self.assertEqual(d['wait_location'],'forward_staging')
        self.assertTrue(any('initial_wait' in (a['failure'] or '') for a in d['attempts']))
        self.assertTrue(all(d['staging_wait_ticks_by_robot'][r]>0 for r in d['follow_robots']))
        self.assertFalse(joint_conflicts(paths,.26))

    def test_no_safe_wait_records_failure_and_keeps_collision_constraints(self):
        d={}
        field=Corridor(danger=lambda a,b,ta,tb,r: (a[0]<0 and a==b and tb>2.5))
        with self.assertRaises(JointPlanningFailure):
            coordinate(field,CFG,STARTS,GOALS,2.,goal_hold=.5,headings=dict.fromkeys(ROBOTS,0.),
                       departure=dict(mode='staggered_pairs',max_schedule_attempts=12),diagnostics=d)
        self.assertTrue(d['attempts'])
        self.assertTrue(any('shadow' in (a['failure'] or '') for a in d['attempts']))

    def test_plan_batch_keeps_four_by_four_hungarian_and_separates_added_time(self):
        from test_exploration import Field
        poses=dict(POSES)
        goals=[Candidate(r,p[0]+1.,p[1],purpose='fixed') for r,p in poses.items()]
        with tempfile.TemporaryDirectory() as root:
            common=dict(field=Field(),pcfg=CFG,poses=poses,candidates=goals,t0=5.,created_sim=1.,hold=1.,
                        robot_order=ROBOTS,cancel_event=threading.Event(),require_all=True)
            baseline=plan_batch(**common,output=Path(root)/'simultaneous',settings={})
            staggered=plan_batch(**common,output=Path(root)/'pairs',settings={'departure':{'mode':'staggered_pairs','clearance_distance_m':.5}})
        self.assertEqual(staggered['status'],'OK',staggered)
        self.assertEqual(staggered['costs'],baseline['costs'])
        self.assertEqual(staggered['assignment'],baseline['assignment'])
        self.assertEqual(len(staggered['costs']),4)
        self.assertTrue(all(len(row)==4 for row in staggered['costs']))
        self.assertEqual(len(staggered['assigned']),4)
        self.assertGreater(staggered['cost_breakdown']['initial_departure_wait_ticks'],0)
        self.assertGreaterEqual(staggered['coordinated_total_ticks'],staggered['original_assignment_total_ticks'])

    def test_delay_is_replanned_against_new_time_not_shifted_old_path(self):
        from test_exploration import Field
        poses={r:p[:2] for r,p in POSES.items()}
        goals={r:(p[0]+1.,p[1]) for r,p in POSES.items()}
        d={};calls=[]
        class TimedField(Field):
            def with_static_robots(self,static):
                obj=TimedField();obj.static=tuple(static);return obj
            def blocked_segment(self,a,b,ta,tb,r):
                calls.append((ta,tb,a,b))
                return super().blocked_segment(a,b,ta,tb,r)
        coordinate(TimedField(),CFG,poses,goals,2.,goal_hold=.5,departure={'mode':'staggered_pairs'},diagnostics=d)
        release=min(d['release_times'][r] for r in d['follow_robots'])
        self.assertTrue(any(abs(ta-release)<1e-9 and a!=b for ta,tb,a,b in calls))
        with self.assertRaises(PlanningCancelled):
            coordinate(TimedField(),CFG,poses,goals,2.,cancelled=lambda:True,departure={'mode':'staggered_pairs'})

    def test_bt_waits_observes_then_moves_and_cancel_retires_old_schedule(self):
        from test_exploration import Field,PCFG
        cfg=config();cfg['departure']={'mode':'staggered_pairs','clearance_distance_m':.5}
        with tempfile.TemporaryDirectory() as root:
            e=Exploration(cfg,Field(),PCFG,root,'fixed',ImmediatePool())
            e.start(1.,0.,POSES)
            for t in (1.,2.,2.1,2.2):
                scans(e,t,POSES);e.step(t,t,POSES)
            self.assertTrue(e.batch)
            followers=e.batch['departure']['follow_robots']
            self.assertEqual(len(followers),2)
            self.assertTrue(all(e.robot_bt[r].status=='WAITING_DEPARTURE' for r in followers))
            self.assertTrue(all(e.observation.valid_frames[r]>=3 for r in followers))
            old=e.plan_id;task=e.team.active[followers[0]].copy()
            e.robot_bt[followers[0]].navigation.cancel()
            scans(e,2.3,POSES);controls=e.step(2.3,2.3,POSES)
            self.assertTrue(all(c.linear==0 and c.angular==0 for c in controls.values()))
            self.assertGreater(e.plan_id,old)
            self.assertFalse(e.accept_result(followers[0],dict(session=e.team.session,task_id=task['task_id'],status='VISITED'),old))
            self.assertFalse(e.metrics.active_departures)
            self.assertTrue(all(d['ended_reason'] is not None for d in e.metrics.departures))

    def test_full_bt_staggered_fixed_mission_completes_with_actual_departure_records(self):
        from test_exploration import Field,PCFG
        cfg=config();cfg['departure']={'mode':'staggered_pairs','clearance_distance_m':.5}
        with tempfile.TemporaryDirectory() as root:
            e=Exploration(cfg,Field(),PCFG,root,'fixed',ImmediatePool());simulate(e)
            self.assertEqual(e.phase,'COMPLETE',e.reason)
            completed=[d for d in e.metrics.departures if d['ended_reason']=='회차 종료']
            self.assertEqual({d['robot'] for d in completed},set(ROBOTS))
            self.assertTrue(all(d['actual_translation_start_sim_seconds'] is not None for d in completed))
            self.assertTrue(all(d['actual_translation_start_sim_seconds']>=d['scheduled_release_sim_seconds'] for d in completed))
            with (e.output/'metrics.csv').open() as f:
                rows=list(csv.DictReader(f))
            self.assertIn('robot1.departure_plan_id',rows[0])
            self.assertIn('initial_departure_wait_ticks',rows[0])
            self.assertTrue(any(r['robot1.actual_translation_start_sim_seconds'] for r in rows))

    def test_policy_rejects_invalid_settings(self):
        for settings in ({'mode':'two_robots_only'},{'delay_candidates_seconds':[-1.]},{'max_schedule_attempts':0}):
            with self.assertRaises(ValueError): DeparturePolicy.parse(settings)

    def test_failed_four_assignment_is_not_replaced_with_two_robot_policy(self):
        from test_exploration import Field
        poses=dict(zip(ROBOTS,[(-1.,-1.,0.),(-1.,1.,0.),(1.,-1.,0.),(1.,1.,0.)]))
        goals=[Candidate(r,0.,0.) for r in ROBOTS]
        tiny=PlannerConfig(resolution=.5,time_step=.5,speed=1.,robot_radius=.26,horizon=2.)
        with tempfile.TemporaryDirectory() as out:
            result=plan_batch(Field(),tiny,poses,goals,5.,1.,.5,ROBOTS,out,threading.Event(),
                {'departure':{'mode':'staggered_pairs','max_schedule_attempts':4},'planning_wall_budget_seconds':3.},False)
        self.assertNotEqual(result['status'],'OK')
        self.assertTrue(result['attempts'])
        self.assertTrue(all(len(a['robots'])==4 and len(a['costs'])==4 for a in result['attempts']))

    def test_observed_leader_clearance_is_required_without_shifting_old_schedule(self):
        from test_exploration import Field,PCFG
        cfg=config();cfg['departure']={'mode':'staggered_pairs','clearance_distance_m':.5}
        with tempfile.TemporaryDirectory() as out:
            e=Exploration(cfg,Field(),PCFG,out,'fixed',ImmediatePool());e.start(1.,0.,POSES)
            for t in (1.,2.,2.1,2.2): scans(e,t,POSES);e.step(t,t,POSES)
            d=e.batch['departure'];old=e.plan_id
            release=min(d['release_times'][r] for r in d['follow_robots'])
            poses=dict(POSES)
            for r in d['lead_robots']:
                poses[r]=(POSES[r][0]+.49,POSES[r][1],0.)
            scans(e,release,poses);controls=e.step(release,release,poses)
            self.assertGreater(e.plan_id,old)
            self.assertTrue(all(c.linear==0 and c.angular==0 for c in controls.values()))
            self.assertIn('실제 통로 해제',next(v for v in reversed(e.metrics.events) if v['event']=='replan')['reason'])
            self.assertFalse(e.paths)


if __name__=='__main__': unittest.main()
