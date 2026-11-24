"""Safety-model and global-frontier regressions; no robot commands."""
import math
import json
import tempfile
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

import numpy as np

from planner import ROBOTS, allocate
from shadow_prediction import Forecast, encode, envelope, triangle_envelope
from mission import coverage, settings
from radio import neighbors, connected, routes_connected, movement_permits
from explore import Exploration
from radio import route_point


class PredictionTests(unittest.TestCase):
    def test_separate_ground_features_do_not_block_gap(self):
        triangles = np.array([[[-20., -1., 1.], [-19., -1., 1.], [-20., 1., 1.]],
                              [[20., -1., 1.], [21., -1., 1.], [20., 1., 1.]]])
        v0 = triangles[:, 0]
        meshes = [(triangles.min(axis=(0,1)), triangles.max(axis=(0,1)),
                   v0, triangles[:,1]-v0, triangles[:,2]-v0)]
        mask = triangle_envelope(meshes, [0., 0., 1.], (-31., -31.), .2, 310, margin=0)
        self.assertTrue(mask[155,55])
        self.assertFalse(mask[155,155])

    def test_updated_southwest_starts_can_receive_cp_sat_tasks(self):
        cells = np.full((180,180), -1, dtype=np.int16)
        cells[10:160,10:160] = 0
        homes = dict(zip(ROBOTS, [(-26.,-26.),(-26.,-24.5),(-24.5,-26.),(-24.5,-24.5)]))
        forecast = Forecast(encode(np.zeros((181,310,310),dtype=np.uint8),
                                   0.,origin=(-31.,-31.),resolution=.2))
        result = allocate(cells,.1,(-30.,-30.),homes,homes,
                          dict.fromkeys(ROBOTS,0.),[],[],radio_range=4.5,
                          budget=100,forecast=forecast,bounds=(-30.,-30.,30.,30.),max_leg=1.5)
        self.assertIn('tasks',result,result)
        self.assertTrue(connected({r:t['goal'] for r,t in result['tasks'].items()},4.5))
        self.assertTrue(all(forecast.safe(t['goal'],result['departure_at'])
                            for t in result['tasks'].values()))
        self.assertTrue(all(t['grid_distance_m'] <= 1.75 for t in result['tasks'].values()))

    def test_box_casts_shadow_away_from_sun(self):
        box = [(np.array([0., 0., .02]), np.array([1., 1., 1.02]))]
        mask = envelope(box, [math.sqrt(.5), 0, math.sqrt(.5)],
                        origin=(-2., -2.), resolution=.05, size=100, margin=0)
        def blocked(x, y):
            return mask[int((y+2)/.05), int((x+2)/.05)]
        self.assertTrue(blocked(-.5, .5))
        self.assertTrue(blocked(.5, .5))
        self.assertFalse(blocked(1.5, .5))

    def test_future_crossing_rejected_even_when_currently_clear(self):
        masks = np.zeros((46, 147, 147), dtype=np.uint8)
        row = col = int(11/.15)
        masks[5:, row-4:row+5, col-4:col+5] = 1
        forecast = Forecast(encode(masks, 100.))
        self.assertTrue(forecast.safe((0., 0.), 100.))
        self.assertFalse(forecast.hold_safe((0., 0.), 100., 20.))
        self.assertFalse(forecast.route_safe([(-1., 0.), (0., 0.)], 100.))
        self.assertFalse(forecast.safe((0., 0.), 200.))
        self.assertFalse(forecast.safe((50., 0.), 100.))

    def test_global_frontier_is_not_limited_to_local_leg(self):
        cells = np.full((160, 160), -1, dtype=np.int16)
        cells[10:140, 10:140] = 0
        homes = dict(zip(ROBOTS, [(-8., -7.6), (-8., -6.2), (-6.6, -7.6), (-6.6, -6.2)]))
        forecast = Forecast(encode(np.zeros((181, 147, 147), dtype=np.uint8), 0.))
        result = allocate(cells, .1, (-12., -12.), homes, homes,
                          dict.fromkeys(ROBOTS, 0.), [], [], radio_range=4.5, budget=40,
                          forecast=forecast,solver_seconds=30.)
        self.assertIn('tasks', result, result)
        tasks = result['tasks']
        self.assertTrue(connected({r:t['goal'] for r,t in tasks.items()},4.5))
        explorers = [t for t in tasks.values() if t['role']=='explorer']
        self.assertGreaterEqual(len(explorers),1)
        self.assertLessEqual(len(explorers),3)
        self.assertTrue(any(math.dist(t['goal'],t['global_goal']) > 1. for t in explorers))
        self.assertTrue(any(math.dist(homes[r],t['goal'])>2.5 for r,t in tasks.items()))
        self.assertTrue(all(math.dist(homes[r],t['goal'])<5 for r,t in tasks.items()))
        self.assertNotIn('tasks', allocate(cells, .1, (-12., -12.), homes, homes,
                         dict.fromkeys(ROBOTS, 40.), [], [], radio_range=4.5, budget=40, forecast=forecast))


class MissionTests(unittest.TestCase):
    def test_failed_path_preserves_other_validated_paths(self):
        peer_path = [(2.,0.),(3.,0.)]
        agent = SimpleNamespace(validating={'robot1'},rejected=[],now=lambda:10.,
                                tasks={'robot1':{'goal':(1.,0.)}},get_logger=Mock(),
                                previous={'robot1':(0.,0.)},shadows={'robot1':False},
                                forecast=SimpleNamespace(hold_safe=lambda *args:True),
                                validated_plans={'robot2':peer_path},validated={'robot2'})
        Exploration.path_failed(agent,'robot1','Nav2 found no path')
        self.assertEqual(agent.tasks['robot1']['goal'],(0.,0.))
        self.assertTrue(agent.tasks['robot1']['path_blocked'])
        self.assertFalse(agent.tasks['robot1']['waiting_for_sun'])
        self.assertEqual(agent.validated_plans['robot2'],peer_path)
        self.assertEqual(agent.validated,{'robot1','robot2'})
        self.assertEqual(agent.rejected[0]['point'],(1.,0.))

    def test_shaded_robot_holds_while_sunlit_peers_receive_moving_goals(self):
        cells = np.full((80,80),-1,dtype=np.int16)
        cells[8:72,8:72] = 0
        homes = dict(zip(ROBOTS,[(-1.5,-1.5),(-1.5,0.),(0.,-1.5),(0.,0.)]))
        masks = np.zeros((181,80,80),dtype=np.uint8)
        masks[:,39:42,39:42] = 1
        forecast = Forecast(encode(masks,0.,origin=(-8.,-8.),resolution=.2))
        self.assertFalse(forecast.safe(homes['robot4'],0.))
        result = allocate(cells,.2,(-8.,-8.),homes,homes,dict.fromkeys(ROBOTS,0.),[],[],
                          radio_range=10.,budget=100,forecast=forecast,max_leg=2.,
                          frozen_robots={'robot4'},movement_reward=1000)
        self.assertIn('tasks',result,result)
        tasks = result['tasks']
        self.assertEqual(tuple(tasks['robot4']['goal']),homes['robot4'])
        self.assertTrue(tasks['robot4']['waiting_for_sun'])
        self.assertTrue(any(math.dist(homes[r],t['goal'])>.3
                            for r,t in tasks.items() if r!='robot4'))
        self.assertTrue(connected({r:t['goal'] for r,t in tasks.items()},10.))
        self.assertTrue(all(forecast.safe(t['goal'],result['departure_at'])
                            for r,t in tasks.items() if r!='robot4'))

    def test_shadow_incident_cancels_only_affected_robot(self):
        isolate = Mock()
        agent = SimpleNamespace(phase='MOVING',shadows={},shadow_times={},now=lambda:10.,
                                allowed={'robot1','robot2'},queue=[],
                                handles={'robot1':Mock(),'robot2':Mock()},canceling=set(),
                                isolate=isolate,defer=Mock())
        Exploration.receive_shadow(agent,'robot1',SimpleNamespace(data=True))
        self.assertEqual(agent.phase,'MOVING')
        self.assertEqual(agent.allowed,{'robot2'})
        isolate.assert_called_once()
        self.assertEqual(isolate.call_args.args[0],'robot1')
        agent.defer.assert_not_called()

    def test_reposition_cannot_report_motion_when_only_holds_are_safe(self):
        class HoldsOnly:
            def departure_delay(self,route,*args,**kwargs):
                return 0. if math.dist(route[0],route[-1])<.25 else None
            def departure_constraints(self,route,*args,**kwargs):
                return [(0,4,30)] if math.dist(route[0],route[-1])<.25 else []
        cells = np.full((80,80),-1,dtype=np.int16)
        cells[8:72,8:72] = 0
        homes = dict(zip(ROBOTS,[(-1.5,-1.5),(-1.5,0.),(0.,-1.5),(0.,0.)]))
        result = allocate(cells,.2,(-8.,-8.),homes,homes,dict.fromkeys(ROBOTS,0.),[],[],
                          radio_range=10.,budget=100,forecast=HoldsOnly(),
                          allow_reposition=True,max_leg=2.)
        self.assertEqual(result['status'],'INFEASIBLE')
        self.assertNotIn('tasks',result)

    def test_rejected_target_cannot_return_as_relay(self):
        cells = np.full((80,80),-1,dtype=np.int16)
        cells[8:72,8:72] = 0
        homes = dict(zip(ROBOTS,[(-1.5,-1.5),(-1.5,0.),(0.,-1.5),(0.,0.)]))
        args = (cells,.2,(-8.,-8.),homes,homes,dict.fromkeys(ROBOTS,0.),[])
        kwargs = dict(radio_range=10.,budget=100,allow_reposition=True,
                      maximum_explorers=0,max_leg=2.,movement_reward=1000)
        first = allocate(*args,[],**kwargs)
        self.assertIn('tasks',first,first)
        blocked = next(t['goal'] for r,t in first['tasks'].items()
                       if math.dist(homes[r],t['goal'])>=.3)
        second = allocate(*args,[blocked],**kwargs)
        self.assertIn('tasks',second,second)
        self.assertTrue(all(t['role']=='relay' for t in second['tasks'].values()))
        for robot,task in second['tasks'].items():
            if math.dist(homes[robot],task['goal'])>.25:
                self.assertGreaterEqual(math.dist(task['goal'],blocked),.75)

    def test_prefetched_plan_rejected_after_stall_or_pose_mismatch(self):
        positions = dict(zip(ROBOTS,[(0.,0.),(2.,0.),(4.,0.),(6.,0.)]))
        agent = SimpleNamespace(round=3,rejected=[],now=lambda:15.,prefetch={
            'positions':positions.copy(),'round':3,'rejections':0,'stamp':10.})
        self.assertTrue(Exploration.prefetch_usable(agent,positions))
        agent.rejected.append({'point':(1.,0.),'until':20.})
        self.assertFalse(Exploration.prefetch_usable(agent,positions))
        agent.rejected.clear()
        shifted = dict(positions,robot1=(1.,0.))
        self.assertFalse(Exploration.prefetch_usable(agent,shifted))
        agent.now = lambda:45.
        self.assertFalse(Exploration.prefetch_usable(agent,positions))

    def test_future_only_windows_do_not_schedule_long_group_wait(self):
        class Windows:
            def departure_delay(self,*args,**kwargs): return 100.
            def departure_constraints(self,*args,**kwargs): return [(20,24,30)]
        cells = np.full((80,80),-1,dtype=np.int16)
        cells[8:72,8:72] = 0
        homes = dict(zip(ROBOTS,[(-1.5,-1.5),(-1.5,0.),(0.,-1.5),(0.,0.)]))
        result = allocate(cells,.2,(-8.,-8.),homes,homes,dict.fromkeys(ROBOTS,0.),[],[],
                          radio_range=10.,budget=100,forecast=Windows(),max_wait_s=10.)
        self.assertEqual(result['status'],'INFEASIBLE')
        self.assertNotIn('tasks',result)

    def test_stalled_robot_does_not_cancel_peer_goals(self):
        peer = Mock()
        stalled = Mock()
        agent = SimpleNamespace(allowed={'robot1','robot2'},plans={'robot1':[(0,0)],'robot2':[(1,0)]},
                                tasks={'robot1':{'goal':(2,0)}},rejected=[],isolated=set(),
                                handles={'robot1':stalled,'robot2':peer},canceling=set(),
                                now=lambda:10.,get_logger=Mock(),phase='MOVING')
        Exploration.isolate(agent,'robot1','stalled')
        stalled.cancel_goal_async.assert_called_once()
        peer.cancel_goal_async.assert_not_called()
        self.assertEqual(agent.phase,'MOVING')
        self.assertIn('robot2',agent.allowed)
        self.assertIn('robot2',agent.plans)

    def test_ten_meter_radio_supports_wider_relay_chain(self):
        positions = dict(zip(ROBOTS,[(0.,0.),(9.,0.),(18.,0.),(27.,0.)]))
        self.assertFalse(connected(positions,4.5))
        self.assertTrue(connected(positions,10.))
        self.assertEqual(neighbors(positions,'robot1',10.),['robot2'])
        projections = {r:(p[0]+.2,p[1]) for r,p in positions.items()}
        self.assertEqual(movement_permits(positions,projections,10.),set(ROBOTS))

    def test_spacing_preference_replaces_compact_team_objective(self):
        cells = np.full((80,80),-1,dtype=np.int16)
        cells[8:72,8:72] = 0
        homes = dict(zip(ROBOTS,[(-1.5,-1.5),(-1.5,0.),(0.,-1.5),(0.,0.)]))
        args = (cells,.2,(-8.,-8.),homes,homes,dict.fromkeys(ROBOTS,0.),[],[])
        compact = allocate(*args,radio_range=10.,budget=100,max_leg=2.,
                           compactness_weight=500,spacing_weight=0)
        spread = allocate(*args,radio_range=10.,budget=100,max_leg=2.,
                          compactness_weight=0,spacing_weight=500,preferred_spacing=10.)
        self.assertIn('tasks',compact,compact)
        self.assertIn('tasks',spread,spread)
        def pair_spacing(result):
            goals = [t['goal'] for t in result['tasks'].values()]
            return sum(math.dist(p,q) for i,p in enumerate(goals) for q in goals[i+1:])
        self.assertGreater(pair_spacing(spread),pair_spacing(compact)+1.)
        self.assertTrue(connected({r:t['goal'] for r,t in spread['tasks'].items()},10.))

    def test_motor_stop_does_not_pause_simulation_clock(self):
        agent = SimpleNamespace(phase='MOVING', heartbeat=Mock(), cancel_all=Mock(),
                                status=Mock(), get_logger=Mock(), round=1, travel={},
                                known=0, coverage_report=None, start_known=0, results=[])
        with tempfile.TemporaryDirectory() as folder:
            command = Path(folder)/'simulation_command'
            command.write_text('resume')
            with patch('explore.Path', return_value=command):
                Exploration.halt(agent,'Test motor stop')
            self.assertEqual(command.read_text(),'resume')
        self.assertEqual(agent.phase,'STOPPED')
        self.assertTrue(agent.heartbeat.publish.call_args.args[0].data)

    def test_shadow_deferral_keeps_full_map_mission_pending(self):
        agent = SimpleNamespace(phase='MOVING',queue=['robot1'],allowed={'robot1'},
                                cancel_all=Mock(),heartbeat=Mock(),get_logger=Mock(),
                                status=Mock(),round=7,coverage_report={'observed_fraction':.09},
                                shadows={'robot1':True})
        Exploration.defer(agent,'Waiting for a safe window')
        self.assertEqual(agent.phase,'WAITING')
        self.assertEqual(agent.queue,[])
        self.assertFalse(agent.allowed)
        self.assertTrue(agent.heartbeat.publish.call_args.args[0].data)
        self.assertFalse(json.loads(agent.status.publish.call_args.args[0].data)['mission_complete'])

    def test_unknown_accessible_area_prevents_completion(self):
        cells = np.zeros((20,20), dtype=np.int16)
        cells[:,10:] = -1
        report = coverage(cells, .1, (0.,0.), {'robot1': (.5,.5)}, (0.,0.,2.,2.))
        self.assertAlmostEqual(report['observed_fraction'], .5)
        self.assertAlmostEqual(report['unknown_potentially_accessible_m2'], 2.)
        self.assertFalse(report['all_accessible_observed'])
        self.assertFalse(report['mission_complete'])

    def test_slam_grid_growth_does_not_inflate_coverage(self):
        small = np.zeros((10,10), dtype=np.int16)
        large = np.full((30,30), -1, dtype=np.int16)
        large[10:20,10:20] = 0
        args = ({'robot1': (.5,.5)}, (0.,0.,2.,2.))
        a = coverage(small, .1, (0.,0.), *args)
        b = coverage(large, .1, (-1.,-1.), *args)
        self.assertAlmostEqual(a['observed_area_m2'], b['observed_area_m2'])
        self.assertAlmostEqual(a['observed_fraction'], .25)
        self.assertFalse(a['all_accessible_observed'])

    def test_mobile_team_has_no_fixed_base_radius_bound(self):
        config = json.loads(Path(__file__).with_name('mission.json').read_text())
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'mission.json'
            for radius in (None, 0):
                config['radio_range_m'] = radius
                path.write_text(json.dumps(config))
                with self.assertRaises(ValueError):
                    settings(path)
            config['radio_range_m'] = 4.5
            path.write_text(json.dumps(config))
            self.assertEqual(settings(path)['connectivity_mode'],'mobile_team')

    def test_multihop_team_can_translate_without_anchor(self):
        positions = dict(zip(ROBOTS,[(0.,0.),(3.,0.),(6.,0.),(9.,0.)]))
        self.assertTrue(connected(positions,4.5))
        self.assertEqual(neighbors(positions,'robot1',4.5),['robot2'])
        moved = {r:(p[0]+100.,p[1]+100.) for r,p in positions.items()}
        self.assertTrue(connected(moved,4.5))
        moved['robot4'] = (130.,100.)
        self.assertFalse(connected(moved,4.5))

    def test_detouring_path_cannot_silently_break_relay(self):
        positions = dict(zip(ROBOTS,[(0.,0.),(3.,0.),(6.,0.),(9.,0.)]))
        plans = {r:[p,(p[0]+.5,p[1]+.2)] for r,p in positions.items()}
        self.assertTrue(routes_connected(plans,positions,4.5))
        plans['robot2'] = [(3.,0.),(3.,10.),(3.,0.)]
        self.assertTrue(connected(positions,4.5))
        self.assertFalse(routes_connected(plans,positions,4.5))

    def test_long_synchronized_translation_and_lag_gate(self):
        positions = dict(zip(ROBOTS,[(0.,0.),(3.,0.),(6.,0.),(9.,0.)]))
        plans = {r:[p,(p[0]+6.,p[1])] for r,p in positions.items()}
        self.assertTrue(routes_connected(plans,positions,4.5))
        positions['robot4'] = (10.2,0.)
        projections = {r:(p[0]+.44,p[1]) for r,p in positions.items()}
        permits = movement_permits(positions,projections,4.5)
        self.assertNotIn('robot4',permits)
        self.assertIn('robot3',permits)

    def test_departure_waits_for_sun_window(self):
        masks = np.zeros((181,147,147),dtype=np.uint8)
        row = col = int(11/.15)
        masks[:5,row-2:row+3,col-2:col+3] = 1
        forecast = Forecast(encode(masks,100.,sun_period_s=240.))
        route = [(-1.,0.),(0.,0.)]
        delay = forecast.departure_delay(route,100.)
        self.assertIsNotNone(delay)
        self.assertGreater(delay,0)
        self.assertTrue(forecast.route_safe(route,100.+delay))

    def test_cp_sat_chooses_common_departure_slot(self):
        class Windows:
            def departure_delay(self,*args,**kwargs):return 0.
            def departure_constraints(self,route,now,base,hold=15,**kwargs):
                return [(i,i+4,i+12) for i in ([1,2] if route[0][0]>-7 else [0,2])]
        cells = np.full((160,160),-1,dtype=np.int16)
        cells[10:140,10:140] = 0
        homes = dict(zip(ROBOTS,[(-8.,-7.6),(-8.,-6.2),(-6.6,-7.6),(-6.6,-6.2)]))
        result = allocate(cells,.1,(-12.,-12.),homes,homes,dict.fromkeys(ROBOTS,0.),[],[],
                          radio_range=4.5,budget=40,forecast=Windows())
        self.assertIn('tasks',result,result)
        self.assertEqual(result['departure_at'],11.)

    def test_waiting_for_peer_can_make_detour_connected(self):
        positions = dict(zip(ROBOTS,[(0.,0.),(3.,0.),(6.,0.),(9.,0.)]))
        plans = {r:[p,(p[0]+6.,0.)] for r,p in positions.items()}
        plans['robot2'] = [(3.,0.),(3.,3.),(9.,3.),(9.,0.)]
        equal_progress = {r:route_point([positions[r]]+plans[r],fraction=.25) for r in ROBOTS}
        self.assertFalse(connected(equal_progress,4.5))
        self.assertTrue(routes_connected(plans,positions,4.5))

    def test_unsolicited_plan_cannot_overwrite_validation_snapshot(self):
        agent = SimpleNamespace(phase='VALIDATING',tasks={'robot1':{'goal':(1.,0.)}},
                                plans={},validated_plans={'robot1':[(0.,0.),(1.,0.)]})
        message = SimpleNamespace(poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=9.,y=9.)))])
        Exploration.receive_plan(agent,'robot1',message)
        self.assertEqual(agent.plans,{})
        agent.phase = 'MOVING'
        Exploration.receive_plan(agent,'robot1',message)
        self.assertEqual(agent.plans,{})
        message.poses[0].pose.position.x,message.poses[0].pose.position.y = 1.,0.
        Exploration.receive_plan(agent,'robot1',message)
        self.assertEqual(agent.plans['robot1'],[(1.,0.)])
        self.assertEqual(agent.validated_plans['robot1'],[(0.,0.),(1.,0.)])

    def test_legal_current_link_may_recover_its_planning_margin(self):
        cells = np.full((200,200),-1,dtype=np.int16)
        cells[10:190,10:190] = 0
        positions = dict(zip(ROBOTS,[(-8.,-7.6),(-8.,-3.4),(-4.,-3.4),(-4.,.9)]))
        self.assertTrue(connected(positions,4.5))
        self.assertFalse(connected(positions,4.2))
        result = allocate(cells,.1,(-12.,-12.),positions,positions,dict.fromkeys(ROBOTS,0.),[],[],
                          radio_range=4.5,budget=100)
        self.assertIn('tasks',result,result)
        self.assertTrue(connected({r:t['goal'] for r,t in result['tasks'].items()},4.2))

    def test_short_relay_goal_must_remain_sunlit_for_long_team_move(self):
        masks = np.zeros((181,147,147),dtype=np.uint8)
        row = col = int(11/.15)
        masks[30:50,row-2:row+3,col-2:col+3] = 1
        forecast = Forecast(encode(masks,0.,sun_period_s=240.))
        self.assertTrue(forecast.route_safe([(0.,0.)],0.,hold=25))
        self.assertFalse(forecast.route_safe([(0.,0.)],0.,hold=25,team_horizon=90))
        windows = forecast.departure_constraints([(0.,0.)],0.,5.)
        first = next(w for w in windows if w[0]==0)
        self.assertLessEqual(5+first[1]*5,40)
        self.assertGreaterEqual(5+first[2]*5,40)
        self.assertLess(5+first[2]*5,90)

    def test_cp_sat_rejects_disjoint_team_finish_windows(self):
        class Windows:
            def departure_delay(self,*args,**kwargs):return 0.
            def departure_constraints(self,route,now,base,hold=15,**kwargs):
                return [(0,4,6)] if route[0][0]>-7 else [(0,8,12)]
        cells = np.full((160,160),-1,dtype=np.int16)
        cells[10:140,10:140] = 0
        homes = dict(zip(ROBOTS,[(-8.,-7.6),(-8.,-6.2),(-6.6,-7.6),(-6.6,-6.2)]))
        result = allocate(cells,.1,(-12.,-12.),homes,homes,dict.fromkeys(ROBOTS,0.),[],[],
                          radio_range=4.5,budget=40,forecast=Windows())
        self.assertEqual(result['status'],'INFEASIBLE')


if __name__ == '__main__':
    unittest.main()
