"""Regression tests for preserving active goals during asynchronous renewal."""
import math
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from builtin_interfaces.msg import Time

from planner import ROBOTS, allocate
from rolling_dispatch import RollingExploration
from shadow_prediction import Forecast, encode, navigation_mask


class RollingTests(unittest.TestCase):
    def test_short_legs_keep_useful_global_target_instead_of_bouncing_back(self):
        cells = np.full((80,80),-1,dtype=np.int16)
        cells[8:72,8:72] = 0
        positions = dict(zip(ROBOTS,[(-1.5,-1.5),(-1.5,0.),(0.,-1.5),(0.,0.)]))
        preferred = {'robot1':(-1.5,-4.9)}
        result = allocate(cells,.2,(-8.,-8.),positions,positions,dict.fromkeys(ROBOTS,0.),[],[],
                          radio_range=10.,budget=100,max_leg=2.,maximum_explorers=4,
                          preferred_goals=preferred,recent_targets={'robot1':[(-3.5,-1.5)]},
                          movement_reward=1000)
        self.assertIn('tasks',result,result)
        task = result['tasks']['robot1']
        self.assertEqual(task['role'],'explorer')
        self.assertLess(math.dist(task['global_goal'],preferred['robot1']),.2)
        self.assertLess(math.dist(task['goal'],preferred['robot1']),
                        math.dist(positions['robot1'],preferred['robot1']))

    def test_startup_cancels_orphaned_nav2_goals_before_reusing_servers(self):
        clients = {r:Mock() for r in ROBOTS}
        futures = {}
        for r,client in clients.items():
            client.service_is_ready.return_value = True
            future = Mock()
            future.done.return_value = False
            future.result.return_value = SimpleNamespace(return_code=0)
            client.call_async.return_value = future
            futures[r] = future
        agent = SimpleNamespace(cleanup_complete=False,cleanup_futures=None,
                                cleanup_clients=clients,heartbeat=Mock(),halt=Mock(),get_logger=Mock())
        self.assertFalse(RollingExploration.clear_orphan_goals(agent))
        self.assertFalse(RollingExploration.clear_orphan_goals(agent))
        self.assertFalse(agent.cleanup_complete)
        for future in futures.values():future.done.return_value = True
        self.assertTrue(RollingExploration.clear_orphan_goals(agent))
        self.assertTrue(agent.cleanup_complete)
        for client in clients.values():client.call_async.assert_called_once()

    def test_nav2_mask_does_not_turn_later_shadow_into_a_current_wall(self):
        masks = np.zeros((20,30,30),dtype=np.uint8)
        masks[4:,15,15] = 1
        # Eight seconds from now is handled by temporal route checks, not a
        # current static wall that prevents an earlier safe departure.
        self.assertEqual(navigation_mask(masks)[15,15],0)
        forecast = Forecast(encode(masks,0.,origin=(-1.5,-1.5),resolution=.1))
        self.assertTrue(forecast.safe((.05,.05),0.))
        self.assertFalse(forecast.hold_safe((.05,.05),0.,10.))
        masks[1,15,15] = 1
        self.assertEqual(navigation_mask(masks)[15,15],100)

    def test_forecast_includes_nav2_footprint_clearance(self):
        masks = np.zeros((20,30,30),dtype=np.uint8)
        masks[:,15,18] = 1
        payload = encode(masks,0.,origin=(-1.5,-1.5),resolution=.1)
        raw = Forecast(payload)
        navigation = Forecast(payload,navigation_clearance_m=.35)
        self.assertTrue(raw.safe((.05,.05),0.))
        self.assertFalse(navigation.safe((.05,.05),0.))
        self.assertTrue(navigation.safe((-.65,.05),0.))

    def test_clearance_does_not_round_35cm_up_to_45cm(self):
        masks = np.zeros((20,30,30),dtype=np.uint8)
        masks[:,15,18] = 1
        f = Forecast(encode(masks,0.,origin=(-2.25,-2.25),resolution=.15),navigation_clearance_m=.35)
        self.assertTrue(f.safe((.075,.075),0.))  # obstacle center .45m away
        self.assertFalse(f.safe((.225,.075),0.))  # obstacle center .30m away

    def test_window_search_only_checks_executable_departure_times(self):
        safe = Mock(return_value=True)
        agent = SimpleNamespace(payload={'sun_period_s':240.},
                                hold_safe=lambda *args:True,route_safe=safe)
        windows = Forecast.departure_windows(agent,[(0.,0.),(1.,0.)],0.,1.,maximum_wait=10.)
        self.assertEqual(windows,[0,1])
        self.assertEqual(safe.call_count,2)

    def test_lunar_navigation_avoids_spin_and_wait_recovery_loops(self):
        root = ET.parse(Path(__file__).with_name('rolling_navigation.xml')).getroot()
        tags = {element.tag for element in root.iter()}
        self.assertTrue({'ComputePathToPose','FollowPath','ClearEntireCostmap'}<=tags)
        self.assertFalse({'Spin','Wait','BackUp'}&tags)
        self.assertEqual(root.find('.//RecoveryNode').attrib['number_of_retries'],'1')

    def test_allocation_keeps_active_goals_exactly(self):
        cells = np.full((80,80),-1,dtype=np.int16)
        cells[8:72,8:72] = 0
        positions = dict(zip(ROBOTS,[(-1.5,-1.5),(-1.5,0.),(0.,-1.5),(0.,0.)]))
        fixed = {'robot2':{'role':'explorer','goal':(-1.5,1.),'grid_distance_m':1.,
                           'return_distance_m':1.,'global_goal':(-1.5,4.)},
                 'robot4':{'role':'relay','goal':(0.,0.),'grid_distance_m':0.,
                           'return_distance_m':0.}}
        result = allocate(cells,.2,(-8.,-8.),positions,positions,dict.fromkeys(ROBOTS,0.),[],[],
                          radio_range=10.,budget=100,max_leg=2.,maximum_explorers=4,
                          fixed_tasks=fixed,movement_reward=1000)
        self.assertIn('tasks',result,result)
        for r,task in fixed.items():
            self.assertEqual(result['tasks'][r],task)
        self.assertTrue(any(math.dist(positions[r],result['tasks'][r]['goal'])>=.3
                            for r in ('robot1','robot3')))

    def test_finished_robot_is_eligible_while_peer_still_far_from_goal(self):
        positions = dict(zip(ROBOTS,[(0.,0.),(2.,0.),(4.,0.),(6.,0.)]))
        agent = SimpleNamespace(pending={},sending=set(),canceling=set(),cooldown={},
                                shadows={'robot4':True},forecast=SimpleNamespace(hold_safe=lambda *args:True),
                                now=lambda:10.,handles={'robot2':Mock(),'robot3':Mock()},
                                tasks={'robot2':{'goal':(5.,0.)},'robot3':{'goal':(4.5,0.)}})
        self.assertEqual(RollingExploration.eligible(agent,positions),{'robot1','robot3'})

    def test_dispatch_does_not_wait_for_peer_arrival_or_cancel_peer(self):
        positions = dict(zip(ROBOTS,[(0.,0.),(2.,0.),(4.,0.),(6.,0.)]))
        peer = Mock()
        client = Mock()
        pending = {'task':{'role':'explorer','goal':(0.,1.)},'anchor':(0.,0.),
                   'stamp':10.,'departure_at':10.,'checking':False,
                   'route':[(0.,0.),(0.,1.)]}
        clock = Mock()
        clock.now.return_value.to_msg.return_value = Time(sec=11)
        agent = SimpleNamespace(pending={'robot1':pending},handles={'robot2':peer},
                                sending=set(),canceling=set(),shadows={},now=lambda:11.,
                                forecast=SimpleNamespace(route_safe=lambda *args,**kwargs:True),
                                plans={'robot2':[(2.,0.),(3.,0.)]},radio_range=10.,
                                tasks={'robot2':{'goal':(3.,0.)}},tokens=dict.fromkeys(ROBOTS,0),
                                nav_clients={'robot1':client},get_clock=lambda:clock,reasons={},
                                get_logger=Mock(),accepted_rolling=Mock(),reject_pending=Mock())
        RollingExploration.dispatch_tick(agent,positions)
        client.send_goal_async.assert_called_once()
        self.assertIn('robot2',agent.handles)
        self.assertEqual(agent.tasks['robot2']['goal'],(3.,0.))
        peer.cancel_goal_async.assert_not_called()
        self.assertEqual(agent.sending,{'robot1'})
        self.assertNotIn('robot1',agent.pending)

    def test_path_rejection_only_removes_its_own_pending_goal(self):
        p = {'task':{'goal':(1.,0.)}}
        peer = Mock()
        agent = SimpleNamespace(pending={'robot1':p,'robot2':{'route':[(2.,0.),(3.,0.)]}},
                                rejected=[],cooldown={},reasons={},now=lambda:10.,
                                handles={'robot3':peer},get_logger=Mock())
        RollingExploration.reject_pending(agent,'robot1',p,'blocked')
        self.assertIn('robot2',agent.pending)
        self.assertIn('robot3',agent.handles)
        peer.cancel_goal_async.assert_not_called()

    def test_planning_in_background_does_not_revoke_motion_permits(self):
        future = Mock()
        future.done.return_value = False
        agent = SimpleNamespace(job={'stage':'planning','future':future},allowed={'robot2'},
                                handles={'robot2':Mock()})
        RollingExploration.job_tick(agent,{})
        self.assertEqual(agent.allowed,{'robot2'})

    def test_late_result_cannot_remove_newer_goal(self):
        peer = Mock()
        future = Mock()
        agent = SimpleNamespace(tokens={'robot1':2},handles={'robot1':peer})
        RollingExploration.finished_rolling(agent,'robot1',1,future)
        self.assertIs(agent.handles['robot1'],peer)
        future.result.assert_not_called()


if __name__=='__main__':
    unittest.main()
