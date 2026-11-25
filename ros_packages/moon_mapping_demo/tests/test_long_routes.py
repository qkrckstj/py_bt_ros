"""긴 탐사 목표의 지형 검사·실제 관측 보존·계산 예산 복귀 계약."""
from dataclasses import replace
import math
from pathlib import Path
import tempfile
import threading
import numpy as np
from types import SimpleNamespace
from unittest.mock import patch

from moon_mapping_demo.exploration_planning import Candidate, plan_batch
from moon_mapping_demo.observation import EvaluationArea, ObservationGrid
from moon_mapping_demo.space_time import PlannerConfig, SpaceTimePlanner
from moon_mapping_demo.work_regions import corridor_candidates
from moon_mapping_demo.risk import RiskPolicy, prognosis
from moon_mapping_demo.space_time import TimedPath, TimedPoint
from synthetic_mesh_scan import simulated_mesh_scan


class OpenField:
    bounds = (-30., 30., -30., 30.)

    def with_static_robots(self, robots):
        assert not robots
        return self

    def static_blocked_segment(self, a, b, radius):
        return any(not (-30.+radius <= v <= 30.-radius) for p in (a,b) for v in p)

    def blocked_segment(self, a, b, ta, tb, radius):
        return self.static_blocked_segment(a,b,radius)


def example():
    poses={'robot1':(0.,0.,0.)}
    observation=ObservationGrid(EvaluationArea(-20.,-20.,20.,20.,.25),poses,4.)
    observation.integrate('robot1',1.,poses['robot1'],[math.inf]*72,-math.pi,2*math.pi/72,.01,6.)
    goal=Candidate('frontier',3.,0.,work_region='east',preferred_robot='robot1')
    settings={'long_route_distance_m':12.,'fallback_target_step_m':4.,'candidate_limit':12}
    return observation,poses,goal,settings


def test_long_goal_extends_beyond_scan_without_creating_observations():
    observation,poses,goal,settings=example()
    before=observation.cells.copy()
    area=observation.observed_area_m2
    goals=corridor_candidates(observation,poses,OpenField(),.22,settings,[goal],{},())
    long=next(g for g in goals if g.route_kind=='corridor')
    assert math.dist(long.xy,poses['robot1'][:2])==12.
    assert observation.cells[observation.area.index(long.xy)]==-1
    assert goal in goals
    assert long.work_region==goal.work_region
    assert (observation.cells==before).all()
    assert observation.observed_area_m2==area


def test_clear_endpoint_does_not_allow_long_segment_through_wall():
    class Wall(OpenField):
        def static_blocked_segment(self,a,b,radius):
            return super().static_blocked_segment(a,b,radius) or min(a[0],b[0])-radius<=5.<=max(a[0],b[0])+radius
    observation,poses,goal,settings=example()
    assert not Wall().static_blocked_segment((12.,0.),(12.,0.),.22)
    goals=corridor_candidates(observation,poses,Wall(),.22,settings,[goal],{},())
    assert goals==[goal]


def test_long_goal_avoids_peer_body_and_keeps_short_alternative():
    observation,poses,goal,settings=example()
    poses['robot2']=(12.,0.,0.)
    goals=corridor_candidates(observation,poses,OpenField(),.22,settings,[goal],{},())
    long=next(g for g in goals if g.route_kind=='corridor')
    assert 6.<long.x<12.
    assert math.dist(long.xy,poses['robot2'][:2])>2*.22+.1
    assert goal in goals


def test_long_search_failure_uses_only_remaining_shared_budget():
    _,poses,goal,_=example()
    long=replace(goal,id='corridor',x=12.,work_region='',route_kind='corridor')
    short=replace(goal,work_region='')
    calls=[]
    def worker(*args):
        calls.append(args)
        return {'status':'PLANNING_BUDGET','attempts':[]} if len(calls)==1 else {'status':'OK','paths':{'verified':object()}}
    with tempfile.TemporaryDirectory() as out, \
            patch('moon_mapping_demo.exploration_planning._plan_batch',side_effect=worker), \
            patch('moon_mapping_demo.exploration_planning.time',SimpleNamespace(monotonic=iter([10.,11.]).__next__)):
        value=plan_batch(OpenField(),PlannerConfig(),poses,[long,short],1.,0.,2.,list(poses),
            Path(out),threading.Event(),{'long_routes':True,'planning_wall_budget_seconds':8.,
            'planning_result_margin_seconds':.4,'max_path_seconds':90.,'fallback_max_path_seconds':45.})
    assert value['status']=='OK'
    assert value['route_phase']=='short_fallback'
    assert calls[0][3]==[long,short] and calls[1][3]==[short]
    assert abs(calls[0][10]['planning_wall_budget_seconds']-4.56)<1e-9
    assert abs(calls[1][10]['planning_wall_budget_seconds']-6.6)<1e-9
    assert calls[1][10]['max_path_seconds']==45.


def test_cancelled_long_search_never_starts_short_fallback():
    _,poses,goal,_=example()
    long=replace(goal,id='corridor',work_region='',route_kind='corridor')
    with tempfile.TemporaryDirectory() as out, patch(
            'moon_mapping_demo.exploration_planning._plan_batch',return_value={'status':'CANCELLED'}) as worker:
        value=plan_batch(OpenField(),PlannerConfig(),poses,[long,replace(goal,work_region='')],
            1.,0.,2.,list(poses),Path(out),threading.Event(),{'long_routes':True})
    assert value['status']=='CANCELLED'
    assert worker.call_count==1


def test_cancel_after_long_result_discards_paths():
    _,poses,goal,_=example()
    event=threading.Event()
    def worker(*args):
        event.set()
        return {'status':'OK','paths':{'robot1':object()}}
    with tempfile.TemporaryDirectory() as out, patch(
            'moon_mapping_demo.exploration_planning._plan_batch',side_effect=worker):
        value=plan_batch(OpenField(),PlannerConfig(),poses,
            [replace(goal,work_region='',route_kind='corridor')],1.,0.,2.,list(poses),
            Path(out),event,{'long_routes':True})
    assert value['status']=='CANCELLED'
    assert 'paths' not in value


def test_mesh_sensor_measures_actual_vertical_triangle_at_lidar_height():
    field=SimpleNamespace(mesh_triangles=np.asarray([[[2.,-1.,0.],[2.,1.,0.],[2.,0.,1.]]]),
        boundary_boxes=np.empty((0,4)))
    origin,ranges,_,_=simulated_mesh_scan(field,'robot1',{'robot1':(0.,0.,0.)},rays=3)
    assert abs(ranges[1]-(2.-origin[0]))<1e-9
    assert math.isinf(ranges[0]) and math.isinf(ranges[2])


def test_mesh_sensor_keeps_clear_rays_when_old_box_covers_robot():
    field=SimpleNamespace(mesh_triangles=np.asarray([[[0.,0.,0.],[2.,0.,0.],[0.,2.,0.]]]),
        boundary_boxes=np.empty((0,4)),obstacle_boxes=np.asarray([[-1.,-1.,1.,1.]]))
    _,ranges,_,_=simulated_mesh_scan(field,'robot1',{'robot1':(0.,0.,0.)},rays=3)
    assert all(math.isinf(r) for r in ranges)


def test_long_path_pays_full_travel_time_and_observation_hold():
    poses={'robot1':(0.,0.,0.)}
    pcfg=PlannerConfig(resolution=.5,time_step=.5,speed=.18,robot_radius=.22,horizon=90.,direct_goal_links=True)
    goal=Candidate('long',12.,0.,route_kind='corridor')
    with tempfile.TemporaryDirectory() as out:
        value=plan_batch(OpenField(),pcfg,poses,[goal],10.,1.,4.,list(poses),Path(out),threading.Event(),
            {'long_routes':True,'max_path_seconds':90.,'turn_aware_coordination':True,
             'risk':{'enabled':True,'safe_goal_windows':True}},True)
    assert value['status']=='OK',value
    path=value['paths']['robot1']
    assert path.points[-1].t-path.points[0].t>=12./.18
    assert (path.points[-1].x,path.points[-1].y)==goal.xy
    assert value['execution_hold_seconds']==4.


def test_straight_graph_waits_for_safe_time_instead_of_crossing_shadow():
    class Shadow(OpenField):
        def blocked_segment(self,a,b,ta,tb,radius):
            return super().blocked_segment(a,b,ta,tb,radius) or (a!=b and ta<5.)
    planner=SpaceTimePlanner(Shadow(),PlannerConfig(speed=1.,time_step=.5,horizon=15.,straight_only=True))
    path=planner.plan((0.,0.),(6.,0.),1.,initial_yaw=0.,goal_hold=2.)
    assert path is not None
    assert path.points[-1].t>=11.
    assert all(not Shadow().blocked_segment((a.x,a.y),(b.x,b.y),a.t,b.t,.24)
               for a,b in zip(path.points,path.points[1:]))


def test_straight_graph_rejects_blocked_line_and_full_graph_keeps_detour():
    class Obstacle(OpenField):
        def blocked_segment(self,a,b,ta,tb,radius):
            return super().blocked_segment(a,b,ta,tb,radius) or (
                min(a[0],b[0])<3.<max(a[0],b[0]) and max(abs(a[1]),abs(b[1]))<1.)
    config=PlannerConfig(speed=1.,time_step=.5,horizon=15.,direct_goal_links=True)
    assert SpaceTimePlanner(Obstacle(),replace(config,straight_only=True)).plan((0.,0.),(6.,0.),1.) is None
    assert SpaceTimePlanner(Obstacle(),config).plan((0.,0.),(6.,0.),1.) is not None


def test_risk_uses_body_radius_in_motion_and_keeps_extra_goal_margin():
    class MarginBand(OpenField):
        def __init__(self,at_goal=False):self.at_goal=at_goal
        def blocked_segment(self,a,b,ta,tb,radius):
            return radius>.28 and ((a==b and ta>=20.) if self.at_goal else a!=b)
    path=TimedPath((TimedPoint(0.,0.,1.),TimedPoint(3.,0.,20.)),19)
    poses={'robot1':(0.,0.,0.)}
    cfg=PlannerConfig(robot_radius=.26)
    policy=RiskPolicy(enabled=True,warning_position_margin_m=.05)
    moving=prognosis(MarginBand(),{'robot1':path},poses,1.,cfg,policy)
    assert moving['robots']['robot1']['planned_warning'] is None
    goal=prognosis(MarginBand(True),{'robot1':path},poses,1.,cfg,policy)
    assert goal['robots']['robot1']['planned_warning']['kind']=='observation_or_followup_hold'
