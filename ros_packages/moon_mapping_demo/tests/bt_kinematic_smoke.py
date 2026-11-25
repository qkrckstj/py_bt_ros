#!/usr/bin/env python3
"""실제 WBT 계획 + 고정 버전 BT + 이상 차동구동 및 모의 거리 센서 검사.

합성 스캔은 WBT 정적 기둥과 동료 원판을 광선 추적한다. 실제 Webots 센서,
ROS TF/DDS, 물리·미끄러짐을 검증하지 않는다. 계산 중 sim 시간이 멈추는 모델이다.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
import numpy as np

from moon_mapping_demo.coordination import moving_discs_collide
from moon_mapping_demo.experiment import load_experiment, make_field, path_data
from moon_mapping_demo.exploration import Exploration
from moon_mapping_demo.metrics import atomic_json
from moon_mapping_demo.risk import RiskPolicy
from test_exploration import ImmediatePool, integrate


def simulated_scan(field, robot, poses, max_range=6., rays=72):
    x,y,yaw=poses[robot]
    x,y=x+.103*math.cos(yaw),y+.103*math.sin(yaw)
    fov=math.radians(220.)
    angles=yaw-fov/2+np.arange(rays)*fov/(rays-1)
    directions=np.stack((np.cos(angles),np.sin(angles)),axis=1)
    boxes=np.concatenate((field.obstacle_boxes,field.boundary_boxes))
    boxes=boxes[np.all(boxes[:,2:] >= np.asarray([x,y])-max_range,axis=1) &
                np.all(boxes[:,:2] <= np.asarray([x,y])+max_range,axis=1)]
    with np.errstate(divide='ignore',invalid='ignore'):
        lo=(boxes[None,:,:2]-[x,y])/directions[:,None,:]
        hi=(boxes[None,:,2:]-[x,y])/directions[:,None,:]
        entry=np.maximum(0.,np.min(np.stack((lo,hi)),axis=0).max(axis=2))
        exit=np.max(np.stack((lo,hi)),axis=0).min(axis=2)
        ranges=np.min(np.where(exit>=entry,entry,np.inf),axis=1,initial=np.inf)
    for r,p in poses.items():
        if r==robot: continue
        offset=np.asarray(p[:2])-[x,y]
        projected=directions@offset
        discriminant=.26**2-(offset@offset-projected**2)
        hits=np.where((discriminant>=0)&(projected>=0),projected-np.sqrt(np.maximum(0,discriminant)),np.inf)
        ranges=np.minimum(ranges,hits)
    ranges=np.where(ranges<=max_range,ranges,np.inf)
    return (x,y,yaw),ranges.tolist(),-fov/2,fov/(rays-1)


class SimulationDelayedPool:
    """계산은 즉시 수행하되 결과 공개를 sim 시간만큼 늦추는 지연 주입."""
    def __init__(self,delay): self.now,self.delay=1.,delay
    def submit(self,function,*args):
        from concurrent.futures import Future
        value=Future()
        try:value.set_result(function(*args))
        except Exception as exc:value.set_exception(exc)
        pool=self;release=self.now+self.delay
        class Delayed:
            def __init__(self):self.cancelled=False
            def done(self):return self.cancelled or pool.now>=release
            def result(self):return value.result()
            def cancel(self):self.cancelled=True;return True
        return Delayed()


def run(config, output, mode='fixed', duration=None, departure_policy=None,risk_stage=None,planning_delay_sim=0.,sensor_geometry='legacy_columns'):
    before=time.monotonic()
    cfg,env,pcfg,share=load_experiment(config)
    if (Path(output)/'bt_kinematic_validation.json').exists():
        raise FileExistsError('기존 실험 결과를 보존하려면 새 output 폴더를 사용하세요.')
    if departure_policy:
        cfg['departure']={**cfg.get('departure',{}),'mode':departure_policy}
    if risk_stage is not None:
        cfg['risk']=RiskPolicy.stage(cfg.get('risk'),risk_stage)
    if duration:
        cfg['exploration']['max_sim_seconds']=duration
    field=make_field(cfg,env,pcfg,share)
    sensor=simulated_scan
    if sensor_geometry=='mesh':
        from synthetic_mesh_scan import simulated_mesh_scan
        sensor=simulated_mesh_scan
    poses={r:tuple(s[k] for k in ('x','y','yaw')) for r,s in env['starts'].items()}
    pool=SimulationDelayedPool(planning_delay_sim) if planning_delay_sim else ImmediatePool()
    engine=Exploration(cfg,field,pcfg,output,mode,pool)
    engine.start(1.,time.monotonic(),poses)
    engine.metrics.metadata.update(synthetic_sensor=True, physical_webots_run=False, planning_sim_clock_paused=True)
    engine.metrics.metadata['injected_planning_delay_sim_seconds']=planning_delay_sim
    engine.metrics.metadata['synthetic_sensor_geometry']=sensor_geometry
    dt=cfg['execution']['control_period']
    minimum=math.inf
    dispersion_samples=[]
    error=None
    for step in range(math.ceil(cfg['exploration']['max_sim_seconds']/dt)+2):
        now=1.+step*dt
        if planning_delay_sim:pool.now=now
        if step%5==0:
            for robot in poses:
                pose,ranges,amin,inc=sensor(field,robot,poses)
                engine.observation.integrate(robot,now,pose,ranges,amin,inc,.1,8.)
        for robot,p in poses.items():
            engine.metrics.pose(robot,now,p[:2])
        before_paths,before_plan,before_replans=dict(engine.paths),engine.plan_id,engine.metrics.replans
        controls=engine.step(now,time.monotonic(),poses)
        if before_paths and engine.metrics.replans>before_replans:
            atomic_json(engine.output/f'replan_state_{before_plan:04d}.json',
                {'sim_time':now,'plan_id':before_plan,'poses':poses,'paths':path_data(before_paths),
                 'settings':dict(engine.settings),'event':next((e for e in reversed(engine.metrics.events) if e['event']=='replan'),None)})
        after={r:integrate(p,controls[r],dt) for r,p in poses.items()}
        for robot,p in poses.items():
            c=controls[robot]
            margin=abs(c.linear*c.angular)*dt*dt/2
            if field.blocked_segment(p[:2],after[robot][:2],now,now+dt,pcfg.robot_radius+margin):
                error=robot+': 적분 선분이 금지 영역에 진입'
        pair_distances=[]
        for i,a in enumerate(poses):
            for b in list(poses)[i+1:]:
                distance=math.dist(after[a][:2],after[b][:2])
                pair_distances.append(distance)
                minimum=min(minimum,distance)
                if moving_discs_collide(poses[a][:2],after[a][:2],poses[b][:2],after[b][:2],2*pcfg.robot_radius):
                    error=f'{a}/{b}: 적분 구간 충돌'
        poses=after
        if step%10==0 or not engine.active:
            dispersion_samples.append({'elapsed_sim_seconds':now-engine.metrics.started_sim,
                'mean_pair_distance_m':sum(pair_distances)/len(pair_distances),
                'minimum_pair_distance_m':min(pair_distances),'maximum_pair_distance_m':max(pair_distances),
                'work_regions':dict(getattr(engine,'work_regions',{}))})
        if error:
            engine.terminate(error,'ERROR')
        if not engine.active:
            break
        if step%100==0:
            engine.persist()
    dispatches=[e for e in engine.metrics.events if e['event']=='dispatch']
    passed=not error and engine.phase in ('COMPLETE','PARTIAL_COMPLETE')
    if mode=='fixed':
        passed=passed and engine.metrics.termination_reason=='fixed_goals_observed'
    else:
        passed=passed and engine.metrics.rounds_completed>=2
    report={'kind':'pinned_BT_WBT_ideal_kinematics_and_synthetic_raycast',
        'passed':passed,'physical_webots_run':False,'real_sensor_run':False,'ros_transport_tested':False,
        'planning_sim_clock_paused':True,'mode':mode,'phase':engine.phase,'reason':engine.reason,
        'sim_time':engine.now,'elapsed_wall_seconds':time.monotonic()-before,
        'rounds_completed':engine.metrics.rounds_completed,'replans':engine.metrics.replans,
        'dispatch_count':engine.dispatch_count,'minimum_sampled_distance_m':minimum,
        'observation':engine.observation.snapshot(),'dispatches':dispatches,
        'history':engine.team.history,'last_poses':poses,'session_directory':str(engine.output)}
    report['dispersion_samples']=dispersion_samples
    report['departure_policy']=engine.departure.mode
    report['risk_stage']=risk_stage
    report['risk_policy']=asdict(engine.risk)
    report['injected_planning_delay_sim_seconds']=planning_delay_sim
    report['synthetic_sensor_geometry']=sensor_geometry
    report['departures']=engine.metrics.departures
    report['plan_costs']=engine.metrics.plan_costs
    engine.close()
    atomic_json(Path(output)/'bt_kinematic_validation.json',report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('history','dispatches','last_poses','departures','plan_costs','dispersion_samples')},ensure_ascii=False,indent=2))
    return passed


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['fixed','explore'],default='fixed')
    parser.add_argument('--config',type=Path,default=Path(__file__).resolve().parents[1]/'config/exploration_short.yaml')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float)
    parser.add_argument('--departure-policy',choices=['simultaneous','staggered_pairs'])
    parser.add_argument('--risk-stage',choices=['baseline','early','goals','candidates','heading'])
    parser.add_argument('--planning-delay-sim',type=float,default=0.)
    parser.add_argument('--sensor-geometry',choices=['legacy_columns','mesh'],default='legacy_columns')
    args=parser.parse_args()
    raise SystemExit(0 if run(args.config,args.output,args.mode,args.duration,args.departure_policy,args.risk_stage,args.planning_delay_sim,args.sensor_geometry) else 2)
