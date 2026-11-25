#!/usr/bin/env python3
"""합성 clock/odom/TF/거리 센서로 전용 BT 통합 노드의 ROS 계약을 검사한다.

같은 rclpy context의 프로세스 내 검사이며 물리 주행·실제 센서·Jazzy 검증은 아니다.
"""
import argparse
import json
from pathlib import Path
import tempfile
import time
import math
import yaml

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from moon_mapping_demo.exploration_ros import ExplorationMission
from moon_mapping_demo.shadow_mission import stamp
from moon_mapping_demo.metrics import atomic_json
from ros_smoke import Probe


def run(case, root, debug=False, departure_policy='simultaneous',risk_stage=''):
    share=Path(__file__).resolve().parents[1]
    cfg=yaml.safe_load((share/'config/exploration_short.yaml').read_text())
    cfg['execution']['startup_stable_seconds']=.2
    cfg['exploration']['bootstrap_seconds']=.5
    config=root/(case+'.yaml')
    config.write_text(yaml.safe_dump(cfg))
    out=root/case
    ros_args=['--ros-args','-p','use_sim_time:=true','-p','execute:=true',
                    '-p','mode:=fixed','-p',f'config:={config}','-p',f'output:={out}',
                    '-p',f'departure_policy:={departure_policy}']
    if risk_stage:ros_args.extend(['-p',f'risk_stage:={risk_stage}'])
    rclpy.init(args=ros_args)
    mission,probe,ex=None,None,None
    try:
        mission=ExplorationMission()
        counts={r:0 for r in mission.robots}
        original_scan=mission.scan
        def count_scan(robot,msg):
            counts[robot]+=1
            original_scan(robot,msg)
        mission.scan=count_scan
        probe=Probe(mission.all_robots)
        tf=TransformBroadcaster(probe)
        static=StaticTransformBroadcaster(probe)
        transforms=[]
        for r,s in mission.env['starts'].items():
            anchor=TransformStamped()
            anchor.header.frame_id='map'; anchor.child_frame_id=f'{r}/odom'
            anchor.transform.translation.x=float(s['x']); anchor.transform.translation.y=float(s['y'])
            anchor.transform.rotation.w=1.
            laser=TransformStamped()
            laser.header.frame_id=f'{r}/base_link'; laser.child_frame_id=f'{r}/laser_link'
            laser.transform.translation.x=.103; laser.transform.translation.z=-.034
            laser.transform.rotation.w=1.
            transforms.extend([anchor,laser])
        static.sendTransform(transforms)
        pubs={r:probe.create_publisher(LaserScan,f'/{r}/scan_observation',qos_profile_sensor_data) for r in mission.robots}
        maps=[]; states={}; btmarkers=[]
        probe.create_subscription(OccupancyGrid,'/exploration/observations',maps.append,probe.transient)
        for r in mission.robots:
            probe.create_subscription(String,f'/{r}/exploration/state',lambda msg,r=r:states.update({r:json.loads(msg.data)}),probe.transient)
        from visualization_msgs.msg import MarkerArray
        probe.create_subscription(MarkerArray,'/exploration/bt_markers',btmarkers.append,probe.transient)
        ex=SingleThreadedExecutor(); ex.add_node(mission); ex.add_node(probe)
        before=time.monotonic(); last=0.; injected=None; late=False; stop_future=None; conflict=None; last_debug=0.
        valid_since=None
        while time.monotonic()-before<15.:
            wall=time.monotonic(); sim=1.+wall-before
            if injected and case=='clock_backward': sim=.25+wall-injected
            if wall-last>=.04:
                probe.publish_inputs(sim,('robot4',) if injected and case=='robot4_odom_stale' else ())
                dynamic=[]
                for r in mission.robots:
                    tr=TransformStamped(); tr.header.frame_id=f'{r}/odom'; tr.child_frame_id=f'{r}/base_link'
                    tr.header.stamp=stamp(sim); tr.transform.rotation.w=1.
                    if not (case=='missing_acquisition_tf' and r=='robot4'):
                        dynamic.append(tr)
                    if not (injected and case=='robot4_scan_stale' and r=='robot4'):
                        scan=LaserScan(); scan.header.frame_id=f'{r}/laser_link'; scan.header.stamp=stamp(sim)
                        scan.angle_min=-math.radians(110.); scan.angle_increment=math.radians(220.)/71
                        scan.angle_max=-scan.angle_min; scan.range_min=.1; scan.range_max=8.
                        scan.ranges=[math.inf]*72
                        pubs[r].publish(scan)
                tf.sendTransform(dynamic); last=wall
            ex.spin_once(timeout_sec=.005)
            e=mission.engine
            if debug and wall-last_debug>.5:
                print({'wall_elapsed':wall-before,'sim_time':e.now,'phase':e.phase,'received':counts,
                    'valid':e.observation.valid_frames,'pending':len(mission.pending_scans),
                    'drops':mission.scan_drops,'future_done':e.future.done() if e.future else None},flush=True)
                last_debug=wall
            if e.phase=='EXECUTING' and not late:
                probe.subscribe_late(mission.robots); late=True
            linked=(e.phase=='EXECUTING' and len(probe.paths)==4 and maps and btmarkers and len(states)==4
                    and probe.heartbeat is False and len(e.started)==4)
            if linked and departure_policy=='staggered_pairs':
                # 경로보다 1Hz 상태·마커가 늦게 도착할 수 있다. 같은 실행
                # 단계의 출발 대기 표시를 받은 뒤 연결 완료를 판정한다.
                linked=(any('출발 대기' in m.text for m in btmarkers[-1].markers) and
                    all(states[r]['status']=='WAITING_DEPARTURE' for r in e.batch['departure']['follow_robots']))
            if linked and injected is None:
                injected=wall
                if case=='user_stop': stop_future=probe.stop_client.call_async(Trigger.Request())
                if case=='command_conflict': conflict=probe.create_publisher(Twist,'/robot4/cmd_vel',10)
            if case=='missing_acquisition_tf':
                success=e.phase=='ERROR' and e.observation.valid_frames['robot4']==0 and e.observation.observed_count>0
            elif case=='connection':
                success=bool(linked)
            elif case=='user_stop':
                success=bool(stop_future and stop_future.done() and stop_future.result().success and e.phase=='STOPPED')
            else:
                success=injected is not None and e.phase=='ERROR'
            if success and (case=='connection' or probe.stopped(mission.robots)):
                valid_since=valid_since or wall
                if wall-valid_since>.15: break
            else: valid_since=None
        else:
            raise AssertionError(f'{case} 시간 초과: {e.snapshot()}')
        if case=='connection':
            assert all(len(path.poses)>1 for path in probe.paths.values())
            t0={path.poses[0].header.stamp.sec+path.poses[0].header.stamp.nanosec/1e9 for path in probe.paths.values()}
            assert len(t0)==1
            assert maps[-1].info.width==80 and maps[-1].info.height==80
            assert sum(v>=0 for v in maps[-1].data)>0
            assert len(btmarkers[-1].markers)==5
            assert set(states)==set(mission.robots)
            if departure_policy=='staggered_pairs':
                assert len(e.batch['assigned'])==4
                followers=e.batch['departure']['follow_robots']
                assert len(followers)==2
                assert all(e.robot_bt[r].status=='WAITING_DEPARTURE' for r in followers)
                assert any('출발 대기' in m.text for m in btmarkers[-1].markers)
        if case=='command_conflict': assert 'cmd_vel' in e.reason
        if case=='clock_backward': assert e.reason=='clock_reversed'
        if case=='robot4_scan_stale': assert '센서' in e.reason
        if case=='robot4_odom_stale': assert 'odom' in e.reason
        result={'case':case,'passed':True,'phase':e.phase,'reason':e.reason,'departure_policy':departure_policy,'risk_stage':risk_stage,
                'observed_area_m2':e.observation.observed_area_m2,
                'valid_frames_by_robot':e.observation.valid_frames,
                'path_topics':sorted(probe.paths),'map_received':bool(maps),'robot_state_topics':sorted(states),
                'bt_marker_count':len(btmarkers[-1].markers) if btmarkers else 0,
                'scan_drops':mission.scan_drops,'physical_webots_run':False}
        atomic_json(out/'ros_contract_validation.json',result)
        return result
    finally:
        if mission:
            mission.halt(); mission.engine.terminate('test_shutdown')
            mission.pool.shutdown(wait=True,cancel_futures=True)
        if ex: ex.shutdown()
        if probe: probe.destroy_node()
        if mission: mission.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',choices=['connection','user_stop','robot4_odom_stale','robot4_scan_stale','clock_backward','command_conflict','missing_acquisition_tf'])
    parser.add_argument('--debug',action='store_true')
    parser.add_argument('--departure-policy',choices=['simultaneous','staggered_pairs'],default='simultaneous')
    parser.add_argument('--risk-stage',choices=['baseline','early','goals','candidates','heading'],default='')
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    cases=['connection','user_stop','robot4_odom_stale','robot4_scan_stale','clock_backward','command_conflict','missing_acquisition_tf']
    if args.case: cases=[args.case]
    results=[]
    for case in cases:
        result=run(case,args.output.resolve(),args.debug,args.departure_policy,args.risk_stage); results.append(result)
        print(json.dumps(result,ensure_ascii=False),flush=True)
    atomic_json(args.output/'bt_ros_validation.json',{'context':'single process shared rclpy; synthetic inputs', 'cases':results})
