#!/usr/bin/env python3
"""Simulator-only localization and public task visuals. Never teleports robots."""
import json
import os
import math
import sys
from pathlib import Path
from controller import Supervisor
from task_visuals import render_panel, panel_values
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def main():
    world=Supervisor();rclpy.init();node=rclpy.create_node('city_simulation_localization')
    tf=TransformBroadcaster(node);pub=node.create_publisher(String,'/disaster/sim_poses',10)
    cfg=json.loads((Path(os.environ.get('CITY_SCENARIO_DIR',str(Path(__file__).resolve().parent/'profiles/junyong_10_damage')))/'scenario.json').read_text())
    robots=[world.getFromDef(f'ROBOT{i+1}') for i in range(len(cfg['starts']))];odom={};observation={};visuals={}
    def receive(m):
        observation.clear();observation.update(json.loads(m.data))
    node.create_subscription(String,'/disaster/observation',receive,10)
    for i in range(len(robots)):node.create_subscription(Odometry,f'/robot{i+1}/odom',lambda m,i=i:odom.update({i:m}),10)
    lasers=[r.getFromProtoDef('EXPLORATION_LIDAR') for r in robots]
    media=Path(os.environ['CITY_MEDIA_DIR']) if os.environ.get('CITY_MEDIA_DIR') else None
    if media:media.mkdir(parents=True,exist_ok=True)
    recording=False;recorded=False;stopped=False;last_shot=-15.;last_event_shot=''
    tick=0
    while world.step(int(world.getBasicTimeStep()))!=-1:
        for _ in range(8):rclpy.spin_once(node,timeout_sec=0.)
        tick+=1
        if tick%5:continue
        now=world.getTime(); poses=[]
        for i,r in enumerate(robots):
            p=r.getPosition();mat=r.getOrientation();yaw=math.atan2(mat[3],mat[0]);vel=r.getVelocity()
            poses.append(dict(xy=p[:2],yaw=yaw,speed=math.hypot(vel[0],vel[1])))
            if i in odom:
                o=odom[i];q=o.pose.pose.orientation;oy=2*math.atan2(q.z,q.w);a=yaw-oy
                ox,oyp=o.pose.pose.position.x,o.pose.pose.position.y
                t=TransformStamped();t.header.frame_id='map';t.child_frame_id=f'robot{i+1}/odom'
                t.header.stamp.sec=int(now);t.header.stamp.nanosec=int((now-int(now))*1e9)
                t.transform.translation.x=p[0]-math.cos(a)*ox+math.sin(a)*oyp
                t.transform.translation.y=p[1]-math.sin(a)*ox-math.cos(a)*oyp
                t.transform.rotation.z=math.sin(a/2);t.transform.rotation.w=math.cos(a/2)
                tf.sendTransform(t)
        pub.publish(String(data=json.dumps(dict(time_s=now,robots=poses))))
        if tick%25:continue
        if tick%1000==0:
            world.exportImage(str(Path(__file__).resolve().parent/'runtime/webots.png'),90)
        world.setLabel(0,f'URBAN DISASTER | {len(robots)} LIMO | '+('KNOWN TASKS' if cfg.get('tasks_known') else 'HIDDEN TASKS'),.02,.02,.035,0xFFFFFF,0,'Arial')
        world.setLabel(1,f"{observation.get('phase','PREPARING')} | {observation.get('time_s',0):.0f}s | {observation.get('policy','Preparing policy')}",.02,.065,.029,0xFFFFFF,0,'Arial')
        complete=sum(t['state']=='done' for t in observation.get('tasks',{}).values())
        world.setLabel(2,f"Tasks complete {complete}/{len(cfg['tasks'])} | fire workload grows until suppression",.02,.105,.024,0xFFFFFF,0,'Arial')
        if media:
            phase=observation.get('phase');elapsed=observation.get('time_s',0.)
            if phase=='RUNNING' and not recorded:
                world.movieStartRecording(str(media/'webots_full_4x.mp4'),1920,1080,0,85,4,True)
                recording=True;recorded=True
            event=next((e for e in reversed(observation.get('events',[])) if e['kind'] in ('started','completed','team_released')),None)
            signature=json.dumps(event,sort_keys=True) if event else ''
            if phase=='RUNNING' and (elapsed-last_shot>=15. or (signature and signature!=last_event_shot)):
                tag=(event['kind']+'_'+str(event.get('task','team'))) if signature!=last_event_shot and event else 'overview'
                name=f'{elapsed:07.1f}_{tag}'
                world.exportImage(str(media/(name+'.png')),100)
                (media/(name+'.json')).write_text(json.dumps(observation,indent=2))
                last_shot=elapsed;last_event_shot=signature
            if recording and (phase in ('COMPLETE','TIME_LIMIT','BATTERY_FAILURE') or (media/'stop_recording').exists()):
                world.exportImage(str(media/'final.png'),100)
                (media/'final_state.json').write_text(json.dumps(observation,indent=2))
                world.movieStopRecording();recording=False;stopped=True
            if stopped and world.movieIsReady():
                (media/'recording_status.json').write_text(json.dumps(dict(ready=True,failed=world.movieFailed(),phase=phase)))
                stopped=False
                if os.environ.get('CITY_EXIT_AFTER_RECORDING')=='1':
                    world.simulationQuit(0)
        for key,task in observation.get('tasks',{}).items():
            signature=panel_values(task)
            if key in visuals and visuals[key]['signature']==signature:continue
            x,y=task['xy']
            old=visuals.get(key,{})
            revision=old.get('revision',0)+1
            texture=Path(__file__).resolve().parent/'runtime'/f'task_{key}_{revision}.png'
            render_panel(task,texture)
            if key not in visuals:
                world.getRoot().getField('children').importMFNodeFromString(-1,f'''DEF TASK_{key} Transform {{ translation {x} {y} 0.55 children [
                  Shape {{ appearance PBRAppearance {{ baseColor 0.85 0.55 0.22 transparency 0.65 roughness 1 }} geometry Sphere {{ radius 0.5 }} castShadows FALSE }}
                  Transform {{ translation 0 1.1 1.0 children [ Shape {{
                    appearance Appearance {{ material Material {{ diffuseColor 1 1 1 emissiveColor 1 1 1 }}
                      texture DEF TASK_TEXTURE_{key} ImageTexture {{ url [ "{texture}" ] repeatS FALSE repeatT FALSE }} }}
                    geometry IndexedFaceSet {{ coord Coordinate {{ point [ -1.9 -0.65 0, 1.9 -0.65 0, 1.9 0.65 0, -1.9 0.65 0 ] }}
                      coordIndex [ 0 1 2 3 -1 ] texCoord TextureCoordinate {{ point [ 0 0, 1 0, 1 1, 0 1 ] }}
                      texCoordIndex [ 0 1 2 3 -1 ]  }} castShadows FALSE
                  }} ] }} ] }}''')
                marker=world.getFromDef('TASK_'+key)
                for laser in lasers:
                    if laser:marker.setVisibility(laser,False)
                field=world.getFromDef('TASK_TEXTURE_'+key).getField('url')
            else:
                field=old['field'];field.setMFString(0,str(texture))
            visuals[key]=dict(signature=signature,revision=revision,field=field)
            # Unique URLs force texture refresh; retain recent files while rendering.
            if revision>3:
                (texture.parent/f'task_{key}_{revision-3}.png').unlink(missing_ok=True)
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
