"""Configurable simulated LIMO fleet, known map, Nav2 and task overlays."""
import json
import os
import math
import re
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess,TimerAction,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.utils import get_webots_home


def generate_launch_description():
    root=Path(__file__).resolve().parent;repo=root.parent;out=root/'runtime'
    profile=Path(os.environ.get('CITY_SCENARIO_DIR',str(root/'profiles/junyong_10_damage'))).resolve()
    out.mkdir(parents=True,exist_ok=True)
    cfg=json.loads((profile/'scenario.json').read_text());port='1257'
    source=(repo/'protos/LimoFourDiff.proto').read_text()
    while match:=re.search(r'\b(?:Camera|RangeFinder|Display)\s*\{',source):
        end,depth=match.end(),1
        while depth:depth+=(source[end]=='{')-(source[end]=='}');end+=1
        source=source[:match.start()]+source[end:]
    source=source.replace('url "meshes/',f'url "{repo}/protos/meshes/')
    source=re.sub(r'\bLidar\s*\{','DEF EXPLORATION_LIDAR Lidar {\n translation 0 0 0.20',source)
    proto=out/'LimoFourDiff.proto';proto.write_text(source)
    world=(profile/'city.wbt').read_text().replace('@CITY_ROOT@',str(root)).replace('WorldInfo {',f'EXTERNPROTO "{proto}"\nWorldInfo {{',1)
    if profile==root:
        world+='\nTransform { translation -16 -9 0.015 children [ Shape { appearance PBRAppearance { baseColor 0.3 0.6 0.4 roughness 1 } geometry Box { size 1.2 1.2 0.02 } } ] }\n'
        world+='\nTransform { translation -14 -12.1 0.008 children [ Shape { appearance PBRAppearance { baseColor 0.24 0.35 0.45 roughness 1 } geometry Box { size 6 3 0.01 } } ] }\n'
    for i,xy in enumerate(cfg['starts']):
        name=f'robot{i+1}'
        world+=f'DEF {name.upper()} LimoFourDiff {{ name "{name}" translation {xy[0]} {xy[1]} 0.145 rotation 0 0 1 {math.pi/2} controller "<extern>" }}\n'
        (out/f'{name}.urdf').write_text(f'''<robot name="{name}"><webots>
<device reference="laser"><ros><enabled>false</enabled></ros></device>
<device reference="imu compass"><ros><enabled>false</enabled></ros></device>
<plugin type="webots_ros2_limo.exploration_flat_driver.FlatDriver"><namespace>{name}</namespace></plugin>
</webots></robot>''')
    world+='Robot { name "city_observer" supervisor TRUE controller "<extern>" }\n'
    (out/'city_six_limo.wbt').write_text(world)
    webots=WebotsLauncher(world=str(out/'city_six_limo.wbt'),gui=True,mode='realtime',port=port,ros2_supervisor=True)
    observer=ExecuteProcess(cmd=[str(Path(get_webots_home())/'webots-controller'),'--robot-name=city_observer','--port='+port,str(root/'supervisor.py')],output='screen')
    actions=[webots,webots._supervisor,observer,
        RegisterEventHandler(OnProcessExit(target_action=webots,on_exit=[EmitEvent(event=Shutdown(reason='City Webots closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=observer,on_exit=[EmitEvent(event=Shutdown(reason='City supervisor ended'))])),
        Node(package='nav2_map_server',executable='map_server',name='city_map_server',parameters=[{'use_sim_time':True,'yaml_filename':str(profile/'city.yaml'),'topic_name':'map','frame_id':'map'}]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='city_map_lifecycle',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['city_map_server']}])]
    servers=[('controller_server','nav2_controller'),('smoother_server','nav2_smoother'),('planner_server','nav2_planner'),('behavior_server','nav2_behaviors'),('bt_navigator','nav2_bt_navigator')]
    for i in range(len(cfg["starts"])):
        name=f'robot{i+1}'
        actions.append(WebotsController(robot_name=name,namespace=name,port=port,parameters=[{'robot_description':str(out/f'{name}.urdf'),'use_sim_time':True,'set_robot_state_publisher':False}],output='screen'))
        actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name=name+'_laser_mount',arguments=['--x','.103','--z','.366','--frame-id',name+'/base_link','--child-frame-id',name+'/laser_link']))
        params=yaml.safe_load((root/'config/nav2.yaml').read_text().replace('robot1',name))
        params.pop('amcl',None)
        c=params['controller_server']['ros__parameters']
        c['progress_checker_plugins']=[c.pop('progress_checker_plugin','progress_checker')]
        c['controller_frequency']=10.
        c['general_goal_checker'].update(xy_goal_tolerance=.22,yaw_goal_tolerance=3.14)
        c['progress_checker'].update(required_movement_radius=.1,movement_time_allowance=25.)
        c['FollowPath'].update(desired_linear_vel=cfg['speed_mps'],min_approach_linear_velocity=.08,rotate_to_heading_angular_vel=.8,max_angular_accel=10.,use_collision_detection=True)
        params['planner_server']['ros__parameters']['expected_planner_frequency']=1.
        params['planner_server']['ros__parameters']['GridBased'].update(allow_unknown=False,tolerance=.15)
        params['bt_navigator']['ros__parameters']['bt_loop_duration']=50
        for cm in ('local_costmap','global_costmap'):
            p=params[cm][cm]['ros__parameters'];p['resolution']=.1;p['update_frequency']=3.
            p['inflation_layer']['inflation_radius']=.45
            p.pop('footprint',None);p['robot_radius']=.24;p['footprint_padding']=.01
        params['global_costmap']['global_costmap']['ros__parameters']['plugins']=['static_layer','inflation_layer']
        path=out/f'{name}_nav2.yaml';path.write_text(yaml.safe_dump({name:params}))
        nodes=[Node(package=p,executable=n,namespace=name,name=n,parameters=[str(path),{'use_sim_time':True}],output='screen') for n,p in servers]
        nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',namespace=name,name='navigation_lifecycle_manager',parameters=[{'use_sim_time':True,'autostart':True,'bond_timeout':15.,'node_names':[n for n,_ in servers]}]))
        actions.append(TimerAction(period=5.+5.*i,actions=nodes))
    actions.append(TimerAction(period=5.*len(cfg['starts'])+15.,actions=[ExecuteProcess(cmd=['python3','-m','webots_ros2_limo.exploration_startup','--ros-args','-p','include_slam:=false','-p','robots:=['+','.join(f'robot{i+1}' for i in range(len(cfg['starts'])))+']'],output='screen')]))
    runtime=ExecuteProcess(cmd=[os.environ.get('CITY_RUNTIME_PYTHON','python3'),'-m','disaster_city.mission_node','--ros-args','-p','use_sim_time:=true'],output='screen')
    if os.environ.get('CITY_TASK_WINDOW','0')=='1':
        actions.append(ExecuteProcess(cmd=['python3','-m','disaster_city.hud'],output='screen'))
    actions.extend([runtime,
        RegisterEventHandler(OnProcessExit(target_action=runtime,on_exit=[EmitEvent(event=Shutdown(reason='City mission process ended'))]))])
    rviz={'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'}], 'Visualization Manager':{'Class':'','Global Options':{'Fixed Frame':'map','Background Color':'48; 48; 48'},
        'Displays':[{'Class':'rviz_default_plugins/Map','Name':'Known city map','Enabled':True,'Topic':{'Value':'/map','Durability Policy':'Transient Local'},'Color Scheme':'map'},
                    {'Class':'rviz_default_plugins/MarkerArray','Name':'Robots and discovered tasks','Enabled':True,'Topic':{'Value':'/disaster/markers','Durability Policy':'Transient Local'}}],
        'Views':{'Current':{'Class':'rviz_default_plugins/TopDownOrtho','Name':'City','Target Frame':'map','Scale':22.,'X':0.,'Y':0.,'Angle':0.}}}}
    (out/'city.rviz').write_text(yaml.safe_dump(rviz))
    actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(out/'city.rviz')],parameters=[{'use_sim_time':True}]))
    return LaunchDescription(actions)
