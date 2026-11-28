from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node
import json,yaml,copy
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    import os
    physical=os.environ.get('HYBRID_PHYSICAL_MODE')=='1'
    if physical and os.environ.get('LIMO_USER_APPROVED')!='YES':
        raise RuntimeError('실제 주행은 사용자 승인 후에만 준비할 수 있습니다')
    root=Path(__file__).resolve().parent.parent
    run=root/'hybrid_moon_12m'
    config=json.loads((run/'config/environment.json').read_text())
    multi=root/'ros_packages/multirobot_sim'
    webots=WebotsLauncher(world=str(run/'moon/worlds/moon_terrain.wbt'),mode='realtime',port='1254',ros2_supervisor=True)
    actions=[SetEnvironmentVariable('ROS_DOMAIN_ID','74'),SetEnvironmentVariable('ROS_LOCALHOST_ONLY','1'),SetEnvironmentVariable('PYTHONPATH',str(run)+':'+str(multi)+':' +__import__('os').environ.get('PYTHONPATH','')),webots,webots._supervisor]
    for name in ('robot1','robot2','robot3','robot4'):
        if name=='robot1':
            robot=ET.Element('robot',name=name)
            plugin=ET.SubElement(ET.SubElement(robot,'webots'),'plugin',type='hybrid_replica.Replica')
            ET.SubElement(plugin,'robot_ip').text=os.environ.get('HYBRID_ROBOT_IP', config['real_robot_ip'])
        else:
            robot=ET.parse(multi/'config'/('robot3.urdf' if name == 'robot4' else f'{name}.urdf')).getroot()
            plugin=robot.find('./webots/plugin')
            plugin.set('type','sim_prepare_driver.MissionDriver')
            robot.attrib['name']=name
            for element in robot.iter():
                if element.text:
                    element.text=element.text.replace('robot3',name) if name=='robot4' else element.text
                for key,value in element.attrib.items():
                    if name=='robot4': element.attrib[key]=value.replace('robot3',name)
            ET.SubElement(plugin,'camera_preview').text='false'
            ET.SubElement(plugin,'require_bt_heartbeat').text='false'
        description=run/f'{name}.urdf'
        ET.ElementTree(robot).write(description,encoding='unicode')
        actions.append(WebotsController(robot_name=name,namespace=name,port='1254',parameters=[{'robot_description':str(description),'use_sim_time':True,'set_robot_state_publisher':False}]))
    nav=[]
    for name, (x,y) in config['sim_starts'].items():
        for index,(parent,child,xyz) in enumerate([
            ('map',f'{name}/odom',(x,y,0.)),
            (f'{name}/base_link',f'{name}/laser_link',(.103,0.,-.034))]):
            actions.append(Node(package='tf2_ros',executable='static_transform_publisher',
                name=f'{name}_anchor_{index}',arguments=['--x',str(xyz[0]),'--y',str(xyz[1]),'--z',str(xyz[2]),'--frame-id',parent,'--child-frame-id',child]))
        source=multi/'config'/('robot3_nav2.yaml' if name=='robot4' else f'{name}_nav2.yaml')
        text=source.read_text()
        if name=='robot4':text=text.replace('robot3','robot4')
        params=yaml.safe_load(text)
        params.pop('amcl',None)
        controller=params['controller_server']['ros__parameters']
        controller['FollowPath'].update(desired_linear_vel=.15,rotate_to_heading_angular_vel=.4,max_angular_accel=.5)
        controller['progress_checker_plugins']=[controller.pop('progress_checker_plugin','progress_checker')]
        for area in ('global_costmap','local_costmap'):
            value=params[area][area]['ros__parameters']
            if area=='global_costmap':value['plugins']=['static_layer','inflation_layer']
            if 'static_layer' in value:value['static_layer']['map_topic']='/hybrid/sim_nav_map'
        planner=params['planner_server']['ros__parameters']['GridBased']
        planner.update(use_astar=True,allow_unknown=False)
        params['bt_navigator']['ros__parameters'].pop('default_bt_xml_filename',None)
        parameter_file=run/'config'/f'{name}_runtime_nav.yaml'
        parameter_file.write_text(yaml.safe_dump({name:params},sort_keys=False))
        servers=[('controller_server','nav2_controller'),('planner_server','nav2_planner'),('behavior_server','nav2_behaviors'),('bt_navigator','nav2_bt_navigator')]
        for server,package in servers:
            nav.append(Node(package=package,executable=server,name=server,namespace=name,
                parameters=[str(parameter_file),{'use_sim_time':True}],remappings=[('cmd_vel','cmd_vel_requested')],output='screen'))
        nav.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',namespace=name,
            name='lifecycle_manager_prepare',parameters=[{'use_sim_time':True,'autostart':True,'node_names':[n for n,_ in servers]}],output='screen'))
    actions.append(ExecuteProcess(cmd=['/usr/bin/python3',str(run/'mapping.py'),'--ros-args','-p','use_sim_time:=true','-p','preparation_only:=false'],output='screen'))
    actions.append(Node(package='rviz2',executable='rviz2',name='hybrid_rviz',additional_env={'LIBGL_ALWAYS_SOFTWARE':'1'},arguments=['-d',str(run/'config/hybrid.rviz')],parameters=[{'use_sim_time':True}],output='screen'))
    actions.append(TimerAction(period=10.,actions=nav))
    mission_args=['--allow-physical','--key-file',os.environ.get('HYBRID_KEY_FILE','')] if physical else ['--virtual-only']
    actions.append(ExecuteProcess(cmd=['/usr/bin/python3',str(run/'mission.py'),*mission_args,'--ros-args','-p','use_sim_time:=true'],output='screen'))
    return LaunchDescription(actions)
