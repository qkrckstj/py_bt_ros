"""Run the three-LIMO factory simulator with optional single-robot mapping."""
import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess,RegisterEventHandler,TimerAction,EmitEvent,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from multirobot_sim.map_grid import MAP_RESOLUTION
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

ROBOTS = ('robot1', 'robot2', 'robot3')

def enabled(context,key):return LaunchConfiguration(key).perform(context).lower() in ('true','1','yes')


def mapping_test_assets(descriptions, rviz_config, active_robot, directory):
    test_descriptions = {}
    for robot, description in descriptions.items():
        urdf = ET.parse(description)
        plugin = urdf.find('./webots/plugin')
        ET.SubElement(plugin, 'camera_preview').text = 'false'
        if robot == active_robot:
            ET.SubElement(plugin, 'require_bt_heartbeat').text = 'false'
        else:
            ET.SubElement(plugin, 'standby').text = 'true'
        test_description = directory / description.name
        urdf.write(test_description, encoding='unicode')
        test_descriptions[robot] = test_description

    rviz = yaml.safe_load(rviz_config.read_text())
    manager = rviz['Visualization Manager']
    inactive_prefixes = tuple(f'{robot} ' for robot in ROBOTS if robot != active_robot)
    manager['Displays'] = [display for display in manager['Displays']
                           if not display.get('Name', '').startswith(inactive_prefixes)]
    manager['Global Options']['Frame Rate'] = 5
    manager['Tools'].append({
        'Class': 'rviz_default_plugins/SetGoal', 'Topic': {'Value': f'/{active_robot}/goal_pose'}})
    test_rviz = directory / 'mapping_test.rviz'
    test_rviz.write_text(yaml.safe_dump(rviz, sort_keys=False))
    return test_descriptions, test_rviz


def runtime(context):
    share=Path(get_package_share_directory('multirobot_sim'))
    env=yaml.safe_load((share/'config/environment.yaml').read_text())
    slam = enabled(context, 'slam')
    robots = list(ROBOTS)
    mapping_robot = LaunchConfiguration('mapping_robot').perform(context)
    if mapping_robot and (not slam or mapping_robot not in robots):
        raise ValueError('mapping_robot requires slam:=true and one of the three LIMO robots')
    if slam and not mapping_robot:
        mapping_robot = 'robot1'
    repo=LaunchConfiguration('repo_path').perform(context)
    python=LaunchConfiguration('python_executable').perform(context)
    temporary=Path(tempfile.mkdtemp(prefix='multirobot_sim_'))
    world = share / 'worlds' / env['world']
    descriptions = {robot: share / 'config' / f'{robot}.urdf' for robot in robots}
    rviz_config = share / 'config/multi.rviz'
    if slam:
        descriptions, rviz_config = mapping_test_assets(
            descriptions, rviz_config, mapping_robot, temporary)
    webots=WebotsLauncher(world=str(world),mode='realtime',ros2_supervisor=True)
    actions=[webots,webots._supervisor];stacks=[]
    if slam:
        slam_share = Path(get_package_share_directory('slam_toolbox'))
        slam_config = slam_share / 'config/mapper_params_online_async.yaml'
        slam_defaults = yaml.safe_load(slam_config.read_text())['slam_toolbox']['ros__parameters']
        stacks.append(Node(
            package='multirobot_sim', executable='live_mapping', output='screen',
            parameters=[{'use_sim_time': True, 'robot': mapping_robot}],
            on_exit=[EmitEvent(event=Shutdown(reason='Live map publisher stopped'))]))
    else:
        stacks.extend([
            Node(package='nav2_map_server',executable='map_server',name='multi_map_server',parameters=[{'yaml_filename':str(share/'config'/env['map']),'use_sim_time':True}]),
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='multi_map_lifecycle',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['multi_map_server']}])])
    for ns in robots:
        actions.append(WebotsController(robot_name=ns,namespace=ns,parameters=[{'robot_description':str(descriptions[ns]),'use_sim_time':True,'set_robot_state_publisher':False}],output='screen'))
        for child,xyz in [('laser_link',(.103,0.,-.034)),('camera_link',(.096,.015,.03))]:
            actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name=ns+'_'+child+'_tf',arguments=['--x',str(xyz[0]),'--y',str(xyz[1]),'--z',str(xyz[2]),'--frame-id',ns+'/base_link','--child-frame-id',ns+'/'+child]))
        if slam and ns != mapping_robot:
            start = env['starts'][ns]
            actions.append(Node(
                package='tf2_ros', executable='static_transform_publisher',
                name=f'{ns}_standby_odom_tf', arguments=[
                    '--x', str(start['x']), '--y', str(start['y']), '--yaw', str(start['yaw']),
                    '--frame-id', 'map', '--child-frame-id', f'{ns}/odom']))
            continue
        params=yaml.safe_load((share/f'config/{ns}_nav2.yaml').read_text())
        nodes = [
            ('controller_server', 'nav2_controller'),
            ('smoother_server', 'nav2_smoother'),
            ('planner_server', 'nav2_planner'),
            ('behavior_server', 'nav2_behaviors'),
            ('bt_navigator', 'nav2_bt_navigator'),
        ]
        lifecycle_nodes = [name for name, _ in nodes]
        if slam:
            params.pop('amcl')
            planner = params['planner_server']['ros__parameters']['GridBased']
            planner['tolerance'] = 0.1
            planner['allow_unknown'] = False
            params['behavior_server']['ros__parameters']['max_rotational_vel'] = 0.55
            global_costmap = params['global_costmap']['global_costmap']['ros__parameters']
            global_costmap['static_layer']['footprint_clearing_enabled'] = True
            slam_params = {
                **slam_defaults,
                'use_sim_time': True,
                'use_lifecycle_manager': True,
                'use_map_saver': False,
                'enable_interactive_mode': False,
                'map_frame': f'{ns}/map',
                'map_name': f'/{ns}/map',
                'odom_frame': f'{ns}/odom',
                'base_frame': f'{ns}/base_link',
                'scan_topic': f'/{ns}/scan_slam',
                'resolution': MAP_RESOLUTION,
                'map_update_interval': 1.0,
                'min_laser_range': 0.1,
                'max_laser_range': 12.0,
                'minimum_travel_distance': 0.2,
                'minimum_travel_heading': 0.2,
                'check_min_dist_and_heading_precisely': True,
            }
            stacks.append(Node(
                package='slam_toolbox', executable='async_slam_toolbox_node',
                namespace=ns, name='slam_toolbox', parameters=[slam_params],
                output='screen'))
            lifecycle_nodes.insert(0, 'slam_toolbox')
        else:
            params['amcl']['ros__parameters']['initial_pose'] = env['starts'][ns]
            nodes.insert(0, ('amcl', 'nav2_amcl'))
            lifecycle_nodes.insert(0, 'amcl')
        file=temporary/(ns+'.yaml');file.write_text(yaml.safe_dump({ns:params},sort_keys=False))
        for name,package in nodes:stacks.append(Node(package=package,executable=name,name=name,namespace=ns,parameters=[str(file),{'use_sim_time':True}],output='screen'))
        stacks.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager',namespace=ns,parameters=[{'autostart':True,'use_sim_time':True,'node_names':lifecycle_nodes,'bond_timeout':10.}]))
        if enabled(context,'yolo'):
            model=str(Path(get_package_share_directory('human_yolo'))/'models/best.pt')
            script="import sys,runpy;sys.path.append('/usr/lib/python3/dist-packages');import cv2,torch;cv2.setNumThreads(1);torch.set_num_interop_threads(1);import ultralytics.utils.torch_utils as tu;tu.NUM_THREADS=1;runpy.run_module('human_yolo.detector',run_name='__main__')"
            cmd=[LaunchConfiguration('yolo_python').perform(context),'-c',script,'--ros-args','-r','__ns:=/'+ns,'-p','use_sim_time:=true','-p','model_path:='+model,'-p','cpu_threads:=1','-p','confidence:=0.65','-p','input_timeout:=2.0']
            for key,value in [('rgb_topic','camera/color/image_raw'),('depth_topic','camera/depth/image_raw'),('camera_info_topic','camera/color/camera_info')]:cmd+=['-p',key+':=/'+ns+'/'+value]
            for topic in ['/human/detected','/human/position_valid','/human/position','/human/detections','/perception/is_person','/perception/person_pose']:cmd+=['-r',topic+':=/'+ns+topic]
            stacks.append(ExecuteProcess(cmd=cmd,additional_env={'OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','YOLO_CONFIG_DIR':str(temporary/'yolo')},output='screen'))
    if not slam:
        bt=Path(get_package_share_directory('multirobot_bt'))/'launch/multirobot_bt.launch.py'
        factory_regions_file = ''
        if enabled(context, 'use_main_regions'):
            factory_regions_file = str(Path(repo) / 'scenarios/limo_factory_search/config_sim.yaml')
        stacks.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bt)),
            launch_arguments={
                'repo_path': repo,
                'python_executable': python,
                'use_sim_time': 'true',
                'pygame': LaunchConfiguration('pygame').perform(context),
                'factory_regions_file': factory_regions_file,
            }.items()))
    stacks.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(rviz_config)],parameters=[{'use_sim_time':True}],additional_env={'LIBGL_ALWAYS_SOFTWARE':'1'},condition=IfCondition(LaunchConfiguration('rviz'))))
    gate = Node(package='multirobot_sim', executable='ready', output='screen',
                parameters=[{'robots': robots, 'scan_robots': [mapping_robot] if slam else robots}])
    def sensors_ready(event,context):
        if context.is_shutdown:return []
        return [TimerAction(period=2.,actions=stacks)] if event.returncode==0 else [EmitEvent(event=Shutdown(reason='Required robot sensors failed'))]
    actions.extend([RegisterEventHandler(OnProcessExit(target_action=gate,on_exit=sensors_ready)),gate,RegisterEventHandler(OnProcessExit(target_action=webots,on_exit=[EmitEvent(event=Shutdown(reason='Webots closed'))]))])
    return actions

def generate_launch_description():
    candidates=[*Path(__file__).resolve().parents,Path.cwd()]
    repo=next((p for p in candidates if (p/'modules/base_bt_nodes.py').is_file()),Path.cwd())
    bt_python=repo.parent/'.venv-view/bin/python';yolo_python=Path.home()/'yolo_project/.venv/bin/python'
    return LaunchDescription([
        DeclareLaunchArgument('repo_path',default_value=str(repo)),
        DeclareLaunchArgument('python_executable',default_value=str(bt_python) if bt_python.exists() else sys.executable),
        DeclareLaunchArgument('yolo_python',default_value=str(yolo_python) if yolo_python.exists() else sys.executable),
        DeclareLaunchArgument('use_main_regions',default_value='true'),
        DeclareLaunchArgument('robots', default_value='3', choices=['3'],
                              description='Run the three-LIMO factory world'),
        DeclareLaunchArgument('mapping_robot', default_value='', choices=['', *ROBOTS],
                              description='Choose the mapping LIMO. Defaults to robot1 in SLAM mode.'),
        DeclareLaunchArgument('slam', default_value='false', choices=['false', 'true'],
                              description='Build and share a live map from one LIMO'),
        DeclareLaunchArgument('yolo',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('pygame',default_value='true'),
        OpaqueFunction(function=runtime)])
