"""Attach Nav2 to existing sensor preparation. Physical movement is opt-in."""
from pathlib import Path
import os,sys,yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,GroupAction,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    here=Path(__file__).resolve().parent
    # Refuse a second copy of this add-on's Nav2 stack.
    for proc in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv=[a.decode() for a in proc.read_bytes().split(b'\0') if a]
        except (OSError,UnicodeError):continue
        if argv and Path(argv[0]).name in ('controller_server','planner_server','bt_navigator') and str(here/'real_nav2_bounded.yaml') in argv:
            raise RuntimeError('기존 hybrid Nav2가 실행 중입니다. 중복 실행을 차단했습니다: PID '+proc.parent.name)
    source=Path(os.environ.get('LIMO_EXPLORATION_DIR', str(Path.home() / 'limo_single_exploration'))) / 'resource/real_single_nav2.yaml'
    params=yaml.safe_load(source.read_text());c=params['controller_server']['ros__parameters']
    c['general_goal_checker'].update(xy_goal_tolerance=.04,yaw_goal_tolerance=3.141592653589793,stateful=False)
    c['progress_checker'].update(required_movement_radius=.025,movement_time_allowance=15.)
    c['FollowPath']={'plugin':'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController','desired_linear_vel':.16,'lookahead_dist':.12,'min_lookahead_dist':.10,'max_lookahead_dist':.20,'use_velocity_scaled_lookahead_dist':False,'min_approach_linear_velocity':.02,'approach_velocity_scaling_dist':.15,'use_collision_detection':True,'max_allowed_time_to_collision_up_to_carrot':.5,'use_rotate_to_heading':True,'rotate_to_heading_angular_vel':.60,'max_angular_accel':.3,'transform_tolerance':.8,'allow_reversing':False}
    for name in ('global_costmap','local_costmap'):
        v=params[name][name]['ros__parameters'];v['transform_tolerance']=.8
        if 'static_layer' in v:v['static_layer']['map_topic']='/hybrid/physical_nav_map'
    s=params['velocity_smoother']['ros__parameters'];s.update(max_velocity=[.20,0.,.70],min_velocity=[0.,0.,-.70],max_accel=[.10,0.,.3],max_decel=[-.10,0.,-.3],velocity_timeout=.3)
    profile=here/'real_nav2_bounded.yaml';profile.write_text(yaml.safe_dump(params))
    env={'PYTHONPATH':str(here)+':'+os.environ.get('PYTHONPATH',''),'PYTHONNOUSERSITE':'1'}
    armed=LaunchConfiguration('allow_motion').perform(context)=='true'
    if armed and os.environ.get('LIMO_USER_APPROVED')!='YES':raise RuntimeError('Explicit user approval is required before arming physical motion')
    args=[sys.executable,'-u',str(here/'real_executor.py'),'--key-file',LaunchConfiguration('key_file').perform(context),'--reference',str(here/'real_reference.json')]
    base = LaunchConfiguration('dashboard_base').perform(context)
    output = LaunchConfiguration('dashboard_output').perform(context)
    if bool(base) != bool(output):
        raise RuntimeError('Set both dashboard_base and dashboard_output')
    if base:
        args += ['--peer-ip', os.environ['HYBRID_LAPTOP_IP']]
        args += ['--dashboard-base', base, '--output-topic', output]
    if armed:args+=['--allow-motion']
    return [
        ExecuteProcess(cmd=[sys.executable,'-u',str(here/'bounded_real_map.py')],additional_env=env,output='screen'),
        # Explicit Humble velocity pipeline; only the executor can reach the base.
        *navigation_nodes(profile),
        ExecuteProcess(cmd=args,additional_env=env,output='screen')]


def navigation_nodes(profile):
    servers=[('controller_server','nav2_controller'),
             ('smoother_server','nav2_smoother'),
             ('planner_server','nav2_planner'),
             ('behavior_server','nav2_behaviors'),
             ('bt_navigator','nav2_bt_navigator'),
             ('waypoint_follower','nav2_waypoint_follower'),
             ('velocity_smoother','nav2_velocity_smoother')]
    actions=[]
    for name,package in servers:
        remaps=[]
        if name in ('controller_server','behavior_server'):
            remaps=[('cmd_vel','/hybrid/nav_raw')]
        elif name=='velocity_smoother':
            remaps=[('cmd_vel','/hybrid/nav_raw'),
                    ('cmd_vel_smoothed','/hybrid/nav_request')]
        actions.append(Node(package=package,executable=name,name=name,
            parameters=[str(profile),{'use_sim_time':False}],
            remappings=remaps,output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_navigation',parameters=[{'use_sim_time':False,
        'autostart':True,'node_names':[name for name,_ in servers]}],output='screen'))
    return actions

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('allow_motion',default_value='false',choices=['true','false']),DeclareLaunchArgument('key_file'),DeclareLaunchArgument('dashboard_base',default_value=''),DeclareLaunchArgument('dashboard_output',default_value=''),OpaqueFunction(function=setup)])
