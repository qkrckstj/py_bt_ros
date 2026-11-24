"""Add three Nav2 stacks to the unchanged robot1 lunar mapping launch.

No goals or motor-enable heartbeat are published. Robot2..4 use the original
start-anchored odometry TF: this is not multi-robot SLAM or drift correction.
Nav2 receives each robot's map assembled from range-gated observation messages.
Nav2 velocity requests pass through per-robot coordination permits.
"""

import importlib.util
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    EmitEvent, DeclareLaunchArgument,
    RegisterEventHandler, TimerAction, ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.utils import get_webots_home


ROBOTS = ('robot1', 'robot2', 'robot3', 'robot4')
NAV_NODES = (
    ('controller_server', 'nav2_controller'),
    ('smoother_server', 'nav2_smoother'),
    ('planner_server', 'nav2_planner'),
    ('behavior_server', 'nav2_behaviors'),
    ('bt_navigator', 'nav2_bt_navigator'),
)


def add_shadow_filter(params, robot):
    import json
    config = json.loads(Path(__file__).with_name('mission.json').read_text())
    follower = params['controller_server']['ros__parameters']['FollowPath']
    follower['desired_linear_vel'] = config.get('navigation_speed_mps',.18)
    follower['rotate_to_heading_angular_vel'] = .6
    follower['rotate_to_heading_min_angle'] = .25
    follower['lookahead_dist'] = .35
    follower['approach_velocity_scaling_dist'] = .35
    if os.environ.get('ROS_DISTRO') == 'humble':
        params['planner_server']['ros__parameters']['GridBased']['plugin'] = 'nav2_navfn_planner/NavfnPlanner'
        behavior = params['behavior_server']['ros__parameters']
        for name in behavior.get('behavior_plugins', []):
            behavior[name]['plugin'] = behavior[name]['plugin'].replace('nav2_behaviors::', 'nav2_behaviors/')
    # Exploration goals need a safe position, not a prescribed final heading.
    params['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance'] = 3.141592653589793
    params['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance'] = 60.0
    for name in ('global_costmap', 'local_costmap'):
        costmap = params[name][name]['ros__parameters']
        costmap['filters'] = ['shadow_filter']
        costmap['static_layer']['map_topic'] = f'/lunar/{robot}/team_map'
        costmap['shadow_filter'] = {
            'plugin': 'nav2_costmap_2d::KeepoutFilter', 'enabled': True,
            'filter_info_topic': '/lunar/shadow_filter_info',
            'transform_tolerance': 1.0,
        }


def generate_launch_description():
    share = Path(get_package_share_directory('moon_mapping_demo'))
    temporary = Path(tempfile.mkdtemp(prefix='lunar_four_nav_'))
    here = Path(__file__).resolve().parent
    original = share / 'launch/moon_mapping.launch.py'
    spec = importlib.util.spec_from_file_location('original_moon_launch', original)
    base = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(base)
    original_runtime = base.runtime
    original_assets = base.mapping_test_assets
    original_node = base.Node
    def gated_node(*args, **kwargs):
        if (os.environ.get('ROS_DISTRO') == 'humble' and
                kwargs.get('package') == 'nav2_lifecycle_manager'):
            # Humble's async SLAM node starts itself and has no lifecycle services.
            for parameters in kwargs.get('parameters', []):
                if isinstance(parameters, dict) and 'node_names' in parameters:
                    parameters['node_names'] = [n for n in parameters['node_names'] if n != 'slam_toolbox']
        if kwargs.get('executable') in ('controller_server','behavior_server'):
            kwargs['remappings'] = list(kwargs.get('remappings',[]))+[('cmd_vel','cmd_vel_requested')]
        return original_node(*args,**kwargs)
    base.Node = gated_node
    runtime_directories = []

    def assets(*args):
        descriptions, rviz = original_assets(*args)
        for path in descriptions.values():
            tree = ET.parse(path)
            tree.find('./webots/plugin/require_bt_heartbeat').text = 'true'
            tree.write(path, encoding='unicode')
        runtime_directories.append(next(iter(descriptions.values())).parent)
        world = runtime_directories[-1]/'moon_runtime.wbt'
        import re
        text = re.sub(r'DEF DEMO_VIEW Viewpoint \{.*?\}',
                      'DEF DEMO_VIEW Viewpoint { orientation 0 0 1 0 position -25 -25 12 description "Four robot start" }',
                      world.read_text(), count=1, flags=re.S)
        world.write_text(text+'\nRobot { name "lunar_runtime_control" controller "<extern>" supervisor TRUE }\n')
        return descriptions, rviz

    def runtime(context):
        actions = original_runtime(context)
        # Modify only the generated launch actions and temporary parameter files.
        # The team's launch/controller files on disk are kept intact.
        actions[2] = ExecuteProcess(
            cmd=[str(Path(get_webots_home()) / 'webots-controller'),
                 '--robot-name=lunar_sun_supervisor', str(here / 'predictive_sun.py'),
                 str(share / 'controllers/lunar_sun/lunar_sun.py')],
            additional_env={'WEBOTS_HOME': get_webots_home()}, output='screen')
        for directory in runtime_directories:
            config = directory / 'robot1.yaml'
            values = yaml.safe_load(config.read_text())
            add_shadow_filter(values['robot1'], 'robot1')
            config.write_text(yaml.safe_dump(values, sort_keys=False))
        actions.append(ExecuteProcess(
            cmd=['/usr/bin/python3','-u',str(here/'radio.py')], output='screen',
            on_exit=[EmitEvent(event=Shutdown(reason='Range-gated robot radio agents stopped'))]))
        actions.append(ExecuteProcess(
            cmd=[str(Path(get_webots_home())/'webots-controller'),
                 '--robot-name=lunar_runtime_control',str(here/'simulation_control.py'),'resume'],
            additional_env={'WEBOTS_HOME':get_webots_home()},output='screen'))
        actions.append(ExecuteProcess(
            cmd=[str(here.parents[1]/'lunar_cp_env/bin/python'),'-u',str(here/'explore.py')],
            condition=IfCondition(LaunchConfiguration('explore')),output='screen'))
        return actions

    base.mapping_test_assets = assets
    base.runtime = runtime
    stacks = []
    for robot in ROBOTS[1:]:
        params = yaml.safe_load((share / f'config/{robot}_nav2.yaml').read_text())
        # The original launch supplies map -> robotN/odom at the lunar start.
        # Do not launch AMCL with the stale factory initial poses in these YAMLs.
        params.pop('amcl', None)
        params['planner_server']['ros__parameters']['GridBased'].update(
            tolerance=0.1, allow_unknown=False)
        params['behavior_server']['ros__parameters']['max_rotational_vel'] = 0.55
        costmap = params['global_costmap']['global_costmap']['ros__parameters']
        costmap['static_layer']['footprint_clearing_enabled'] = True
        add_shadow_filter(params, robot)
        config = temporary / f'{robot}.yaml'
        config.write_text(yaml.safe_dump({robot: params}, sort_keys=False))
        for name, package in NAV_NODES:
            stacks.append(Node(
                package=package, executable=name, name=name, namespace=robot,
                remappings=[('cmd_vel','cmd_vel_requested')] if name in ('controller_server','behavior_server') else [],
                parameters=[str(config), {'use_sim_time': True}], output='screen'))
        stacks.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager', namespace=robot, output='screen',
            parameters=[{
                'use_sim_time': True, 'autostart': True,
                'node_names': [name for name, _ in NAV_NODES],
                'bond_timeout': 10.0,
            }]))

    gate = Node(
        package='moon_mapping_demo', executable='moon_ready',
        name='lunar_four_robot_ready', output='screen',
        parameters=[{'robots': list(ROBOTS), 'scan_robots': list(ROBOTS)}])

    def sensors_ready(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Four-robot sensors unavailable'))]
        return [TimerAction(period=2.0, actions=stacks)]

    return LaunchDescription([
        DeclareLaunchArgument('explore',default_value='false'),
        *base.generate_launch_description().entities,
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=sensors_ready)),
        gate,
    ])
