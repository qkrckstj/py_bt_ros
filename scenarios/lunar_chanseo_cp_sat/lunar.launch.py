"""CP-SAT goal assignment plus selectable spatial routes and coverage RViz."""
import importlib.util
import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def rviz_config(directory):
    """Reuse the four sensor/path displays and add live exploration evidence."""
    share = Path(get_package_share_directory('moon_mapping_demo'))
    config = yaml.safe_load((share/'config/multi.rviz').read_text())
    manager = config['Visualization Manager']
    manager['Global Options']['Frame Rate'] = 5
    for display in manager['Displays']:
        if display['Class'] == 'rviz_default_plugins/Map':
            display['Topic']['Value'] = '/lunar/robot1/team_map'
            display['Name'] = 'OccupancyGrid map (optional GPU renderer)'
            display['Enabled'] = display['Value'] = False
    def topic(value, latched=True):
        return {'Value': value, 'Depth': 1, 'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable',
                'Durability Policy': 'Transient Local' if latched else 'Volatile'}
    manager['Displays'].extend([
        {'Class': 'rviz_default_plugins/Map', 'Name': 'Observed area (purple)',
         'Enabled': False, 'Value': False, 'Alpha': .25, 'Color Scheme': 'costmap',
         'Draw Behind': False, 'Use Timestamp': False,
         'Topic': topic('/exploration/coverage')},
        {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Exploration goals',
         'Enabled': True, 'Value': True, 'Topic': topic('/exploration/goals')},
        {'Class': 'rviz_default_plugins/Marker', 'Name': 'Observed percentage',
         'Enabled': True, 'Value': True, 'Topic': topic('/exploration/coverage_label')},
    ])
    manager['Displays'].extend([
        {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Lunar obstacles and observed free space',
         'Enabled': True, 'Value': True, 'Topic': topic('/exploration/obstacle_map')},
        {'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Robot positions and headings',
         'Enabled': True, 'Value': True, 'Topic': topic('/exploration/robot_markers')},
    ])
    manager['Views']['Current'].update(X=-24., Y=-23., Scale=45., Angle=0.)
    manager['Views']['Saved'] = [
        {**manager['Views']['Current'], 'Name': 'Whole lunar mission (60m)', 'X': 0., 'Y': 0., 'Scale': 14.},
        {**manager['Views']['Current'], 'Name': 'Start area and robots', 'X': -24., 'Y': -23., 'Scale': 45.},
    ]
    for i, color in enumerate(('255; 40; 40', '40; 255; 40', '255; 180; 40', '40; 100; 255'), 1):
        manager['Displays'].append({
            'Class': 'rviz_default_plugins/Path', 'Name': f'robot{i} selected exploration route',
            'Enabled': True, 'Value': True, 'Color': color, 'Alpha': 1.,
            'Line Style': 'Lines', 'Line Width': .07,
            'Topic': topic(f'/exploration/robot{i}/planned_path')})
    target = directory/'lunar_exploration.rviz'
    target.write_text(yaml.safe_dump(config, sort_keys=False))
    return target


def runtime(context):
    here = Path(__file__).resolve().parent
    repo = here.parents[1]
    default_runtime = repo if (repo/'scenarios/lunar_exploration/four_robot_nav.launch.py').exists() else repo.parent/'py_bt_ros'
    legacy = Path(os.environ.get('LUNAR_LEGACY_REPO', str(default_runtime))).resolve()
    method = LaunchConfiguration('method').perform(context)
    if method not in ('cp_sat', 'space_astar'):
        raise ValueError('method must be cp_sat or space_astar')
    planner = LaunchConfiguration('path_planner').perform(context)
    if planner == 'auto':
        planner = 'distance_field' if method == 'cp_sat' else 'astar'
    if planner not in ('astar', 'distance_field'):
        raise ValueError('path_planner must be astar or distance_field')
    if method == 'space_astar' and planner != 'astar':
        raise ValueError('space_astar requires path_planner:=astar or auto')
    show_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    source = legacy/'scenarios/lunar_exploration/four_robot_nav.launch.py'
    spec = importlib.util.spec_from_file_location('legacy_lunar_launch', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original_filter = module.add_shadow_filter
    def configure_navigation(params, robot):
        original_filter(params, robot)
        follower = params['controller_server']['ros__parameters']['FollowPath']
        follower.update(lookahead_dist=.6, max_lookahead_dist=.8,
                        rotate_to_heading_min_angle=.6)
        # All four Nav2 planner servers use the same selected search algorithm.
        params['planner_server']['ros__parameters']['GridBased'].update(
            use_astar=planner == 'astar', allow_unknown=False)
    module.add_shadow_filter = configure_navigation
    description = module.generate_launch_description()
    actions = [
        SetEnvironmentVariable('RMW_FASTRTPS_PUBLICATION_MODE', 'ASYNCHRONOUS'),
        # This integration is the only task owner. The legacy runtime supplies
        # sensors/navigation; our RViz below includes coverage and CP-SAT goals.
        # Sensor-ready handlers create legacy actions later, after a scoped
        # group would have restored rviz=true. Keep these launch values in the
        # shared context; show_rviz above controls our dedicated window.
        SetLaunchConfiguration('explore', 'false'),
        SetLaunchConfiguration('rviz', 'false'),
        *description.entities,
        ExecuteProcess(
            cmd=[str(legacy/'lunar_cp_env/bin/python'), '-u', str(here/'bridge.py'),
                 '--legacy-repo', str(legacy), '--path-planner', planner, '--method', method],
            additional_env={'PYTHONPATH': str(repo/'simulation/webots_ros2_limo')+os.pathsep+os.environ.get('PYTHONPATH', '')},
            output='screen'),
        ExecuteProcess(cmd=['/usr/bin/python3', '-u', str(here/'map_visualizer.py'),
                            '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
    ]
    if show_rviz:
        config = rviz_config(Path(tempfile.mkdtemp(prefix='lunar_exploration_rviz_')))
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(config)], parameters=[{'use_sim_time': True}],
                            output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='cp_sat', choices=['cp_sat', 'space_astar']),
        DeclareLaunchArgument('path_planner', default_value='auto',
                              choices=['auto', 'astar', 'distance_field']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=runtime),
    ])
