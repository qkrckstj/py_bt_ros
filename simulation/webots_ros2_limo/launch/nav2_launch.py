#!/usr/bin/env python

# Nav2 + AMCL + RViz for the Webots AgileX LIMO.
#
# Wraps `nav2_bringup/bringup_launch.py` with this package's map, Nav2
# parameters and RViz view, so navigation starts with a single command.
#
# This deliberately does not use `turtlebot3_navigation2/navigation2.launch.py`:
# that file is the same bringup_launch.py plus an RViz node you cannot switch
# off, and it reads os.environ['TURTLEBOT3_MODEL'] at import time, raising
# KeyError when the variable is unset -- even though params_file overrides
# everything the variable would select.

import os
import tempfile
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def start_navigation(context):
    nav2_dir = get_package_share_directory('nav2_bringup')
    selected = LaunchConfiguration('params_file').perform(context)
    cleanup = []
    if os.environ.get('ROS_DISTRO') == 'jazzy':
        package_dir = Path(get_package_share_directory('webots_ros2_limo'))
        # Adapt only bundled Humble parameters; explicit custom configs stay untouched.
        if Path(selected).resolve() in [
                (package_dir / 'resource' / drive / 'nav2_params.yaml').resolve()
                for drive in ('diff', 'mecanum')]:
            from webots_ros2_limo.nav2_compat import prepare_jazzy_params
            directory = tempfile.TemporaryDirectory(prefix='limo-nav2-')
            target = str(Path(directory.name) / 'nav2_params.yaml')
            prepare_jazzy_params(selected, Path(nav2_dir) / 'params/nav2_params.yaml', target)
            selected = target
            cleanup.append(RegisterEventHandler(OnShutdown(
                on_shutdown=lambda event, context: directory.cleanup())))
    # A node-specific yaml_filename in the parameters can override Nav2's
    # wildcard CLI override. Write the selected map into that exact node entry.
    directory_map = tempfile.TemporaryDirectory(prefix='limo-map-params-')
    params = yaml.safe_load(Path(selected).read_text())
    params.setdefault('map_server', {}).setdefault('ros__parameters', {})['yaml_filename'] = str(
        Path(LaunchConfiguration('map').perform(context)).expanduser().resolve())
    selected = str(Path(directory_map.name) / 'nav2_params.yaml')
    Path(selected).write_text(yaml.safe_dump(params))
    cleanup.append(RegisterEventHandler(OnShutdown(
        on_shutdown=lambda event, context: directory_map.cleanup())))
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch/bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': selected,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_composition': LaunchConfiguration('use_composition').perform(context).capitalize(),
            'slam': LaunchConfiguration('slam').perform(context).capitalize(),
        }.items()), *cleanup]


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_limo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(package_dir, 'resource', 'limo_world_map.yaml')
    # resource/diff/nav2_params.yaml or resource/mecanum/nav2_params.yaml
    default_params = PathJoinSubstitution(
        [package_dir, 'resource', LaunchConfiguration('drive'), 'nav2_params.yaml'])
    default_rviz = os.path.join(nav2_bringup_dir, 'rviz',
                                'nav2_default_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config = LaunchConfiguration('rviz_config', default=default_rviz)

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time,
            'qos_overrides./tf.subscription.reliability': 'best_effort',
            'qos_overrides./tf.subscription.depth': 20,
        }],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': LaunchConfiguration('rviz_software_rendering')},
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        DeclareLaunchArgument(
            'drive', default_value='diff', choices=['diff', 'mecanum'],
            description='Drive type of the LIMO; selects the default params_file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Webots /clock as ROS 2 time'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz with the Nav2 view'),
        DeclareLaunchArgument('use_composition', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('shutdown_on_rviz_exit', default_value='false'),
        DeclareLaunchArgument(
            'rviz_software_rendering', default_value='0', choices=['0', '1'],
            description='Use software OpenGL for RViz when GPU map shaders fail'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Map yaml for AMCL to localise against'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='RViz configuration file'),

        OpaqueFunction(function=start_navigation),

        RegisterEventHandler(OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='RViz exited; stopping navigation'))]),
            condition=IfCondition(LaunchConfiguration('shutdown_on_rviz_exit'))),
        rviz,
    ])
