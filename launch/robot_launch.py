#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots TurtleBot3 Burger driver."""

import os
import sys
from pathlib import Path
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnShutdown
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def launch_robot(context):
    package_dir = get_package_share_directory('webots_ros2_turtlebot')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    homework = launch.conditions.IfCondition(use_nav).evaluate(context)
    root = Path(os.environ.get('PY_BT_ROS_ROOT', str(Path.home() / 'ros2_ws/src/py_bt_ros')))
    setup_actions = []
    world_source = PathJoinSubstitution([package_dir, 'worlds', world])
    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')
    if homework:
        if not (root / 'main.py').is_file():
            raise RuntimeError(f'py_bt_ros not found: {root}. Set PY_BT_ROS_ROOT to its source directory.')
        sys.path.insert(0, str(root))
        from scenarios.turtlebot3_photo.launch_support import prepare_resources
        selected_world = Path(world.perform(context))
        if not selected_world.is_absolute():
            selected_world = Path(package_dir) / 'worlds' / selected_world
        sim_time = launch.conditions.IfCondition(use_sim_time).evaluate(context)
        resources = prepare_resources(root, Path(package_dir),
            Path(get_package_share_directory('nav2_bringup')), selected_world, sim_time)
        resource_dir = Path(resources.name)
        world_source = str(resource_dir / 'photo_mission.wbt')
        robot_description_path = str(resource_dir / 'robot.urdf')
        setup_actions.extend([
            SetEnvironmentVariable('PYTHONPATH', str(root) + os.pathsep + os.environ.get('PYTHONPATH', '')),
            RegisterEventHandler(OnShutdown(on_shutdown=lambda event, ctx: resources.cleanup())),
        ])

    webots = WebotsLauncher(
        world=world_source,
        mode=mode,
        ros2_supervisor=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name="placeholder"><link name="base_link"/></robot>',
            'use_sim_time': use_sim_time,
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    if homework:
        mappings.append(('/robot_description', '/control_robot_description'))
    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': not homework},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # Navigation
    navigation_nodes = []
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    nav2_map = os.path.join(package_dir, 'resource', 'turtlebot3_burger_example_map.yaml')
    if homework:
        # Nav2 embeds `slam` in PythonExpression, which requires True/False.
        # IfCondition also accepts lowercase CLI values such as slam:=false.
        nav2_slam = str(launch.conditions.IfCondition(use_slam).evaluate(context))
        turtlebot_navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('params_file', str(resource_dir / 'nav2_params.yaml')),
                ('use_sim_time', use_sim_time),
                ('slam', nav2_slam),
                ('use_composition', 'False'),
            ],
            condition=launch.conditions.IfCondition(use_nav))
        navigation_nodes.append(turtlebot_navigation)
        navigation_nodes.extend([
            ExecuteProcess(cmd=[sys.executable, str(root / 'main.py'),
                                '--config', str(resource_dir / 'bt_config.yaml')],
                           cwd=str(root), output='screen'),
            ExecuteProcess(cmd=[sys.executable,
                                str(root / 'scenarios/turtlebot3_photo/capture_image_server.py'),
                                '--ros-args', '-p', ['use_sim_time:=', use_sim_time],
                                '-p', ['output_dir:=', LaunchConfiguration('output_dir')]],
                           cwd=str(root), output='screen'),
            Node(package='rviz2', executable='rviz2',
                 arguments=['-d', str(root / 'scenarios/turtlebot3_photo/photo_mission.rviz')],
                 parameters=[{'use_sim_time': use_sim_time}], output='screen'),
        ])

    # SLAM
    if 'turtlebot3_cartographer' in get_packages_with_prefixes():
        turtlebot_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_slam))
        navigation_nodes.append(turtlebot_slam)

    # Wait for the simulation to be ready to start navigation nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=navigation_nodes + ros_control_spawners
    )

    return setup_actions + [
        webots,
        webots._supervisor,

        robot_state_publisher,
        footprint_publisher,

        turtlebot_driver,
        waiting_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ]


def generate_launch_description():
    root = Path(os.environ.get('PY_BT_ROS_ROOT', str(Path.home() / 'ros2_ws/src/py_bt_ros')))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='turtlebot3_burger_example.wbt'),
        DeclareLaunchArgument('mode', default_value='realtime'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('slam', default_value='False'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value=str(root / 'output/captures')),
        OpaqueFunction(function=launch_robot),
    ])
