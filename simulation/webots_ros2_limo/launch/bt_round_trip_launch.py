"""Start Nav2 and the external py_bt_ros LIMO round-trip scenario.

Start robot_launch.py first and wait for Webots /odom before running this launch.
"""
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start_bt(context):
    root = Path(LaunchConfiguration('py_bt_ros_root').perform(context)).expanduser().resolve()
    config = root / 'scenarios/limo_round_trip/config_sim.yaml'
    for required in (root / 'main.py', config, root / 'scenarios/limo_round_trip/agent.py'):
        if not required.is_file():
            raise RuntimeError(f'Missing {required}. Use py_bt_ros with the limo_round_trip scenario; '
                               'set py_bt_ros_root:=/path/to/py_bt_ros if needed.')
    return [ExecuteProcess(
        cmd=[sys.executable, str(root / 'main.py'), '--config', str(config)],
        cwd=str(root), output='screen', name='limo_round_trip_bt',
        additional_env={'PYTHONUNBUFFERED': '1'})]


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_limo')
    return LaunchDescription([
        DeclareLaunchArgument('py_bt_ros_root', default_value=os.environ.get(
            'PY_BT_ROS_ROOT', str(Path.home() / 'ros2_ws/src/py_bt_ros'))),
        DeclareLaunchArgument('drive', default_value='diff', choices=['diff', 'mecanum']),
        DeclareLaunchArgument('map', default_value=os.path.join(
            package_dir, 'resource/limo_world_map.yaml')),
        DeclareLaunchArgument('nav', default_value='true',
                              description='Start Nav2 too; false if already running'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch/nav2_launch.py')),
            launch_arguments={
                'drive': LaunchConfiguration('drive'),
                'map': LaunchConfiguration('map'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_sim_time': 'true',
                'rviz_config': os.path.join(package_dir, 'rviz/limo_round_trip.rviz'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('nav'))),
        OpaqueFunction(function=start_bt),
    ])
