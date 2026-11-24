"""One real LIMO: live SLAM, frontier allocation and existing robot BT.

Run on the robot's ROS Humble host. Its existing hardware stack is reused by
default. No saved map, AMCL, Webots, radio peers or second base driver starts.
"""
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


ROOT = Path(__file__).resolve().parents[1]


def setup(context):
    if os.environ.get('ROS_DISTRO') != 'humble':
        raise RuntimeError('Real single-LIMO exploration requires ROS Humble on the robot')
    profile = str(Path(LaunchConfiguration('nav2_params').perform(context)).expanduser().resolve())
    slam = str(Path(LaunchConfiguration('slam_params').perform(context)).expanduser().resolve())
    for file in (profile, slam):
        if not Path(file).is_file():
            raise FileNotFoundError(file)
    bt_script = str(ROOT / 'scripts/real_single_bt.py')
    env = {'PYTHONPATH': str(ROOT) + os.pathsep + os.environ.get('PYTHONPATH', ''),
           'PYTHONNOUSERSITE': '1'}
    hardware = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / 'launch' / file)))
        for package, file in (
            ('limo_description', 'load_urdf.launch.py'),
            ('limo_base', 'limo_base.launch.py'),
            ('robot_localization', 'limo_ekf_launch.py'),
        )
    ] if LaunchConfiguration('hardware').perform(context) == 'true' else []
    if hardware:
        hardware.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('ydlidar_ros2_driver')) / 'launch/ydlidar.launch.py'))))
    scan_adapter = ExecuteProcess(cmd=[sys.executable, '-u', '-m',
        'webots_ros2_limo.slam_scan_adapter', '--ros-args',
        '-p', 'real_no_return:=true', '-p', 'min_period:=0.5'],
        additional_env=env, output='screen')
    slam_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory('slam_toolbox')) / 'launch/online_async_launch.py')),
        launch_arguments={'slam_params_file': slam, 'use_sim_time': 'false',
                          'autostart': 'true', 'use_lifecycle_manager': 'false'}.items())
    nav_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory('nav2_bringup')) / 'launch/navigation_launch.py')),
        launch_arguments={'params_file': profile, 'use_sim_time': 'false',
                          'autostart': 'true', 'use_composition': 'False'}.items())
    team = ExecuteProcess(cmd=[sys.executable, '-u', bt_script, 'team', '--ros-args',
        '-r', '__node:=exploration_team',
        '-p', 'robots:=[robot1]',
        '-p', 'map_frame:=map',
        '-p', 'autostart:=false',
        '-p', 'strategy:=frontier',
        '-p', 'radio_enabled:=false',
        '-p', 'require_mapping_control:=false',
        '-p', 'return_home:=false',
        '-p', 'completion_mode:=frontier',
        '-p', 'completion_hold:=10.0',
        '-p', 'input_timeout:=10.0',
        '-p', 'clearance:=0.24',
        '-p', 'min_goal_distance:=0.3',
        '-p', 'goal_timeout:=120.0'], additional_env=env, output='screen')
    robot = ExecuteProcess(cmd=[sys.executable, '-u', bt_script, 'robot', '--ros-args',
        '-r', '__node:=exploration_robot', '-r', '__ns:=/robot1',
        '-p', 'map_frame:=map', '-p', 'base_frame:=base_link',
        '-p', 'nav_action:=/navigate_to_pose',
        '-p', 'radio_enabled:=false',
        '-p', 'pose_timeout:=2.0',
        '-p', 'observe_seconds:=0.5'], additional_env=env, output='screen')
    return [*hardware, scan_adapter, slam_launch, nav_launch, team, robot]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='false', choices=['true', 'false'],
                              description='Start chassis, lidar, URDF and odom if none already run'),
        DeclareLaunchArgument('slam_params', default_value=str(ROOT / 'resource/real_slam.yaml')),
        DeclareLaunchArgument('nav2_params', default_value=str(ROOT / 'resource/real_single_nav2.yaml')),
        OpaqueFunction(function=setup),
    ])
