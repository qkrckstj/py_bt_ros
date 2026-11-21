"""SLAM for real scan + existing odom TF, without starting any motor driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package = get_package_share_directory('webots_ros2_limo')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            package, 'resource', 'real_slam.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch',
                'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'false', 'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items()),
    ])
