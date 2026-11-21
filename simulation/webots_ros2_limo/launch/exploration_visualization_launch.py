"""Attach read-only RViz and pygame views to an existing exploration run."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('webots_ros2_limo'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=str(share/'rviz/exploration_flat.rviz')),
        DeclareLaunchArgument('bt_viewer', default_value='true'),
        DeclareLaunchArgument('bt_snapshot', default_value=''),
        DeclareLaunchArgument('bt_repo', default_value=str(Path.home()/'ros2_ws/src/py_bt_ros')),
        Node(package='rviz2', executable='rviz2', name='exploration_rviz',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
        Node(package='webots_ros2_limo', executable='exploration_viewer',
             arguments=['--snapshot', LaunchConfiguration('bt_snapshot'),
                        '--repo', LaunchConfiguration('bt_repo')],
             condition=IfCondition(LaunchConfiguration('bt_viewer')),
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
