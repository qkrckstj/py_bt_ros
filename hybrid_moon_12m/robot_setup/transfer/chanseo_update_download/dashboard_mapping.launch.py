"""Add mapping to the hardware already started by the LIMO dashboard."""
import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(os.environ.get('LIMO_EXPLORATION_DIR', str(Path.home() / 'limo_single_exploration')))
    folder = Path(__file__).resolve().parent
    return LaunchDescription([
        ExecuteProcess(
            cmd=[sys.executable, '-u', '-m', 'webots_ros2_limo.slam_scan_adapter',
                 '--ros-args', '-p', 'real_no_return:=true', '-p', 'min_period:=0.2'],
            additional_env={'PYTHONPATH': str(root) + ':' + os.environ.get('PYTHONPATH', '')},
            output='screen',
        ),
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=[str(root / 'resource/real_slam.yaml'), {
                'use_sim_time': False, 'scan_topic': '/scan_slam',
                'minimum_time_interval': 0.2,
                'minimum_travel_distance': 0.0, 'minimum_travel_heading': 0.15,
                'map_update_interval': 1.0, 'map_file_name': '',
            }],
        ),
        ExecuteProcess(
            cmd=[sys.executable, '-u', str(folder / 'real_observer.py'),
                 '--target', os.environ['HYBRID_LAPTOP_IP']],
            output='screen',
        ),
    ])
