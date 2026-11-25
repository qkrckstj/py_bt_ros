"""사전 지형 기반 시간 경로 실험: 기본은 시각화, execute:=true로 주행."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('moon_mapping_demo'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/shadow_experiment.yaml')),
        DeclareLaunchArgument('execute', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('output', default_value='results/shadow_hungarian_ros'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/moon_mapping.launch.py')),
            launch_arguments={'shadow_mode': 'true', 'shadow_config': LaunchConfiguration('config'),
                              'rviz': LaunchConfiguration('rviz')}.items()),
        Node(package='moon_mapping_demo', executable='moon_shadow_mission', output='screen',
             parameters=[{'use_sim_time': True, 'config': LaunchConfiguration('config'),
                          'execute': LaunchConfiguration('execute'), 'output': LaunchConfiguration('output')}]),
    ])
