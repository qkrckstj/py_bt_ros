"""TeamBT·RobotBT와 공동 시간 경로의 전용 통합 실행. 기존 BT/Nav2 동시 실행 금지."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('moon_mapping_demo'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/exploration_short.yaml')),
        DeclareLaunchArgument('mode', default_value='explore', choices=['fixed', 'explore']),
        DeclareLaunchArgument('departure_policy', default_value='', choices=['','simultaneous','staggered_pairs']),
        DeclareLaunchArgument('risk_stage', default_value='', choices=['','baseline','early','goals','candidates','heading']),
        DeclareLaunchArgument('execute', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('output', default_value='results/bt_exploration_ros'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/moon_mapping.launch.py')),
            launch_arguments={'shadow_mode': 'true', 'exploration_mode': 'true',
                              'shadow_config': LaunchConfiguration('config'),
                              'rviz': LaunchConfiguration('rviz')}.items()),
        Node(package='moon_mapping_demo', executable='moon_exploration', output='screen',
             parameters=[{'use_sim_time': True, 'config': LaunchConfiguration('config'),
                          'execute': LaunchConfiguration('execute'), 'mode': LaunchConfiguration('mode'),
                          'departure_policy':ParameterValue(LaunchConfiguration('departure_policy'),value_type=str),
                          'risk_stage':ParameterValue(LaunchConfiguration('risk_stage'),value_type=str),
                          'output': LaunchConfiguration('output')}]),
    ])
