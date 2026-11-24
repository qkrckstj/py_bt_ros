"""Robot-side sensors and saved-map localization, without navigation or camera tilt.

Run as an absolute launch file on the confirmed WEGO Humble installation.
An operator must set /initialpose in the saved map before mirroring can arm.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    actions = [DeclareLaunchArgument('map', description='Saved occupancy map YAML')]
    for package, name in [
        ('limo_description', 'load_urdf.launch.py'),
        ('limo_base', 'limo_base.launch.py'),
        ('ydlidar_ros2_driver', 'ydlidar.launch.py'),
        ('robot_localization', 'limo_ekf_launch.py'),
    ]:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', name))))
    params = os.path.join(get_package_share_directory('wego_2d_nav'),
                          'params', 'diff_navigation_params.yaml')
    actions.extend([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': LaunchConfiguration('map'),
                                          'use_sim_time': False}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, {'use_sim_time': False, 'set_initial_pose': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='twin_localization_manager', output='screen',
             parameters=[{'autostart': True, 'use_sim_time': False,
                          'node_names': ['map_server', 'amcl']}]),
    ])
    return LaunchDescription(actions)
