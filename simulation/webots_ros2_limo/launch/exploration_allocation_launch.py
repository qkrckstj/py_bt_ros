"""Only team/robot BTs; attach to an existing four-robot SLAM/Nav2 simulation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        Node(package='webots_ros2_limo', executable='exploration_team', output='screen',
             parameters=[{'use_sim_time': sim_time, 'map_frame': LaunchConfiguration('map_frame'),
                          'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool)}],
             remappings=[('/map', LaunchConfiguration('map_topic'))]),
        *[Node(package='webots_ros2_limo', executable='exploration_robot', namespace=robot,
               output='screen', parameters=[{'use_sim_time': sim_time,
                   'map_frame': LaunchConfiguration('map_frame'), 'base_frame': robot + '/base_link'}])
          for robot in ('robot1', 'robot2', 'robot3', 'robot4')],
    ])
