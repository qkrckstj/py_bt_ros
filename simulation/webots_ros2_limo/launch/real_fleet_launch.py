"""One sensorless real-pose replica and two physical simulated peers."""
import ipaddress
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_limo.map_to_webots import load_map, render_world, validate_spawn


def build_assets(map_path, share, multi, output, robot_ip, *, wall_height=0.5, starts=None):
    occupancy = load_map(map_path)
    # robot1 is a display only: it may overlap mapped walls when real localization
    # is wrong. Its pose is never constrained by simulated collisions.
    if starts is None:
        starts = {'robot2': (.8, 0., 0.), 'robot3': (1.6, 0., 0.)}
    for pose in starts.values():
        validate_spawn(occupancy, *pose)
    world, _ = render_world(occupancy, robot_x=0., robot_y=0., robot_yaw=0.,
                            proto=share / 'protos/LimoFourDiff.proto', wall_height=wall_height)
    world = world[:world.index('DEF LIMO LimoFourDiff {')]
    world = world.replace('WorldInfo {',
        f'EXTERNPROTO "{share / "protos/LimoReplica.proto"}"\nWorldInfo {{', 1)
    world += 'DEF REAL_LIMO LimoReplica { name "robot1" }\n'
    for name, (x, y, yaw) in starts.items():
        world += (f'LimoFourDiff {{ name "{name}" translation {x} {y} 0.145 '
                  f'rotation 0 0 1 {yaw} controller "<extern>" }}\n')
    world_path = output / 'real_fleet.wbt'
    world_path.write_text(world)
    descriptions = {}
    for name in ('robot1', 'robot2', 'robot3'):
        if name == 'robot1':
            root = ET.Element('robot', name=name)
            webots = ET.SubElement(root, 'webots')
            plugin = ET.SubElement(webots, 'plugin', type='webots_ros2_limo.pose_replica.PoseReplica')
            ET.SubElement(plugin, 'robot_ip').text = robot_ip
        else:
            root = ET.parse(multi / 'config' / f'{name}.urdf').getroot()
            plugin = root.find('./webots/plugin')
            ET.SubElement(plugin, 'camera_preview').text = 'false'
            ET.SubElement(plugin, 'require_bt_heartbeat').text = 'false'
        descriptions[name] = output / f'{name}.urdf'
        ET.ElementTree(root).write(descriptions[name], encoding='unicode')
    return world_path, descriptions, starts


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('webots_ros2_limo'))
    multi = Path(arg('multirobot_source')).expanduser().resolve(strict=True)
    if not (multi / 'multirobot_sim/driver.py').is_file():
        raise ValueError('multirobot_source must contain multirobot_sim/driver.py')
    robot_ip = str(ipaddress.IPv4Address(arg('robot_ip')))
    sim_domain, real_domain = int(arg('sim_domain')), int(arg('real_domain'))
    if not (0 <= sim_domain <= 232 and 0 <= real_domain <= 232) or sim_domain == real_domain:
        raise ValueError('real and simulation ROS domains must be valid and distinct')
    directory = Path(tempfile.mkdtemp(prefix='limo_real_fleet_'))
    starts = {name: tuple(float(arg(f'{name}_{axis}')) for axis in ('x', 'y', 'yaw'))
              for name in ('robot2', 'robot3')}
    world, descriptions, starts = build_assets(
        Path(arg('map')).expanduser(), share, multi, directory, robot_ip,
        wall_height=float(arg('wall_height')), starts=starts)
    webots = WebotsLauncher(world=str(world), mode='realtime', gui=arg('gui'),
                           port=arg('webots_port'), ros2_supervisor=True)
    actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(sim_domain)),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('PYTHONPATH', str(multi) + ':' + os.environ.get('PYTHONPATH', '')),
        webots, webots._supervisor,
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda context: shutil.rmtree(directory, ignore_errors=True))])),
        RegisterEventHandler(OnProcessExit(target_action=webots, on_exit=[
            EmitEvent(event=Shutdown(reason='Webots closed'))])),
    ]
    for name, description in descriptions.items():
        actions.append(WebotsController(robot_name=name, namespace=name, port=arg('webots_port'),
                       parameters=[{'robot_description': str(description), 'use_sim_time': True,
                                    'set_robot_state_publisher': False}]))
    for name, (x, y, yaw) in starts.items():
        transforms = [('map', name + '/odom', x, y, 0., yaw),
                      (name + '/base_link', name + '/laser_link', .103, 0., -.034, 0.),
                      (name + '/base_link', name + '/camera_link', .096, .015, .03, 0.)]
        for i, (parent, child, x, y, z, yaw) in enumerate(transforms):
            actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                name=f'{name}_mount_{i}', arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                '--yaw', str(yaw), '--frame-id', parent, '--child-frame-id', child]))
    actions.append(Node(package='webots_ros2_limo', executable='peer_relay',
                        arguments=['send', '--robot-ip', robot_ip], output='screen', respawn=True))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map'), DeclareLaunchArgument('real_domain', default_value='0'),
        DeclareLaunchArgument('sim_domain', default_value='74'),
        DeclareLaunchArgument('robot_ip', default_value='172.20.10.5'),
        DeclareLaunchArgument('wall_height', default_value='0.5'),
        DeclareLaunchArgument('robot2_x', default_value='0.8'),
        DeclareLaunchArgument('robot2_y', default_value='0.0'),
        DeclareLaunchArgument('robot2_yaw', default_value='0.0'),
        DeclareLaunchArgument('robot3_x', default_value='1.6'),
        DeclareLaunchArgument('robot3_y', default_value='0.0'),
        DeclareLaunchArgument('robot3_yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('webots_port', default_value='1254'),
        DeclareLaunchArgument('multirobot_source', default_value=str(Path.home() /
            'limo_multi_allocator_ws/src/py_bt_ros/ros_packages/multirobot_sim')),
        OpaqueFunction(function=setup),
    ])
