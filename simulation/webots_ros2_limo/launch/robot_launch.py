#!/usr/bin/env python

# Launch Webots AgileX LIMO driver.
#
# Structure mirrors the official Cyberbotics example
# `webots_ros2/webots_ros2_turtlebot/launch/robot_launch.py`:
#   * `slam:=true`  -> includes turtlebot3_cartographer (cartographer SLAM + RViz)
#   * `nav:=true`   -> includes turtlebot3_navigation2 (Nav2/AMCL + RViz) with a saved map
# Both are optional and only activate when the corresponding package is installed.
#
# `drive:=diff` (default) is the four-wheel differential LIMO, `drive:=mecanum`
# the omnidirectional one. The drive selects the world (robot model), the
# ros2_control controller and the Nav2 parameters in `resource/<drive>/`.

import os
import xml.etree.ElementTree as ET
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_limo.world_variants import without_scenario_props

# Per drive: default world, ros2_control controller and the topic on which
# that controller listens for velocity commands (remapped to /cmd_vel).
DRIVES = {
    'diff': {
        'world': 'limo_world.wbt',
        'controller': 'diffdrive_controller',
        'cmd_vel_topic': '/diffdrive_controller/cmd_vel_unstamped',
    },
    'mecanum': {
        'world': 'limo_world_mecanum.wbt',
        'controller': 'mecanum_drive_controller',
        'cmd_vel_topic': '/mecanum_drive_controller/reference_unstamped',
    },
}


def controller_connection_callback(controller_names, navigation_nodes):
    """Create fresh spawner actions after each driver restart.

    A launch Action cannot execute twice. Navigation starts once, while a newly
    created controller manager after a Webots reload needs fresh spawners.
    """
    connected_pids = set()
    output_buffers = {}
    navigation_started = False

    def connected(event):
        nonlocal navigation_started
        if event.pid in connected_pids:
            return None
        text = output_buffers.get(event.pid, b'') + event.text
        output_buffers[event.pid] = text[-1024:]
        if b'Controller successfully connected to robot in Webots simulation.' not in text:
            return None
        connected_pids.add(event.pid)
        output_buffers.pop(event.pid, None)
        actions = [Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[name, '--controller-manager-timeout', '50'],
        ) for name in controller_names]
        if not navigation_started:
            actions = list(navigation_nodes) + actions
            navigation_started = True
        return actions

    return connected


def launch_setup(context):
    package_dir = get_package_share_directory('webots_ros2_limo')
    drive = LaunchConfiguration('drive').perform(context)
    if drive not in DRIVES:
        raise RuntimeError(f"drive must be one of {list(DRIVES)}, got '{drive}'")
    world = LaunchConfiguration('world').perform(context) or DRIVES[drive]['world']
    mode = LaunchConfiguration('mode')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    nav2_map = LaunchConfiguration(
        'map', default=os.path.join(package_dir, 'resource', 'limo_world_map.yaml'))

    world_path = os.path.join(package_dir, 'worlds', world)
    temporary_world = None
    if LaunchConfiguration('scenario_props').perform(context).lower() == 'false':
        temporary_world = without_scenario_props(world_path)
    cleanup = []
    if temporary_world:
        def remove_temporary_world(_context, **_kwargs):
            if os.path.exists(temporary_world):
                os.unlink(temporary_world)
            return []
        cleanup.append(launch.actions.RegisterEventHandler(
            launch.event_handlers.OnShutdown(on_shutdown=[
                OpaqueFunction(function=remove_temporary_world)])))

    webots = WebotsLauncher(
        world=temporary_world or world_path,
        mode=mode,
        ros2_supervisor=True
    )

    # Jazzy can consume the latched bootstrap description before Webots' async
    # update arrives. An empty robot then permanently initializes zero hardware
    # interfaces. Seed a valid control model; Webots replaces the visual model.
    control = ET.parse(os.path.join(package_dir, 'resource/limo.urdf')).getroot().find('ros2_control')
    bootstrap = ET.Element('robot', name='LIMO')
    ET.SubElement(bootstrap, 'link', name='base_link')
    # The driver's full description can arrive before the parameter service is
    # ready. Keep lidar localization usable even if the bootstrap remains active.
    # Mount matches laser_link in both LIMO PROTOs; RSP remains its sole publisher.
    ET.SubElement(bootstrap, 'link', name='laser_link')
    laser_joint = ET.SubElement(bootstrap, 'joint', name='laser_joint', type='fixed')
    ET.SubElement(laser_joint, 'parent', link='base_link')
    ET.SubElement(laser_joint, 'child', link='laser_link')
    ET.SubElement(laser_joint, 'origin', xyz='0.103 0 -0.034', rpy='0 0 0')
    for controlled_joint in control.findall('joint'):
        name = controlled_joint.attrib['name']
        child = name + '_bootstrap_link'
        ET.SubElement(bootstrap, 'link', name=child)
        joint = ET.SubElement(bootstrap, 'joint', name=name, type='continuous')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'axis', xyz='0 1 0')
        ET.SubElement(joint, 'limit', effort='10', velocity='100')
    bootstrap.append(control)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ET.tostring(bootstrap, encoding='unicode')
        }],
    )

    # No base_link -> base_footprint TF: the real LIMO has no base_footprint, and a
    # config that relies on it (e.g. Nav2 AMCL's default base_frame_id) would work
    # here but fail on the robot.

    # Camera mount TFs: same frames and chain as the real robot (wego/camera_tilt_launch.py,
    # degree:=0), but base_link -> camera_mount is set so that camera_link lands on the Webots
    # camera, i.e. the camera block of the LIMO body mesh: camera_link = (0.096, 0.015, 0.03).
    # (The robot's own values, (0.2, 0.1, 0.06), would put the camera 7 cm in front of the
    # bumper and 5 cm to the left of where it is mounted.)
    camera_mount_tfs = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            output='screen',
            arguments=['--x', x, '--y', y, '--z', z, '--frame-id', parent, '--child-frame-id', child],
        )
        for name, x, y, z, parent, child in [
            ('base_link_to_camera_mount', '0.066', '0.065', '0.03', 'base_link', 'camera_mount'),
            ('camera_mount_to_roate', '0.0', '0.0', '0.0', 'camera_mount', 'camera_rotate'),
            ('rotate_to_camera_link', '0.03', '-0.05', '0.0', 'camera_rotate', 'camera_link'),
        ]
    ]

    robot_description_path = os.path.join(package_dir, 'resource', 'limo.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', drive, 'ros2control.yaml')
    mappings = [(DRIVES[drive]['cmd_vel_topic'], '/cmd_vel')]
    velocity_adapters = []
    if os.environ.get('ROS_DISTRO') == 'jazzy' and drive == 'diff':
        # Jazzy removed cmd_vel_unstamped from diff_drive_controller.
        mappings = []
        velocity_adapters.append(Node(
            package='webots_ros2_limo', executable='cmd_vel_stamper',
            parameters=[{'use_sim_time': use_sim_time}], output='screen'))
    limo_driver = WebotsController(
        robot_name='LIMO',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # /odom and the odom -> base_link TF come from the limo_sensors plugin (see
    # resource/limo.urdf), not from the drive controller's own odometry.

    # Optional stacks. Both launch files live in this package and each brings
    # its own RViz, so these shortcut arguments run exactly what the README
    # asks you to type in a second terminal.
    navigation_nodes = []
    if 'nav2_bringup' in get_packages_with_prefixes():
        limo_navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                package_dir, 'launch', 'nav2_launch.py')),
            launch_arguments=[
                ('drive', drive),
                ('map', nav2_map),
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_nav))
        navigation_nodes.append(limo_navigation)

    if 'cartographer_ros' in get_packages_with_prefixes():
        limo_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                package_dir, 'launch', 'cartographer_launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_slam))
        navigation_nodes.append(limo_slam)

    # Wait for the simulation to be ready to start navigation nodes.
    #
    # NOTE: starting the simulator and the SLAM/Nav stack from this single
    # command loses a startup race in roughly 1 launch in 5 — Nav2's
    # global_costmap can latch the robot at the map origin and never recover.
    # The README documents starting the two stacks in separate terminals,
    # which avoids the race entirely. On reconnect, use new spawner Actions.
    waiting_nodes = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessIO(
            target_action=limo_driver,
            on_stderr=controller_connection_callback(
                [DRIVES[drive]['controller'], 'joint_state_broadcaster'],
                navigation_nodes),
        ))

    return [
        *cleanup,
        webots,
        webots._supervisor,

        robot_state_publisher,
        *camera_mount_tfs,

        limo_driver,
        *velocity_adapters,
        waiting_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_limo')
    return LaunchDescription([
        DeclareLaunchArgument(
            'drive',
            default_value='diff',
            choices=list(DRIVES),
            description='Drive type of the LIMO'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='World file from `webots_ros2_limo/worlds`. Empty: the '
                        'default world of the chosen drive'
        ),
        DeclareLaunchArgument(
            'scenario_props',
            default_value='true',
            choices=['true', 'false'],
            description='Include scenario people/boxes; false for static mapping'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Also start cartographer SLAM + RViz (cartographer_launch.py)'
        ),
        DeclareLaunchArgument(
            'nav',
            default_value='false',
            description='Also start Nav2/AMCL + RViz (nav2_launch.py)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(package_dir, 'resource', 'limo_world_map.yaml'),
            description='Map yaml used when nav:=true'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Webots /clock as ROS 2 time'
        ),
        OpaqueFunction(function=launch_setup),
    ])
