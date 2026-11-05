"""Nav2 + RViz + the py_bt_ros factory person-search BT (simulator separate)."""
import os
from pathlib import Path
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    from webots_ros2_limo.session_guard import acquire_factory_session
    session = acquire_factory_session()
    # Hold the lock through child-process shutdown, not just until Shutdown fires.
    # The launch process owns this descriptor until it exits.
    context.extend_globals({'factory_navigation_session': session})
    package = Path(get_package_share_directory('webots_ros2_limo'))
    root = Path(LaunchConfiguration('py_bt_ros_root').perform(context)).expanduser().resolve()
    config_path = root / 'scenarios/limo_factory_search/config_sim.yaml'
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    for required in (config_path, root / 'main.py', map_path):
        if not required.is_file():
            raise RuntimeError(f'Missing required factory-search file: {required}')
    config = yaml.safe_load(config_path.read_text())
    if LaunchConfiguration('mock_detector').perform(context).lower() != 'true':
        yolo_python = Path(LaunchConfiguration('yolo_python').perform(context)).expanduser()
        if not yolo_python.is_file():
            raise RuntimeError(f'YOLO Python not found: {yolo_python}. See the factory-search README installation steps.')
    directory = tempfile.TemporaryDirectory(prefix='limo-factory-nav-')
    target = Path(directory.name) / 'params.yaml'
    source = package / 'resource/diff/nav2_params.yaml'
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    if os.environ.get('ROS_DISTRO') == 'jazzy':
        from webots_ros2_limo.nav2_compat import prepare_jazzy_params
        prepare_jazzy_params(source, nav2 / 'params/nav2_params.yaml', target)
    else:
        target.write_text(source.read_text())
    params = yaml.safe_load(target.read_text())
    from webots_ros2_limo.factory_navigation import configure_factory, relative_regions
    configure_factory(params, float(LaunchConfiguration('goal_tolerance').perform(context)))
    slam = LaunchConfiguration('slam').perform(context).lower() == 'true'
    if slam:
        slam_share = Path(get_package_share_directory('slam_toolbox'))
        params['slam_toolbox'] = yaml.safe_load((slam_share / 'config/mapper_params_online_sync.yaml').read_text())['slam_toolbox']
        params['slam_toolbox']['ros__parameters'].update(
            base_frame='base_link', use_sim_time=True, scan_topic='/scan_slam',
            min_laser_range=0.1, max_laser_range=12.0, map_update_interval=1.0,
            minimum_travel_distance=0.2, minimum_travel_heading=0.2)
        relative_regions(config)
        config_path = Path(directory.name) / 'bt_config.yaml'
        config_path.write_text(yaml.safe_dump(config))
    amcl = params['amcl']['ros__parameters']
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = config['mission']['start_pose']
    target.write_text(yaml.safe_dump(params))
    regions = config['mission']['mock_person_regions']
    bt_process = ExecuteProcess(
        cmd=[sys.executable, str(root / 'main.py'), '--config', str(config_path)],
        cwd=str(root), output='screen', name='factory_search_bt',
        additional_env={'PYTHONUNBUFFERED': '1'})
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=bt_process,
            on_exit=[EmitEvent(event=Shutdown(reason='Factory BT exited; stopping its Nav2 stack'))])),
        Node(package='webots_ros2_limo', executable='slam_scan_adapter',
             parameters=[{'use_sim_time': True}], output='screen',
             condition=IfCondition(LaunchConfiguration('slam'))),
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: directory.cleanup())),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch/nav2_launch.py')),
            launch_arguments={'drive': 'diff', 'map': str(map_path), 'params_file': str(target),
                              'use_sim_time': 'true', 'use_rviz': LaunchConfiguration('use_rviz'),
                              'rviz_software_rendering': '1',
                              'shutdown_on_rviz_exit': 'true',
                              'use_composition': 'false',
                              'slam': str(slam),
                              'rviz_config': str(package / 'rviz/limo_factory_search.rviz')}.items()),
        bt_process,
        ExecuteProcess(
            cmd=['bash', str(root / 'ros_packages/human_yolo/scripts/run_detector.sh'),
                 '-p', 'use_sim_time:=true',
                 '-r', f'/perception/is_person:={config["mission"]["person_topic"]}',
                 '-r', f'/perception/person_pose:={config["mission"]["person_pose_topic"]}'],
            cwd=str(root), output='screen', name='human_yolo',
            additional_env={'YOLO_PYTHON': LaunchConfiguration('yolo_python'),
                            'PYTHONUNBUFFERED': '1'},
            condition=UnlessCondition(LaunchConfiguration('mock_detector'))),
        ExecuteProcess(
            cmd=[sys.executable, '-m', 'scenarios.limo_factory_search.mock_person_detector',
                 '--ros-args', '-p', f'person_regions:={regions}',
                 '-p', f'person_topic:={config["mission"]["person_topic"]}',
                 '-p', 'use_sim_time:=true',
                 '-p', f'person_pose_topic:={config["mission"]["person_pose_topic"]}',
                 '-p', f'person_x:={config["mission"]["mock_person_pose"]["x"]}',
                 '-p', f'person_y:={config["mission"]["mock_person_pose"]["y"]}',
                 '-p', f'person_z:={config["mission"]["mock_person_pose"]["z"]}',
                 '-p', f'detection_distance:={config["mission"]["mock_detection_distance"]}'],
            cwd=str(root), output='screen', name='mock_person_detector',
            condition=IfCondition(LaunchConfiguration('mock_detector'))),
    ]


def generate_launch_description():
    package = Path(get_package_share_directory('webots_ros2_limo'))
    return LaunchDescription([
        DeclareLaunchArgument('py_bt_ros_root', default_value=os.environ.get(
            'PY_BT_ROS_ROOT', str(Path.home() / 'ros2_ws/src/py_bt_ros'))),
        DeclareLaunchArgument('map', default_value=str(package / 'resource/limo_factory_map.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false', choices=['true', 'false'],
                              description='Build a new live map instead of loading the saved map/AMCL'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.30'),
        DeclareLaunchArgument('mock_detector', default_value='false', choices=['true', 'false'],
                              description='Use the distance-based mock instead of the trained YOLO model'),
        DeclareLaunchArgument('yolo_python', default_value=os.environ.get(
            'YOLO_PYTHON', str(Path.home() / '.venvs/human-yolo/bin/python'))),
        OpaqueFunction(function=setup),
    ])
