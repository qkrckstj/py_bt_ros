from pathlib import Path
from setuptools import find_packages, setup

package_name = 'webots_ros2_limo'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/protos', [
    'protos/LimoFourDiff.proto',
    'protos/LimoMecanum.proto',
    'protos/LimoReplica.proto',
]))
data_files.append(('share/' + package_name + '/protos/meshes', [
    'protos/meshes/limo_base.dae',
    'protos/meshes/limo_wheel.dae',
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/limo_world.wbt',
    'worlds/limo_world_mecanum.wbt',
    'worlds/limo_factory_hall.wbt',
    'worlds/.limo_factory_hall.wbproj',
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/limo.urdf',
    'resource/limo_lds_2d.lua',
    'resource/limo_world_map.yaml',
    'resource/limo_world_map.pgm',
    'resource/limo_factory_map.yaml',
    'resource/limo_factory_map.pgm',
    'resource/limo_factory_map.survey.json',
    'resource/exploration_nav2_template.yaml',
    'resource/real_slam.yaml',
    'resource/real_single_nav2.yaml',
]))
for drive in ('diff', 'mecanum'):
    data_files.append(('share/' + package_name + '/resource/' + drive, [
        'resource/' + drive + '/ros2control.yaml',
        'resource/' + drive + '/nav2_params.yaml',
    ]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/robot_launch.py',
    'launch/cartographer_launch.py',
    'launch/nav2_launch.py',
    'launch/bt_round_trip_launch.py',
    'launch/factory_search_launch.py',
    'launch/exploration_allocation_launch.py',
    'launch/exploration_flat_launch.py',
    'launch/exploration_visualization_launch.py',
    'launch/real_mapping_launch.py',
    'launch/real_single_exploration_launch.py',
    'launch/real_fleet_launch.py',
    'launch/real_twin_robot.launch.py',
]))
data_files.append(('share/' + package_name + '/rviz', ['rviz/limo_cartographer.rviz', 'rviz/limo_round_trip.rviz', 'rviz/limo_factory_search.rviz', 'rviz/exploration_flat.rviz', 'rviz/real_single_exploration.rviz']))
data_files.append(('share/' + package_name + '/scripts', [
    'scripts/real_single_bt.py',
    'scripts/real_single_exploration.sh',
    'scripts/real_twin.sh',
    'scripts/twin_robot.sh',
    'scripts/twin_keyboard.py',
    'scripts/package_twin_robot.py',
]))
data_files.append(('share/' + package_name, ['package.xml']))
moon_root = Path('resource/moon_map_release')
for directory in [moon_root, *sorted(p for p in moon_root.rglob('*') if p.is_dir())]:
    files = [str(p) for p in sorted(directory.iterdir()) if p.is_file()]
    if files:
        data_files.append(('share/' + package_name + '/' + str(directory), files))


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inmojang',
    maintainer_email='inmo.jang@kau.ac.kr',
    description='AgileX LIMO (four-wheel diff / mecanum) Webots simulation with webots_ros2_driver + ros2_control + Nav2.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'cmd_vel_stamper = webots_ros2_limo.cmd_vel_stamper:main',
            'slam_scan_adapter = webots_ros2_limo.slam_scan_adapter:main',
            'exploration_team = webots_ros2_limo.exploration_ros:team_main',
            'exploration_robot = webots_ros2_limo.exploration_ros:robot_main',
            'exploration_flat_map = webots_ros2_limo.exploration_flat_map:main',
            'exploration_traffic = webots_ros2_limo.exploration_traffic:main',
            'exploration_viewer = webots_ros2_limo.exploration_viewer:main',
            'exploration_startup = webots_ros2_limo.exploration_startup:main',
            'map_to_webots = webots_ros2_limo.map_to_webots:main',
            'twin_bridge = webots_ros2_limo.twin_bridge:main',
            'peer_relay = webots_ros2_limo.peer_relay:main',
        ],
    },
)
