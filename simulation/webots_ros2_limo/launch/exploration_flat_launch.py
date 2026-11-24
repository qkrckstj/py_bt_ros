"""Physical four-LIMO flat-world allocation experiment, no preloaded map."""
import json
import math
import re
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, TimerAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_limo.exploration_localization import slam_prior
from webots_ros2_limo.exploration_world import layout
from webots_ros2_limo.exploration_moon import MOON_BASE, wide_moon, circular_radar_moon


def assets(share, output, kind='branched', radio_range=0., base=MOON_BASE, moon_seed=42):
    starts, walls = layout(kind)
    # A test-only PROTO removes unused display overlays. Preserve the real model.
    proto = output / 'LimoFourDiff.proto'
    source = (share / 'protos/LimoFourDiff.proto').read_text()
    while match := re.search(r'\b(?:Camera|RangeFinder|Display)\s*\{', source):
        end, depth = match.end(), 1
        while depth:
            depth += (source[end] == '{') - (source[end] == '}')
            end += 1
        source = source[:match.start()] + source[end:]
    source = source.replace('url "meshes/', f'url "{share / "protos/meshes"}/')
    # Webots returns +inf both for too-near and too-far ranges. Retain near
    # measurements in the test sensor so the mapper can reject them explicitly.
    source = re.sub(r'\bnear\s+0\.1\b', 'near 0.01', source)
    source = re.sub(r'\bminRange\s+0\.1\b', 'minRange 0.01', source)
    # The legacy model places its lidar origin inside the decorative cover.
    # At a 1 cm near plane that cover occludes most rays. Remove only that
    # visual Shape in this temporary model; retain the sensor/collision mount.
    cover = re.search(r'# Lidar sensor.*?\bShape\s*\{', source, re.DOTALL)
    if cover:
        begin = source.rfind('Shape', cover.start(), cover.end())
        end, depth = cover.end(), 1
        while depth:
            depth += (source[end] == '{') - (source[end] == '}')
            end += 1
        source = source[:begin]+source[end:]
    # Raise the optical origin above the legacy chassis mesh. Its former
    # 10 cm near clip hid self-occlusion rather than modelling a clear lens.
    source = re.sub(r'\bLidar\s*\{', 'DEF EXPLORATION_LIDAR Lidar {\n            translation 0 0 0.20', source)
    proto.write_text(source)
    world = f'''#VRML_SIM R2025a utf8
EXTERNPROTO "{proto}"
WorldInfo {{ basicTimeStep 20 contactProperties [ ContactProperties {{ material1 "wheel" material2 "default" coulombFriction 1 forceDependentSlip 0.01 bounce 0 }} ] }}
Viewpoint {{ orientation 0 1 0 1.57079632679 position 0 0 48 fieldOfView 0.9 }}
Background {{ skyColor [0.15 0.18 0.23] }}
DirectionalLight {{ direction -0.5 -0.3 -1 intensity 1 }}
Solid {{ name "flat_ground" translation 0 0 -0.05 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.55 0.57 0.60 roughness 1 metalness 0 }} geometry Box {{ size 26 26 0.1 }} }} ] boundingObject Box {{ size 26 26 0.1 }} }}
'''
    for index, (x, y, sx, sy) in enumerate(walls):
        world += f'Solid {{ name "boundary_{index}" translation {x} {y} 0.25 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.3 0.35 0.4 roughness 1 }} geometry Box {{ size {sx} {sy} 0.5 }} }} ] boundingObject Box {{ size {sx} {sy} 0.5 }} }}\n'
    if kind in ('moon', 'moon_radar'):
        moon = share/'resource/moon_map_release/worlds/moon_terrain.wbt'
        world, terrain = (circular_radar_moon if kind == 'moon_radar' else wide_moon)(moon.read_text(), moon_seed)
        (output/'moon_layout.json').write_text(json.dumps(terrain, indent=2))
        world = re.sub(r'EXTERNPROTO\s+"[^"]+"', f'EXTERNPROTO "{proto}"', world, count=1)
        world = world.replace('"textures/moon_ground.jpg"', f'"{moon.parent}/textures/moon_ground.jpg"')
        world = re.sub(r'DEF LIMO_\d LimoFourDiff\s*\{[^{}]*\}', '', world)
        # The existing observer owns the sun and radio visuals, avoiding another
        # Supervisor/controller search path after WebotsLauncher copies the world.
        world = re.sub(r'Robot\s*\{\s*name "lunar_sun_supervisor"[^{}]*\}', '', world)
        world = world.replace('WorldInfo {', 'WorldInfo { basicTimeStep 20\n', 1)
        height = 140 if kind == 'moon_radar' else 52
        world = re.sub(r'DEF DEMO_VIEW Viewpoint\s*\{[^{}]*\}',
            f'DEF DEMO_VIEW Viewpoint {{ orientation -0.577350269 0.577350269 0.577350269 2.094395102 position 0 0 {height} fieldOfView 0.9 }}', world)
        world += f'\nDEF BASE_MARKER Transform {{ translation {base[0]} {base[1]} 0.3 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.2 0.6 1 emissiveColor 0.1 0.3 0.5 }} geometry Box {{ size 0.35 0.35 0.6 }} }} ] }}\n'
    if radio_range > 0:
        colors = ['1 1 1', '0.95 0.25 0.25', '0.25 0.9 0.4', '0.3 0.55 1', '1 0.75 0.2']
        for name, color in zip(['BASE', *[n.upper() for n in starts]], colors):
            points = ', '.join(f'{radio_range*math.cos(i*math.tau/96):.4f} {radio_range*math.sin(i*math.tau/96):.4f} 0' for i in range(97))
            indices = ' '.join(str(i) for i in range(97))+' -1'
            world += f'DEF RADIO_{name} Transform {{ children [ Shape {{ appearance Appearance {{ material Material {{ diffuseColor {color} emissiveColor {color} }} }} geometry IndexedLineSet {{ coord Coordinate {{ point [ {points} ] }} coordIndex [ {indices} ] }} castShadows FALSE }} ] }}\n'
        link_points = ', '.join(['0 0 0']*20)
        link_indices = ', '.join(f'{2*i} {2*i+1} -1' for i in range(10))
        world += f'DEF RADIO_LINKS Shape {{ appearance Appearance {{ material Material {{ emissiveColor 0.2 1 0.5 }} }} geometry IndexedLineSet {{ coord Coordinate {{ point [{link_points}] }} coordIndex [{link_indices}] }} castShadows FALSE }}\n'
    descriptions = {}
    if kind == 'branched':
        # Visual-only staging pad; no collision shape or prior occupancy map.
        world += 'Transform { translation 0 -9.5 0.002 children [ Shape { appearance PBRAppearance { baseColor 0.2 0.4 0.6 roughness 1 metalness 0 } geometry Box { size 5.5 3 0.003 } } ] }\n'
    for name, (x, y, yaw) in starts.items():
        world += f'DEF {name.upper()} LimoFourDiff {{ name "{name}" translation {x} {y} 0.145 rotation 0 0 1 {yaw} controller "<extern>" }}\n'
        descriptions[name] = output / f'{name}.urdf'
        descriptions[name].write_text(f'''<robot name="{name}"><webots>
<device reference="laser"><ros><enabled>false</enabled></ros></device>
<device reference="imu compass"><ros><enabled>false</enabled></ros></device>
<plugin type="webots_ros2_limo.exploration_flat_driver.FlatDriver"><namespace>{name}</namespace></plugin>
</webots></robot>''')
    world += 'Robot { name "observer" supervisor TRUE controller "<extern>" }\n'
    descriptions['observer'] = output / 'observer.urdf'
    sun_path = str(share/'resource/moon_map_release/controllers/lunar_sun') if kind in ('moon', 'moon_radar') else ''
    descriptions['observer'].write_text('<robot name="observer"><webots><plugin type="webots_ros2_limo.exploration_observer.FlatObserver"><starts_json>'+json.dumps(starts)+f'</starts_json><sun_path>{sun_path}</sun_path><radio_range>{radio_range}</radio_range><base_json>{json.dumps(base)}</base_json><overview_height>{140 if kind == "moon_radar" else 0}</overview_height></plugin></webots></robot>')
    path = output / 'flat_four_limo.wbt'
    path.write_text(world)
    return path, descriptions, starts


def runtime(context):
    share = Path(get_package_share_directory('webots_ros2_limo'))
    output = Path(tempfile.mkdtemp(prefix='limo_flat_'))
    kind = LaunchConfiguration('world').perform(context)
    radio_enabled = LaunchConfiguration('radio_enabled').perform(context).lower() == 'true'
    radio_range = float(LaunchConfiguration('radio_range').perform(context))
    base = list(MOON_BASE) if kind == 'moon' else [0., 0.] if kind == 'moon_radar' else [0., -10.5]
    radio_params = {'radio_enabled': radio_enabled, 'radio_range': radio_range,
                    'radio_margin': .5, 'base_position': base}
    world, descriptions, starts = assets(share, output, kind, radio_range if radio_enabled else 0., base,
                                        int(LaunchConfiguration('moon_seed').perform(context)))
    rviz_config = share/'rviz/exploration_flat.rviz'
    if kind in ('moon', 'moon_radar'):
        config = yaml.safe_load(rviz_config.read_text())
        config['Visualization Manager']['Views']['Current']['Scale'] = 72 if kind == 'moon_radar' else 46
        rviz_config = output/'moon.rviz'
        rviz_config.write_text(yaml.safe_dump(config))
    port = LaunchConfiguration('webots_port').perform(context)
    webots = WebotsLauncher(world=str(world), gui=LaunchConfiguration('gui'), mode='realtime',
                            port=port, ros2_supervisor=True)
    actions = [webots, webots._supervisor]
    actions.append(WebotsController(robot_name='observer', port=port,
        parameters=[{'robot_description': str(descriptions['observer']),
                     'set_robot_state_publisher': False}], output='screen'))
    stacks = [Node(package='webots_ros2_limo', executable='exploration_flat_map', output='screen',
                   parameters=[{'use_sim_time': True, 'starts_json': json.dumps(starts)}]),
              Node(package='webots_ros2_limo', executable='exploration_traffic', output='screen',
                   parameters=[{'use_sim_time': True, 'route_clearance': .85, **radio_params}])]
    defaults = yaml.safe_load((Path(get_package_share_directory('slam_toolbox')) /
                               'config/mapper_params_online_async.yaml').read_text())['slam_toolbox']['ros__parameters']
    robot_stacks = []
    for robot_index, name in enumerate(starts):
        group_start = len(stacks)
        actions.append(WebotsController(robot_name=name, namespace=name, port=port,
            parameters=[{'robot_description': str(descriptions[name]), 'use_sim_time': True,
                         'set_robot_state_publisher': False}], output='screen'))
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name=name+'_laser_mount', arguments=['--x', '.103', '--z', '.166',
            '--frame-id', name+'/base_link', '--child-frame-id', name+'/laser_link']))
        slam = {**defaults, 'use_sim_time': True, 'use_lifecycle_manager': True, 'use_map_saver': False,
                'enable_interactive_mode': False, 'map_frame': name+'/map', 'map_name': '/'+name+'/map', 'odom_frame': name+'/odom',
                'base_frame': name+'/base_link', 'scan_topic': '/'+name+'/scan_slam',
                'resolution': .1, 'map_update_interval': 1., 'min_laser_range': .1, 'max_laser_range': 12.,
                # Gating a tilted/turning laser scan must not freeze map->odom
                # at the last accepted scan timestamp. Hold its correction
                # between scans while live wheel/IMU odometry propagates pose.
                'restamp_tf': True, 'transform_publish_period': .05,
                'transform_timeout': .3, 'scan_queue_size': 1,
                'min_pass_through': 2, 'minimum_travel_distance': .2, 'minimum_travel_heading': .2}
        slam.update(slam_prior(LaunchConfiguration('scan_matching_profile').perform(context)))
        if kind in ('moon', 'moon_radar'):
            # Repetitive circular terrain can produce ambiguous long-range
            # loop matches. Isolate local scan matching/odometry first; do not
            # apply a global pose-graph jump while validating this scene.
            slam['do_loop_closing'] = False
        params = yaml.safe_load((share/'resource/exploration_nav2_template.yaml').read_text().replace('robot1', name))
        params.pop('amcl', None)
        params['slam_toolbox'] = {'ros__parameters': slam}
        # 20 Hz is sufficient for this slow flat-world navigation experiment.
        params['bt_navigator']['ros__parameters']['bt_loop_duration'] = 50
        controller = params['controller_server']['ros__parameters']
        controller['controller_frequency'] = 15.
        # Positional arrival must not wait for a precise frontier heading.
        # Coverage is checked from actual scans, never inferred from arrival.
        controller['general_goal_checker'].update(
            plugin='nav2_controller::PositionGoalChecker', xy_goal_tolerance=.25, stateful=True)
        controller['progress_checker'].update(plugin='nav2_controller::PoseProgressChecker',
            required_movement_radius=.1, required_movement_angle=.2, movement_time_allowance=30.)
        controller['FollowPath'].update(desired_linear_vel=.40, lookahead_dist=.65,
            min_lookahead_dist=.35, max_lookahead_dist=.9, min_approach_linear_velocity=.10,
            approach_velocity_scaling_dist=.35, rotate_to_heading_angular_vel=.9,
            # At 15 Hz the former 1.5 cap issued only 0.1 rad/s from rest.
            # Four-wheel static friction kept feedback near zero indefinitely.
            max_angular_accel=12.)
        if kind == 'moon_radar':
            controller['FollowPath'].update(min_approach_linear_velocity=.2,
                                             approach_velocity_scaling_dist=.3)
            params['behavior_server']['ros__parameters'].update(
                max_rotational_vel=.9, min_rotational_vel=.3, rotational_acc_lim=3.)
        params['planner_server']['ros__parameters']['GridBased'].update(allow_unknown=False, tolerance=.1)
        params['planner_server']['ros__parameters']['expected_planner_frequency'] = 1.
        for costmap in ('global_costmap', 'local_costmap'):
            p = params[costmap][costmap]['ros__parameters']
            p['resolution'] = .1
            p['inflation_layer']['inflation_radius'] = .4
            if costmap == 'global_costmap':
                # Jazzy StaticLayer reads use_maximum at costmap scope, not
                # under the plugin name. Both static layers must combine costs.
                p['use_maximum'] = True
                # Use the fixed global costmap's maximum-combination semantics:
                # unknown means no contribution. Keep the rolling local map's
                # ordinary live lidar avoidance unchanged.
                p['plugins'].insert(p['plugins'].index('inflation_layer'), 'reservation_layer')
                p['reservation_layer'] = {
                    'plugin': 'nav2_costmap_2d::StaticLayer', 'enabled': True,
                    'map_topic': f'/{name}/reservation_map', 'map_subscribe_transient_local': True,
                    'subscribe_to_updates': False, 'use_maximum': True,
                    'trinary_costmap': True, 'track_unknown_space': True}
        params['local_costmap']['local_costmap']['ros__parameters']['update_frequency'] = 5.
        file = output / f'{name}_nav2.yaml'
        file.write_text(yaml.safe_dump({name: params}, sort_keys=False))
        # This node supports an acknowledged mapping -> localization-only switch
        # against its existing pose graph; TF stays live while map writing stops.
        servers = [('slam_toolbox', 'slam_toolbox', 'map_and_localization_slam_toolbox_node'),
                   ('controller_server', 'nav2_controller', 'controller_server'),
                   ('smoother_server', 'nav2_smoother', 'smoother_server'),
                   ('planner_server', 'nav2_planner', 'planner_server'),
                   ('behavior_server', 'nav2_behaviors', 'behavior_server'),
                   ('bt_navigator', 'nav2_bt_navigator', 'bt_navigator')]
        for node, package, executable in servers:
            remappings = ([('/map', '/'+name+'/map'), ('/map_metadata', '/'+name+'/map_metadata')]
                          if node == 'slam_toolbox' else
                          [('cmd_vel', 'cmd_vel_nav')] if node in ('controller_server', 'behavior_server') else [])
            stacks.append(Node(package=package, executable=executable, namespace=name, name=node,
                               parameters=[str(file), {'use_sim_time': True}], output='screen',
                               remappings=remappings))
        # SLAM must activate independently of Nav2's map-dependent configuration.
        stacks.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace=name,
            name='slam_lifecycle_manager',
            parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 10.,
                         'node_names': ['slam_toolbox']}], output='screen'))
        stacks.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', namespace=name,
            name='navigation_lifecycle_manager',
            parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 10.,
                         'node_names': [n for n, _, _ in servers if n != 'slam_toolbox']}], output='screen'))
        stacks.append(Node(package='webots_ros2_limo', executable='exploration_robot', namespace=name,
            parameters=[{'use_sim_time': True, 'base_frame': name+'/base_link',
                         'observe_seconds': .1 if kind == 'moon_radar' else .5,
                         'radio_enabled': radio_enabled}], output='screen'))
        # Four simultaneous Nav2/SLAM startups can time out lifecycle replies
        # under load. Stagger startup while the team still waits for all peers.
        robot_stacks.append(TimerAction(period=15.+8.*robot_index, actions=stacks[group_start:]))
        del stacks[group_start:]
    stacks.append(Node(package='webots_ros2_limo', executable='exploration_team',
        parameters=[{'use_sim_time': True, 'autostart': LaunchConfiguration('autostart').perform(context).lower() == 'true', 'input_timeout': 10.,
                     'allocator': LaunchConfiguration('allocator').perform(context),
                     'goal_separation': 3. if kind == 'branched' else 2., 'goal_timeout': 180.,
                     'min_goal_distance': 1.2,
                     'exploration_claim_radius': float(LaunchConfiguration('work_area_radius').perform(context)),
                     'require_radio_measurement': radio_enabled,
                     'strategy': 'radar' if kind == 'moon_radar' else 'frontier',
                     'sweep_bounds': [-29., -29., 29., 29.],
                     'sweep_radius': float(LaunchConfiguration('sweep_radius').perform(context)),
                     'sweep_slot_slack': float(LaunchConfiguration('sweep_slot_slack').perform(context)),
                     'homes_json': json.dumps(starts), 'completion_hold': 5.,
                     'completion_mode': 'frontier', 'return_home': True,
                     'require_mapping_control': True, **radio_params}], output='screen'))
    actions.extend([TimerAction(period=15., actions=stacks), *robot_stacks,
        TimerAction(period=45., actions=[Node(package='webots_ros2_limo', executable='exploration_startup',
                                              output='screen')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/exploration_visualization_launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz'),
                              'rviz_config': str(rviz_config),
                              'bt_viewer': LaunchConfiguration('bt_viewer')}.items()),
        RegisterEventHandler(OnProcessExit(target_action=webots, on_exit=[
            EmitEvent(event=Shutdown(reason='Flat-world Webots closed'))]))])
    return actions


def generate_launch_description():
    return LaunchDescription([
        # Keep reliable writers from blocking the simultaneous local ROS stacks.
        SetEnvironmentVariable('RMW_FASTRTPS_PUBLICATION_MODE', 'ASYNCHRONOUS'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('bt_viewer', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('allocator', default_value='greedy', choices=['greedy', 'cp_sat']),
        DeclareLaunchArgument('world', default_value='branched', choices=['branched', 'plain', 'moon', 'moon_radar']),
        DeclareLaunchArgument('radio_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('radio_range', default_value='7.0'),
        DeclareLaunchArgument('work_area_radius', default_value='2.5'),
        DeclareLaunchArgument('sweep_radius', default_value='24.0'),
        DeclareLaunchArgument('sweep_slot_slack', default_value='0.75'),
        DeclareLaunchArgument('moon_seed', default_value='42'),
        DeclareLaunchArgument('scan_matching_profile', default_value='imu_prior', choices=['standard', 'imu_prior']),
        DeclareLaunchArgument('webots_port', default_value='1254'), OpaqueFunction(function=runtime)])
