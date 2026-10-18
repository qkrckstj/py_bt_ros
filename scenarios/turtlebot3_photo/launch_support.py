"""Prepare homework resources for the existing Webots robot_launch entry point."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml


def prepare_resources(root, webots, nav2, world_path, use_sim_time, ros_distro):
    if ros_distro not in ('humble', 'jazzy'):
        raise RuntimeError(f'Unsupported ROS_DISTRO: {ros_distro}. Source Humble or Jazzy first.')
    directory = tempfile.TemporaryDirectory(prefix='py-bt-webots-')
    output = Path(directory.name)
    params = yaml.safe_load((nav2 / 'params/nav2_params.yaml').read_text())
    overrides = yaml.safe_load((root / 'scenarios/turtlebot3_photo/nav2_overrides.yaml').read_text())

    def merge(target, values):
        for key, value in values.items():
            previous = target.get(key)
            if isinstance(value, dict) and isinstance(previous, dict):
                merge(previous, value)
            else:
                target[key] = value

    merge(params, overrides)
    controller = params['controller_server']['ros__parameters']['FollowPath']
    plugin = controller['plugin']
    if plugin == 'dwb_core::DWBLocalPlanner':
        controller.update(max_vel_x=0.22, max_vel_theta=1.0, max_speed_xy=0.22,
                          xy_goal_tolerance=0.15, trans_stopped_velocity=0.05)
    elif plugin == 'nav2_mppi_controller::MPPIController':
        controller.update(vx_max=0.22, vx_min=-0.15, wz_max=1.0)
    else:
        raise RuntimeError(f'Unsupported default Nav2 controller: {plugin}')
    if ros_distro == 'jazzy':
        for name in ('controller_server', 'behavior_server', 'velocity_smoother',
                     'collision_monitor', 'docking_server'):
            params.setdefault(name, {}).setdefault('ros__parameters', {})['enable_stamped_cmd_vel'] = True
        params['velocity_smoother']['ros__parameters']['stamp_smoothed_velocity_with_smoothing_time'] = True
    (output / 'nav2_params.yaml').write_text(yaml.safe_dump(params))

    # Humble's diff-drive controller has separate Twist/TwistStamped inputs.
    control = yaml.safe_load((webots / 'resource/ros2control.yml').read_text())
    control['diffdrive_controller']['ros__parameters']['use_stamped_vel'] = ros_distro == 'jazzy'
    (output / 'ros2control.yaml').write_text(yaml.safe_dump(control))

    if ros_distro == 'humble':
        rviz = yaml.safe_load((nav2 / 'rviz/nav2_default_view.rviz').read_text())
        tools = rviz['Visualization Manager']['Tools']
        goal = next((tool for tool in tools if tool['Class'] == 'rviz_default_plugins/SetGoal'), None)
        if goal is None:
            goal = {'Class': 'rviz_default_plugins/SetGoal'}
            tools.append(goal)
        goal['Topic'] = {'Value': '/bt/goal_pose', 'Depth': 10,
                         'Durability Policy': 'Volatile', 'History Policy': 'Keep Last',
                         'Reliability Policy': 'Reliable'}
        (output / 'photo_mission.rviz').write_text(yaml.safe_dump(rviz, sort_keys=False))
    else:
        (output / 'photo_mission.rviz').write_text(
            (root / 'scenarios/turtlebot3_photo/photo_mission.rviz').read_text())

    robot = ET.parse(webots / 'resource/turtlebot_webots.urdf')
    plugins = robot.getroot().find('webots')
    for plugin in list(plugins.findall('plugin')):
        if plugin.get('type', '').startswith('webots_ros2_turtlebot.'):
            plugins.remove(plugin)
    ET.SubElement(plugins, 'plugin', {
        'type': 'scenarios.turtlebot3_photo.control_description.ControlDescriptionPlugin'})
    for device in list(plugins.findall('device')):
        if device.get('reference') == 'front_camera':
            plugins.remove(device)
    camera = ET.SubElement(plugins, 'device', {'reference': 'front_camera', 'type': 'Camera'})
    settings = ET.SubElement(camera, 'ros')
    for key, value in {'enabled': 'true', 'alwaysOn': 'false', 'updateRate': '10',
                       'topicName': '/TurtleBot3Burger/front_camera', 'frameName': 'front_camera'}.items():
        ET.SubElement(settings, key).text = value
    robot.write(output / 'robot.urdf')

    world = world_path.read_text()
    if 'name "front_camera"' not in world:
        if 'extensionSlot [' not in world:
            raise RuntimeError('TurtleBot3 world has no extensionSlot for the front camera')
        camera_text = (root / 'scenarios/turtlebot3_photo/front_camera.wbt').read_text()
        world = world.replace('extensionSlot [', 'extensionSlot [\n' + camera_text, 1)
    # Resolve relative resource paths before moving the world to a temporary directory.
    import re
    def absolute_resource(match):
        resource = match.group(1)
        candidate = world_path.parent / resource
        if '://' not in resource and not Path(resource).is_absolute() and candidate.exists():
            return '"' + str(candidate.resolve()) + '"'
        return match.group(0)
    world = re.sub(r'"([^"\n]+)"', absolute_resource, world)
    (output / 'photo_mission.wbt').write_text(world)

    config = yaml.safe_load((root / 'config.yaml').read_text())
    config['ros']['use_sim_time'] = use_sim_time
    (output / 'bt_config.yaml').write_text(yaml.safe_dump(config))
    return directory
