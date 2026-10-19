"""Adapt generated resources to the sourced ROS distribution and controller."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET


def ros_distribution(value=None):
    distro = value or os.environ.get('ROS_DISTRO', '')
    if distro not in ('humble', 'jazzy'):
        raise RuntimeError('Source ROS 2 Humble or Jazzy before launching.')
    return distro


def tf2_library_directory(root, distro):
    """Select the upstream fix for Humble's tf2 0.25.21-23 lock inversion."""
    if distro != 'humble':
        return None
    from ament_index_python.packages import get_package_share_directory
    installed = Path(get_package_share_directory('tf2')) / 'package.xml'
    version = tuple(int(part) for part in ET.parse(installed).findtext('version').split('.'))
    if not (0, 25, 21) <= version < (0, 25, 24):
        return None
    prefix = Path(root) / 'output/compat/tf2/opt/ros/humble'
    description = prefix / 'share/tf2/package.xml'
    if description.is_file() and (prefix / 'lib/libtf2.so').is_file():
        fixed = tuple(int(part) for part in ET.parse(description).findtext('version').split('.'))
        if (0, 25, 24) <= fixed < (0, 26, 0):
            return prefix / 'lib'
    raise RuntimeError(
        'Installed Humble tf2 has a known deadlock. Prepare the official fix with: '
        f'bash "{Path(root) / "scripts/prepare_humble_tf2.sh"}"')


def configure_navigation(params, distro):
    # Humble Nav2 uses Twist; Jazzy ros2_control requires TwistStamped.
    for name in ('controller_server', 'behavior_server', 'velocity_smoother',
                 'collision_monitor', 'docking_server'):
        if name in params:
            values = params[name]['ros__parameters']
            if distro == 'jazzy':
                values['enable_stamped_cmd_vel'] = True
                if name == 'velocity_smoother':
                    values['stamp_smoothed_velocity_with_smoothing_time'] = True
            else:
                values.pop('enable_stamped_cmd_vel', None)
                values.pop('stamp_smoothed_velocity_with_smoothing_time', None)

    controller = params['controller_server']['ros__parameters']['FollowPath']
    plugin = controller.get('plugin', '')
    if plugin == 'dwb_core::DWBLocalPlanner':
        controller.update(max_vel_x=0.22, min_vel_x=0.0, max_vel_y=0.0,
                          max_vel_theta=1.0, max_speed_xy=0.22,
                          xy_goal_tolerance=0.15, trans_stopped_velocity=0.05)
    elif plugin in ('nav2_mppi_controller::MPPIController',
                    'nav2_mppi_controller/MPPIController'):
        controller.update(vx_max=0.22, vx_min=-0.15, wz_max=1.0)
    else:
        raise RuntimeError(f'Unsupported FollowPath controller: {plugin}')
