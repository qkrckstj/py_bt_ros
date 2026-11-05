"""Prepare Jazzy Nav2 parameters while preserving this package's LIMO tuning."""
from pathlib import Path
import yaml


def prepare_jazzy_params(source, installed_defaults, output):
    params = yaml.safe_load(Path(installed_defaults).read_text())
    limo = yaml.safe_load(Path(source).read_text())
    # Keep distribution-specific servers (smoother, collision monitor, etc.).
    # Existing LIMO controller, costmap and localization tuning takes precedence.
    for name in ('amcl', 'controller_server', 'local_costmap', 'global_costmap',
                 'map_server', 'planner_server', 'behavior_server', 'velocity_smoother',
                 'waypoint_follower'):
        if name in limo:
            params[name] = limo[name]
    controller = params['controller_server']['ros__parameters']
    controller['progress_checker_plugins'] = [controller.pop('progress_checker_plugin', 'progress_checker')]
    # Jazzy automatically loads built-in Nav2 BT plugins; use its navigator defaults.
    params['bt_navigator']['ros__parameters'].update(
        global_frame='map', robot_base_frame='base_link', odom_topic='/odom')

    def update(value):
        if not isinstance(value, dict):
            return
        for key, child in list(value.items()):
            if key == 'plugin' and isinstance(child, str) and '/' in child:
                value[key] = child.replace('/', '::')
            elif key in ('base_frame_id', 'robot_base_frame', 'base_frame') and child == 'base_footprint':
                value[key] = 'base_link'
            else:
                update(child)
        if 'ros__parameters' in value:
            value['ros__parameters']['use_sim_time'] = True
            # Keep /cmd_vel as Twist, matching the real LIMO interface.
            value['ros__parameters']['enable_stamped_cmd_vel'] = False

    update(params)
    Path(output).write_text(yaml.safe_dump(params))
