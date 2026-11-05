"""Factory-specific planar lidar collision geometry and position-only goals."""
import math


def configure_factory(params, tolerance=0.30):
    if not 0.05 <= tolerance <= 0.75:
        raise ValueError('goal_tolerance must be between 0.05 and 0.75 metres')
    controller = params['controller_server']['ros__parameters']
    controller['general_goal_checker'].update(
        plugin='nav2_controller::PositionGoalChecker', xy_goal_tolerance=tolerance, stateful=False)
    dwb = controller['FollowPath']
    # CPU inference can delay localization by ~0.3 s. Bound the allowance;
    # stale transforms still fail instead of silently using arbitrary old poses.
    dwb['transform_tolerance'] = 0.5
    dwb['xy_goal_tolerance'] = tolerance
    dwb['critics'] = ['Oscillation', 'ObstacleFootprint', 'PathAlign', 'PathDist', 'GoalDist']
    dwb['ObstacleFootprint.scale'] = 1.0
    controller['progress_checker'].update(required_movement_radius=0.2, movement_time_allowance=20.0)
    # Body/wheels span roughly 0.32 x 0.22 m; include body overhang and clearance.
    footprint = '[ [0.20, 0.15], [0.20, -0.15], [-0.20, -0.15], [-0.20, 0.15] ]'
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        costmap.pop('robot_radius', None)
        costmap.update(footprint=footprint, footprint_padding=0.03,
                       plugins=(['static_layer'] if name == 'global_costmap' else []) +
                       ['obstacle_layer', 'inflation_layer'])
        # /odom is planar at z=0, while laser_link is at z=-0.034 relative to base_link.
        layer = costmap['obstacle_layer']
        layer.update(min_obstacle_height=-0.2, max_obstacle_height=2.0)
        layer['scan'].update(min_obstacle_height=-0.2, max_obstacle_height=2.0,
                             obstacle_max_range=8.0, raytrace_max_range=10.0,
                             obstacle_min_range=0.1, raytrace_min_range=0.1)
        # Plan with more clearance than the local controller needs for tracking.
        # These are obstacle cost radii, not a replacement for the body footprint.
        inflation_radius = 0.30 if name == 'global_costmap' else 0.05
        costmap['inflation_layer'].update(
            inflation_radius=inflation_radius, cost_scaling_factor=3.0)
        if name == 'local_costmap':
            costmap.update(width=5, height=5, update_frequency=10.0, publish_frequency=5.0)
    monitor = params.get('collision_monitor', {}).get('ros__parameters')
    if monitor:
        monitor['scan'].update(min_height=-0.2)
        monitor['FootprintApproach']['min_points'] = 2
    return params


def relative_regions(config):
    """A new SLAM map starts at the simulator's initial robot pose, not world (0,0)."""
    start = config['mission']['start_pose']
    c, s = math.cos(start['yaw']), math.sin(start['yaw'])
    points = list(config['mission']['regions'].values())
    if 'mock_person_pose' in config['mission']:
        points.append(config['mission']['mock_person_pose'])
    for point in points:
        dx, dy = point['x'] - start['x'], point['y'] - start['y']
        point['x'], point['y'] = c*dx+s*dy, -s*dx+c*dy
        if 'yaw' in point:
            point['yaw'] -= start['yaw']
    config['mission']['start_pose'] = dict(x=0.0, y=0.0, yaw=0.0)
