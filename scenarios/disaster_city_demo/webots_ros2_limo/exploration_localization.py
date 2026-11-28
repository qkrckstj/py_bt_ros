"""Sensor geometry and SLAM priors for the physical flat-world experiment."""
import math


class MappingScanGate:
    """Reject tilted/rapidly changing scan planes, with a short settle period.

    At a ~0.31 m laser height and 12 m range, a 2-degree downward tilt can
    intersect the flat floor. Raw scans for local obstacle avoidance stay live.
    """
    def __init__(self, tilt_limit=math.radians(1.), settle=.25):
        self.tilt_limit, self.settle = tilt_limit, settle
        self.last, self.until, self.reason = None, -math.inf, 'settling'

    def accept(self, now, roll, pitch, yaw_rate):
        values = (now, roll, pitch, yaw_rate)
        reason = None
        if not all(math.isfinite(v) for v in values):
            reason = 'invalid_imu'
        elif math.hypot(roll, pitch) > self.tilt_limit:
            reason = 'tilted_scan_plane'
        elif abs(yaw_rate) > .5:
            reason = 'fast_rotation'
        elif self.last and now > self.last[0] and abs(yaw_rate-self.last[1])/(now-self.last[0]) > .9:
            reason = 'angular_transient'
        self.last = (now, yaw_rate)
        if reason:
            self.until, self.reason = now+self.settle, reason
            return False
        if now < self.until:
            self.reason = 'settling'
            return False
        self.reason = 'stable'
        return True


def scan_geometry(field_of_view, count):
    """Webots fixed lidar samples pixel centers, not the FOV's outer edges.

    WbLidar::updatePointCloud uses dtheta=-FOV/count and theta0=FOV/2+dtheta/2.
    The ROS range vector is reversed to increase counterclockwise.
    """
    if count < 2 or not 0 < field_of_view <= 2*math.pi:
        raise ValueError('A positive lidar FOV and at least two rays are required')
    increment = field_of_view/count
    return -(field_of_view-increment)/2, (field_of_view-increment)/2, increment


def integrate(x, y, distance, previous_heading, heading):
    """Midpoint integration with a wrapped IMU heading change."""
    change = math.atan2(math.sin(heading-previous_heading), math.cos(heading-previous_heading))
    midpoint = previous_heading+change/2
    return x+distance*math.cos(midpoint), y+distance*math.sin(midpoint), change


def slam_prior(profile):
    if profile == 'standard':
        return {}
    if profile != 'imu_prior':
        raise ValueError(f'Unknown scan matching profile: {profile}')
    # These are standard deviations; Karto squares them internally. This
    # profile assumes the simulator's accurate absolute IMU heading. Repeated
    # rims/sparse filtered scans can trigger the default 20/40/60-degree
    # response expansion. Bound its search around the IMU prior instead.
    # Both angular offset and resolution must remain positive in Karto.
    return {'distance_variance_penalty': .05,
            'use_scan_barycenter': False,
            'angle_variance_penalty': math.radians(1.),
            'minimum_distance_penalty': .05, 'minimum_angle_penalty': .05,
            'use_response_expansion': False,
            'coarse_search_angle_offset': math.radians(3.),
            'coarse_angle_resolution': math.radians(.5),
            'fine_search_angle_offset': math.radians(.2),
            'link_match_minimum_response_fine': .3,
            'loop_match_minimum_response_coarse': .6,
            'loop_match_minimum_response_fine': .65}
