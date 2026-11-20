"""Controlled rotating sunlight. No ROS, navigation, SPICE or trial evaluator."""
import json
import math
from pathlib import Path
from controller import Supervisor


def sun_direction(elapsed, elevation_deg, initial_azimuth_deg, period):
    """Site-to-Sun unit vector: +X east, +Y north, +Z up."""
    azimuth = (initial_azimuth_deg + 360.0 * elapsed / period) % 360.0
    az = math.radians(azimuth)
    el = math.radians(elevation_deg)
    return [math.cos(el)*math.sin(az), math.cos(el)*math.cos(az), math.sin(el)], azimuth


def main():
    config = json.loads(Path(__file__).with_name('sun_config.json').read_text())
    elevation = float(config['elevation_deg'])
    initial_azimuth = float(config['initial_azimuth_deg'])
    period = float(config['azimuth_period_sim_seconds'])
    if config.get('mode') != 'constant_elevation':
        raise ValueError('This map release supports constant_elevation mode only.')
    if not all(math.isfinite(v) for v in (elevation, initial_azimuth, period)):
        raise ValueError('Sun parameters must be finite.')
    if not 0 < elevation < 90 or period <= 0:
        raise ValueError('Elevation must be between 0 and 90 degrees; period must be positive.')
    robot = Supervisor()
    sun = robot.getFromDef('LUNAR_SUN')
    direction = sun.getField('direction')
    sun.getField('intensity').setSFFloat(2.0)
    marker = robot.getFromDef('SUN_MARKER').getField('translation')
    view = robot.getFromDef('DEMO_VIEW')
    for side in ('WEST', 'EAST', 'SOUTH', 'NORTH'):
        # Hide only from main viewport; retain lidar/camera/collision boundary.
        robot.getFromDef('BOUNDARY_'+side).setVisibility(view, False)
    origin = robot.getTime()
    print('Lunar map ready: four stationary LIMOs, rotating sunlight, no time limit.', flush=True)
    while robot.step(int(robot.getBasicTimeStep())) != -1:
        vector, azimuth = sun_direction(robot.getTime()-origin, elevation, initial_azimuth, period)
        direction.setSFVec3f([-value for value in vector])
        marker.setSFVec3f([30*value for value in vector])
        robot.setLabel(0, f'LUNAR SUN | CONTROLLED DAYLIGHT\n'
                       f'Az {azimuth:.1f} deg | El {elevation:.1f} deg | Period {period:.0f} s',
                       .02, .02, .035, 0xFFFFFF, 0, 'Arial')


if __name__ == '__main__':
    main()
