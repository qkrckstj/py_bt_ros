"""Controlled rotating sunlight. No ROS, navigation, SPICE or trial evaluator."""
import json
import math
from pathlib import Path
from controller import Supervisor
import numpy as np
import rclpy
from std_msgs.msg import Bool


class TerrainShadow:
    """Ground-truth triangle ray tests, restricted to terrain DEF nodes."""
    def __init__(self, supervisor):
        self.meshes = []
        names = ['LUNAR_GROUND'] + [f'DEMO_{kind}_{i}' for kind in ('HILL', 'CRATER') for i in range(1,7)]
        for name in names:
            node = supervisor.getFromDef(name)
            if node is None:
                continue
            mesh = supervisor.getFromDef(name+'_MESH')
            points = mesh.getField('coord').getSFNode().getField('point')
            vertices = np.array([points.getMFVec3f(i) for i in range(points.getCount())])
            rotation = np.array(node.getOrientation()).reshape(3, 3)
            vertices = vertices @ rotation.T + np.array(node.getPosition())
            field = mesh.getField('coordIndex')
            faces, face = [], []
            for i in range(field.getCount()):
                value = field.getMFInt32(i)
                if value == -1:
                    for j in range(1, len(face)-1):
                        faces.append((face[0], face[j], face[j+1]))
                    face = []
                else:
                    face.append(value)
            triangles = vertices[np.array(faces)]
            self.meshes.append((vertices.min(axis=0), vertices.max(axis=0),
                                triangles[:, 0], triangles[:, 1]-triangles[:, 0],
                                triangles[:, 2]-triangles[:, 0]))
    def shaded(self, origin, direction):
        origin, direction = np.asarray(origin), np.asarray(direction)
        for low, high, v0, e1, e2 in self.meshes:
            # Broad phase: positive ray against mesh bounding box.
            near, far = 0.0, float('inf')
            for axis in range(3):
                if abs(direction[axis]) < 1e-10:
                    if origin[axis] < low[axis] or origin[axis] > high[axis]:
                        far = -1; break
                else:
                    a, b = sorted(((low[axis]-origin[axis])/direction[axis],
                                   (high[axis]-origin[axis])/direction[axis]))
                    near, far = max(near, a), min(far, b)
            if far < near:
                continue
            h = np.cross(direction, e2)
            determinant = np.einsum('ij,ij->i', e1, h)
            valid = np.abs(determinant) > 1e-10
            inverse = np.divide(1., determinant, out=np.zeros_like(determinant), where=valid)
            offset = origin-v0
            u = inverse*np.einsum('ij,ij->i', offset, h)
            q = np.cross(offset, e1)
            v = inverse*(q @ direction)
            distance = inverse*np.einsum('ij,ij->i', e2, q)
            if np.any(valid & (u >= -1e-8) & (v >= -1e-8) &
                      (u+v <= 1+1e-8) & (distance > .001)):
                return True
        return False

    def robot_shaded(self, node, direction):
        # Nine ground footprint samples. Any shaded sample means TRUE.
        position = np.array(node.getPosition())
        rotation = np.array(node.getOrientation()).reshape(3, 3)
        for x in (-.16, 0., .16):
            for y in (-.105, 0., .105):
                offset = rotation @ np.array([x, y, 0.])
                point = [position[0]+offset[0], position[1]+offset[1], max(.02, position[2]-.13)]
                if self.shaded(point, direction):
                    return True
        return False



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
    shadow = TerrainShadow(robot)
    rclpy.init()
    ros = rclpy.create_node('terrain_shadow_monitor')
    publishers = [ros.create_publisher(Bool, f'/robot{i}/in_terrain_shadow', 10)
                  for i in range(1, 5)]
    robots = [robot.getFromDef(f'LIMO_{i}') for i in range(1, 5)]
    next_check = 0.
    last_states = None
    origin = robot.getTime()
    print('Lunar map ready: four stationary LIMOs, rotating sunlight, no time limit.', flush=True)
    while robot.step(int(robot.getBasicTimeStep())) != -1:
        vector, azimuth = sun_direction(robot.getTime()-origin, elevation, initial_azimuth, period)
        direction.setSFVec3f([-value for value in vector])
        marker.setSFVec3f([70*value for value in vector])
        if robot.getTime() >= next_check:
            states = [shadow.robot_shaded(node, vector) for node in robots]
            for i, (publisher, state) in enumerate(zip(publishers, states), 1):
                publisher.publish(Bool(data=bool(state)))
                robot.setLabel(i, f'LIMO {i} | Terrain shadow: {str(bool(state)).upper()}',
                               .02, .14+.045*(i-1), .028,
                               0xFF7777 if state else 0x88FF88, 0, 'Arial')
            if states != last_states:
                print('Terrain shadow states: '+str(dict(zip(range(1,5), states))), flush=True)
                last_states = states
            next_check = robot.getTime()+.5
        robot.setLabel(0, f'LUNAR SUN | CONTROLLED DAYLIGHT\n'
                       f'Az {azimuth:.1f} deg | El {elevation:.1f} deg | Period {period:.0f} s',
                       .02, .02, .035, 0xFFFFFF, 0, 'Arial')


if __name__ == '__main__':
    main()
