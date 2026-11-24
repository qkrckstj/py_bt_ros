#!/usr/bin/env python3
"""Original terrain-shadow monitor plus conservative future shadow grids.

Uses the team's controller module as a read-only library. Navigation receives
no terrain obstacle map; only this explicitly labeled hazard oracle is exposed.
"""
import importlib.util
import json
import os
import sys
import uuid
from pathlib import Path

import numpy as np
import rclpy
from controller import Supervisor
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from shadow_prediction import encode, triangle_envelope, navigation_mask
from mission import settings


def main():
    original = Path(sys.argv[1])
    spec = importlib.util.spec_from_file_location('team_lunar_sun', original)
    team = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(team)
    config = json.loads(original.with_name('sun_config.json').read_text())
    supervisor = Supervisor()
    terrain = team.TerrainShadow(supervisor)
    mission = settings()
    bounds = mission['bounds_m']
    resolution = .15
    grid_origin = (bounds[0]-1., bounds[1]-1.)
    size = int(np.ceil(max(bounds[2]-bounds[0]+2., bounds[3]-bounds[1]+2.)/resolution))
    robots = [supervisor.getFromDef(f'LIMO_{i}') for i in range(1, 5)]
    sun = supervisor.getFromDef('LUNAR_SUN')
    sun.getField('intensity').setSFFloat(2.)
    view = supervisor.getFromDef('DEMO_VIEW')
    for side in ('WEST', 'EAST', 'SOUTH', 'NORTH'):
        supervisor.getFromDef('BOUNDARY_'+side).setVisibility(view, False)
    rclpy.init()
    node = rclpy.create_node('predictive_terrain_shadow_monitor')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    forecast_pub = node.create_publisher(String, '/lunar/shadow_forecast', qos)
    mask_pub = node.create_publisher(OccupancyGrid, '/lunar/shadow_keepout', qos)
    info_pub = node.create_publisher(CostmapFilterInfo, '/lunar/shadow_filter_info', qos)
    truth_pubs = [node.create_publisher(Bool, f'/robot{i}/in_terrain_shadow', 10) for i in range(1, 5)]
    # Explicit runtime reattachment may preserve the existing Sun phase and
    # checkpoint identity. A fresh launch still creates a new world identity.
    origin = float(os.environ.get('LUNAR_SUN_ORIGIN',supervisor.getTime()))
    world_id = os.environ.get('LUNAR_FORECAST_WORLD_ID') or uuid.uuid4().hex
    cache = {}
    period = config['azimuth_period_sim_seconds']
    # Bound shadow-tip displacement over a 2-second angular bin by the
    # maximum mesh height and sun elevation. Include footprint/raster reserve.
    max_height = max(float(mesh[1][2]) for mesh in terrain.meshes)
    margin = .65 + max_height/np.tan(np.deg2rad(config['elevation_deg']))*2*np.pi/period
    period_bins = round(period/2)
    if abs(period_bins*2-period)>1e-6:
        raise ValueError('Prediction cache requires a sun period divisible by 2 seconds')
    mission_state = {}
    node.create_subscription(String,'/lunar_exploration/status',
                             lambda m:mission_state.update(json.loads(m.data)),qos)
    next_truth = next_forecast = 0.
    last = None
    while supervisor.step(int(supervisor.getBasicTimeStep())) != -1:
        now = supervisor.getTime()
        vector, azimuth = team.sun_direction(now-origin, config['elevation_deg'],
                                            config['initial_azimuth_deg'], config['azimuth_period_sim_seconds'])
        sun.getField('direction').setSFVec3f([-v for v in vector])
        supervisor.getFromDef('SUN_MARKER').getField('translation').setSFVec3f([30*v for v in vector])
        if now >= next_truth:
            states = [terrain.robot_shaded(robot, vector) for robot in robots]
            for i, (publisher, state) in enumerate(zip(truth_pubs, states), 1):
                publisher.publish(Bool(data=bool(state)))
                supervisor.setLabel(i, f'LIMO {i} | Terrain shadow: {bool(state)}',
                                    .02, .14+.045*(i-1), .028,
                                    0xFF7777 if state else 0x88FF88, 0, 'Arial')
            if states != last:
                print('Terrain shadow states: '+str(states), flush=True)
                last = states
            next_truth = now+.5
        if now >= next_forecast:
            index = int((now-origin)//2)
            for k in range(index, index+181):
                phase = k%period_bins
                if phase not in cache:
                    vector_k = team.sun_direction(phase*2, config['elevation_deg'],
                        config['initial_azimuth_deg'], config['azimuth_period_sim_seconds'])[0]
                    cache[phase] = triangle_envelope(terrain.meshes, vector_k,
                        grid_origin, resolution, size, margin=margin)
            masks = [cache[k%period_bins] for k in range(index,index+181)]
            forecast_pub.publish(String(data=encode(masks, origin+index*2,
                origin=grid_origin,resolution=resolution,
                sun_period_s=period,sun_phase_deg=azimuth,elevation_deg=config['elevation_deg'],
                world_id=world_id,sun_origin_s=origin)))
            # Reactive keepout covers the current and next 2-second bin.
            # Longer temporal constraints remain in CP-SAT and runtime guards.
            mask = OccupancyGrid()
            mask.header.frame_id = 'map'
            mask.header.stamp.sec = int(now)
            mask.header.stamp.nanosec = int((now-int(now))*1e9)
            mask.info.resolution = resolution
            mask.info.width = mask.info.height = size
            mask.info.origin.position.x, mask.info.origin.position.y = grid_origin
            mask.info.origin.orientation.w = 1.
            mask.data = navigation_mask(masks).ravel().tolist()
            mask_pub.publish(mask)
            info = CostmapFilterInfo()
            info.header = mask.header
            info.type = 0
            info.filter_mask_topic = '/lunar/shadow_keepout'
            info.base = 0.
            info.multiplier = 1.
            if next_forecast == 0:
                info_pub.publish(info)
            next_forecast = now+2
        rclpy.spin_once(node,timeout_sec=0)
        fraction = (mission_state.get('coverage') or {}).get('observed_fraction',0)
        wait = max(0,(mission_state.get('departure_at') or now)-now) if mission_state.get('phase')=='WAITING_SUN' else 0
        supervisor.setLabel(20,f'TEAM {mission_state.get("phase","STARTING")} | observed {fraction:.1%} | wait {wait:.0f}s',
                            .02,.36,.027,0xFFFFFF,0,'Arial')
        supervisor.setLabel(0, f'LUNAR SUN | SHADOW FORECAST ORACLE\nAz {azimuth:.1f} deg | period {period:.0f}s | forecast 360s',
                            .02, .02, .035, 0xFFFFFF, 0, 'Arial')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
