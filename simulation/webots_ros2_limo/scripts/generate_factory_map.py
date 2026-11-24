#!/usr/bin/env python3
"""Generate a simulation reference map from a temporary Webots lidar survey.

This is ground-truth pose sampling, not SLAM. The source world is never changed.
Added mission props and the LIMO are excluded. Run with Webots closed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def remove_node(text, name):
    match = re.search(r'(?m)^DEF ' + re.escape(name) + r' \w+ \{', text)
    if not match:
        return text
    depth = 0
    for token in re.finditer(r'"(?:\\.|[^"\\])*"|#[^\n]*|[{}]', text[match.start():]):
        if token.group() == '{':
            depth += 1
        elif token.group() == '}':
            depth -= 1
            if depth == 0:
                return text[:match.start()] + text[match.start() + token.end():]
    raise ValueError(f'Unbalanced node: {name}')


def survey(args):
    import cv2
    import numpy as np
    from controller import Supervisor

    robot = Supervisor()
    robot.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)
    step = int(robot.getBasicTimeStep())
    lidar = robot.getDevice('survey_lidar')
    lidar.enable(step)
    position = robot.getSelf().getField('translation')
    resolution = 0.05
    xmin, ymin, xmax, ymax = -9., -13., 8., 13.
    width, height = round((xmax - xmin) / resolution), round((ymax - ymin) / resolution)
    free = np.zeros((height, width), np.uint8)
    hits = np.zeros((height, width), np.uint16)
    angles = np.linspace(lidar.getFov() / 2, -lidar.getFov() / 2, lidar.getHorizontalResolution())
    directions = np.column_stack([np.cos(angles), np.sin(angles)])
    poses = [(x, y) for y in np.arange(ymin + .4, ymax, .8)
             for x in np.arange(xmin + .4, xmax, .8)]
    accepted = 0
    for index, (x, y) in enumerate(poses):
        position.setSFVec3f([float(x), float(y), .111])
        if robot.step(step) == -1 or robot.step(step) == -1:
            raise RuntimeError('Webots closed during survey')
        ranges = np.asarray(lidar.getRangeImage())
        finite = np.isfinite(ranges) & (ranges < lidar.getMaxRange() - .01)
        # Avoid sampling too close to a wall or inside a small prop.
        if finite.any() and np.nanmin(ranges[finite]) < .18:
            continue
        accepted += 1
        lengths = np.where(finite, ranges, lidar.getMaxRange())
        endpoints = np.array([x, y]) + directions * lengths[:, None]
        cells = np.floor((endpoints - [xmin, ymin]) / resolution).astype(int)
        start = (int((x - xmin) / resolution), int((y - ymin) / resolution))
        for endpoint in cells:
            cv2.line(free, start, tuple(endpoint), 1, 1)
        valid = finite & (cells[:, 0] >= 0) & (cells[:, 0] < width) & (cells[:, 1] >= 0) & (cells[:, 1] < height)
        np.add.at(hits, (cells[valid, 1], cells[valid, 0]), 1)
        if index % 100 == 0:
            print(f'Survey {index}/{len(poses)}; valid poses {accepted}', flush=True)
    image = np.full((height, width), 205, np.uint8)
    image[free > 0] = 254
    image[hits >= 2] = 0
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output.with_suffix('.pgm')), np.flipud(image))
    output.with_suffix('.yaml').write_text(
        f'image: {output.name}.pgm\nmode: trinary\nresolution: {resolution}\n'
        f'origin: [{xmin}, {ymin}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    output.with_suffix('.survey.json').write_text(json.dumps({
        'method': 'Webots lidar survey with simulator ground-truth poses; not SLAM',
        'world_sha256': args.world_hash, 'source_world': str(args.world),
        'excluded_defs': ['LIMO', 'RESCUE_DUMMY', 'OBSTACLE_BOX_1', 'OBSTACLE_BOX_2'],
        'lidar_height': .111, 'sample_spacing': .8, 'accepted_poses': accepted,
        'resolution': resolution, 'origin': [xmin, ymin, 0],
    }, indent=2) + '\n')
    print(f'Saved {output.with_suffix(".yaml")}: {accepted} survey poses', flush=True)
    robot.simulationQuit(0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, default=ROOT / 'worlds/limo_factory_hall.wbt')
    parser.add_argument('--output', type=Path, default=ROOT / 'resource/limo_factory_map')
    parser.add_argument('--controller', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--world-hash', default='', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.controller:
        survey(args)
        return
    from webots_ros2_driver.utils import get_webots_home
    home = Path(get_webots_home())
    text = args.world.read_text()
    digest = hashlib.sha256(text.encode()).hexdigest()
    for name in ('LIMO', 'RESCUE_DUMMY', 'OBSTACLE_BOX_1', 'OBSTACLE_BOX_2'):
        text = remove_node(text, name)
    text = re.sub(r'EXTERNPROTO "(\.\./[^"]+)"',
                  lambda m: f'EXTERNPROTO "{(args.world.parent / m[1]).resolve()}"', text)
    text += '''\nRobot {
  name "FactorySurvey"
  supervisor TRUE
  controller "<extern>"
  children [ Lidar {
    name "survey_lidar"
    horizontalResolution 720
    fieldOfView 6.28318530718
    numberOfLayers 1
    near 0.02
    minRange 0.02
    maxRange 30
    noise 0
  } ]
}\n'''
    with tempfile.TemporaryDirectory(prefix='limo-survey-') as directory:
        world = Path(directory) / 'survey.wbt'
        world.write_text(text)
        env = dict(os.environ, WEBOTS_HOME=str(home))
        env['PYTHONPATH'] = str(home / 'lib/controller/python') + ':' + env.get('PYTHONPATH', '')
        log_path = args.output.with_suffix('.webots.log')
        with log_path.open('w') as log:
            process = subprocess.Popen([str(home / 'webots'), '--batch', '--mode=fast', str(world)],
                                       env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                subprocess.run([str(home / 'webots-controller'), '--robot-name=FactorySurvey',
                                '--protocol=ipc', '--port=1234', str(Path(__file__).resolve()),
                                '--controller', '--world', str(args.world.resolve()),
                                '--world-hash', digest, '--output', str(args.output.resolve())],
                               env=env, check=True, timeout=240)
                process.wait(timeout=30)
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=10)


if __name__ == '__main__':
    main()
