#!/usr/bin/env python3
"""Package the real-robot feed, localization configuration, and operator keyboard helper."""
import argparse
from pathlib import Path
import tarfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map-dir', type=Path, help='Private directory containing limo_real.pgm and limo_real_sim.yaml')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    map_dir = args.map_dir or root / 'resource/real_maps'
    files = [
        'webots_ros2_limo/__init__.py', 'webots_ros2_limo/twin_bridge.py',
        'webots_ros2_limo/twin_protocol.py', 'webots_ros2_limo/peer_relay.py', 'resource/real_slam.yaml',
        'scripts/twin_robot.sh',
        'scripts/twin_keyboard.py',
        'launch/real_twin_robot.launch.py',
    ]
    for item in files:
        if not (root / item).is_file():
            raise SystemExit(f'Missing source file: {item}')
    private_maps = ('limo_real_sim.yaml', 'limo_real.pgm')
    for name in private_maps:
        if not (map_dir / name).is_file():
            raise SystemExit(f'Missing private map file: {map_dir / name}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('xb') as output:
        with tarfile.open(fileobj=output, mode='w:gz') as archive:
            for item in files:
                archive.add(root / item, arcname='limo_twin_robot/' + item)
            for name in private_maps:
                archive.add(map_dir / name, arcname='limo_twin_robot/resource/real_maps/' + name)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
