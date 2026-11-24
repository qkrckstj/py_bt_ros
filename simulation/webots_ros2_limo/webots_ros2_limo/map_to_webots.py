"""Extrude a saved ROS 2 trinary occupancy map into a physical Webots world.

Map coordinates are Webots ENU coordinates: x/y remain x/y, z is up.  This is
a geometric approximation of the laser slice, not a reconstruction of room
heights, textures or objects hidden from the scanner. Unknown cells are dark
floor, never invented walls. No robot or simulator is started by this module.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile

import numpy as np
import yaml


@dataclass(frozen=True)
class Rectangle:
    x: int
    y: int
    width: int
    height: int


@dataclass(frozen=True)
class OccupancyMap:
    # Bottom row first, as in nav_msgs/OccupancyGrid, not image coordinates.
    cells: np.ndarray
    resolution: float
    origin: tuple[float, float, float]
    source: Path

    @property
    def width(self):
        return self.cells.shape[1]

    @property
    def height(self):
        return self.cells.shape[0]

    def grid_to_world(self, x, y):
        """Convert continuous cell coordinates (cell edges at integers)."""
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return (ox + self.resolution * (c * x - s * y),
                oy + self.resolution * (s * x + c * y))

    def world_to_grid(self, x, y):
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = x - ox, y - oy
        return ((c * dx + s * dy) / self.resolution,
                (-s * dx + c * dy) / self.resolution)


def _positive(value, name):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return number


def _read_pgm(path):
    """Read P2/P5 without assuming maxval=255 (including 16-bit PGM)."""
    data = path.read_bytes()
    offset = 0

    def token():
        nonlocal offset
        while offset < len(data):
            if data[offset:offset + 1] == b'#':
                end = data.find(b'\n', offset)
                offset = len(data) if end < 0 else end + 1
            elif data[offset:offset + 1].isspace():
                offset += 1
            else:
                break
        start = offset
        while offset < len(data) and not data[offset:offset + 1].isspace():
            offset += 1
        if start == offset:
            raise ValueError('Truncated PGM header or pixels')
        return data[start:offset]

    magic = token()
    width, height, maximum = int(token()), int(token()), int(token())
    if magic not in (b'P2', b'P5') or min(width, height) <= 0 or not 1 <= maximum <= 65535:
        raise ValueError('Invalid PGM header')
    if magic == b'P2':
        pixels = np.array([int(token()) for _ in range(width * height)], dtype=np.float64)
    else:
        if offset >= len(data) or not data[offset:offset + 1].isspace():
            raise ValueError('Missing PGM binary raster separator')
        # Consume exactly the header separator, not whitespace-valued pixels.
        offset += 2 if data[offset:offset + 2] == b'\r\n' else 1
        dtype = np.uint8 if maximum < 256 else np.dtype('>u2')
        needed = width * height * np.dtype(dtype).itemsize
        if len(data) - offset < needed:
            raise ValueError('Truncated PGM raster')
        pixels = np.frombuffer(data, dtype=dtype, count=width * height, offset=offset)
    if np.any(pixels < 0) or np.any(pixels > maximum):
        raise ValueError('PGM sample exceeds its declared range')
    return pixels.reshape(height, width).astype(np.float64) / maximum, None


def _read_image(path):
    if path.suffix.lower() == '.pgm':
        return _read_pgm(path)
    if path.suffix.lower() != '.png':
        raise ValueError('Map image must be PGM or PNG')
    import cv2

    pixels = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if pixels is None:
        raise ValueError(f'Cannot read map image: {path}')
    if pixels.dtype not in (np.dtype('uint8'), np.dtype('uint16')):
        raise ValueError('Expected an 8-bit or 16-bit map image')
    maximum = np.iinfo(pixels.dtype).max
    alpha = None
    if pixels.ndim == 2:
        gray = pixels
    elif pixels.ndim == 3 and pixels.shape[2] in (3, 4):
        # Nav2 uses the average of the color channels, not luma weighting.
        gray = pixels[:, :, :3].astype(np.float64).mean(axis=2)
        if pixels.shape[2] == 4:
            alpha = pixels[:, :, 3] < maximum
    else:
        raise ValueError('Unsupported PNG channel layout')
    return gray.astype(np.float64) / maximum, alpha


def load_map(yaml_path):
    source = Path(yaml_path).expanduser().resolve()
    metadata = yaml.safe_load(source.read_text())
    if not isinstance(metadata, dict):
        raise ValueError('Map YAML must be a mapping')
    if metadata.get('mode', 'trinary') != 'trinary':
        raise ValueError('Only trinary map mode is supported; save a trinary map first')
    resolution = _positive(metadata['resolution'], 'resolution')
    origin = tuple(float(value) for value in metadata['origin'])
    if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
        raise ValueError('origin must contain finite [x, y, yaw]')
    free, occupied = float(metadata['free_thresh']), float(metadata['occupied_thresh'])
    if not 0 <= free < occupied <= 1:
        raise ValueError('Expected 0 <= free_thresh < occupied_thresh <= 1')
    negate = metadata.get('negate', 0)
    if negate not in (0, 1):
        raise ValueError('negate must be 0 or 1')
    image_path = Path(metadata['image']).expanduser()
    if not image_path.is_absolute():
        image_path = source.parent / image_path
    brightness, transparent = _read_image(image_path)
    probability = brightness if negate else 1.0 - brightness
    cells = np.full(probability.shape, -1, dtype=np.int8)
    cells[probability < free] = 0
    cells[probability > occupied] = 100
    if transparent is not None:
        cells[transparent] = -1
    return OccupancyMap(np.flipud(cells).copy(), resolution, origin, source)


def merge_rectangles(mask):
    """Merge identical horizontal runs on consecutive rows, with no lost cells."""
    rectangles, active = [], {}
    for y, row in enumerate(np.asarray(mask, dtype=bool)):
        edges = np.diff(np.concatenate(([False], row, [False])).astype(np.int8))
        runs = list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))
        following = {}
        for start, end in runs:
            key = (int(start), int(end))
            previous = active.pop(key, None)
            following[key] = Rectangle(key[0], previous.y if previous else y,
                                       key[1] - key[0], previous.height + 1 if previous else 1)
        rectangles.extend(active.values())
        active = following
    rectangles.extend(active.values())
    return rectangles


def validate_spawn(occupancy, x, y, yaw, clearance=0.25):
    """Require known free floor under a conservative circular robot footprint."""
    if not all(math.isfinite(v) for v in (x, y, yaw, clearance)) or clearance < 0:
        raise ValueError('Robot pose and clearance must be finite; clearance must be >= 0')
    gx, gy = occupancy.world_to_grid(x, y)
    radius = clearance / occupancy.resolution
    if not (radius <= gx < occupancy.width - radius and radius <= gy < occupancy.height - radius):
        raise ValueError('Robot spawn/clearance extends outside the saved map')
    if occupancy.cells[math.floor(gy), math.floor(gx)] != 0:
        raise ValueError('Robot spawn must be in a known free map cell')
    if radius == 0:
        return
    x0, x1 = math.floor(gx - radius), math.floor(gx + radius)
    y0, y1 = math.floor(gy - radius), math.floor(gy + radius)
    for row in range(max(0, y0), min(occupancy.height, y1 + 1)):
        for column in range(max(0, x0), min(occupancy.width, x1 + 1)):
            dx = max(column - gx, 0, gx - (column + 1))
            dy = max(row - gy, 0, gy - (row + 1))
            if dx * dx + dy * dy < radius * radius and occupancy.cells[row, column] != 0:
                raise ValueError('Robot spawn clearance overlaps occupied or unknown cells; choose a clear pose')


def resolve_proto(path=None):
    if path is not None:
        result = Path(path).expanduser().resolve()
    else:
        result = Path(__file__).resolve().parents[1] / 'protos' / 'LimoFourDiff.proto'
        if not result.is_file():
            try:
                from ament_index_python.packages import get_package_share_directory
                result = Path(get_package_share_directory('webots_ros2_limo')) / 'protos' / 'LimoFourDiff.proto'
            except (ImportError, LookupError) as exc:
                raise ValueError('Cannot locate LimoFourDiff.proto; use --proto PATH') from exc
    if not result.is_file():
        raise ValueError(f'Robot PROTO does not exist: {result}')
    return result


def render_world(occupancy, *, robot_x=0.0, robot_y=0.0, robot_yaw=0.0,
                 robot_clearance=0.25, wall_height=2.0, proto=None, max_rectangles=50000):
    wall_height = _positive(wall_height, 'wall_height')
    validate_spawn(occupancy, robot_x, robot_y, robot_yaw, robot_clearance)
    proto_path = resolve_proto(proto)
    obstacles = merge_rectangles(occupancy.cells == 100)
    floors = merge_rectangles(occupancy.cells == 0)
    if len(obstacles) + len(floors) > max_rectangles:
        raise ValueError(f'Map requires {len(obstacles) + len(floors)} rectangles, exceeding {max_rectangles}; '
                         'use a cleaner/coarser saved map or increase --max-rectangles')
    center_x, center_y = occupancy.grid_to_world(occupancy.width / 2, occupancy.height / 2)
    yaw = occupancy.origin[2]
    width, height = occupancy.width * occupancy.resolution, occupancy.height * occupancy.resolution
    camera_height = max(math.hypot(width, height), 3.0) * 1.4 + wall_height
    lines = [
        '#VRML_SIM R2025a utf8',
        '# Generated from ' + json.dumps(str(occupancy.source), ensure_ascii=True),
        '# Map frame = Webots x/y; z up. Tan floor = known free; dark floor = unknown.',
        '# Obstacles are a constant-height extrusion of occupied laser-map cells.',
        f'EXTERNPROTO {json.dumps(str(proto_path), ensure_ascii=True)}',
        'WorldInfo {', '  title "LIMO real-map digital twin"',
        '  coordinateSystem "ENU"', '  basicTimeStep 10',
        '  contactProperties [ ContactProperties { material1 "wheel" material2 "default"',
        '    coulombFriction 1 forceDependentSlip 0.01 bounce 0 } ]', '}',
        'Viewpoint {',
        f'  position {center_x:.9g} {center_y:.9g} {camera_height:.9g}',
        # Webots Viewpoint's unrotated forward axis is +X, not OpenGL -Z.
        '  orientation 0 1 0 1.5707963267948966', '  near 0.05', '}',
        'Background { skyColor [ 0.18 0.21 0.25 ] }',
        'DirectionalLight { direction -0.3 -0.5 -1 intensity 1 ambientIntensity 0.65 }',
        'DEF MAP_FLOOR Solid {',
        f'  translation {center_x:.9g} {center_y:.9g} -0.05',
        f'  rotation 0 0 1 {yaw:.12g}',
        '  name "map floor (dark areas are unknown)"',
        '  children [ Shape { appearance PBRAppearance { baseColor 0.18 0.18 0.20 roughness 1 metalness 0 }',
        f'    geometry DEF FLOOR_GEOMETRY Box {{ size {width:.9g} {height:.9g} 0.1 }} }} ]',
        '  boundingObject USE FLOOR_GEOMETRY', '}',
    ]
    # Free floor overlays are visual only; a continuous floor supports physics.
    for i, rectangle in enumerate(floors):
        x, y = occupancy.grid_to_world(rectangle.x + rectangle.width / 2, rectangle.y + rectangle.height / 2)
        w, h = rectangle.width * occupancy.resolution, rectangle.height * occupancy.resolution
        lines.extend([
            f'Pose {{ translation {x:.9g} {y:.9g} 0.0005 rotation 0 0 1 {yaw:.12g}',
            '  children [ Shape { appearance PBRAppearance { baseColor 0.66 0.62 0.52 roughness 1 metalness 0 }',
            f'    geometry Box {{ size {w:.9g} {h:.9g} 0.001 }} }} ] }}',
        ])
    for i, rectangle in enumerate(obstacles):
        x, y = occupancy.grid_to_world(rectangle.x + rectangle.width / 2, rectangle.y + rectangle.height / 2)
        w, h = rectangle.width * occupancy.resolution, rectangle.height * occupancy.resolution
        lines.extend([
            f'DEF MAP_WALL_{i} Solid {{',
            f'  translation {x:.9g} {y:.9g} {wall_height / 2:.9g}',
            f'  rotation 0 0 1 {yaw:.12g}', f'  name "map wall {i}"',
            '  children [ Shape { appearance PBRAppearance { baseColor 0.75 0.77 0.78 roughness 1 metalness 0 }',
            f'    geometry DEF WALL_GEOMETRY_{i} Box {{ size {w:.9g} {h:.9g} {wall_height:.9g} }} }} ]',
            f'  boundingObject USE WALL_GEOMETRY_{i}', '}',
        ])
    lines.extend([
        'DEF LIMO LimoFourDiff {',
        f'  translation {robot_x:.9g} {robot_y:.9g} 0.145',
        f'  rotation 0 0 1 {robot_yaw:.12g}',
        '  name "LIMO"', '  controller "<extern>"', '  supervisor TRUE', '}', '',
    ])
    return '\n'.join(lines), {'occupied_rectangles': len(obstacles), 'free_rectangles': len(floors),
                              'occupied_cells': int(np.count_nonzero(occupancy.cells == 100)),
                              'unknown_cells': int(np.count_nonzero(occupancy.cells == -1))}


def generate_world(map_yaml, output, *, force=False, **render_options):
    """Write a world atomically and return conversion metadata.

    render_options are the keyword arguments of render_world (including the
    map-frame robot_x/robot_y/robot_yaw). This function never launches Webots.
    """
    output = Path(output).expanduser().resolve()
    if output.suffix != '.wbt':
        raise ValueError('Output filename must end in .wbt')
    if output.exists() and not force:
        raise ValueError(f'Output already exists: {output}; use --force to replace it')
    occupancy = load_map(map_yaml)
    world, counts = render_world(occupancy, **render_options)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=output.parent,
                                         prefix=output.name + '.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(world)
        if force:
            os.replace(temporary, output)
        else:
            os.link(temporary, output)
            temporary.unlink()
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return {'world': str(output), 'map': str(occupancy.source), 'map_frame': 'map',
            'resolution': occupancy.resolution, 'origin': occupancy.origin,
            'initial_pose': {key: render_options.get('robot_' + key, 0.0) for key in ('x', 'y', 'yaw')},
            'spawn_clearance': render_options.get('robot_clearance', 0.25),
            'wall_height': render_options.get('wall_height', 2.0), **counts}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_yaml', help='Saved Nav2/SLAM map YAML (PGM or PNG, trinary mode)')
    parser.add_argument('--output', required=True, help='Destination .wbt world; existing files require --force')
    parser.add_argument('--robot-x', type=float, required=True, help='Initial robot map-frame x (metres)')
    parser.add_argument('--robot-y', type=float, required=True, help='Initial robot map-frame y (metres)')
    parser.add_argument('--robot-yaw', type=float, required=True, help='Initial robot map-frame yaw (radians)')
    parser.add_argument('--robot-clearance', type=float, default=0.25, help='Known-free spawn radius in metres')
    parser.add_argument('--wall-height', type=float, default=2.0)
    parser.add_argument('--proto', help='LimoFourDiff.proto path (auto-detected in source/install)')
    parser.add_argument('--max-rectangles', type=int, default=50000)
    parser.add_argument('--force', action='store_true', help='Replace an existing output world')
    args = parser.parse_args(argv)
    try:
        if args.max_rectangles < 1:
            raise ValueError('max_rectangles must be positive')
        report = generate_world(
            args.map_yaml, args.output, force=args.force,
            robot_x=args.robot_x, robot_y=args.robot_y, robot_yaw=args.robot_yaw,
            robot_clearance=args.robot_clearance, wall_height=args.wall_height, proto=args.proto,
            max_rectangles=args.max_rectangles)
        print(json.dumps(report, ensure_ascii=False))
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        parser.exit(2, f'map_to_webots: {exc}\n')


if __name__ == '__main__':
    main()
