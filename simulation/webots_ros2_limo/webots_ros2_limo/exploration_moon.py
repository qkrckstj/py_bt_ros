"""Adapt the imported moon scene, without exposing terrain to the allocator.

The upstream assets stay unchanged. A seeded layout makes each run reproducible;
mesh coordinates and their USE collision geometry are enlarged together.
"""
import math
import random
import re


MOON_BASE = (-18.8, 0.)


def solid_blocks(world, pattern):
    for match in re.finditer(pattern, world):
        end, depth = match.end(), 1
        while depth:
            depth += (world[end] == '{') - (world[end] == '}')
            end += 1
        yield match, end, world[match.start():end]


def mesh_points(block):
    match = re.search(r'point\s*\[([^]]+)\]', block, re.DOTALL)
    values = [float(v) for v in re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', match[1])]
    return match, list(zip(values[::3], values[1::3], values[2::3]))


def wide_moon(world, seed=42):
    """40 x 20 m, three hills and three raised-rim craters, clear west staging.

These craters are the upstream above-ground bowl meshes, not excavations in the
flat floor. Their raised rim crosses the ~0.31 m horizontal laser scan plane.
"""
    rng = random.Random(seed)
    # Match the contact model used to tune this ROS driver's four-wheel skid
    # steering. Reading the upstream WorldInfo otherwise silently drops it.
    world = world.replace('WorldInfo {', 'WorldInfo {\n  contactProperties [ ContactProperties { '
        'material1 "wheel" material2 "default" coulombFriction 1 '
        'forceDependentSlip 0.01 bounce 0 } ]\n', 1)
    blocks = list(solid_blocks(world, r'DEF (DEMO_(HILL|CRATER)_\d+) Solid\s*\{'))
    if len(blocks) != 6:
        raise ValueError('Expected the six imported hill/crater meshes')
    # Stratified random placement spreads features along the long axis instead
    # of permitting all six to cluster at one end. Shuffle which terrain goes
    # into each slot; keep a known clearance between their full footprints.
    slots = [-10.5, -5.3, 0., 5.3, 10.5, 15.5]
    rng.shuffle(slots)
    features, replacements = [], []
    for (match, end, block), slot in zip(blocks, slots):
        kind = match[2].lower()
        radius = rng.uniform(2.2, 2.6) if kind == 'hill' else rng.uniform(1.8, 2.2)
        height = rng.uniform(1.5, 1.8) if kind == 'hill' else rng.uniform(.65, .8)
        for _ in range(2000):
            x, y = slot+rng.uniform(-.5, .5), rng.uniform(-6., 6.)
            if all(math.dist((x, y), f['position'][:2]) >= radius+f['radius']+1.5
                   for f in features):
                break
        else:
            raise ValueError(f'Cannot place {match[1]} with safe terrain spacing')
        points_match, points = mesh_points(block)
        old_radius = max(math.hypot(p[0], p[1]) for p in points)
        old_height = max(p[2] for p in points)
        xy_scale, z_scale = radius/old_radius, (height-.02)/old_height
        coordinates = '\n'+''.join(f'          {px*xy_scale:.5f} {py*xy_scale:.5f} {pz*z_scale:.5f},\n'
                                    for px, py, pz in points)+'      '
        block = block[:points_match.start(1)]+coordinates+block[points_match.end(1):]
        block = re.sub(r'translation\s+[-\d.eE+]+\s+[-\d.eE+]+\s+[-\d.eE+]+',
                       f'translation {x:.5f} {y:.5f} 0.02', block, count=1)
        features.append({'def': match[1], 'kind': kind, 'position': [x, y, .02],
                         'radius': radius, 'height': height})
        replacements.append((match.start(), end, block))
    for start, end, block in reversed(replacements):
        world = world[:start]+block+world[end:]
    world = world.replace('size 20 20 0.1', 'size 40 20 0.1')
    world = world.replace('TextureTransform { scale 4 4 }', 'TextureTransform { scale 8 4 }')
    for match, end, block in reversed(list(solid_blocks(world, r'DEF BOUNDARY_(WEST|EAST|SOUTH|NORTH) Solid\s*\{'))):
        if match[1] in ('WEST', 'EAST'):
            block = block.replace('translation -10 0', 'translation -20 0').replace('translation 10 0', 'translation 20 0')
        else:
            block = block.replace('size 20.2 0.2 0.5', 'size 40.2 0.2 0.5')
        world = world[:match.start()]+block+world[end:]
    return world, {'seed': seed, 'size': [40., 20.], 'base': list(MOON_BASE), 'features': features}


def radar_moon(world, seed=42):
    """60 x 60 m (nine original tiles of area), central base, 36 spaced features.

    Reuse the upstream meshes with unique DEF/USE identifiers. No terrain
    positions are passed to the exploration policy, only the mission boundary.
    """
    world, source = wide_moon(world, seed)
    templates = list(solid_blocks(world, r'DEF (DEMO_(HILL|CRATER)_\d+) Solid\s*\{'))
    rng = random.Random(seed)
    slots = [(x, y) for x in (-25., -15., -5., 5., 15., 25.)
             for y in (-25., -15., -5., 5., 15., 25.)]
    rng.shuffle(slots)
    features, blocks = [], []
    for i, (sx, sy) in enumerate(slots):
        match, _, template = templates[i % len(templates)]
        original = source['features'][i % len(templates)]
        x, y = sx+rng.uniform(-.6, .6), sy+rng.uniform(-.6, .6)
        name = f'DEMO_{match[2]}_{i+1}'
        block = template.replace(match[1], name)
        block = re.sub(r'\bname\s+"[^"]+"', f'name "{name.lower()}"', block, count=1)
        block = re.sub(r'translation\s+[-\d.eE+]+\s+[-\d.eE+]+\s+[-\d.eE+]+',
                       f'translation {x:.5f} {y:.5f} 0.02', block, count=1)
        blocks.append(block)
        features.append({**original, 'def': name, 'position': [x, y, .02]})
    for i in reversed(range(len(templates))):
        match, end, _ = templates[i]
        world = world[:match.start()]+('\n\n'.join(blocks) if i == 0 else '')+world[end:]
    world = world.replace('size 40 20 0.1', 'size 60 60 0.1')
    world = world.replace('TextureTransform { scale 8 4 }', 'TextureTransform { scale 12 12 }')
    for match, end, block in reversed(list(solid_blocks(world, r'DEF BOUNDARY_(WEST|EAST|SOUTH|NORTH) Solid\s*\{'))):
        block = block.replace('translation -20 0', 'translation -30 0').replace('translation 20 0', 'translation 30 0')
        block = block.replace('translation 0 -10', 'translation 0 -30').replace('translation 0 10', 'translation 0 30')
        block = block.replace('size 0.2 20.2 0.5', 'size 0.2 60.2 0.5').replace('size 40.2 0.2 0.5', 'size 60.2 0.2 0.5')
        world = world[:match.start()]+block+world[end:]
    return world, {'seed': seed, 'size': [60., 60.], 'base': [0., 0.], 'features': features}


def circular_radar_moon(world, seed=42):
    """Diameter 60 m disk with a physical polygonal perimeter, no square floor.

    The rim is hidden only from the main overview by the observer; it remains
    visible to lidar and collidable. Its geometry is not an allocator input.
    """
    world, meta = radar_moon(world, seed)
    keep = {f['def'] for f in meta['features']
            if math.hypot(*f['position'][:2])+f['radius'] < 29.}
    for match, end, _ in reversed(list(solid_blocks(world, r'DEF (DEMO_(HILL|CRATER)_\d+) Solid\s*\{'))):
        if match[1] not in keep:
            world = world[:match.start()]+world[end:]
    meta['features'] = [f for f in meta['features'] if f['def'] in keep]
    count, radius = 96, 30.
    points = [(0., 0., .05), (0., 0., -.05)]
    points += [(radius*math.cos(i*math.tau/count), radius*math.sin(i*math.tau/count), z)
               for z in (.05, -.05) for i in range(count)]
    faces = []
    for i in range(count):
        j = (i+1) % count
        faces += [(0, 2+i, 2+j), (1, 2+count+j, 2+count+i),
                  (2+i, 2+count+i, 2+count+j), (2+i, 2+count+j, 2+j)]
    coordinates = ', '.join(f'{x:.5f} {y:.5f} {z}' for x, y, z in points)
    indices = ', '.join(' '.join(map(str, face))+' -1' for face in faces)
    texture = ', '.join(f'{x/60+.5:.5f} {y/60+.5:.5f}' for x, y, z in points)
    floor = f'''DEF LUNAR_GROUND Solid {{ translation 0 0 -0.05 name "lunar_ground" locked TRUE
      children [ Shape {{ appearance PBRAppearance {{ baseColor 0.65 0.65 0.65
        baseColorMap ImageTexture {{ url [ "textures/moon_ground.jpg" ] }}
        textureTransform TextureTransform {{ scale 12 12 }} roughness 1 metalness 0 }}
        geometry DEF CIRCULAR_GROUND_MESH IndexedFaceSet {{ coord Coordinate {{ point [ {coordinates} ] }}
          coordIndex [ {indices} ] texCoord TextureCoordinate {{ point [ {texture} ] }} }} }} ]
      boundingObject USE CIRCULAR_GROUND_MESH }}'''
    match, end, _ = next(solid_blocks(world, r'DEF LUNAR_GROUND Solid\s*\{'))
    world = world[:match.start()]+floor+world[end:]
    for match, end, _ in reversed(list(solid_blocks(world, r'DEF BOUNDARY_(WEST|EAST|SOUTH|NORTH) Solid\s*\{'))):
        world = world[:match.start()]+world[end:]
    for i in range(count):
        angle = (i+.5)*math.tau/count
        # Facet centers sit on the polygon apothem. Adjacent segments overlap
        # slightly so a lidar ray cannot leak between collision boxes.
        r = radius*math.cos(math.pi/count)
        size = f'{2*radius*math.sin(math.pi/count)+.03:.6f} 0.2 0.5'
        world += f'''\nDEF BOUNDARY_RING_{i} Solid {{ translation {r*math.cos(angle):.6f} {r*math.sin(angle):.6f} 0.25
          rotation 0 0 1 {angle+math.pi/2:.8f} name "boundary_ring_{i}" locked TRUE
          children [ Shape {{ appearance PBRAppearance {{ baseColor 0.55 0.55 0.55 roughness 1 }}
            geometry Box {{ size {size} }} castShadows FALSE }} ] boundingObject Box {{ size {size} }} }}'''
    return world, {**meta, 'shape': 'disk', 'radius': radius, 'boundary_segments': count}
