"""Small physical test worlds. Geometry is never an allocator input."""
import math


def layout(kind='branched'):
    if kind == 'moon_radar':
        return {'robot1': [1.5, -.7, 0.], 'robot2': [1.5, .7, 0.],
                'robot3': [2.9, -.7, 0.], 'robot4': [2.9, .7, 0.]}, []
    if kind == 'moon':
        return {'robot1': [-17.5, -.7, 0.], 'robot2': [-17.5, .7, 0.],
                'robot3': [-16.1, -.7, 0.], 'robot4': [-16.1, .7, 0.]}, []
    walls = [(0., -12., 24., .15), (0., 12., 24., .15),
             (-12., 0., .15, 24.), (12., 0., .15, 24.)]
    if kind == 'plain':
        starts = {'robot1': [-8., -8., -3*math.pi/4], 'robot2': [-8., 8., 3*math.pi/4],
                  'robot3': [8., -8., -math.pi/4], 'robot4': [8., 8., math.pi/4]}
        return starts, walls
    if kind != 'branched':
        raise ValueError(f'Unknown physical world: {kind}')
    starts = {f'robot{i+1}': [x, -9.5, math.pi/2] for i, x in enumerate((-1.8, -.6, .6, 1.8))}
    # Central passage, side rooms, wide door openings, and several occluding
    # structures. All walls/pillars are 0.5 m high, with no closed-off rooms.
    for x, spans in ((-3., [(-8.2, -6.2), (-3.8, 4.), (6.5, 10.8)]),
                     (3., [(-8.2, -6.2), (-3.8, 3.), (5.5, 10.8)])):
        walls.extend((x, (lo+hi)/2, .22, hi-lo) for lo, hi in spans)
    walls += [(-9.9, 0., 3.8, .22), (-4.25, 0., 2.5, .22),
              (4.5, 1., 3., .22), (10.15, 1., 3.3, .22),
              (-7.8, 5.5, 3.2, .22), (8.3, 7., .22, 3.),
              (-8., -5., 1.2, 1.2), (7.5, -6., 1.4, 1.4),
              (-7., 8.5, 1., 1.), (0., 3.5, 1.2, 1.2)]
    return starts, walls
