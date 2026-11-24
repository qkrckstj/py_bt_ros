"""Mission settings and fixed-area map coverage; independent of ROS."""
import json
import math
from collections import deque
from pathlib import Path

import numpy as np


def settings(path=None):
    config = json.loads(Path(path or Path(__file__).with_name('mission.json')).read_text())
    value = config.get('radio_range_m')
    if value is None or not math.isfinite(value) or value <= 0:
        raise ValueError('Set the lunar radio_range_m in mission.json; do not inherit the factory demo radius')
    if config.get('connectivity_mode') != 'mobile_team' or config.get('base_xy') is not None:
        raise ValueError('This lunar executor supports a mobile connected team; fixed-base access needs a separate mission model')
    return config


def coverage(cells, resolution, origin, positions, bounds, sample_resolution=.1):
    """Resample into fixed mission bounds so changing SLAM extents cannot
    masquerade as percent coverage. Unknown cells reachable through unknown
    space are still required. Enclosed unknown is reported separately, never
    silently counted as observed. This is a 2D map estimate, not terrain truth.
    """
    xmin, ymin, xmax, ymax = bounds
    width, height = math.ceil((xmax-xmin)/sample_resolution), math.ceil((ymax-ymin)/sample_resolution)
    ys, xs = np.mgrid[:height, :width]
    columns = np.floor((xmin+(xs+.5)*sample_resolution-origin[0])/resolution).astype(int)
    rows = np.floor((ymin+(ys+.5)*sample_resolution-origin[1])/resolution).astype(int)
    inside = (rows >= 0) & (rows < cells.shape[0]) & (columns >= 0) & (columns < cells.shape[1])
    sampled = np.full((height, width), -1, dtype=np.int16)
    sampled[inside] = cells[rows[inside], columns[inside]]
    possible = sampled < 20
    queue = deque()
    reached = np.zeros(sampled.shape, dtype=bool)
    # Pose-local seeds avoid declaring a region inaccessible just because the
    # robots themselves appear as occupied returns in another robot's map.
    for x, y in positions.values():
        row, col = int((y-ymin)/sample_resolution), int((x-xmin)/sample_resolution)
        radius = math.ceil(.4/sample_resolution)
        for r in range(max(0,row-radius), min(height,row+radius+1)):
            for c in range(max(0,col-radius), min(width,col+radius+1)):
                if possible[r, c] and not reached[r, c]:
                    reached[r, c] = True
                    queue.append((r, c))
    while queue:
        row, col = queue.popleft()
        for r, c in ((row-1,col), (row+1,col), (row,col-1), (row,col+1)):
            if 0 <= r < height and 0 <= c < width and possible[r,c] and not reached[r,c]:
                reached[r,c] = True
                queue.append((r,c))
    unknown = sampled < 0
    remaining = int(np.count_nonzero(reached & unknown))
    area = sample_resolution**2
    return {
        'mission_area_m2': width*height*area,
        'observed_area_m2': float(np.count_nonzero(~unknown)*area),
        'observed_fraction': float(np.count_nonzero(~unknown)/sampled.size),
        'unknown_potentially_accessible_m2': remaining*area,
        'unknown_enclosed_m2': float(np.count_nonzero(unknown & ~reached)*area),
        'all_accessible_observed': bool(np.any(reached) and remaining == 0),
        'mission_complete': False,
        'resolution_m': sample_resolution,
    }
