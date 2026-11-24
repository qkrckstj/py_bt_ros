"""Adapt the test Webots lidar's no-return rays to Karto's clearing semantics."""
import math
from collections import deque

import numpy as np
from scipy.spatial import cKDTree


class PersistentMappingHits:
    """Confirm static endpoints in odometry space without classifying robots.

    Unconfirmed hits are unknown (NaN), never free rays through an obstacle.
    Raw Nav2 scans are unaffected. Only historical measured endpoints are used.
    """
    def __init__(self, age=1.5, tolerance=.12):
        self.age, self.tolerance, self.history = age, tolerance, deque()

    def filter(self, now, ranges, angle_min, increment, origin, yaw, maximum=12.):
        values = np.asarray(ranges, dtype=float)
        valid = np.isfinite(values) & (values > .1) & (values < maximum)
        indices = np.flatnonzero(valid)
        angles = yaw+angle_min+indices*increment
        points = np.column_stack((origin[0]+values[valid]*np.cos(angles),
                                  origin[1]+values[valid]*np.sin(angles)))
        while self.history and now-self.history[0][0] > self.age+.6:
            self.history.popleft()
        matches = np.zeros(len(points), dtype=int)
        for stamp, tree in self.history:
            if now-stamp >= self.age and len(points):
                distances, _ = tree.query(points, distance_upper_bound=self.tolerance)
                matches += np.isfinite(distances)
        output = values.copy()
        output[indices[matches < 2]] = math.nan
        if len(points):
            self.history.append((now, cKDTree(points)))
        return output.tolist()


def mapping_ranges(ranges, minimum=.1, maximum=12.):
    """Preserve real hits, ignore invalid/too-close rays, retain far no-returns.

    Karto ignores r >= scan.range_max. It clears without an endpoint hit only
    when threshold <= r < scan.range_max. Use a private sentinel strictly above
    the physical threshold, so it is also excluded from scan-matching points.
    /scan remains untouched; only /scan_slam uses this internal representation.
    The test lidar uses near=minRange=0.01 m so near returns can be rejected
    explicitly; it assumes no external surface can be inside that 1 cm blind zone.
    """
    sentinel = maximum+.01
    output = []
    for value in ranges:
        if value == math.inf or value == maximum:
            output.append(sentinel)
        elif math.isfinite(value) and minimum < value < maximum:
            output.append(float(value))
        else:
            output.append(math.nan)
    return output, maximum+.02
