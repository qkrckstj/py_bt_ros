"""Observed-map coverage, never inferred from goals or planned robot paths."""
import math

import numpy as np


def coverage_snapshot(grid):
    """Overlay observed cells; count a fixed ROI including unmapped exterior.

    Percent is sampled at map resolution on a world-aligned mission rectangle.
    An observed obstacle counts as observed, but not as traversed free space.
    """
    overlay = np.where((grid.cells >= 0) & grid.in_bounds, 100, -1).astype(np.int8)
    if grid.bounds:
        xmin, ymin, xmax, ymax = grid.bounds
        xs = np.arange(xmin+grid.resolution/2, xmax, grid.resolution)
        ys = np.arange(ymin+grid.resolution/2, ymax, grid.resolution)
        x, y = np.meshgrid(xs-grid.origin[0], ys-grid.origin[1])
        c, s = math.cos(grid.origin[2]), math.sin(grid.origin[2])
        rows = np.floor((-s*x+c*y)/grid.resolution).astype(int)
        cols = np.floor((c*x+s*y)/grid.resolution).astype(int)
        inside = ((rows >= 0) & (rows < grid.cells.shape[0])
                  & (cols >= 0) & (cols < grid.cells.shape[1]))
        samples = np.full(rows.shape, -1, dtype=np.int8)
        samples[inside] = grid.cells[rows[inside], cols[inside]]
    else:
        samples = grid.cells
    total = int(samples.size)
    observed = int(np.count_nonzero(samples >= 0))
    free = int(np.count_nonzero((samples >= 0) & (samples < 25)))
    return overlay, {
        'definition': 'Observed cells in mission bounds; includes observed obstacles, not robot visits',
        'total_cells': total, 'observed_cells': observed,
        'observed_free_cells': free, 'unknown_cells': total-observed,
        'observed_percent': 100.*observed/total if total else 0.,
        'observed_area_m2': observed*grid.resolution**2,
        'bounds_fully_known': grid.bounds_fully_known(),
    }
