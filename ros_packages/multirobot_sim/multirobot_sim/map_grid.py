"""Align a planar occupancy grid with the shared map axes."""

import math
from dataclasses import dataclass

import numpy as np


MAP_RESOLUTION = 0.05


@dataclass(frozen=True)
class Grid:
    cells: np.ndarray
    resolution: float
    x: float
    y: float
    yaw: float = 0.0


def align_grid(grid, resolution=MAP_RESOLUTION):
    """Resample one SLAM grid on the axis-aligned map used by Nav2."""
    cosine, sine = math.cos(grid.yaw), math.sin(grid.yaw)
    source_height, source_width = grid.cells.shape
    height, width = source_height * grid.resolution, source_width * grid.resolution
    corners_x = (grid.x, grid.x + cosine * width,
                 grid.x - sine * height, grid.x + cosine * width - sine * height)
    corners_y = (grid.y, grid.y + sine * width,
                 grid.y + cosine * height, grid.y + sine * width + cosine * height)
    # Avoid an extra cell from roundoff at exact grid boundaries.
    x0 = math.floor(min(corners_x) / resolution + 1e-9)
    y0 = math.floor(min(corners_y) / resolution + 1e-9)
    x1 = math.ceil(max(corners_x) / resolution - 1e-9)
    y1 = math.ceil(max(corners_y) / resolution - 1e-9)
    if (x1 - x0) * (y1 - y0) > 4_000_000:
        raise ValueError('Shared map exceeds 4,000,000 cells. Check the map origin and start pose.')

    cells = np.full((y1 - y0, x1 - x0), -1, dtype=np.int8)
    # Sample each output cell center in the input grid's coordinates.
    dx = (np.arange(x0, x1) + 0.5) * resolution - grid.x
    dy = (np.arange(y0, y1)[:, None] + 0.5) * resolution - grid.y
    columns = np.floor((cosine * dx + sine * dy) / grid.resolution).astype(np.intp)
    rows = np.floor((-sine * dx + cosine * dy) / grid.resolution).astype(np.intp)
    inside = ((rows >= 0) & (rows < source_height)
              & (columns >= 0) & (columns < source_width))
    cells[inside] = grid.cells[rows[inside], columns[inside]]

    return Grid(cells, resolution, x0 * resolution, y0 * resolution)
