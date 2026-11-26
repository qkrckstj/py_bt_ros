"""Frontier goals and distance-based allocation, independent of ROS and world assets.

Only observed free cells are traversable. The policy deliberately has no shadow,
radio-range, YOLO, LLM, fixed room or simulator ground-truth inputs.
"""
from collections import OrderedDict, deque
from dataclasses import dataclass
import math

import numpy as np
from scipy import ndimage


@dataclass(frozen=True)
class Goal:
    x: float
    y: float
    yaw: float
    gain: float

    @property
    def xy(self):
        return self.x, self.y


def exploration_claim(position, goal, radius=2.5, approach=2.):
    """A work area around the target and its final approach, not the whole route."""
    length = math.dist(position[:2], goal[:2])
    fraction = min(1., approach/max(length, 1e-9))
    start = [goal[i]+(position[i]-goal[i])*fraction for i in range(2)]
    return {'start': start, 'end': list(goal[:2]), 'radius': radius}


def in_exploration_claim(point, claim):
    a, b = claim['start'], claim['end']
    delta = [b[i]-a[i] for i in range(2)]
    squared = sum(d*d for d in delta)
    t = max(0., min(1., sum((point[i]-a[i])*delta[i] for i in range(2))/squared)) if squared else 0.
    nearest = [a[i]+t*delta[i] for i in range(2)]
    return math.dist(point[:2], nearest) < claim['radius']


class FrontierGrid:
    def __init__(self, cells, resolution, origin=(0., 0., 0.), *,
                 clearance=.32, spacing=.8, gain_radius=1.5, bounds=(), min_frontier_length=.5,
                 cache_distances=False):
        self.cells = np.asarray(cells)
        if self.cells.ndim != 2 or not self.cells.size:
            raise ValueError('Expected a nonempty 2D occupancy grid')
        if not np.isfinite(self.cells).all() or np.any((self.cells < -1) | (self.cells > 100)):
            raise ValueError('Occupancy values must be -1 or 0..100')
        if not np.equal(self.cells, np.floor(self.cells)).all():
            raise ValueError('Occupancy values must be integers')
        if (not all(math.isfinite(v) for v in (resolution, clearance, spacing, gain_radius, min_frontier_length, *origin))
                or resolution <= 0 or clearance < 0 or spacing <= 0 or gain_radius <= 0
                or min_frontier_length < 0 or len(origin) != 3):
            raise ValueError('Invalid grid geometry or frontier parameters')
        if bounds and (len(bounds) != 4 or not all(math.isfinite(v) for v in bounds)
                       or bounds[0] >= bounds[2] or bounds[1] >= bounds[3]):
            raise ValueError('bounds must be [xmin, ymin, xmax, ymax]')
        self.cells = self.cells.astype(np.int8, copy=True)
        self.resolution, self.origin, self.clearance = resolution, tuple(origin), clearance
        self.bounds = tuple(bounds)
        free = (self.cells >= 0) & (self.cells < 25)
        distance = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * resolution
        self.safe = free & (distance >= clearance)
        # A ROS map is a snapshot: completion and allocation can reuse the same
        # distance field until the next map object or a different start cell.
        # Protect both inputs from accidental edits that would stale the cache.
        self._distance_cache = OrderedDict() if cache_distances else None
        if cache_distances:
            self.cells.setflags(write=False)
            self.safe.setflags(write=False)
        unknown = self.cells == -1
        yy, xx = np.indices(self.cells.shape)
        wx, wy = self.world(yy, xx)
        self.in_bounds = np.ones_like(free) if not bounds else (
            (wx >= bounds[0]) & (wy >= bounds[1]) & (wx < bounds[2]) & (wy < bounds[3]))
        # The exterior is unobserved, never silently assumed to be a wall.
        padded_unknown = np.pad(unknown & self.in_bounds, 1, constant_values=not bool(bounds))
        adjacent = ndimage.binary_dilation(padded_unknown)[1:-1, 1:-1]
        boundary = free & adjacent & self.in_bounds
        self.frontier_cells = int(boundary.sum())
        labels, _ = ndimage.label(boundary, structure=np.ones((3, 3)))
        sizes = np.bincount(labels.ravel())
        keep = sizes * resolution >= min_frontier_length
        keep[0] = False
        boundary &= keep[labels]
        self.significant_frontier_cells = int(boundary.sum())
        if not boundary.any():
            self.candidates = []
            return
        band = ndimage.distance_transform_edt(~boundary) * resolution
        possible = self.safe & self.in_bounds & (band <= clearance + resolution * 2)
        # Keep several samples along long frontiers; one cluster may span a room.
        groups = {}
        for row, col in zip(*np.nonzero(possible)):
            x, y = self.world(row, col)
            groups.setdefault((math.floor(x / spacing), math.floor(y / spacing)), []).append((row, col))
        self.candidates = []
        radius = max(1, math.ceil(gain_radius / resolution))
        for group in groups.values():
            row, col = max(group, key=lambda p: (distance[p], -band[p], -p[0], -p[1]))
            y0, y1 = max(0, row - radius), min(free.shape[0], row + radius + 1)
            x0, x1 = max(0, col - radius), min(free.shape[1], col + radius + 1)
            uy, ux = np.nonzero((unknown & self.in_bounds)[y0:y1, x0:x1])
            if not len(ux):
                continue
            # Gain is an approximate count, not an occlusion-aware sensor model.
            yaw = math.atan2(float(uy.mean() + y0 - row), float(ux.mean() + x0 - col)) + origin[2]
            x, y = self.world(row, col)
            self.candidates.append(Goal(float(x), float(y), yaw, len(ux) * resolution ** 2))
        self.candidates.sort(key=lambda g: (g.x, g.y))

    def exploration_remaining(self, positions):
        """Ignore assignment cooldowns and separation when checking exhaustion.

        A significant frontier without a safe/reachable observation goal is
        unresolved, not evidence of full coverage. Map edges remain unknown.
        """
        if not self.significant_frontier_cells:
            return 'EXHAUSTED'
        for xy in positions.values():
            distance = self.distances(xy)
            if any(math.isfinite(distance[self.index(g.xy)]) for g in self.candidates):
                return 'AVAILABLE'
        return 'BLOCKED'

    def world(self, row, col):
        x, y = (col + .5) * self.resolution, (row + .5) * self.resolution
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return self.origin[0] + c*x - s*y, self.origin[1] + s*x + c*y

    def index(self, xy):
        x, y = xy[0] - self.origin[0], xy[1] - self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return math.floor((-s*x + c*y) / self.resolution), math.floor((c*x + s*y) / self.resolution)

    def distances(self, xy):
        """Four-neighbor paths; do not jump across walls or unknown cells."""
        cell = self.index(xy)
        cache = self._distance_cache
        if cache is not None and cell in cache:
            cache.move_to_end(cell)
            return cache[cell]
        result = self._distances_from_cell(*cell)
        if cache is not None:
            result.setflags(write=False)
            cache[cell] = result
            # Four robots can each retain their current and previous cell;
            # moving through a large map must not retain an unbounded history.
            if len(cache) > 8:
                cache.popitem(last=False)
        return result

    def _distances_from_cell(self, row, col):
        result = np.full(self.cells.shape, np.inf)
        h, w = result.shape
        if not (0 <= row < h and 0 <= col < w):
            return result
        # SLAM cannot observe through the robot body. Only its footprint may
        # contain unknown start cells; the route beyond that must be observed.
        yy, xx = np.ogrid[:h, :w]
        squared = ((yy-row)**2+(xx-col)**2)*self.resolution**2
        seed = squared <= .38**2
        free = (self.cells >= 0) & (self.cells < 25)
        traversable = self.safe | (seed & free)
        if self.cells[row, col] == -1:
            # A forward-looking lidar leaves the cell under the robot body
            # unobserved. Bridge only that physical footprint to nearby
            # *observed* free cells, and never bridge over a measured obstacle.
            body = squared <= self.clearance**2 + 1e-9
            if not np.any(body & free) or np.any(body & (self.cells >= 25)):
                return result
            traversable |= body & (self.cells == -1)
        if not traversable[row, col]:
            return result
        result[row, col] = 0.
        queue = deque([(row, col)])
        while queue:
            y, x = queue.popleft()
            for ny, nx in ((y-1, x), (y+1, x), (y, x-1), (y, x+1)):
                if 0 <= ny < h and 0 <= nx < w and traversable[ny, nx] and not math.isfinite(result[ny, nx]):
                    result[ny, nx] = result[y, x] + self.resolution
                    queue.append((ny, nx))
        return result

    def bounds_fully_known(self):
        """Strict completion certificate for a specified rectangle, never auto-grown extent."""
        if not self.bounds:
            return False
        xmin, ymin, xmax, ymax = self.bounds
        # All four ROI corners must lie inside this map (including rotated maps).
        eps = self.resolution * 1e-6
        for xy in ((xmin+eps, ymin+eps), (xmin+eps, ymax-eps),
                   (xmax-eps, ymin+eps), (xmax-eps, ymax-eps)):
            row, col = self.index(xy)
            if not (0 <= row < self.cells.shape[0] and 0 <= col < self.cells.shape[1]):
                return False
        return bool(self.in_bounds.any() and np.all(self.cells[self.in_bounds] >= 0))

    def path(self, position, goal, distances):
        row, col = self.index(goal)
        if not math.isfinite(distances[row, col]):
            return []
        route = [(row, col)]
        while distances[row, col] > 1e-6:
            neighbors = [(y, x) for y, x in ((row-1, col), (row+1, col), (row, col-1), (row, col+1))
                         if 0 <= y < distances.shape[0] and 0 <= x < distances.shape[1]
                         and distances[y, x] < distances[row, col]-1e-6]
            if not neighbors:
                return []
            row, col = min(neighbors, key=lambda p: distances[p])
            route.append((row, col))
        return [tuple(position), *[self.world(y, x) for y, x in reversed(route)]]


def allocate(grid, positions, *, reserved=(), excluded=(), separation=2., min_distance=.5,
             transform_goal=None, validate_goal=None):
    """Deterministic greedy matching using reachable path distance and bounded gain.

    `reserved` are active goal xy pairs. `excluded` are temporary (xy, radius)
    entries from finished/failed visits; they never modify the map or completion.
    """
    choices = []
    for robot, xy in sorted(positions.items()):
        distance = grid.distances(xy)
        for goal in grid.candidates:
            if not math.isfinite(distance[grid.index(goal.xy)]):
                continue
            if transform_goal:
                goal = transform_goal(robot, goal, distance)
                if goal is None:
                    continue
            if math.dist(xy[:2], goal.xy) < min_distance:
                continue
            if any(math.dist(goal.xy, p) < radius for p, radius in excluded):
                continue
            cost = distance[grid.index(goal.xy)]
            if math.isfinite(cost):
                choices.append((cost - .12 * min(goal.gain, 8.), robot, goal))
    choices.sort(key=lambda item: (item[0], item[1], item[2].x, item[2].y))
    result, occupied = {}, list(reserved)
    for _, robot, goal in choices:
        if (robot not in result and all(math.dist(goal.xy, p) >= separation for p in occupied)
                and (validate_goal is None or validate_goal(robot, goal, result))):
            result[robot] = goal
            occupied.append(goal.xy)
    return result
