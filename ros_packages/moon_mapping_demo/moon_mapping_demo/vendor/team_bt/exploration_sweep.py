"""Map-driven radial formation sweep; no simulator terrain or hidden map input.

The rectangle is a mission boundary, not an occupancy prior. Slots describe a
preferred formation. Every issued waypoint must still be reachable through
observed free space, and the existing radio and traffic guards remain in charge.
"""
import math

import numpy as np

from .exploration_policy import Goal


class RadarSweep:
    def __init__(self, robots, base=(0., 0.), bounds=(-29., -29., 29., 29.), step_degrees=5., radius=28., slot_slack=0.):
        if (not robots or len(set(robots)) != len(robots) or len(base) != 2 or len(bounds) != 4
                or not all(math.isfinite(v) for v in (*base, *bounds, step_degrees, radius, slot_slack))
                or not bounds[0] < base[0] < bounds[2] or not bounds[1] < base[1] < bounds[3]
                or not 1 <= step_degrees <= 10 or radius <= 0 or not 0 <= slot_slack <= 1.5):
            raise ValueError('Invalid radial sweep geometry')
        self.robots, self.base, self.bounds = tuple(robots), tuple(base), tuple(bounds)
        self.step = math.radians(step_degrees)
        self.limit = radius
        self.slot_slack = slot_slack
        self.angle, self.radius, self.stage = 0., radius, 'DEPLOY'
        self.stage_since, self.best_error, self.last_progress = None, math.inf, None
        self.reason = ''
        self.targets = {}
        self.unavailable_since = None
        self.skipped_sectors = []

    def advance_sector(self, now):
        self.stage = 'ROTATE'
        self.angle += self.step
        if self.angle >= math.tau-1e-9:
            self.angle, self.stage = math.tau, 'CLEANUP'
        self.stage_since, self.last_progress, self.best_error = now, now, math.inf
        self.unavailable_since = None

    def skip_unavailable_sector(self, idle, active, chosen, now):
        """Try the next preferred line after 2 s with no admissible waypoint.

        This is not arrival or exploration completion: unknown cells stay
        unknown, and the frontier cleanup must still inspect remaining space.
        Deployment still requires measured predecessor progress.
        """
        if (self.stage != 'ROTATE' or set(idle) != set(self.robots) or active or chosen):
            self.unavailable_since = None
            return False
        if self.unavailable_since is None or now < self.unavailable_since:
            self.unavailable_since = now
        if now-self.unavailable_since < 2.:
            return False
        self.reason = 'Skipped preferred sector: no admissible known-free waypoint; retain for frontier cleanup'
        self.skipped_sectors.append({'angle_degrees': math.degrees(self.angle),
                                     'time': now, 'reason': self.reason})
        self.advance_sector(now)
        return True

    def reach(self):
        c, s = math.cos(self.angle), math.sin(self.angle)
        distances = []
        for component, center, low, high in ((c, self.base[0], self.bounds[0], self.bounds[2]),
                                             (s, self.base[1], self.bounds[1], self.bounds[3])):
            if abs(component) > 1e-9:
                distances.append(((high if component > 0 else low)-center)/component)
        return min(self.limit, *distances)

    def slots(self):
        radius = min(self.radius, self.reach()) if self.stage == 'DEPLOY' else self.reach()
        return {name: (self.base[0]+radius*(i+1)/len(self.robots)*math.cos(self.angle),
                       self.base[1]+radius*(i+1)/len(self.robots)*math.sin(self.angle))
                for i, name in enumerate(self.robots)}

    def deployment_targets(self, positions):
        """Outer leader goes first; each follower uses its predecessor's pose.

        A dispatched future goal is never treated as a reached relay position.
        The final radial slots stay fixed so the inner robot remains near base
        when the deployed line starts rotating.
        """
        slots = self.slots()
        axis = (math.cos(self.angle), math.sin(self.angle))
        gap = self.reach()/len(self.robots)
        targets = {self.robots[-1]: slots[self.robots[-1]]}
        for i, name in enumerate(self.robots[:-1]):
            front = positions[self.robots[i+1]]
            progress = sum((front[k]-self.base[k])*axis[k] for k in (0, 1))
            own = sum((positions[name][k]-self.base[k])*axis[k] for k in (0, 1))
            distance = min(gap*(i+1), progress-gap)
            # Collect a useful leg instead of repeatedly finishing tiny goals
            # as the predecessor moves. Allow the last short alignment leg.
            final = math.dist(positions[name], slots[name]) <= 1.5
            if distance-own < (0.5 if final else 1.5):
                # A terrain detour may finish at the correct forward radius
                # but off the line. Once the predecessor actually reaches its
                # final radius, permit sideways alignment within the same
                # 1.25 m formation tolerance; never chase it backwards.
                at_final_radius = abs(own-gap*(i+1)) <= 1.25+self.slot_slack
                front_deployed = progress >= gap*(i+2)-1.25-self.slot_slack
                aligned = tuple(self.base[k]+max(own, distance)*axis[k] for k in (0, 1))
                if at_final_radius and front_deployed and math.dist(positions[name], aligned) > .6+self.slot_slack:
                    targets[name] = aligned
                else:
                    targets[name] = positions[name]
            else:
                targets[name] = tuple(self.base[k]+distance*axis[k] for k in (0, 1))
        return targets

    def update(self, positions, idle, now, grid):
        if self.stage == 'CLEANUP':
            return
        if self.stage_since is None:
            self.stage_since = self.last_progress = now
        slots = self.slots()
        errors = []
        for name, slot in slots.items():
            error = math.dist(positions[name], slot)
            # Nearby occupied terrain may force a formation slot aside. Unknown
            # slots never count as visited just because a robot cannot reach one.
            row, col = grid.index(slot)
            occupied = (0 <= row < grid.cells.shape[0] and 0 <= col < grid.cells.shape[1]
                        and grid.cells[row, col] >= 25)
            # Formation arrival is deliberately looser than Nav2's 0.25 m
            # positional arrival: snapped safe cells, the 0.5 m minimum next
            # leg and controller tolerance otherwise leave a sub-metre dead
            # zone in which no goal is useful but the stage can never advance.
            errors.append(max(0., error-(2.5 if occupied else 1.25+self.slot_slack)))
        error = sum(errors)
        if error < self.best_error-.2:
            self.best_error, self.last_progress = error, now
        if not any(errors):
            # Advance on measured formation proximity, without waiting for
            # every last Nav2 result/observation timer. Existing actions finish
            # normally; idle peers can already receive the next sector's leg.
            self.advance_sector(now)
        elif now-self.last_progress > 90.:
            # Do not deadlock on an occluded/unreachable ideal line, and never
            # equate formation failure or one revolution with map completion.
            self.stage, self.reason = 'CLEANUP', 'Formation stalled; continue reachable frontier exploration'

    def choose(self, grid, positions, idle, active, radio=None, excluded=()):
        """Reachable known waypoints, up to 6 m deploying / 4 m rotating.

        Separate per-robot slots prevent nearest-frontier clustering. Path and
        radio constraints can shorten a leg while scans reveal the next section.
        """
        chosen = {}
        if not idle:
            return chosen
        projected = {**positions, **{n: t['goal'][:2] for n, t in active.items()}}
        rows, cols = np.nonzero(grid.safe & grid.in_bounds)
        if not len(rows):
            return chosen
        xs, ys = grid.world(rows, cols)
        slots = self.deployment_targets(positions) if self.stage == 'DEPLOY' else self.slots()
        self.targets = slots
        order = reversed(self.robots) if self.stage == 'DEPLOY' else self.robots
        for name in order:
            if name not in idle or math.dist(positions[name], slots[name]) <= .6+self.slot_slack:
                continue
            distance = grid.distances(positions[name])
            costs = distance[rows, cols]
            valid = np.isfinite(costs) & (costs >= .5)
            # Reserve actual peers and accepted targets; global costmaps and the
            # existing traffic coordinator perform detailed transit avoidance.
            for peer, p in projected.items():
                if peer != name:
                    valid &= (xs-p[0])**2+(ys-p[1])**2 >= 1.1**2
            for point, radius in excluded:
                valid &= (xs-point[0])**2+(ys-point[1])**2 >= radius**2
            error = np.hypot(xs-slots[name][0], ys-slots[name][1])
            valid &= error < math.dist(positions[name], slots[name])-.25
            indices = np.flatnonzero(valid)
            if not len(indices):
                continue
            # Searching several alternatives permits a radio-safe sideways
            # approach instead of repeatedly insisting on a blocked endpoint.
            # A small region around the preferred slot is equally useful. Pick
            # the cheaper reachable approach there instead of pulling every
            # robot back onto the mathematical line after a terrain detour.
            # Connectivity, observed-free paths and clearance remain mandatory.
            score = np.maximum(0., error[indices]-self.slot_slack)+.04*costs[indices]
            ordered = indices[np.argsort(score)[:24]]
            for index in ordered:
                target = (float(xs[index]), float(ys[index]))
                route = grid.path(positions[name], target, distance)
                if not route:
                    continue
                leg = [route[0]]
                length = 0.
                for point in route[1:]:
                    length += math.dist(leg[-1], point)
                    if length > (6. if self.stage == 'DEPLOY' else 4.):
                        break
                    leg.append(point)
                point = radio.advance(positions, name, leg) if radio else leg[-1]
                if math.dist(point, positions[name]) < .5:
                    continue
                if any(peer != name and math.dist(point, p) < 1.1 for peer, p in projected.items()):
                    continue
                if radio and not radio.allows(projected, name, point):
                    continue
                yaw = math.atan2(point[1]-positions[name][1], point[0]-positions[name][0])
                chosen[name] = Goal(*point, yaw, 0.)
                projected[name] = point
                break
        return chosen

    def snapshot(self):
        return {'stage': self.stage, 'angle_degrees': math.degrees(self.angle),
                'radius': min(self.radius, self.reach()) if self.stage == 'DEPLOY' else self.reach(),
                'base': self.base, 'bounds': self.bounds, 'slots': self.slots(),
                'slot_slack': self.slot_slack,
                'targets': self.targets, 'leader': self.robots[-1], 'reason': self.reason,
                'skipped_sectors': self.skipped_sectors}
