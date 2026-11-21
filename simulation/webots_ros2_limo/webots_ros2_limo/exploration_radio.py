"""Distance-only multihop radio model and conservative connectivity shield.

Edges join centers within ONE radio radius, not two overlapping discs. Positions
can be localized planning poses or simulator radio-environment positions.
The latter model radio availability and a final simulated collision veto;
they never populate navigation, allocation or maps.
"""
import math
from collections import deque


def physical_radio_permit(radio, positions, yaws, commands, robots, horizon=1.2, clearance=0.):
    """Validate a complete joint command against measured radio and separation.

    This is only a simulator interlock, not a navigation pose source. Pairwise
    clearance can be requested when stopping selected peers changes the motion
    that the upstream traffic coordinator checked.
    """
    if set(positions) != set(robots) or set(yaws) != set(robots):
        return False
    if not all(math.isfinite(v) for p in positions.values() for v in p):
        return False
    if not all(math.isfinite(v) for v in yaws.values()):
        return False
    if not all(len(c) == 2 and all(math.isfinite(v) for v in c) for c in commands.values()):
        return False
    if not radio.graph(positions)['all_connected']:
        return False
    safe = radio.graph(positions, safe=True)['all_connected']
    names = tuple(robots)
    for i in range(13):
        future = {n: radio.project(p, yaws[n], *commands.get(n, (0., 0.)), horizon*i/12)
                  for n, p in positions.items()}
        if not radio.graph(future, safe=safe)['all_connected']:
            return False
        if clearance and any(math.dist(future[a], future[b]) < clearance
                             for j, a in enumerate(names) for b in names[j+1:]):
            return False
    return True


def select_physical_commands(radio, positions, yaws, commands, robots, *,
                             horizon=1.2, clearance=.85, inputs_fresh=True):
    """Retain the largest safe subset of at most four upstream robot commands.

    Each output is either the unchanged requested (v, w) or zero. Recheck every
    combination as a whole, including collisions with peers stopped by this
    interlock. This allows independent safe motion without inventing a recovery
    route or feeding world poses into navigation. Ties use stable robot order.
    """
    names = tuple(robots)
    if len(names) > 4 or len(set(names)) != len(names):
        raise ValueError('The bounded physical interlock supports at most four unique robots')
    stopped = {n: (0., 0.) for n in names}
    requested = {n: tuple(commands.get(n, (0., 0.))) for n in names}
    movers = [n for n in names if any(requested[n])]
    # No subset can repair missing data, an already disconnected network or an
    # existing overlap. Preserve the complete hold in those cases.
    if (not inputs_fresh or not physical_radio_permit(
            radio, positions, yaws, stopped, names, horizon, clearance)):
        return stopped, movers
    for mask in sorted(range(1 << len(movers)), key=lambda m: (-m.bit_count(), m)):
        trial = dict(stopped)
        for i, name in enumerate(movers):
            if mask & (1 << i):
                trial[name] = requested[name]
        if physical_radio_permit(radio, positions, yaws, trial, names, horizon, clearance):
            return trial, [n for n in movers if trial[n] == (0., 0.)]
    return stopped, movers


class RadioNetwork:
    def __init__(self, base, radius=7., margin=.5):
        if len(base) != 2 or not all(math.isfinite(v) for v in (*base, radius, margin)) or not 0 <= margin < radius:
            raise ValueError('Invalid radio base, range or margin')
        self.base, self.radius, self.margin = tuple(base), radius, margin

    def graph(self, positions, *, safe=False):
        points = {'base': self.base, **positions}
        radius = self.radius-self.margin if safe else self.radius
        links = [(a, b) for i, a in enumerate(points) for b in list(points)[i+1:]
                 if math.dist(points[a], points[b]) <= radius]
        neighbors = {n: [] for n in points}
        for a, b in links:
            neighbors[a].append(b)
            neighbors[b].append(a)
        parents, hops, queue = {'base': None}, {'base': 0}, deque(['base'])
        while queue:
            a = queue.popleft()
            for b in neighbors[a]:
                if b not in parents:
                    parents[b], hops[b] = a, hops[a]+1
                    queue.append(b)
        return {'base': list(self.base), 'radius': self.radius, 'margin': self.margin,
                'positions': {n: list(p) for n, p in positions.items()}, 'links': links,
                'parents': parents, 'hops': hops,
                'connected': {n: n in parents for n in positions},
                'all_connected': all(n in parents for n in positions)}

    def allows(self, positions, robot, point, *, safe=True):
        return self.graph({**positions, robot: point}, safe=safe)['all_connected']

    def advance(self, positions, robot, path):
        """Furthest contiguous radio-safe point on an observed-free grid route."""
        last = positions[robot]
        for point in path:
            if not self.allows(positions, robot, point):
                break
            last = point
        return tuple(last)

    @staticmethod
    def project(position, yaw, v, w, seconds):
        if abs(w) < 1e-6:
            return (position[0]+v*seconds*math.cos(yaw), position[1]+v*seconds*math.sin(yaw))
        return (position[0]+v/w*(math.sin(yaw+w*seconds)-math.sin(yaw)),
                position[1]-v/w*(math.cos(yaw+w*seconds)-math.cos(yaw)))

    def constrain(self, positions, yaws, commands, order, *, horizon=.8):
        """Test the combined motion, including relays and downstream robots.

        Grant velocities in priority order only if all sampled intermediate team
        configurations retain a base-rooted graph. A relay cannot leave merely
        because its own neighbor still exists. Stale/missing inputs are handled
        by the caller. Rotation of circular radio discs does not change links.
        """
        granted = {n: (0., commands.get(n, (0., 0.))[1]) for n in positions}
        blocked = []
        if not self.graph(positions)['all_connected']:
            return {n: (0., 0.) for n in positions}, list(commands)
        for name in order:
            proposed = {**granted, name: commands.get(name, (0., 0.))}
            valid = True
            # Use the margin while safely inside it; allow recovery toward a
            # connected configuration if small estimation drift consumes it.
            safe = self.graph(positions, safe=True)['all_connected']
            for i in range(1, 9):
                future = {n: self.project(p, yaws[n], *proposed[n], horizon*i/8)
                          for n, p in positions.items()}
                if not self.graph(future, safe=safe)['all_connected']:
                    valid = False
                    break
            if valid:
                granted = proposed
            else:
                blocked.append(name)
        return granted, blocked
