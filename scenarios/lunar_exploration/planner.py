"""Observed-grid CP-SAT allocation; distances and radio are simulation proxies."""
import math
from collections import deque

import numpy as np
from ortools.sat.python import cp_model

ROBOTS = ('robot1', 'robot2', 'robot3', 'robot4')
ROLES = ('explorer', 'relay')


def safe_cells(cells, resolution, clearance=0.4):
    blocked = (cells < 0) | (cells >= 20)
    padded = np.pad(blocked, math.ceil(clearance / resolution), constant_values=True)
    radius = math.ceil(clearance / resolution)
    unsafe = np.zeros(cells.shape, dtype=bool)
    height, width = cells.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if math.hypot(dx, dy) * resolution <= clearance:
                unsafe |= padded[radius + dy:radius + dy + height,
                                 radius + dx:radius + dx + width]
    return ~unsafe


def distances(safe, start, resolution):
    """Four-connected observed-free-space distance; not a Nav2 path guarantee."""
    height, width = safe.shape
    result = np.full(safe.shape, np.inf)
    row, col = start
    if not (0 <= row < height and 0 <= col < width and safe[row, col]):
        return result
    result[row, col] = 0
    queue = deque([(row, col)])
    while queue:
        row, col = queue.popleft()
        value = result[row, col] + resolution
        for nr, nc in ((row-1, col), (row+1, col), (row, col-1), (row, col+1)):
            if 0 <= nr < height and 0 <= nc < width and safe[nr, nc] and result[nr, nc] == np.inf:
                result[nr, nc] = value
                queue.append((nr, nc))
    return result


def trace_route(field, target, start, origin, resolution, position, max_leg=4.0):
    """Backtrack a global frontier route, then execute only its next short leg."""
    cell = target
    backwards = [cell]
    height, width = field.shape
    for _ in range(field.size):
        center = (origin[0]+(cell[1]+.5)*resolution, origin[1]+(cell[0]+.5)*resolution)
        if cell == start or math.dist(center, position) < .15:
            break
        row, col = cell
        neighbors = [(r, c) for r, c in ((row-1, col), (row+1, col), (row, col-1), (row, col+1))
                     if 0 <= r < height and 0 <= c < width and field[r, c] < field[cell]-1e-6]
        if not neighbors:
            return []
        cell = min(neighbors, key=lambda rc: field[rc])
        backwards.append(cell)
    points = [position]
    length = 0.
    for row, col in reversed(backwards):
        p = (origin[0]+(col+.5)*resolution, origin[1]+(row+.5)*resolution)
        length += math.dist(points[-1], p)
        if length > max_leg:
            break
        points.append(p)
    return points


def visible_unknown(cells, row, col, resolution, radius=6.0):
    """Potential observations along rays; known walls occlude unknown cells."""
    angles = np.linspace(0, 2*math.pi, 48, endpoint=False)[:, None]
    steps = np.arange(0, radius/resolution, .8)[None, :]
    rows = np.rint(row+np.sin(angles)*steps).astype(int)
    cols = np.rint(col+np.cos(angles)*steps).astype(int)
    inside = (rows >= 0) & (rows < cells.shape[0]) & (cols >= 0) & (cols < cells.shape[1])
    rows = np.clip(rows, 0, cells.shape[0]-1)
    cols = np.clip(cols, 0, cells.shape[1]-1)
    values = cells[rows, cols]
    clear = np.cumsum((values >= 20) | ~inside, axis=1) == 0
    ids = (rows*cells.shape[1]+cols)[clear & (values < 0)]
    return len(np.unique(ids))*resolution**2


def allocate(cells, resolution, origin, positions, homes, travelled, visited,
             rejected, radio_range, budget, forecast=None, now=0., bounds=None,
             allow_reposition=False, max_leg=4.0, staging_hold=15., solver_seconds=15.,
             compactness_weight=60, spacing_weight=0, preferred_spacing=10.,
             max_wait_s=240., maximum_explorers=3, movement_reward=0,
             frozen_robots=(), fixed_tasks=None, preferred_goals=None,
             recent_targets=None):
    """Choose global frontier targets and roles for a mobile connected team.

    All robots exchange sensor maps. Single-commodity flow enforces a connected
    endpoint network, rooted at robot1's moving pose, without a fixed base.
    Forecasts use simulator terrain truth, while navigation uses observed cells.
    Actual energy consumption is not modeled.
    """
    frozen_robots = set(frozen_robots)
    fixed_tasks = fixed_tasks or {}
    preferred_goals = preferred_goals or {}
    recent_targets = recent_targets or {}
    if bounds is not None:
        cells = cells.copy()
        rows, cols = np.mgrid[:cells.shape[0], :cells.shape[1]]
        xs = origin[0]+(cols+.5)*resolution
        ys = origin[1]+(rows+.5)*resolution
        outside = (xs < bounds[0]) | (ys < bounds[1]) | (xs > bounds[2]) | (ys > bounds[3])
        cells[outside] = 100
    else:
        cells = cells.copy()
    # Teammates' current lidar returns are moving footprints, not permanent
    # terrain. Remove only observed occupied cells within the known footprints
    # before inflation; unknown space stays unknown. Nav2 and the transit gate
    # still check actual obstacles and robot separation before executing.
    for position in positions.values():
        row = math.floor((position[1]-origin[1])/resolution)
        col = math.floor((position[0]-origin[0])/resolution)
        radius = math.ceil(.45/resolution)
        for r in range(max(0,row-radius),min(cells.shape[0],row+radius+1)):
            for c in range(max(0,col-radius),min(cells.shape[1],col+radius+1)):
                center = (origin[0]+(c+.5)*resolution,origin[1]+(r+.5)*resolution)
                if cells[r,c] >= 20 and math.dist(position,center) <= .45:
                    cells[r,c] = 0
    safe = safe_cells(cells, resolution)
    height, width = cells.shape

    def index(point):
        return (math.floor((point[1]-origin[1])/resolution),
                math.floor((point[0]-origin[0])/resolution))

    def xy(cell):
        row, col = cell
        return (origin[0]+(col+.5)*resolution, origin[1]+(row+.5)*resolution)

    def nearby(point):
        row, col = index(point)
        # The live SLAM grid contains robot returns at the current footprint.
        # Snap the grid search seed outside that footprint; Nav2 still validates
        # the actual route from the real pose, including the initial segment.
        radius = math.ceil(1.0/resolution)
        choices = [(r, c) for r in range(max(0, row-radius), min(height, row+radius+1))
                   for c in range(max(0, col-radius), min(width, col+radius+1)) if safe[r, c]]
        return min(choices, key=lambda rc: math.dist(point, xy(rc))) if choices else None

    starts = {r: nearby(positions[r]) for r in ROBOTS}
    home_cells = {r: nearby(homes[r]) for r in ROBOTS}
    if starts['robot1'] is None and 'robot1' not in frozen_robots and 'robot1' not in fixed_tasks:
        return {'status': 'WAITING_FOR_SAFE_START_OR_HOME'}
    paths = {}
    returns = {}
    for r in ROBOTS:
        for grids, seeds, source in ((paths, starts, positions), (returns, home_cells, homes)):
            if seeds[r] is None:
                grids[r] = np.full(safe.shape, np.inf)
            else:
                grids[r] = distances(safe, seeds[r], resolution)
                grids[r] += math.dist(source[r], xy(seeds[r]))
        # Holding an existing position is allowed even if its footprint appears
        # occupied in robot1's scan. It is only available to that same robot.
        row, col = index(positions[r])
        if 0 <= row < height and 0 <= col < width:
            paths[r][row, col] = 0.0
            if math.dist(positions[r], homes[r]) < .25:
                returns[r][row, col] = 0.0

    # A frontier is observed free space adjacent to unknown, with the target
    # itself set back into the inflated safe area.
    unknown = cells < 0
    adjacent = np.zeros(cells.shape, dtype=bool)
    adjacent[1:] |= unknown[:-1]
    adjacent[:-1] |= unknown[1:]
    adjacent[:, 1:] |= unknown[:, :-1]
    adjacent[:, :-1] |= unknown[:, 1:]
    frontier = np.argwhere(adjacent & (cells == 0))[::4]
    if not len(frontier):
        return {'status': 'NO_FRONTIER_IN_CURRENT_MAP'}
    step = max(1, round(.6/resolution))
    candidates = {cell for cell in starts.values() if cell is not None}
    candidates.update(index(positions[r]) for r in ROBOTS)
    for robot, task in fixed_tasks.items():
        cell = index(task['goal'])
        if not (0<=cell[0]<height and 0<=cell[1]<width):
            return {'status':'FIXED_TARGET_OUTSIDE_MAP'}
        candidates.add(cell)
        # These legs are already executing under runtime shadow/radio gates.
        # They are constraints on this solve, never new goal suggestions.
        paths[robot][cell] = task['grid_distance_m']
        returns[robot][cell] = task['return_distance_m']
    options = []
    for row in range(0, height, step):
        for col in range(0, width, step):
            if not safe[row, col]:
                continue
            point = xy((row, col))
            if any(math.dist(point, q) < .75 for q in rejected):
                continue
            if not any(.8 <= paths[r][row, col] < np.inf for r in ROBOTS):
                continue
            frontier_distance = float(np.sqrt(np.min(np.sum((frontier-(row, col))**2, axis=1))) * resolution)
            if frontier_distance > 1.2:
                continue
            score = visible_unknown(cells, row, col, resolution)
            if score < .1:
                continue
            score -= 20*sum(max(0, 1-math.dist(point, q)/1.5) for q in visited)
            options.append((score, (row, col)))
    if not options:
        return {'status': 'NO_REACHABLE_FRONTIER_IN_CURRENT_MAP'}
    centroid = tuple(sum(p[k] for p in positions.values())/len(positions) for k in (0,1))
    guide = xy(max(options,key=lambda item:item[0]-.08*math.dist(xy(item[1]),centroid))[1])
    proposals = {}
    for robot in ROBOTS:
        if robot in fixed_tasks or robot in frozen_robots:
            continue
        if starts[robot] is None:
            continue
        reachable = [(score-.08*paths[robot][cell], cell) for score, cell in options
                     if .8 <= paths[robot][cell] < np.inf]
        reachable.sort(reverse=True)
        # Keep pursuing a useful global frontier across short Nav2 legs.
        # Rechoosing it at every leg otherwise causes east/west oscillation.
        preferred = preferred_goals.get(robot)
        if preferred is not None:
            cell = index(preferred)
            if (0<=cell[0]<height and 0<=cell[1]<width and
                    .8<=paths[robot][cell]<np.inf and
                    visible_unknown(cells,*cell,resolution)>.1):
                reachable.insert(0,(visible_unknown(cells,*cell,resolution)-.08*paths[robot][cell],cell))
        accepted = []
        for score, target in reachable:
            global_goal = xy(target)
            if any(math.dist(global_goal, other) < 1.0 for other in accepted):
                continue
            route = trace_route(paths[robot], target, starts[robot], origin, resolution, positions[robot], max_leg=max_leg)
            if len(route) < 2 or math.dist(route[-1], positions[robot]) < .5:
                continue
            if any(math.dist(route[-1],p)<.6 for p in rejected):
                continue
            if forecast is not None and forecast.departure_delay(route,now,hold=staging_hold,maximum_wait=max_wait_s) is None:
                continue
            local = index(route[-1])
            accepted.append(global_goal)
            candidates.add(local)
            old = proposals.get((robot,local))
            preferred = preferred_goals.get(robot)
            old_preferred = old is not None and preferred is not None and math.dist(old['global_goal'],preferred)<.3
            new_preferred = preferred is not None and math.dist(global_goal,preferred)<.3
            # Several far frontiers may share this same first short leg.
            # Do not overwrite its persistent target with a later proposal.
            if (old is None or (new_preferred and not old_preferred) or
                    (new_preferred==old_preferred and score>old['gain'])):
                proposals[robot, local] = {'global_goal': global_goal, 'route': route, 'gain': score}
            if len(accepted) >= 8:
                break
    # Relay candidates span all currently observed safe space within the radio
    # chain's reach. Their actual travel is still limited to a short leg.
    for robot in ROBOTS:
        if robot in fixed_tasks or robot in frozen_robots:
            continue
        relay_options = []
        relay_cells = {(row,col) for row in range(0,height,step) for col in range(0,width,step)}
        # Include pose-aligned exits: the global sampling grid can otherwise
        # force a sideways detour into shadow before a clear north/south exit.
        for angle in np.linspace(0,2*math.pi,8,endpoint=False):
            for distance in np.arange(.5,4.51,.5):
                cell = index((positions[robot][0]+distance*math.cos(angle),
                              positions[robot][1]+distance*math.sin(angle)))
                if 0<=cell[0]<height and 0<=cell[1]<width:
                    relay_cells.add(cell)
        for row,col in sorted(relay_cells):
                if not safe[row,col] or not .25 <= paths[robot][row,col] <= max_leg:
                    continue
                if any(math.dist(xy((row,col)), p)<.75 for p in rejected):
                    continue
                route = trace_route(paths[robot],(row,col),starts[robot],origin,resolution,
                                    positions[robot],max_leg=max_leg+.25)
                if forecast is not None and forecast.departure_delay(route,now,hold=staging_hold,maximum_wait=max_wait_s) is None:
                    continue
                score = math.dist(positions[robot],guide)-math.dist(xy((row,col)),guide)
                relay_options.append((score-.1*paths[robot][row,col],(row,col)))
        ranked = sorted(relay_options,reverse=True)
        # A relay may need to move away from the exploration target to keep
        # another robot connected or escape an approaching shadow. Preserve
        # alternatives around the robot instead of only forward candidates.
        diverse = {}
        for score,cell in ranked:
            point = xy(cell)
            angle = math.atan2(point[1]-positions[robot][1],point[0]-positions[robot][0])
            sector = int((angle+math.pi)/(math.pi/4)) % 8
            band = int(paths[robot][cell] >= 2.)
            diverse.setdefault((sector,band),cell)
        candidates.update(cell for _,cell in ranked[:4])
        candidates.update(diverse.values())
    candidates = sorted(candidates)
    points = [xy(cell) for cell in candidates]
    model = cp_model.CpModel()
    departure_base = math.ceil(now+1)
    last_departure_slot = max(0,min(48,math.floor((now+max_wait_s-departure_base)/5)))
    departure_slot = model.new_int_var(0,last_departure_slot,'team_departure_slot')
    finish_slot = model.new_int_var(0,120,'team_finish_slot')
    choose = {}
    delays = {}
    windows_cache = {}
    costs = []
    for robot in ROBOTS:
        for role in ROLES:
            for j, cell in enumerate(candidates):
                fixed = robot in fixed_tasks
                if fixed and (role!=fixed_tasks[robot]['role'] or
                              cell!=index(fixed_tasks[robot]['goal'])):
                    continue
                frozen = robot in frozen_robots
                if frozen and (role!='relay' or cell!=index(positions[robot])):
                    continue
                length = paths[robot][cell]
                back = returns[robot][cell]
                if not math.isfinite(length+back) or (not fixed and length > max_leg+.25):
                    continue
                # A failed goal must stay excluded for both roles. Previously
                # only frontier proposals respected rejection, so the same
                # blocked goal could immediately return labelled as a relay.
                if not fixed and length>.25 and any(math.dist(points[j], p)<.75 for p in rejected):
                    continue
                if not fixed and length>.25 and any(other!=robot and math.dist(points[j],positions[other])<.75 for other in ROBOTS):
                    continue
                if travelled[robot]+length+back+1.0 > budget:
                    continue
                if not fixed and role == 'explorer' and (robot, cell) not in proposals:
                    continue
                if forecast is not None and not frozen and not fixed:
                    route = trace_route(paths[robot], cell, starts[robot], origin, resolution,
                                        positions[robot], max_leg=max_leg+.25) if length > .25 else [positions[robot]]
                    if (robot,j) not in windows_cache:
                        windows = forecast.departure_constraints(route,now,departure_base,hold=staging_hold,maximum_wait=max_wait_s) if route else []
                        windows_cache[robot,j] = [w for w in windows if w[0]<=last_departure_slot]
                    windows = windows_cache[robot,j]
                    if not windows:
                        continue
                    delays[robot,j] = departure_base+min(w[0] for w in windows)*5-now
                var = model.new_bool_var(f'{robot}_{role}_{j}')
                choose[robot, role, j] = var
                if forecast is not None and not frozen and not fixed:
                    earliest = model.new_int_var(0,120,f'earliest_{robot}_{role}_{j}')
                    latest = model.new_int_var(0,120,f'latest_{robot}_{role}_{j}')
                    model.add_allowed_assignments([departure_slot,earliest,latest],windows_cache[robot,j]).only_enforce_if(var)
                    model.add(finish_slot>=earliest).only_enforce_if(var)
                    model.add(finish_slot<=latest).only_enforce_if(var)
                gain = proposals.get((robot, cell), {}).get('gain', 0) if role == 'explorer' else 0
                progress = math.dist(positions[robot], guide)-math.dist(points[j], guide)
                preferred = preferred_goals.get(robot)
                global_goal = proposals.get((robot,cell),{}).get('global_goal')
                continuity = (40000 if preferred is not None and global_goal is not None
                              and math.dist(preferred,global_goal)<.3 else 0)
                revisit = max((max(0.,1.-math.dist(points[j],p)/1.0)
                               for p in recent_targets.get(robot,[])),default=0.)
                costs.append(round(300*gain+80*progress-30*length-20*delays.get((robot,j),0)
                                   +movement_reward*int(length>=.3)+continuity
                                   -4000*revisit)*var)
    for robot in ROBOTS:
        model.add(sum(v for (r, _, _), v in choose.items() if r == robot) == 1)
    # Repositioning is useful only if somebody changes position. Returning an
    # all-hold OPTIMAL plan otherwise causes endless announce/validate rounds
    # while the interface misleadingly reports MOVING.
    model.add(sum(v for (r, _, j), v in choose.items()
                  if r not in fixed_tasks and math.dist(positions[r], points[j]) >= .3) >= 1)
    # Prefer a team that can move together. Pure endpoint connectivity rewards
    # long fragile chains: a rear robot then cannot follow the leader before
    # rotating shadows close the gap. This is a soft objective, not a smaller
    # radio range or an extra fixed-base constraint.
    coordinates = {axis:[] for axis in (0,1)}
    for robot in ROBOTS:
        for axis in (0,1):
            values = [round(p[axis]*10) for p in points]
            coordinate = model.new_int_var(min(values),max(values),f'{robot}_coordinate_{axis}')
            model.add(coordinate==sum(values[j]*v for (r,_,j),v in choose.items() if r==robot))
            coordinates[axis].append(coordinate)
    spans = []
    for axis,values in coordinates.items():
        low = model.new_int_var(min(round(p[axis]*10) for p in points),max(round(p[axis]*10) for p in points),f'team_low_{axis}')
        high = model.new_int_var(min(round(p[axis]*10) for p in points),max(round(p[axis]*10) for p in points),f'team_high_{axis}')
        model.add_min_equality(low,values)
        model.add_max_equality(high,values)
        spans.append(high-low)
    crowding = []
    if spacing_weight:
        target = round(preferred_spacing*10)
        for a in range(len(ROBOTS)):
            for b in range(a+1,len(ROBOTS)):
                separation = []
                for axis in (0,1):
                    span = max(round(p[axis]*10) for p in points)-min(round(p[axis]*10) for p in points)
                    difference = model.new_int_var(0,span,f'separation_{a}_{b}_{axis}')
                    model.add_abs_equality(difference,coordinates[axis][a]-coordinates[axis][b])
                    separation.append(difference)
                # Manhattan-distance surrogate rewards spreading without
                # forcing separation through hazards or outside radio range.
                shortfall = model.new_int_var(0,target,f'crowding_{a}_{b}')
                model.add_max_equality(shortfall,[0,target-sum(separation)])
                crowding.append(shortfall)
    explorers = sum(v for (_, role, _), v in choose.items() if role == 'explorer')
    model.add(explorers >= (0 if allow_reposition else 1))
    model.add(explorers <= maximum_explorers)
    selected = {(r, j): sum(v for (robot, _, k), v in choose.items() if robot == r and k == j)
                for r in ROBOTS for j in range(len(points))}
    edges = {}
    flows = {}
    for a, robot_a in enumerate(ROBOTS):
        for robot_b in ROBOTS[a+1:]:
            edge = model.new_bool_var(f'link_{robot_a}_{robot_b}')
            edges[robot_a,robot_b] = edge
            for r,s in ((robot_a,robot_b),(robot_b,robot_a)):
                flows[r,s] = model.new_int_var(0,3,f'flow_{r}_{s}')
                model.add(flows[r,s] <= 3*edge)
            for j, p in enumerate(points):
                if not any((robot_a,role,j) in choose for role in ROLES):
                    continue
                for k, q in enumerate(points):
                    if not any((robot_b,role,k) in choose for role in ROLES):
                        continue
                    distance = math.dist(p, q)
                    if distance < .85:
                        model.add(selected[robot_a,j]+selected[robot_b,k] <= 1)
                    # Stronger than endpoint-only connectivity: an active link
                    # must also tolerate either robot moving before the other.
                    if distance > radio_range-.3 or math.dist(positions[robot_a],positions[robot_b])>radio_range:
                        model.add(selected[robot_a,j]+selected[robot_b,k]+edge <= 2)
    for robot in ROBOTS:
        outgoing = sum(v for (r,_),v in flows.items() if r == robot)
        incoming = sum(v for (_,r),v in flows.items() if r == robot)
        model.add(outgoing-incoming == (3 if robot == 'robot1' else -1))
    model.maximize(sum(costs)-10*departure_slot-finish_slot
                   -compactness_weight*sum(spans)-spacing_weight*sum(crowding))
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = solver_seconds
    solver.parameters.num_search_workers = 2
    status = solver.solve(model)
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return {'status': solver.status_name(status),
                'candidate_counts': {r: {role: sum(1 for robot, task, _ in choose if robot == r and task == role)
                                        for role in ROLES} for r in ROBOTS}}
    tasks = {}
    chosen_departure = departure_base+solver.value(departure_slot)*5
    for (robot, role, j), var in choose.items():
        if solver.value(var):
            tasks[robot] = {'role': role, 'goal': points[j],
                            'wait_s':max(0,chosen_departure-now),
                            'grid_distance_m': float(paths[robot][candidates[j]]),
                            'return_distance_m': float(returns[robot][candidates[j]])}
            if robot in frozen_robots:
                tasks[robot]['goal'] = tuple(positions[robot])
                tasks[robot]['waiting_for_sun'] = True
            if robot in fixed_tasks:
                tasks[robot] = dict(fixed_tasks[robot])
            elif role == 'explorer':
                tasks[robot]['global_goal'] = proposals[robot, candidates[j]]['global_goal']
    return {'status': solver.status_name(status), 'tasks': tasks,
            'departure_at': chosen_departure,
            'safe_until': departure_base+solver.value(finish_slot)*5}
