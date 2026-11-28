"""Team allocation BT and per-robot execution BT, with injectable transports."""
import math
import uuid

from .exploration_bt import Leaf, Status, fallback, sequence
from .exploration_policy import Goal, allocate, exploration_claim, in_exploration_claim
from .exploration_sweep import RadarSweep


class TeamBT:
    TERMINAL = ('IDLE', 'VISITED', 'RETURNED', 'FAILED', 'CANCELED', 'REJECTED')

    def __init__(self, robots, send, *, autostart=False, timeout=4., separation=2.,
                 complete_hold=5., goal_timeout=120., completion_mode='frontier',
                 return_home=True, homes=None, require_mapping=False,
                 return_timeout=600., return_retries=2, home_tolerance=.3, radio=None,
                 min_goal_distance=.5, claim_radius=0., require_radio_measurement=False,
                 strategy='frontier', sweep_base=(0., 0.), sweep_bounds=(-29., -29., 29., 29.),
                 sweep_radius=28., sweep_slot_slack=0., allocator=None):
        self.robots, self.send = tuple(robots), send
        self.allocator = allocator or allocate
        if len(set(self.robots)) != len(self.robots) or not self.robots:
            raise ValueError('Robot names must be nonempty and unique')
        if completion_mode not in ('frontier', 'bounds') or complete_hold <= 0:
            raise ValueError('Invalid completion policy')
        self.session = uuid.uuid4().hex
        self.enabled, self.timeout, self.separation = autostart, timeout, separation
        self.complete_hold, self.goal_timeout = complete_hold, goal_timeout
        self.completion_mode, self.return_home = completion_mode, return_home
        self.require_mapping = require_mapping
        self.radio = radio
        if strategy not in ('frontier', 'radar'):
            raise ValueError('Unknown exploration strategy')
        self.sweep = RadarSweep(self.robots, sweep_base, sweep_bounds,
                               radius=sweep_radius, slot_slack=sweep_slot_slack) if strategy == 'radar' else None
        self.require_radio_measurement = require_radio_measurement
        self.radio_measurement, self.radio_received = {}, -math.inf
        if not math.isfinite(min_goal_distance) or min_goal_distance < .3:
            raise ValueError('Minimum exploration goal distance must be at least 0.3 m')
        self.min_goal_distance = min_goal_distance
        if not math.isfinite(claim_radius) or claim_radius < 0:
            raise ValueError('Invalid exploration claim radius')
        self.claim_radius, self.recent_claims = claim_radius, []
        self.return_timeout, self.return_retries, self.home_tolerance = return_timeout, return_retries, home_tolerance
        self.homes = {n: list(p) for n, p in (homes or {}).items() if n in self.robots}
        if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in self.homes.values()):
            raise ValueError('Home poses must be [x, y, yaw]')
        self.states, self.active, self.history, self.excluded = {}, {}, [], []
        self.grid, self.map_time, self.revision = None, -math.inf, 0
        self.now, self.mission_now, self.counter, self.complete_since = 0., 0., 0, None
        self.phase, self.finished, self.reason = 'WAITING', False, ''
        self.exploration_done, self.exploration_result = False, None
        self.remaining = 'AVAILABLE'
        self.mapping_state, self.mapping_time = {}, -math.inf
        self.returned, self.return_failed, self.return_attempts, self.retry_after = set(), set(), {}, {}
        self.tree = fallback('TeamBT',
            sequence('Stopped', Leaf('Disabled', lambda: self.condition(not self.enabled)),
                     Leaf('CancelAndPause', lambda: self.pause('STOPPED'))),
            sequence('InputGuard', Leaf('MissingInputs', lambda: self.condition(not self.ready())),
                     Leaf('CancelAndWait', lambda: self.pause('WAITING'))),
            sequence('CommunicationGuard', Leaf('BaseDisconnected', lambda: self.condition(
                not self.communication_available())),
                Leaf('HoldForBaseConnection', lambda: self.pause('COMMUNICATION_LOST'))),
            sequence('Completed', Leaf('MissionFinished', lambda: self.condition(self.finished)),
                     Leaf('RemainStopped', self.finish)),
            sequence('ReturnMission', Leaf('ExplorationEnded', lambda: self.condition(self.exploration_done)),
                     Leaf('FreezeMapAndReturnHome', self.return_to_base)),
            sequence('CompletionCheck', Leaf('NoExplorationGoals', lambda: self.condition(self.can_complete())),
                     Leaf('ConfirmStableCompletion', self.confirm_completion)),
            Leaf('RadarSweepAndMonitor' if self.sweep else 'AllocateAndMonitor', self.plan))

    @staticmethod
    def condition(value):
        return Status.SUCCESS if value else Status.FAILURE

    def positions(self):
        return {n: self.states[n][0]['position'] for n in self.robots}

    def communication_available(self):
        if self.radio is None:
            return True
        if self.require_radio_measurement:
            return (self.now-self.radio_received <= 1. and self.radio_measurement.get('ready', False)
                    and all(self.radio_measurement.get('connected', {}).get(n, False) for n in self.robots))
        return self.radio.graph(self.positions())['all_connected']

    @property
    def mapping_mode(self):
        return 'LOCALIZATION' if self.exploration_done and self.return_home else 'MAPPING'

    def update_mapping(self, state, now):
        if state.get('session') == self.session:
            self.mapping_state, self.mapping_time = dict(state), now

    def mapping_ready(self, mode):
        return (self.now-self.mapping_time <= self.timeout
                and self.mapping_state.get('mode') == mode
                and self.mapping_state.get('ready') is True
                and (mode != 'LOCALIZATION' or self.mapping_state.get('frozen') is True))

    def update_map(self, grid, now):
        # During return only the frozen producer's heartbeat is needed. Never
        # replace the allocator's final snapshot with a later incoming grid.
        if not self.exploration_done:
            self.grid = grid
            self.revision += 1
        self.map_time = now

    def update_robot(self, name, state, now):
        if name not in self.robots:
            return
        self.states[name] = (dict(state), now)
        task = self.active.get(name)
        if not task or state.get('task_id') != task['task_id'] or state.get('session') != self.session:
            return
        if state.get('status') in self.TERMINAL[1:]:
            self.active.pop(name)
            result = state['status']
            if task.get('purpose') == 'return':
                p = state.get('position')
                # A Nav2 success alone must not mark a robot at the wrong pose home.
                if result == 'RETURNED' and p and math.dist(p, self.homes[name][:2]) <= self.home_tolerance:
                    self.returned.add(name)
                else:
                    result = 'FAILED' if result == 'RETURNED' else result
                    if result == 'CANCELED' and not self.enabled:
                        self.return_attempts[name] -= 1
                    self.retry_after[name] = now + 5.
                    if self.return_attempts.get(name, 0) >= 1 + self.return_retries:
                        self.return_failed.add(name)
            elif not (task.get('formation') and result == 'VISITED'):
                self.excluded.append((tuple(task['goal'][:2]), .75, now + (20 if result == 'VISITED' else 10)))
                if result == 'VISITED' and task.get('claim'):
                    self.recent_claims.append((name, task['claim'], now+15.))
            self.history.append({**task, 'robot': name, 'result': result, 'finished_at': now})
            self.history = self.history[-200:]

    def ready(self):
        missing = []
        if self.grid is None or self.now-self.map_time > self.timeout:
            missing.append('shared map')
        for name in self.robots:
            state, received = self.states.get(name, ({}, -math.inf))
            p = state.get('position')
            if (self.now-received > self.timeout or not state.get('ready') or not p
                    or len(p) != 2 or not all(math.isfinite(v) for v in p)):
                missing.append(name)
            elif name not in self.homes:
                yaw = state.get('yaw', 0.)
                if math.isfinite(yaw):
                    self.homes[name] = [*p, yaw]
        # The return branch handles the localization handshake while stopped.
        if self.require_mapping and not self.exploration_done and not self.mapping_ready('MAPPING'):
            missing.append('fresh maps and mapping acknowledgement from all robots')
        self.reason = 'Waiting for: ' + ', '.join(missing) if missing else ''
        return not missing

    def command(self, name, task=None):
        self.send(name, {'session': self.session, 'mode': 'goal' if task else 'stop',
                         **({k: v for k, v in task.items() if k != 'created_at'} if task else {})})

    def pause(self, phase):
        self.complete_since = None
        if self.sweep:
            self.sweep.unavailable_since = None
        self.phase = phase
        for name in self.robots:
            if name in self.active:
                self.active[name]['cancel_requested'] = True
            self.command(name)
        return Status.RUNNING

    def can_complete(self):
        if self.active or any(self.states[r][0].get('status') not in self.TERMINAL for r in self.robots):
            return False
        if self.completion_mode == 'bounds':
            self.remaining = 'EXHAUSTED' if self.grid.bounds_fully_known() else 'AVAILABLE'
        else:
            self.remaining = self.grid.exploration_remaining(
                {r: self.states[r][0]['position'] for r in self.robots})
        return self.remaining != 'AVAILABLE'

    def confirm_completion(self):
        if self.complete_since is None:
            self.complete_since = self.mission_now
        self.phase = 'VERIFYING_COMPLETE' if self.remaining == 'EXHAUSTED' else 'VERIFYING_BLOCKED'
        for name in self.robots:
            self.command(name)
        if self.mission_now-self.complete_since >= self.complete_hold:
            self.exploration_done = True
            self.exploration_result = 'COMPLETE' if self.remaining == 'EXHAUSTED' else 'PARTIAL'
            if self.return_home:
                return self.return_to_base()
            self.finished = True
            return self.finish()
        return Status.RUNNING

    def new_task(self, name, goal, purpose):
        self.counter += 1
        self.active[name] = {'task_id': f'{self.session}:{self.counter}',
                             'goal': list(goal), 'purpose': purpose, 'created_at': self.now,
                             'map_revision': self.revision}
        if purpose == 'explore' and self.claim_radius:
            self.active[name]['claim'] = exploration_claim(self.positions()[name], goal, self.claim_radius)

    def return_to_base(self):
        if not self.mapping_ready('LOCALIZATION'):
            self.reason = 'Waiting for frozen shared map and localization-only acknowledgement from all robots'
            return self.pause('FREEZING_MAP')
        if any(name not in self.homes for name in self.robots):
            self.reason = 'Missing starting poses; no return goal will be guessed'
            return self.pause('RETURN_FAILED')
        if not self.active:
            pending = [n for n in self.robots if n not in self.returned and n not in self.return_failed]
            if not pending:
                if self.return_failed:
                    self.reason = 'Return failed: ' + ', '.join(sorted(self.return_failed))
                    return self.pause('RETURN_FAILED')
                self.finished = True
                return self.finish()
            # Serial return prevents four arrivals from blocking the shared base.
            name = pending[0]
            if self.radio:
                positions = self.positions()
                graph = self.radio.graph(positions)
                leaves = [n for n in pending if self.radio.graph(
                    {peer: p for peer, p in positions.items() if peer != n})['all_connected']]
                if not leaves:
                    self.reason = 'A remote robot still depends on a relay; keep the base-rooted network'
                    return self.pause('WAITING_FOR_RELAY_RETURN')
                name = max(leaves, key=lambda n: (graph['hops'].get(n, 0), math.dist(positions[n], self.radio.base)))
            if self.now >= self.retry_after.get(name, -math.inf):
                self.return_attempts[name] = self.return_attempts.get(name, 0) + 1
                self.new_task(name, self.homes[name], 'return')
        for name in self.robots:
            task = self.active.get(name)
            if task and (task.get('cancel_requested') or self.now-task['created_at'] > self.return_timeout):
                task['cancel_requested'] = True
                self.command(name)
            else:
                self.command(name, task)
        self.phase = 'RETURNING'
        return Status.RUNNING

    def finish(self):
        self.phase = 'COMPLETE' if self.exploration_result == 'COMPLETE' else 'PARTIAL_COMPLETE'
        for name in self.robots:
            self.command(name)
        return Status.SUCCESS

    def plan(self):
        self.complete_since = None
        self.excluded = [entry for entry in self.excluded if entry[2] > self.now]
        self.recent_claims = [entry for entry in self.recent_claims if entry[2] > self.now]
        idle = {name: self.states[name][0]['position'] for name in self.robots
                if name not in self.active and self.states[name][0].get('status') in self.TERMINAL}
        positions = self.positions()
        projected = {**positions, **{n: t['goal'][:2] for n, t in self.active.items()}}
        if self.sweep:
            self.sweep.update(positions, idle, self.mission_now, self.grid)
            if self.sweep.stage != 'CLEANUP':
                chosen = self.sweep.choose(self.grid, positions, idle, self.active, self.radio,
                                          [(p, radius) for p, radius, _ in self.excluded])
                advanced = self.sweep.skip_unavailable_sector(idle, self.active, chosen, self.mission_now)
                if advanced and self.sweep.stage != 'CLEANUP':
                    chosen = self.sweep.choose(self.grid, positions, idle, self.active, self.radio,
                                              [(p, radius) for p, radius, _ in self.excluded])
            if self.sweep.stage != 'CLEANUP':
                for name, goal in chosen.items():
                    self.new_task(name, [goal.x, goal.y, goal.yaw], 'explore')
                    self.active[name].pop('claim', None)
                    self.active[name]['formation'] = True
                self.dispatch_tasks()
                self.phase = 'EXPLORING' if self.active else 'FORMING_RADAR_LINE'
                self.reason = f'Radar {self.sweep.stage}: {math.degrees(self.sweep.angle):.0f} deg'
                return Status.RUNNING

        def radio_goal(name, goal, distances):
            route = self.grid.path(positions[name], goal.xy, distances)
            point = self.radio.advance(positions, name, route)
            if math.dist(point, positions[name]) < .5:
                return None
            return Goal(*point, goal.yaw, goal.gain)

        def radio_joint(name, goal, chosen):
            joint = {**projected, **{n: g.xy for n, g in chosen.items()}}
            return self.radio is None or self.radio.allows(joint, name, goal.xy)

        def available_workspace(name, goal, chosen):
            claims = [(peer, task['claim']) for peer, task in self.active.items()
                      if task.get('claim') and not task.get('cancel_requested')]
            claims += [(peer, claim) for peer, claim, _ in self.recent_claims]
            if self.claim_radius:
                claims += [(peer, exploration_claim(positions[peer], g.xy, self.claim_radius))
                           for peer, g in chosen.items()]
            return (not any(peer != name and in_exploration_claim(goal.xy, claim)
                            for peer, claim in claims) and radio_joint(name, goal, chosen))

        constraints = dict(
                          reserved=[tuple(t['goal'][:2]) for t in self.active.values()],
                          excluded=[(p, radius) for p, radius, _ in self.excluded],
                          separation=self.separation,
                          transform_goal=radio_goal if self.radio else None,
                          validate_goal=available_workspace)
        chosen = self.allocator(self.grid, idle, min_distance=self.min_goal_distance, **constraints)
        if not chosen and self.min_goal_distance > .5:
            # Prefer useful longer legs, but do not strand a small remaining
            # frontier or a relay whose radio-safe advance is shorter.
            chosen = self.allocator(self.grid, idle, min_distance=.5, **constraints)
        for name, goal in chosen.items():
            self.new_task(name, [goal.x, goal.y, goal.yaw], 'explore')
        self.dispatch_tasks()
        self.phase = 'EXPLORING' if self.active else (
            'WAITING_FOR_CONNECTIVITY' if self.radio else 'WAITING_FOR_RETRY')
        return Status.RUNNING

    def dispatch_tasks(self):
        for name in self.robots:
            task = self.active.get(name)
            if task and (task.get('cancel_requested') or self.now-task['created_at'] > self.goal_timeout):
                task['cancel_requested'] = True
                self.command(name)
            else:
                self.command(name, task)

    def tick(self, now, mission_time=None):
        self.now, self.mission_now = now, now if mission_time is None else mission_time
        if self.complete_since is not None and self.mission_now < self.complete_since:
            self.complete_since = None
        return self.tree.tick()

    def snapshot(self):
        return {'phase': self.phase, 'reason': self.reason, 'session': self.session,
                'map_revision': self.revision, 'frontier_cells': self.grid.frontier_cells if self.grid else 0,
                'significant_frontier_cells': self.grid.significant_frontier_cells if self.grid else 0,
                'mapping_mode': self.mapping_mode, 'exploration_result': self.exploration_result,
                'completion_hold': self.complete_hold, 'homes': self.homes,
                'claim_radius': self.claim_radius,
                'sweep': self.sweep.snapshot() if self.sweep else None,
                'claims': {n: t['claim'] for n, t in self.active.items() if t.get('claim')},
                'radio': self.radio.graph(self.positions()) if self.radio and len(self.states) == len(self.robots)
                and all(s[0].get('position') for s in self.states.values()) else None,
                'returned': sorted(self.returned), 'return_failed': sorted(self.return_failed),
                'active': self.active, 'history': self.history, 'tree': self.tree.snapshot()}


class RobotBT:
    """Transport interface: start(goal), poll()->str, cancel(), ready()->bool.

    poll states: IDLE, SENDING, NAVIGATING, SUCCEEDED, FAILED, CANCELED,
    CANCELING. cancel must also handle an outstanding goal-send request.
    """
    def __init__(self, navigation, *, lease=3., observe_seconds=2., radio_enabled=False):
        self.navigation, self.lease, self.observe_seconds = navigation, lease, observe_seconds
        self.task, self.last_command, self.now = None, -math.inf, 0.
        self.mode, self.status, self.observing_since = 'stop', 'IDLE', None
        self.retired, self.highest_sequence, self.session = set(), 0, None
        self.radio_enabled, self.link_ok, self.link_received = radio_enabled, False, -math.inf
        self.tree = fallback('RobotBT',
            sequence('CommunicationGuard', Leaf('BaseLinkUnavailable', lambda: TeamBT.condition(
                self.radio_enabled and (not self.link_ok or self.now-self.link_received > 2.))),
                Leaf('StopUntilConnected', self.stop)),
            sequence('SafetyGuard', Leaf('UnavailableOrStopped', lambda: TeamBT.condition(
                self.mode == 'stop' or self.now-self.last_command > self.lease or not self.navigation.ready())),
                     Leaf('CancelAndWait', self.stop)),
            sequence('ReturnHome', Leaf('ReturnGoalAssigned', lambda: TeamBT.condition(
                bool(self.task) and self.task.get('purpose') == 'return')),
                Leaf('NavigateToStartPose', self.execute, self.navigation.cancel)),
            Leaf('ExecuteAssignedFrontier', self.execute, self.navigation.cancel))

    def receive(self, command, now):
        session = command.get('session')
        if not isinstance(session, str) or not session or session in self.retired:
            return False
        mode = command.get('mode')
        if mode not in ('stop', 'goal'):
            return False
        if mode == 'goal':
            goal, identifier = command.get('goal'), command.get('task_id')
            if (not isinstance(identifier, str) or not identifier.startswith(session + ':')
                    or not isinstance(goal, list) or len(goal) != 3
                    or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in goal)):
                return False
            try:
                seq = int(identifier.rsplit(':', 1)[1])
            except ValueError:
                return False
            if seq <= 0 or (session == self.session and seq < self.highest_sequence):
                return False
        if session != self.session:
            if self.session:
                self.retired.add(self.session)
            self.session, self.highest_sequence = session, 0
            self.navigation.cancel()
            self.task, self.status = None, 'IDLE'
        self.last_command, self.mode = now, mode
        if mode == 'goal':
            if self.task and identifier == self.task['task_id']:
                return True  # Heartbeat retransmission; never restart the action.
            if seq <= self.highest_sequence:
                return False
            self.highest_sequence = seq
            self.navigation.cancel()
            self.task = {'task_id': identifier, 'session': session, 'goal': goal,
                         'purpose': command.get('purpose', 'explore')}
            self.status, self.observing_since = 'RECEIVED', None
        return True

    def stop(self):
        self.navigation.cancel()
        if self.navigation.poll() in ('SENDING', 'NAVIGATING', 'CANCELING'):
            self.status = 'CANCELING'
        elif self.task and self.status not in ('VISITED', 'RETURNED', 'FAILED', 'CANCELED', 'REJECTED'):
            self.status = 'CANCELED'
        elif not self.task:
            self.status = 'IDLE'
        return Status.RUNNING

    def execute(self):
        if not self.task or self.status in ('VISITED', 'RETURNED', 'FAILED', 'CANCELED', 'REJECTED'):
            return Status.RUNNING
        if self.status == 'RECEIVED':
            if self.navigation.poll() in ('SENDING', 'NAVIGATING', 'CANCELING'):
                return Status.RUNNING
            self.navigation.start(self.task['goal'])
            self.status = 'NAVIGATING'
        state = self.navigation.poll()
        if state in ('FAILED', 'CANCELED'):
            self.status = state
            return Status.FAILURE
        if state == 'SUCCEEDED':
            if self.task.get('purpose') == 'return':
                self.status = 'RETURNED'
                return Status.SUCCESS
            if self.observing_since is None:
                self.observing_since = self.now
            self.status = 'OBSERVING'
            if self.now-self.observing_since >= self.observe_seconds:
                # VISITED is a local visit, not a certificate that a region is mapped.
                self.status = 'VISITED'
                return Status.SUCCESS
        return Status.RUNNING

    def tick(self, now):
        self.now = now
        return self.tree.tick()

    def snapshot(self):
        return {'status': self.status, 'session': self.task['session'] if self.task else self.session,
                'task_id': self.task['task_id'] if self.task else None,
                'tree': self.tree.snapshot()}
