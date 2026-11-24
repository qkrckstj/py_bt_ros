"""Per-robot CP-SAT goal replenishment without a team arrival barrier.

Active Nav2 goals are fixed constraints of each new allocation. Planning,
radio acknowledgments and path checks never revoke their motion permits.
Only a robot's own shadow, stale input, collision or link gate stops it.
"""
import json
import math
import time
from pathlib import Path

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from std_msgs.msg import Bool, String

from explore import Exploration
from planner import ROBOTS
from radio import movement_permits, route_point, routes_connected
from shadow_prediction import Forecast


def remaining_route(position, route):
    if not route:
        return [position]
    nearest = min(range(len(route)),key=lambda j:math.dist(position,route[j]))
    return [position]+list(route[nearest:])


class RollingExploration(Exploration):
    def __init__(self, config):
        super().__init__(config)
        self.job = None
        self.pending = {}
        self.cooldown = {}
        self.tokens = dict.fromkeys(ROBOTS,0)
        self.progress_points = {}
        self.started_at = {}
        self.reasons = dict.fromkeys(ROBOTS,'initializing')
        self.assignment_serial = 0
        self.cleanup_complete = False
        self.cleanup_futures = None
        self.cleanup_clients = {r:self.create_client(CancelGoal,
            f'/{r}/navigate_to_pose/_action/cancel_goal') for r in ROBOTS}
        self.get_logger().info('ROLLING dispatcher: no shared arrival or departure barrier')

    def clear_orphan_goals(self):
        """Cancel goals owned by a previous executor before changing its BT.

        Nav2 rejects preemption with a different XML tree while an old goal
        is active. Local action handles alone cannot detect that old goal.
        """
        if self.cleanup_complete:
            return True
        self.heartbeat.publish(Bool(data=True))
        if self.cleanup_futures is None:
            if not all(c.service_is_ready() for c in self.cleanup_clients.values()):
                return False
            self.cleanup_futures = {r:c.call_async(CancelGoal.Request())
                                    for r,c in self.cleanup_clients.items()}
            self.cleanup_started = time.monotonic()
            return False
        if not all(f.done() for f in self.cleanup_futures.values()):
            if time.monotonic()-self.cleanup_started>10:
                self.halt('Previous Nav2 goal cancellation timed out')
            return False
        for future in self.cleanup_futures.values():
            response = future.result()
            if response is None or response.return_code==CancelGoal.Response.ERROR_REJECTED:
                self.halt('Previous Nav2 goal cancellation rejected')
                return False
        self.cleanup_complete = True
        self.get_logger().info('STARTUP_NAV2 previous goal cancellations acknowledged')
        return True

    def receive_forecast(self, msg):
        try:
            self.forecast = Forecast(msg.data,navigation_clearance_m=.35)
        except Exception as exc:
            self.halt(f'Invalid navigation shadow forecast: {exc}')

    def isolate(self, robot, reason):
        if robot in self.canceling:
            self.allowed.discard(robot)
            return
        self.allowed.discard(robot)
        self.pending.pop(robot,None)
        task = self.tasks.get(robot)
        if task:
            self.rejected.append({'point':task['goal'],'until':self.now()+90})
        self.cooldown[robot] = self.now()+2
        self.reasons[robot] = reason
        if robot in self.handles and robot not in self.canceling:
            self.canceling.add(robot)
            self.handles[robot].cancel_goal_async()
        self.get_logger().warning(f'ISOLATE {robot}: {reason}; other goals retained')

    def finished_rolling(self, robot, token, future):
        if self.tokens[robot]!=token:
            return
        self.handles.pop(robot,None)
        self.canceling.discard(robot)
        self.allowed.discard(robot)
        status = future.result().status
        self.results.append({'round':self.tasks[robot].get('revision',self.round),
                             'robot':robot,'status':status})
        if status==GoalStatus.STATUS_SUCCEEDED:
            self.reasons[robot] = 'arrived; next goal may dispatch independently'
            task = self.tasks[robot]
            self.rejected.append({'point':task['goal'],'robot':robot,
                                  'kind':'visited','until':self.now()+120})
            if 'global_goal' in task and math.dist(task['goal'],task['global_goal'])<.7:
                self.visited.append(task['global_goal'])
            self.get_logger().info(f'ARRIVED robot={robot} token={token}; peers still active={sorted(self.handles)}')
        elif status!=GoalStatus.STATUS_CANCELED:
            self.isolate(robot,f'Nav2 result status={status}')

    def accepted_rolling(self, robot, token, future):
        handle = future.result()
        if self.tokens[robot]!=token or self.phase=='STOPPED':
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self.sending.discard(robot)
        if not handle.accepted:
            self.isolate(robot,'goal rejected')
            return
        self.handles[robot] = handle
        self.started_at[robot] = self.now()
        self.rejected.append({'point':self.previous[robot],'robot':robot,
                              'kind':'visited','until':self.now()+120})
        self.progress_points[robot] = (self.previous[robot],self.now())
        handle.get_result_async().add_done_callback(
            lambda f,r=robot,t=token:self.finished_rolling(r,t,f))

    def eligible(self, positions):
        result = set()
        for r in ROBOTS:
            if (r in self.pending or r in self.sending or r in self.canceling or
                    self.now()<self.cooldown.get(r,0) or self.shadows.get(r,False)):
                continue
            if not self.forecast.hold_safe(positions[r],self.now(),5):
                if hasattr(self,'reasons'):
                    self.reasons[r] = 'waiting for predicted footprint clearance'
                continue
            if r not in self.handles or math.dist(positions[r],self.tasks[r]['goal'])<1.3:
                result.add(r)
        return result

    def begin_job(self, positions, eligible):
        anchors = dict(positions)
        fixed = {}
        traveled = dict(self.travel)
        for r in ROBOTS:
            if r in eligible:
                if r in self.handles:
                    anchors[r] = tuple(self.tasks[r]['goal'])
                    traveled[r] += math.dist(positions[r],anchors[r])
            elif r in self.handles:
                task = dict(self.tasks[r])
                route = remaining_route(positions[r],self.plans.get(r,[]))
                task['grid_distance_m'] = sum(math.dist(a,b) for a,b in zip(route,route[1:]))
                fixed[r] = task
            else:
                fixed[r] = {'role':'relay','goal':positions[r],'grid_distance_m':0.,
                            'return_distance_m':math.dist(positions[r],self.homes[r])}
        self.job = {'stage':'planning','future':self.plan_from(anchors,traveled,fixed),
                    'eligible':eligible,'anchors':anchors,'fixed':fixed,'stamp':self.now(),
                    'wall_started':time.monotonic()}
        self.get_logger().info(f'ROLLING_PLAN candidates={sorted(eligible)} active_preserved={sorted(self.handles)}')

    def job_tick(self, positions):
        if self.job is None:
            eligible = self.eligible(positions)
            if eligible:
                self.begin_job(positions,eligible)
            return
        job = self.job
        if job['stage']=='planning':
            if not job['future'].done():
                return
            result = job['future'].result()
            self.get_logger().info(f'ROLLING_SOLVED status={result["status"]} wall_s={time.monotonic()-job["wall_started"]:.3f}')
            self.coverage_report = result['coverage']
            if 'tasks' not in result:
                for r in job['eligible']:
                    self.cooldown[r] = self.now()+2
                self.get_logger().info(f'ROLLING_PLAN {result["status"]}; existing navigation continues')
                self.job = None
                if self.coverage_report['all_accessible_observed'] and not self.handles:
                    self.phase = 'COMPLETE'
                    self.heartbeat.publish(Bool(data=True))
                return
            # Discard stale suggestions, or any solve whose fixed goals changed.
            if self.now()-job['stamp']>25 or any(
                    r in self.handles and math.dist(self.tasks[r]['goal'],t['goal'])>.1
                    for r,t in job['fixed'].items()):
                self.job = None
                return
            self.round = max(self.round,*(int(h.get('assigned_round',0))
                                         for h in self.radio_health.values()))+1
            tasks = {r:dict(self.tasks.get(r,result['tasks'][r])) for r in ROBOTS}
            proposed = {}
            for r in job['eligible']:
                task = result['tasks'][r]
                if task.get('waiting_for_sun') or math.dist(task['goal'],job['anchors'][r])<.3:
                    self.cooldown[r] = self.now()+1
                    continue
                proposed[r] = dict(task,revision=self.round)
                tasks[r] = proposed[r]
            if not proposed:
                self.job = None
                return
            job.update(stage='announcing',revision=self.round,tasks=proposed,
                       announcement_at=time.monotonic(),
                       departure_at=result.get('departure_at',self.now()))
            self.assignment_pub.publish(String(data=json.dumps({
                'round':self.round,'tasks':tasks,'departure_at':job['departure_at']})))
        elif job['stage']=='announcing':
            acknowledgments = self.radio_health['robot1'].get('assignment_acks',{})
            if all(acknowledgments.get(r)==job['revision'] for r in ROBOTS[1:]):
                for r,task in job['tasks'].items():
                    pending = {'task':task,'anchor':job['anchors'][r],
                               'departure_at':job['departure_at'],'stamp':self.now(),
                               'route':None,'checking':True}
                    self.pending[r] = pending
                    self.request_path(r,pending)
                self.job = None
            elif time.monotonic()-job['announcement_at']>10:
                self.get_logger().warning('ROLLING assignment ack timed out; active goals retained')
                self.job = None

    def request_path(self, robot, pending):
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = goal.start.header.frame_id = 'map'
        goal.goal.header.stamp = goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x,goal.goal.pose.position.y = map(float,pending['task']['goal'])
        goal.start.pose.position.x,goal.start.pose.position.y = map(float,pending['anchor'])
        goal.goal.pose.orientation.w = goal.start.pose.orientation.w = 1.
        goal.use_start = True
        goal.planner_id = 'GridBased'
        future = self.path_clients[robot].send_goal_async(goal)
        future.add_done_callback(lambda f,r=robot,p=pending:self.path_handle(r,p,f))

    def reject_pending(self, robot, pending, reason):
        if self.pending.get(robot) is not pending:
            return
        self.pending.pop(robot,None)
        self.rejected.append({'point':pending['task']['goal'],'until':self.now()+30})
        self.cooldown[robot] = self.now()+1
        self.reasons[robot] = reason
        self.get_logger().warning(f'ROLLING_PATH {robot}: {reason}; peers retained')

    def path_handle(self, robot, pending, future):
        if self.pending.get(robot) is not pending:
            return
        handle = future.result()
        if not handle.accepted:
            self.reject_pending(robot,pending,'path request rejected')
            return
        handle.get_result_async().add_done_callback(
            lambda f,r=robot,p=pending:self.path_result(r,p,f))

    def path_result(self, robot, pending, future):
        if self.pending.get(robot) is not pending:
            return
        route = [(p.pose.position.x,p.pose.position.y) for p in future.result().result.path.poses]
        if not route:
            self.reject_pending(robot,pending,'Nav2 returned no path')
            return
        pending.update(route=route,checking=False)

    def dispatch_tick(self, positions):
        for r,pending in list(self.pending.items()):
            if self.now()-pending['stamp']>30:
                self.reject_pending(r,pending,'suggestion expired')
                continue
            if pending['checking'] or r in self.handles or r in self.sending or r in self.canceling:
                continue
            if self.now()<pending['departure_at']:
                continue
            if self.shadows.get(r,False) or math.dist(positions[r],pending['anchor'])>.4:
                self.reject_pending(r,pending,'start pose or shadow changed')
                continue
            route = remaining_route(positions[r],pending['route'])
            if not self.forecast.route_safe(route,self.now(),hold=8):
                self.reject_pending(r,pending,'live shadow check rejected route')
                continue
            plans = {s:remaining_route(positions[s],self.plans.get(s,[]))
                     if s in self.handles else [positions[s]] for s in ROBOTS}
            plans[r] = route
            if not routes_connected(plans,positions,self.radio_range):
                # Peers may move into relay range without stopping; retry here.
                self.reasons[r] = 'waiting for a connected path with active peers'
                continue
            self.pending.pop(r)
            self.tasks[r] = pending['task']
            self.plans[r] = route
            self.tokens[r] += 1
            token = self.tokens[r]
            goal = NavigateToPose.Goal()
            goal.behavior_tree = str(Path(__file__).with_name('rolling_navigation.xml'))
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x,goal.pose.pose.position.y = map(float,self.tasks[r]['goal'])
            goal.pose.pose.orientation.w = 1.
            self.sending.add(r)
            self.nav_clients[r].send_goal_async(goal).add_done_callback(
                lambda f,r=r,t=token:self.accepted_rolling(r,t,f))
            self.reasons[r] = 'navigating independently'
            self.get_logger().info(f'ROLLING_GOAL robot={r} token={token} peers_active={sorted(self.handles)} goal={self.tasks[r]["goal"]}')

    def gate_motion(self, positions):
        active = set(self.handles)|self.sending
        routes = {r:remaining_route(positions[r],self.plans.get(r,[]))
                  if r in active else [positions[r]] for r in ROBOTS}
        projections = {}
        for r in ROBOTS:
            command,stamp = self.commands.get(r,(Twist(),-100))
            distance = min(.44,abs(command.linear.x)*2+.05) if self.now()-stamp<.5 and abs(command.linear.x)>.01 else 0.
            projections[r] = route_point(routes[r],distance=distance)
            if command.linear.x<-.01:
                yaw = self.headings[r]
                projections[r] = (positions[r][0]-distance*math.cos(yaw),positions[r][1]-distance*math.sin(yaw))
        self.allowed = movement_permits(positions,projections,self.radio_range)&active
        for r in list(active):
            if self.shadows.get(r,False):
                self.isolate(r,'terrain shadow; waiting for sunlight')
                continue
            upcoming = [positions[r]]
            length = 0.
            for point in routes[r][1:]:
                length += math.dist(upcoming[-1],point)
                if length>1.:
                    break
                upcoming.append(point)
            if not self.forecast.route_safe(upcoming,self.now(),turn_delay=0,hold=2):
                self.isolate(r,'shadow predicted on upcoming path')
                continue
            last,stamp = self.progress_points.get(r,(positions[r],self.now()))
            if math.dist(last,positions[r])>.12:
                self.progress_points[r] = (positions[r],self.now())
                stamp = self.now()
            if r in self.handles and r in self.allowed and self.now()-stamp>12:
                self.isolate(r,'no measured translation for 12s; replacing only this goal')
            elif r in self.handles and self.now()-self.started_at[r]>120:
                self.isolate(r,'individual goal timeout')

    def step(self):
        if self.phase in ('STOPPED','COMPLETE'):
            self.heartbeat.publish(Bool(data=True))
            return
        try:
            positions = self.positions()
        except Exception as exc:
            self.wait_for_data(f'Missing or stale TF: {exc}')
            return
        ready = (self.map is not None and self.now()-self.map_time<10
                 and self.forecast is not None and -.5<=self.now()-self.forecast.stamp<5
                 and all(self.now()-self.odom_times.get(r,-100)<2 and
                         self.now()-self.shadow_times.get(r,-100)<2 and
                         self.now()-self.radio_health.get(r,{}).get('stamp',-100)<5 and
                         self.radio_health.get(r,{}).get('network_ok',False) for r in ROBOTS)
                 and all(c.server_is_ready() for c in (*self.nav_clients.values(),*self.path_clients.values())))
        if not ready:
            self.wait_for_data('Waiting for fresh sensors, radio and Nav2')
            return
        self.unavailable_since = None
        if not self.clear_orphan_goals():
            return
        if not self.resume_checked and not self.resume(positions):
            return
        for r,p in positions.items():
            if r in self.previous:
                self.travel[r] += math.dist(p,self.previous[r])
            self.previous[r] = p
            if self.travel[r]+math.dist(p,self.homes[r])>self.budget-1:
                self.halt(f'{r}: distance reserve reached')
                return
        if not self.connected(positions):
            self.halt('Actual radio graph disconnected')
            return
        self.phase = 'MOVING'
        self.job_tick(positions)
        if self.phase=='COMPLETE':
            return
        self.dispatch_tick(positions)
        self.gate_motion(positions)
        self.heartbeat.publish(Bool(data=False))
        if time.monotonic()-self.last_status>3:
            self.last_status = time.monotonic()
            moving = [r for r in ROBOTS if abs(self.measured_velocity.get(r,(0.,0.))[0])>.01]
            turning = [r for r in ROBOTS if r in self.allowed and abs(self.measured_velocity.get(r,(0.,0.))[1])>.02]
            phase = 'MOVING' if moving else 'TURNING' if turning else 'PLANNING' if self.job else 'WAITING'
            payload = {'phase':phase,'dispatch_mode':'per_robot','round':self.round,
                       'positions':positions,'travel_m':self.travel,'coverage':self.coverage_report,
                       'known_cells':self.known,'initial_known_cells':self.start_known,
                       'moving_robots':moving,'turning_robots':turning,
                       'active_goals':sorted(self.handles),'pending_goals':sorted(self.pending),
                       'motion_permits':sorted(self.allowed),
                       'measured_velocity':self.measured_velocity,'shadow':self.shadows,
                       'reasons':self.reasons,'tasks':self.tasks,'recent_results':self.results[-8:],
                       'mission_complete':False,'sun_period_s':self.forecast.payload.get('sun_period_s'),
                       'sun_phase_deg':self.forecast.payload.get('sun_phase_deg'),
                       'radio':{r:{k:v for k,v in h.items() if k in ('network_ok','ack_routes','assigned_round')}
                                for r,h in self.radio_health.items()}}
            self.status.publish(String(data=json.dumps(payload)))
            self.save_checkpoint()
            self.get_logger().info('STATE '+json.dumps(payload))
