#!/usr/bin/env python3
"""Global area exploration with periodic shadow departure scheduling.

Range-gated agents exchange all four scan maps. Robot1 uses SLAM localization;
robot2..4 retain start-anchored odometry. Shadows use simulator terrain truth.
Run with the four_robot_nav launch, and never with multirobot_bt concurrently.
"""
import json
import argparse
import math
import signal
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from planner import ROBOTS, allocate
from shadow_prediction import Forecast
from mission import settings, coverage
from radio import connected, routes_connected, route_point, movement_permits


class Exploration(Node):
    def __init__(self, config):
        super().__init__('lunar_exploration')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.config = config
        self.radio_range = config['radio_range_m']
        self.budget = config['travel_budget_m']
        environment = yaml.safe_load((Path(get_package_share_directory('moon_mapping_demo'))/'config/environment.yaml').read_text())
        self.homes = {r: (environment['starts'][r]['x'], environment['starts'][r]['y']) for r in ROBOTS}
        self.coverage_report = None
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.nav_clients = {r: ActionClient(self, NavigateToPose, f'/{r}/navigate_to_pose') for r in ROBOTS}
        self.path_clients = {r: ActionClient(self, ComputePathToPose, f'/{r}/compute_path_to_pose') for r in ROBOTS}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.heartbeat = self.create_publisher(Bool, '/multi_bt/emergency_stop', qos)
        self.status = self.create_publisher(String, '/lunar_exploration/status', qos)
        self.permit_pubs = {r:self.create_publisher(Bool,f'/lunar/{r}/motion_permit',qos) for r in ROBOTS}
        self.allowed = set()
        self.commands = {}
        self.headings = {}
        self.map = None
        self.map_time = None
        self.forecast = None
        self.plans = {}
        self.validated_plans = {}
        self.validating = set()
        self.validated = set()
        self.odom_times = {}
        self.translation_times = {}
        self.measured_velocity = {}
        self.goal_progress = {}
        self.shadows = {}
        self.shadow_times = {}
        self.radio_health = {}
        self.assignment_pub = self.create_publisher(String,'/lunar/robot1/assignment',10)
        self.subs = [self.create_subscription(OccupancyGrid, '/lunar/robot1/team_map', self.receive_map, qos)]
        self.subs.append(self.create_subscription(String, '/lunar/shadow_forecast', self.receive_forecast, qos))
        for robot in ROBOTS:
            self.subs.append(self.create_subscription(Twist,f'/{robot}/cmd_vel_requested',
                lambda m,r=robot:self.commands.update({r:(m,self.now())}),10))
            self.subs.append(self.create_subscription(String,f'/lunar/{robot}/radio_status',
                lambda m,r=robot:self.radio_health.update({r:json.loads(m.data)}),qos))
            self.subs.append(self.create_subscription(
                Odometry, f'/{robot}/odom', lambda m, r=robot:self.receive_odom(r,m),
                qos_profile_sensor_data))
            self.subs.append(self.create_subscription(
                Bool, f'/{robot}/in_terrain_shadow', lambda m, r=robot: self.receive_shadow(r, m), 10))
            self.subs.append(self.create_subscription(
                NavPath, f'/{robot}/plan', lambda m, r=robot:self.receive_plan(r,m), 10))
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.prefetch = None
        self.phase = 'WAITING'
        self.round = 0
        self.tasks = {}
        self.handles = {}
        self.sending = set()
        self.canceling = set()
        self.isolated = set()
        self.queue = []
        self.visited = []
        self.rejected = []
        self.travel = {r: 0.0 for r in ROBOTS}
        self.previous = {}
        self.results = []
        self.failures = 0
        self.round_started = 0.0
        self.start_known = None
        self.known = 0
        self.last_status = 0.0
        self.retry_at = 0.0
        self.unavailable_since = None
        self.checkpoint = Path('/tmp/lunar_exploration_checkpoint.json')
        self.resume_checked = False
        self.watchdog = self.create_timer(
            .2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('MISSION '+json.dumps(config))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def receive_forecast(self, msg):
        try:
            self.forecast = Forecast(msg.data)
        except Exception as exc:
            self.halt(f'Invalid shadow forecast: {exc}')

    def receive_odom(self,robot,msg):
        self.odom_times[robot] = self.now()
        self.measured_velocity[robot] = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if abs(msg.twist.twist.linear.x)>.01:
            self.translation_times[robot] = self.now()

    def receive_plan(self,robot,msg):
        route = [(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        if (self.phase=='MOVING' and route and robot in self.tasks and
                math.dist(route[-1],self.tasks[robot]['goal'])<.4):
            self.plans[robot] = route

    def receive_shadow(self, robot, msg):
        self.shadows[robot] = msg.data
        self.shadow_times[robot] = self.now()
        if msg.data and self.phase not in ('STOPPED', 'WAITING'):
            self.allowed.discard(robot)
            if robot in self.queue:
                self.queue.remove(robot)
            if robot in self.handles and robot not in self.canceling:
                self.isolate(robot,'SHADOW INCIDENT; this robot waits for sunlight, peers keep safe routes')

    def defer(self, reason):
        """Keep the mission pending while the simulator and Sun advance."""
        self.phase = 'WAITING'
        self.queue = []
        self.allowed = set()
        self.cancel_all()
        self.heartbeat.publish(Bool(data=True))
        self.retry_at = time.monotonic()+1
        self.unavailable_since = None
        self.get_logger().warning(reason)
        self.status.publish(String(data=json.dumps({
            'phase': self.phase, 'reason': reason, 'round': self.round,
            'coverage': self.coverage_report, 'shadow': self.shadows,
            'mission_complete': False})))

    def receive_map(self, msg):
        if not msg.info.width or not msg.info.height:
            return
        self.map = (np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width),
                    msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_time = self.now()
        self.known = int(np.count_nonzero(self.map[0] >= 0))
        if self.start_known is None:
            self.start_known = self.known

    def positions(self):
        positions = {}
        for robot in ROBOTS:
            transform = self.buffer.lookup_transform('map', f'{robot}/base_link', Time())
            stamp = transform.header.stamp
            # /clock and /tf arrive independently; permit a small future skew.
            if not -.5 <= self.now()-(stamp.sec+stamp.nanosec/1e9) < 3:
                raise ValueError(f'{robot} stale TF')
            p = transform.transform.translation
            q = transform.transform.rotation
            self.headings[robot] = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            positions[robot] = (p.x, p.y)
        return positions

    def connected(self, positions):
        return connected(positions,self.radio_range)

    def cancel_all(self):
        for robot, handle in list(self.handles.items()):
            if robot not in self.canceling:
                self.canceling.add(robot)
                handle.cancel_goal_async()

    def halt(self, reason):
        self.phase = 'STOPPED'
        self.heartbeat.publish(Bool(data=True))
        self.cancel_all()
        self.status.publish(String(data=json.dumps({
            'phase': self.phase, 'reason': reason, 'round': self.round,
            'travel_m': self.travel, 'known_cells': self.known,
            'coverage': self.coverage_report, 'mission_complete': False,
            'initial_known_cells': self.start_known, 'recent_results': self.results[-8:]})))
        self.get_logger().error(reason)
        # A navigation stop disables motors. The rotating Sun and simulation
        # clock keep running so forecasts can advance to a later safe window.

    def save_checkpoint(self):
        state = {'round': self.round, 'travel_m': self.travel,
                 'positions': self.previous, 'visited': self.visited,
                 'rejected': self.rejected, 'initial_known_cells': self.start_known,
                 'results': self.results, 'sim_time': self.now()}
        state['world_id'] = self.forecast.payload.get('world_id') if self.forecast else None
        temporary = self.checkpoint.with_suffix('.tmp')
        temporary.write_text(json.dumps(state))
        temporary.replace(self.checkpoint)

    def resume(self, positions):
        self.resume_checked = True
        if not self.checkpoint.exists():
            return True
        saved = json.loads(self.checkpoint.read_text())
        current_world = self.forecast.payload.get('world_id')
        if current_world and saved.get('world_id') != current_world:
            archive = self.checkpoint.with_name(f'lunar_checkpoint_previous_{int(time.time())}.json')
            self.checkpoint.rename(archive)
            self.get_logger().info('New simulator world; previous mission checkpoint archived')
            return True
        if any(math.dist(positions[r], saved['positions'][r]) > .5 for r in ROBOTS):
            self.halt('Checkpoint positions differ; use a new checkpoint after resetting the simulation')
            return False
        if self.now() < saved.get('sim_time', 0)-1:
            self.halt('Simulation clock reset; checkpoint belongs to a previous simulation')
            return False
        self.round = saved['round']
        self.travel = saved['travel_m']
        self.previous = {r: tuple(p) for r, p in saved['positions'].items()}
        self.visited = saved['visited']
        self.rejected = saved['rejected']
        self.results = saved['results']
        self.start_known = saved['initial_known_cells']
        self.get_logger().info(f'Resumed after round {self.round}; preserving accumulated travel')
        return True

    def send_next(self, positions):
        if self.phase != 'MOVING' or not self.queue:
            return
        robot = self.queue.pop(0)
        point = self.tasks[robot]['goal']
        if self.tasks[robot].get('waiting_for_sun') or self.shadows.get(robot,False):
            self.get_logger().info(f'{robot} waiting for sunlight; peers may navigate')
            return
        if math.dist(positions[robot], point) < .25:
            self.get_logger().info(f'{robot} holding relay position {point}')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = map(float, point)
        goal.pose.pose.orientation.w = 1.0
        if 'global_goal' in self.tasks[robot]:
            target = self.tasks[robot]['global_goal']
            yaw = math.atan2(target[1]-point[1], target[0]-point[0])
            goal.pose.pose.orientation.z = math.sin(yaw/2)
            goal.pose.pose.orientation.w = math.cos(yaw/2)
        self.sending.add(robot)
        future = self.nav_clients[robot].send_goal_async(goal)
        future.add_done_callback(lambda f, r=robot: self.accepted(r, f))
        self.get_logger().info(f'GOAL round={self.round} {robot} {self.tasks[robot]["role"]} -> {point}')

    def validate_paths(self):
        self.phase = 'VALIDATING'
        self.validated.clear()
        self.validated_plans = {}
        self.validating = set(ROBOTS)
        self.validation_started = time.monotonic()
        for robot, task in self.tasks.items():
            if math.dist(self.previous[robot],task['goal']) < .25:
                self.validating.discard(robot)
                if (task.get('waiting_for_sun') or
                        self.forecast.hold_safe(self.previous[robot],self.now(),20)):
                    self.validated.add(robot)
                    self.validated_plans[robot] = [self.previous[robot]]
                else:
                    self.path_failed(robot,'holding position would enter future shadow')
                continue
            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = 'map'
            goal.goal.header.stamp = self.get_clock().now().to_msg()
            goal.goal.pose.position.x, goal.goal.pose.position.y = map(float, task['goal'])
            goal.goal.pose.orientation.w = 1.
            goal.planner_id = 'GridBased'
            goal.use_start = False
            future = self.path_clients[robot].send_goal_async(goal)
            future.add_done_callback(lambda f, r=robot: self.path_accepted(r, f))

    def path_accepted(self, robot, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.path_failed(robot, 'path request rejected')
                return
            handle.get_result_async().add_done_callback(lambda f, r=robot: self.path_checked(r, f))
        except Exception as exc:
            self.path_failed(robot, str(exc))

    def path_failed(self, robot, reason):
        self.validating.discard(robot)
        self.rejected.append({'point':self.tasks[robot]['goal'],'until':self.now()+15})
        self.get_logger().warning(f'{robot} path rejected before moving: {reason}')
        # A failed leg must not discard the other robots' valid paths. Keep
        # this agent at its measured pose, then check connectivity and the
        # remaining paths together before permitting anybody to move.
        point = self.previous[robot]
        self.tasks[robot] = {'role':'relay','goal':point,'grid_distance_m':0.,
                             'path_blocked':True,
                             'waiting_for_sun':bool(self.shadows.get(robot,False) or
                                 not self.forecast.hold_safe(point,self.now(),20))}
        self.validated_plans[robot] = [point]
        self.validated.add(robot)

    def path_checked(self, robot, future):
        self.validating.discard(robot)
        try:
            result = future.result().result
            route = [(p.pose.position.x, p.pose.position.y) for p in result.path.poses]
            if not route:
                self.path_failed(robot,f'Nav2 returned no path: code={getattr(result,"error_code","unavailable")} message={getattr(result,"error_msg","")}')
                return
            if not self.forecast.route_safe(route,self.now(),hold=15):
                length = sum(math.dist(a,b) for a,b in zip(route,route[1:]))
                self.path_failed(robot,f'predicted shadow on actual {length:.2f}m Nav2 route')
                return
            self.validated_plans[robot] = route
            self.validated.add(robot)
        except Exception as exc:
            self.path_failed(robot, str(exc))

    def accepted(self, robot, future):
        self.sending.discard(robot)
        try:
            handle = future.result()
            if not handle.accepted:
                self.isolate(robot, 'goal rejected')
                return
            self.handles[robot] = handle
            self.goal_progress[robot] = (math.dist(self.previous[robot],self.tasks[robot]['goal']),self.now())
            handle.get_result_async().add_done_callback(lambda f, r=robot: self.finished(r, f))
            if self.phase != 'MOVING':
                self.cancel_all()
        except Exception as exc:
            self.halt(f'Goal submission failed: {exc}')

    def failed(self, robot, reason, count_failure=True):
        self.failures += int(count_failure)
        if math.dist(self.previous[robot],self.tasks[robot]['goal'])>.25:
            self.rejected.append({'point':self.tasks[robot]['goal'],'until':self.now()+15})
        self.queue = []
        self.phase = 'REPLAN'
        self.retry_at = time.monotonic()+2
        self.heartbeat.publish(Bool(data=True))
        self.cancel_all()
        self.get_logger().warning(f'{robot}: {reason}; replan with rejected target excluded')
        if self.failures >= 5:
            self.halt('Five navigation failures; inspect logs before continuing')

    def isolate(self, robot, reason):
        """Cancel one stalled goal while peers keep their validated routes."""
        self.allowed.discard(robot)
        self.plans.pop(robot,None)
        self.rejected.append({'point':self.tasks[robot]['goal'],'until':self.now()+60})
        if robot in self.handles:
            self.isolated.add(robot)
            if robot not in self.canceling:
                self.canceling.add(robot)
                self.handles[robot].cancel_goal_async()
        self.get_logger().warning(f'{robot}: {reason}; canceling this goal only, peers continue under shadow/radio gates')

    def finished(self, robot, future):
        self.handles.pop(robot, None)
        self.canceling.discard(robot)
        try:
            status = future.result().status
        except Exception as exc:
            self.halt(f'Navigation result failed: {exc}')
            return
        self.results.append({'round': self.round, 'robot': robot, 'status': status})
        if robot in self.isolated:
            self.isolated.discard(robot)
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.failures = 0
            point = self.tasks[robot]['goal']
            target = self.tasks[robot].get('global_goal')
            if target is not None and math.dist(point, target) < .7:
                self.visited.append(target)
            self.get_logger().info(f'ARRIVED round={self.round} {robot} at {point}')
        elif self.phase == 'MOVING':
            self.isolate(robot, f'Nav2 status={status}')

    def plan_from(self, positions, traveled=None, fixed_tasks=None):
        """Snapshot planner inputs so calculation may overlap safe motion."""
        cells, resolution, x, y = self.map
        snapshot = cells.copy()
        location = dict(positions)
        traveled = dict(self.travel if traveled is None else traveled)
        visited = list(self.visited)
        now = self.now()
        rejected = [p['point'] for p in self.rejected
                    if isinstance(p,dict) and p['until']>now and p.get('kind')!='visited']
        recent_targets = {r:[p['point'] for p in self.rejected if isinstance(p,dict)
                            and p.get('kind')=='visited' and p.get('robot')==r and p['until']>now]
                          for r in ROBOTS}
        preferred_goals = {r:t['global_goal'] for r,t in self.tasks.items() if 'global_goal'in t}
        forecast = self.forecast
        frozen = {r for r in ROBOTS if self.shadows.get(r,False)
                  or not forecast.hold_safe(location[r],now,5)}
        fixed_tasks = {r:dict(t) for r,t in (fixed_tasks or {}).items()}
        frozen -= set(fixed_tasks)
        config = dict(self.config)
        homes = dict(self.homes)
        radio_range, budget = self.radio_range, self.budget

        def work():
            report = coverage(snapshot, resolution, (x, y), location,
                              config['bounds_m'], config['coverage_resolution_m'])
            kwargs = dict(radio_range=radio_range, budget=budget,
                          forecast=forecast, now=now, bounds=config['bounds_m'],
                          max_leg=config.get('max_leg_m',3.),
                          staging_hold=config.get('staging_hold_s',15.),
                          solver_seconds=config.get('solver_seconds',5.),
                          compactness_weight=config.get('compactness_weight',0),
                          spacing_weight=config.get('spacing_weight',0),
                          preferred_spacing=config.get('preferred_spacing_m',10.),
                          max_wait_s=config.get('maximum_wait_s',10.),
                          maximum_explorers=config.get('maximum_explorers',4),
                          movement_reward=config.get('movement_reward',1000))
            kwargs['frozen_robots'] = frozen
            kwargs['fixed_tasks'] = fixed_tasks
            kwargs['preferred_goals'] = preferred_goals
            kwargs['recent_targets'] = recent_targets
            result = allocate(snapshot,resolution,(x,y),location,homes,traveled,
                              visited,rejected,**kwargs)
            if result['status']=='INFEASIBLE':
                result = allocate(snapshot,resolution,(x,y),location,homes,traveled,
                                  visited,rejected,allow_reposition=True,**kwargs)
            result['coverage'] = report
            return result

        return self.pool.submit(work)

    def prefetch_usable(self, positions):
        """An early plan is advisory; actual paths still need live validation."""
        return (self.prefetch is not None
                and self.prefetch['round']==self.round
                and self.prefetch['rejections']==len(self.rejected)
                and self.prefetch.get('shadows',())==tuple(r for r in ROBOTS
                    if getattr(self,'shadows',{}).get(r,False))
                and self.now()-self.prefetch['stamp']<30
                and all(math.dist(positions[r],self.prefetch['positions'][r])<.4
                        for r in ROBOTS))

    def preplan(self, positions):
        if self.prefetch is not None or self.isolated or not self.handles:
            return
        active = set(self.handles)|self.sending|set(self.queue)
        if not all(math.dist(positions[r],self.tasks[r]['goal'])<1.5 for r in active):
            return
        # Plan from the expected end poses while the current, independently
        # validated legs continue. Never replace in-flight goals here.
        endpoints = {r:tuple(self.tasks[r]['goal']) if r in active else positions[r]
                     for r in ROBOTS}
        traveled = {r:self.travel[r]+math.dist(positions[r],endpoints[r]) for r in ROBOTS}
        self.prefetch = {'future':self.plan_from(endpoints,traveled),
                         'positions':endpoints,'stamp':self.now(),
                         'shadows':tuple(r for r in ROBOTS if self.shadows.get(r,False)),
                         'round':self.round,'rejections':len(self.rejected)}
        self.get_logger().info('PREPLAN next allocation while robots finish current legs')

    def tick(self):
        self.allowed = set()
        try:
            self.step()
        except Exception as exc:
            self.halt(f'Unexpected exploration error: {exc}')
        finally:
            for robot,pub in self.permit_pubs.items():
                pub.publish(Bool(data=robot in self.allowed and self.phase=='MOVING'))

    def wait_for_data(self, reason):
        self.heartbeat.publish(Bool(data=True))
        if time.monotonic()-getattr(self,'last_wait_log',0)>5:
            self.get_logger().warning(f'{self.phase}: {reason}')
            self.last_wait_log = time.monotonic()
        if self.unavailable_since is None:
            self.unavailable_since = time.monotonic()
        if self.phase != 'WAITING' and time.monotonic()-self.unavailable_since > 3:
            self.halt(reason)

    def step(self):
        if self.phase in ('STOPPED', 'COMPLETE'):
            self.heartbeat.publish(Bool(data=True))
            return
        if self.phase == 'MOVING' and self.now()-self.round_started > 120:
            self.halt('Navigation round timed out; mission is not complete')
            return
        try:
            positions = self.positions()
        except Exception as exc:
            self.wait_for_data(f'Missing or stale robot TF: {str(exc)[:160]}')
            return
        ready = (self.map is not None and self.now()-self.map_time < 10
                 and self.forecast is not None and -.5 <= self.now()-self.forecast.stamp < 5
                 and all(self.now()-self.odom_times.get(r, -100) < 2 for r in ROBOTS)
                 and all(self.now()-self.shadow_times.get(r, -100) < 2 for r in ROBOTS)
                 and all(self.now()-self.radio_health.get(r,{}).get('stamp',-100)<5
                         and self.radio_health.get(r,{}).get('network_ok',False) for r in ROBOTS)
                 and all(c.server_is_ready() for c in (*self.nav_clients.values(), *self.path_clients.values())))
        if not ready:
            reason = ('Waiting for terrain shadow to pass; Sun and clock continue'
                      if any(self.shadows.values()) else 'Waiting for fresh map, odometry, forecast, radio and Nav2')
            self.wait_for_data(reason)
            return
        self.unavailable_since = None
        if not self.resume_checked and not self.resume(positions):
            return
        for robot, point in positions.items():
            if robot in self.previous:
                self.travel[robot] += math.dist(point, self.previous[robot])
            self.previous[robot] = point
            if self.travel[robot]+math.dist(point, self.homes[robot]) > self.budget-1:
                self.halt(f'{robot}: travel budget reserve reached; return requires a separate command')
                return
        if not self.connected(positions):
            self.halt('Ideal range graph disconnected; stopped all robots')
            return
        if self.phase == 'MOVING':
            remaining = {}
            active = set(self.handles)|self.sending|set(self.queue)
            for robot in ROBOTS:
                route = self.plans.get(robot,[]) if robot in active else []
                nearest = min(range(len(route)),key=lambda j:math.dist(positions[robot],route[j])) if route else 0
                remaining[robot] = route[nearest:]
            projections = {}
            for robot in ROBOTS:
                command,stamp = self.commands.get(robot,(Twist(),-100))
                distance = min(.44,abs(command.linear.x)*2+.05) if abs(command.linear.x)>.01 and self.now()-stamp<.5 else 0.
                projections[robot] = route_point([positions[robot]]+remaining[robot],distance=distance)
                if command.linear.x<-.01:
                    yaw = self.headings[robot]
                    projections[robot] = (positions[robot][0]-distance*math.cos(yaw),positions[robot][1]-distance*math.sin(yaw))
            self.allowed = movement_permits(positions,projections,self.radio_range)&active
            self.allowed -= {r for r in ROBOTS if self.shadows.get(r,False)}
            for robot in self.handles:
                error = math.dist(positions[robot],self.tasks[robot]['goal'])
                best,last_motion = self.goal_progress.get(robot,(error,self.round_started))
                if error < best-.15:
                    best,last_motion = error,self.now()
                    self.goal_progress[robot] = (best,last_motion)
                if (robot in self.allowed and self.now()-last_motion>12 and
                        error>.4):
                    self.isolate(robot,'Nav2 did not advance for 12s; retry after peers finish')
            for robot in ROBOTS:
                stale_progress = robot in self.handles and self.now()-self.goal_progress.get(robot,(0,self.now()))[1]>8
                if (robot not in active or self.tasks[robot].get('waiting_for_sun')
                        or self.shadows.get(robot,False)):
                    continue
                if (robot not in self.allowed or stale_progress) and not self.forecast.hold_safe(positions[robot],self.now(),20):
                    self.failed(robot,'waiting here would enter shadow; rescheduling the team',count_failure=False)
                    return
            for robot in self.handles:
                route = self.plans.get(robot, [])
                if not route:
                    continue
                nearest = min(range(len(route)), key=lambda j: math.dist(positions[robot], route[j]))
                upcoming = [positions[robot]]
                length = 0.
                for p in route[nearest:]:
                    length += math.dist(upcoming[-1], p)
                    if length > 1.0:
                        break
                    upcoming.append(p)
                if not self.forecast.route_safe(upcoming, self.now(), turn_delay=0, hold=2):
                    self.failed(robot, 'predicted shadow on upcoming Nav2 path; stopped before entry',count_failure=False)
                    return
        self.heartbeat.publish(Bool(data=self.phase != 'MOVING'))
        if time.monotonic()-self.last_status > 3:
            self.last_status = time.monotonic()
            payload = {'phase': self.phase, 'round': self.round, 'positions': positions,
                       'travel_m': self.travel, 'known_cells': self.known,
                       'initial_known_cells': self.start_known, 'shadow': self.shadows,
                       'coverage': self.coverage_report, 'mission_complete': False,
                       'sun_period_s': self.forecast.payload.get('sun_period_s'),
                       'sun_phase_deg': self.forecast.payload.get('sun_phase_deg'),
                       'departure_at': getattr(self,'departure_at',None),
                       'motion_permits': sorted(self.allowed),
                       'measured_velocity': {r:list(self.measured_velocity.get(r,(0.,0.))) for r in ROBOTS},
                       'moving_robots': [r for r in ROBOTS if abs(self.measured_velocity.get(r,(0.,0.))[0])>.01],
                       'radio': {r:{k:v for k,v in h.items() if k in ('network_ok','ack_routes','assigned_round')}
                                 for r,h in self.radio_health.items()},
                       'sun': {k:self.forecast.payload.get(k) for k in ('sun_period_s','sun_phase_deg','elevation_deg')},
                       'tasks': self.tasks, 'recent_results': self.results[-8:]}
            self.status.publish(String(data=json.dumps(payload)))
            self.save_checkpoint()
            self.get_logger().info('STATE '+json.dumps(payload))
        if self.phase in ('WAITING', 'REPLAN') and not self.handles and not self.sending:
            if time.monotonic() < self.retry_at:
                return
            if self.prefetch_usable(positions):
                self.future = self.prefetch['future']
                self.get_logger().info('PREPLAN reused; skipping stationary allocation')
            else:
                if self.prefetch is not None:
                    self.prefetch['future'].cancel()
                self.future = self.plan_from(positions)
            self.prefetch = None
            self.phase = 'PLANNING'
        elif self.phase == 'PLANNING' and self.future.done():
            result = self.future.result()
            self.coverage_report = result['coverage']
            self.get_logger().info('CP-SAT '+json.dumps(result))
            if 'tasks' not in result:
                if result['status'] in ('NO_FRONTIER_IN_CURRENT_MAP', 'NO_REACHABLE_FRONTIER_IN_CURRENT_MAP') and self.coverage_report['all_accessible_observed']:
                    self.phase = 'COMPLETE'
                    self.coverage_report['mission_complete'] = True
                    self.status.publish(String(data=json.dumps({'phase': self.phase, 'coverage': self.coverage_report})))
                    self.get_logger().info('Map-based mission completion criterion satisfied')
                    if Path('/tmp/lunar_simulation_command').exists():
                        Path('/tmp/lunar_simulation_command').write_text('pause')
                    return
                if all(self.forecast.hold_safe(p,self.now(),10) for p in positions.values()):
                    self.phase = 'REPLAN'
                    self.retry_at = time.monotonic()+1
                    self.get_logger().warning('No safe target in this forecast; retrying, mission remains incomplete')
                else:
                    self.defer('No feasible sunlit route yet; keeping motors stopped and retrying as the Sun advances')
                return
            self.tasks = result['tasks']
            self.preferred_departure = result.get('departure_at')
            # Radio agents survive a planner restart and reject older rounds.
            self.round = max(self.round, *(int(h.get('assigned_round', 0))
                                           for h in self.radio_health.values())) + 1
            # Dispatch the short, prevalidated legs within a few control ticks.
            # Nav2 handles obstacles; range is checked throughout transit.
            self.queue = sorted(ROBOTS, key=lambda r: self.tasks[r]['role'] == 'explorer')
            self.departure_at = max(self.preferred_departure or self.now(),self.now()+1)
            self.phase = 'ANNOUNCING'
            self.announcement_started = time.monotonic()
            self.assignment_pub.publish(String(data=json.dumps({'round':self.round,'tasks':self.tasks,'departure_at':self.departure_at})))
        elif self.phase == 'VALIDATING':
            if time.monotonic()-self.validation_started > 15:
                self.halt('Nav2 path validation timed out')
                return
            if not self.validating:
                if len(self.validated) == 4:
                    if not any(math.dist(positions[r],self.tasks[r]['goal'])>=.25
                               for r in ROBOTS if not self.tasks[r].get('waiting_for_sun')):
                        self.phase = 'REPLAN'
                        self.retry_at = time.monotonic()+1
                        self.get_logger().warning('No moving path validated; retrying without a false MOVING state')
                        return
                    if not routes_connected(self.validated_plans,positions,self.radio_range):
                        Path('/tmp/lunar_rejected_paths.json').write_text(json.dumps({
                            'positions':positions,'tasks':self.tasks,'plans':self.validated_plans,
                            'radio_range':self.radio_range,'sim_time':self.now()}))
                        robot = max(ROBOTS,key=lambda r:self.tasks[r]['grid_distance_m'])
                        self.failed(robot,'actual Nav2 paths cannot preserve a radio spanning network',count_failure=False)
                        return
                    longest = max(sum(math.dist(a,b) for a,b in zip(route,route[1:])) for route in self.validated_plans.values())
                    horizon = 5+longest/.08+15
                    if not all(self.tasks[r].get('waiting_for_sun') or
                               self.forecast.route_safe(route,self.now(),hold=15,team_horizon=horizon)
                               for r,route in self.validated_plans.items()):
                        robot = max(ROBOTS,key=lambda r:sum(math.dist(a,b) for a,b in zip(self.validated_plans[r],self.validated_plans[r][1:])))
                        self.failed(robot,'actual route leaves insufficient sunlit time for the team; changing target',count_failure=False)
                        return
                    self.plans = {r:list(route) for r,route in self.validated_plans.items()}
                    self.phase = 'MOVING'
                    self.round_started = self.now()
                    self.get_logger().info(f'DEPART round={self.round} sun_phase={self.forecast.payload.get("sun_phase_deg")}')
                else:
                    self.phase = 'REPLAN'
                    self.retry_at = time.monotonic()+1
        elif self.phase == 'ANNOUNCING':
            acknowledgments = self.radio_health['robot1'].get('assignment_acks',{})
            if all(acknowledgments.get(r)==self.round for r in ROBOTS[1:]):
                self.get_logger().info(f'RADIO assignment round={self.round} acknowledged by all robots')
                self.phase = 'WAITING_SUN'
            elif time.monotonic()-self.announcement_started > 15:
                self.halt('Task assignment radio acknowledgments timed out')
        elif self.phase == 'WAITING_SUN':
            if self.now()>=self.departure_at:
                self.validate_paths()
            elif not all(self.tasks[r].get('waiting_for_sun') or
                         self.forecast.hold_safe(p,self.now(),self.departure_at-self.now())
                         for r,p in positions.items()):
                self.phase = 'REPLAN'
        elif self.phase == 'MOVING':
            self.preplan(positions)
            if not self.queue and not self.handles and not self.sending:
                self.phase = 'REPLAN'
                self.retry_at = time.monotonic()
            else:
                self.send_next(positions)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config')
    args, ros_args = parser.parse_known_args()
    config = settings(args.config)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    if config.get('rolling_dispatch',False):
        from rolling_dispatch import RollingExploration
        node = RollingExploration(config)
    else:
        node = Exploration(config)
    stopping = [False]
    signal.signal(signal.SIGINT, lambda *_: stopping.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__(0, True))
    try:
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.halt('Exploration process stopped')
        if node.previous:
            node.save_checkpoint()
        end = time.monotonic()+3
        while rclpy.ok() and (node.handles or node.sending) and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        node.pool.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
