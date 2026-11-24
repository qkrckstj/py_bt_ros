#!/usr/bin/env python3
"""Run Chan Seo's TeamBT/RobotBT on the existing 60m lunar ROS runtime.

The original checkout supplies sensors, range-gated maps, shadow forecasts and
Nav2. This process is the sole task owner and sole motion-permit publisher.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path as NavPath, OccupancyGrid
from visualization_msgs.msg import Marker
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from webots_ros2_limo.exploration_methods import allocate_exploration
from webots_ros2_limo.exploration_policy import Goal, FrontierGrid
from webots_ros2_limo.exploration_planning import PlanningWorker, observed_reposition_goals, recent_visit_exclusions
from webots_ros2_limo.exploration_radio import MobileRadioNetwork, select_physical_commands
from webots_ros2_limo.exploration_ros import ExplorationTeam, ExplorationRobot

ROBOTS = ('robot1', 'robot2', 'robot3', 'robot4')
QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class LunarTeam(ExplorationTeam):
    def __init__(self, legacy, path_planner='astar', method='cp_sat', resume_status=None):
        if method not in ('cp_sat', 'space_astar'):
            raise ValueError('Unknown exploration method')
        if method == 'space_astar' and path_planner != 'astar':
            raise ValueError('space_astar requires the astar path planner')
        self.method = method
        if path_planner not in ('astar', 'distance_field'):
            raise ValueError('Unknown path planner')
        self.path_planner = path_planner
        sys.path.insert(0, str(legacy / 'scenarios/lunar_exploration'))
        from mission import settings
        from shadow_prediction import Forecast
        self.Forecast, self.config = Forecast, settings()
        config = self.config
        overrides = {'use_sim_time': True, 'autostart': True,
                     'allocator': 'cp_sat' if method == 'cp_sat' else 'greedy',
                     'radio_enabled': True, 'radio_range': float(config['radio_range_m']),
                     'radio_margin': .3, 'connectivity_mode': 'mobile_team',
                     'require_mapping_control': False, 'return_home': False,
                     'completion_mode': 'bounds', 'exploration_bounds': config['bounds_m'],
                     'clearance': .4, 'goal_timeout': 120., 'input_timeout': 5.}
        super().__init__(parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()],
                         cli_args=['--ros-args', '-r', '/map:=/lunar/robot1/team_map'])
        self.bt.allocator = self.shadow_allocate
        self.forecast = None
        self.shadow, self.health, self.commands, self.plans = {}, {}, {}, {}
        self.travel, self.last_position = dict.fromkeys(ROBOTS, 0.), {}
        if resume_status is not None:
            previous = json.loads(resume_status.read_text())
            travel = previous.get('travel_m', {})
            if set(travel) != set(ROBOTS) or any(not math.isfinite(v) or v < 0 for v in travel.values()):
                raise ValueError('Invalid travel counters in resume status')
            self.travel, self.last_position = travel, previous.get('positions', {})
            self.bt.history = previous.get('history', [])[-200:]
            self.get_logger().info('Retained travel/history from current-world status; active tasks will be replanned')
        self.diagnostics = {}
        self.planning = PlanningWorker()
        self.plan_retry_at = 0.
        self.progress = {}
        self.last_report = 0.
        self.report_path = Path(__file__).parents[2]/"artifacts/lunar_chanseo_cp_sat/live_status.json"
        self.enable_pub = self.create_publisher(Bool, '/multi_bt/emergency_stop', QOS)
        self.permits = {n: self.create_publisher(Bool, f'/lunar/{n}/motion_permit', QOS) for n in ROBOTS}
        self.coverage_pub = self.create_publisher(OccupancyGrid, '/exploration/coverage', QOS)
        self.coverage_status = self.create_publisher(String, '/exploration/coverage_status', QOS)
        self.coverage_label = self.create_publisher(Marker, '/exploration/coverage_label', QOS)
        self.route_pubs = {n: self.create_publisher(NavPath, f'/exploration/{n}/planned_path', QOS)
                           for n in ROBOTS}
        self.coverage = {}
        self.selected_routes = {}
        self.create_subscription(String, '/lunar/shadow_forecast', self.receive_forecast, QOS)
        self.subscriptions_keep = []
        for n in ROBOTS:
            self.subscriptions_keep += [
                self.create_subscription(Bool, f'/{n}/in_terrain_shadow',
                    lambda m, n=n: self.shadow.update({n: (m.data, self.now())}), 10),
                self.create_subscription(String, f'/lunar/{n}/radio_status',
                    lambda m, n=n: self.health.update({n: json.loads(m.data)}), QOS),
                self.create_subscription(Twist, f'/{n}/cmd_vel_requested',
                    lambda m, n=n: self.commands.update({n: (m.linear.x, m.angular.z, self.now())}), 10),
                self.create_subscription(NavPath, f'/{n}/plan',
                    lambda m, n=n: self.plans.update({n: ([(p.pose.position.x, p.pose.position.y)
                        for p in m.poses], self.now())}), 10)]
        self.get_logger().info(f'Lunar {method} / {path_planner}: 60m map; moving 10m network; shadow gates')

    def route(self, grid, position, goal, distances=None):
        if self.path_planner == 'astar':
            return grid.astar_path(position, goal)
        return grid.path(position, goal, grid.distances(position) if distances is None else distances)

    def map(self, message):
        grid = super().map(message)
        # Publish only accepted maps, retaining their origin and rotation.
        if not hasattr(self, 'coverage_pub') or grid is None:
            return
        from webots_ros2_limo.exploration_coverage import coverage_snapshot
        cells, self.coverage = coverage_snapshot(grid)
        overlay = OccupancyGrid(header=message.header, info=message.info,
                                data=cells.ravel().tolist())
        self.coverage_pub.publish(overlay)
        self.coverage_status.publish(String(data=json.dumps(self.coverage)))
        label = Marker()
        label.header = message.header
        label.ns, label.id, label.type, label.action = 'coverage', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.orientation.w = 1.
        label.pose.position.x, label.pose.position.y = self.bounds[0], self.bounds[3]+1.
        label.pose.position.z, label.scale.z, label.color.a = .5, .8, 1.
        label.color.r, label.color.g, label.color.b = .2, 1., .4
        label.text = f"Observed {self.coverage['observed_percent']:.1f}% | {self.method} / {self.path_planner}"
        self.coverage_label.publish(label)

    def publish_routes(self, routes):
        for name, pub in self.route_pubs.items():
            message = NavPath()
            message.header.frame_id, message.header.stamp = self.frame, self.get_clock().now().to_msg()
            for x, y in routes.get(name, []):
                pose = PoseStamped(header=message.header)
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.w = 1.
                message.poses.append(pose)
            pub.publish(message)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def receive_forecast(self, msg):
        try:
            self.forecast = self.Forecast(msg.data, navigation_clearance_m=.3)
        except (KeyError, ValueError, TypeError) as exc:
            self.forecast = None
            self.get_logger().error(f'Invalid shadow forecast: {exc}')

    def inputs_safe(self):
        now = self.now()
        return (self.forecast is not None and -.5 <= now-self.forecast.stamp < 5.
                and self.bt.ready()
                and all(-.5 <= now-self.shadow.get(n, (False, -100))[1] < 2.
                        and -.5 <= now-self.health.get(n, {}).get('stamp', -100) < 5.
                        and self.health.get(n, {}).get('network_ok', False) for n in ROBOTS))

    def shadow_allocate(self, grid, positions, **constraints):
        if not positions or not self.inputs_safe():
            return {}
        active = {n: tuple(t['goal']) for n, t in self.bt.active.items()}
        context = (tuple(sorted(positions)), tuple(sorted((n, t['task_id']) for n, t in self.bt.active.items())))
        try:
            done, value = self.planning.poll(context)
        except Exception as exc:
            self.get_logger().error(f'Background planning failed: {exc}')
            self.plan_retry_at = time.monotonic()+1.
            return {}
        if done and value is not None:
            goals, diagnostics, starts = value
            self.diagnostics = diagnostics
            self.plan_retry_at = time.monotonic()+.5
            current = self.bt.positions()
            if (not set(goals).issubset(positions)
                    or any(math.dist(current[n], starts[n]) > .4 for n in goals)):
                return {}
            projected = {**current, **{n: p[:2] for n, p in active.items()},
                         **{n: g.xy for n, g in goals.items()}}
            if not self.bt.radio.graph(projected, safe=True)['all_connected']:
                return {}
            routes = {}
            for n, goal in goals.items():
                row, col = grid.index(goal.xy)
                if not (0 <= row < grid.cells.shape[0] and 0 <= col < grid.cells.shape[1]
                        and grid.safe[row, col]):
                    return {}
                route = self.route(grid, current[n], goal.xy)
                if not route or not self.forecast.route_safe(route, self.now(), hold=8.):
                    return {}
                routes[n] = route
            self.selected_routes.update(routes)
            self.publish_routes(self.selected_routes)
            return goals
        if self.planning.future is not None or time.monotonic() < self.plan_retry_at:
            return {}
        # Own immutable inputs. The worker never reads live BT state or invokes
        # its transport callbacks while those can change on the ROS thread.
        snapshot = FrontierGrid(grid.cells, grid.resolution, grid.origin,
                                clearance=grid.clearance, bounds=grid.bounds,
                                cache_distances=True)
        all_positions = {n: tuple(p) for n, p in self.bt.positions().items()}
        positions = {n: tuple(p) for n, p in positions.items()}
        forecast, now, shadows, travel = self.forecast, self.now(), dict(self.shadow), dict(self.travel)
        network, config = self.bt.radio, dict(self.config)
        options = {k: constraints[k] for k in ('reserved', 'excluded', 'separation')}
        revisit_excluded = recent_visit_exclusions(self.bt.history, time.monotonic())
        options['excluded'] = [*options['excluded'], *revisit_excluded]
        def compute():
            diagnostics = {'allocator': self.method, 'path_planner': self.path_planner,
                           'frontier_scoring': 'full_observed_route', 'recent_visit_exclusions': len(revisit_excluded),
                           'frontiers': len(snapshot.candidates), 'checked': 0,
                           'shadow_rejected': 0, 'radio_rejected': 0, 'forecast_rejected': 0}
            def short_leg(name, goal, distances):
                diagnostics["checked"] += 1
                if shadows.get(name, (True,))[0] or not forecast.hold_safe(positions[name], now, 5.):
                    diagnostics["shadow_rejected"] += 1
                    return None
                route = self.route(snapshot, positions[name], goal.xy, distances)
                if not route:
                    return None
                point = network.advance(all_positions, name, route)
                if math.dist(point, positions[name]) < .5:
                    diagnostics["radio_rejected"] += 1
                    return None
                # Truncate the actual observed route, never use a straight line.
                short, length = [positions[name]], 0.
                for p in route[1:]:
                    segment = math.dist(short[-1], p)
                    if length+segment > config.get('max_leg_m', 3.):
                        break
                    length += segment
                    short.append(p)
                    if math.dist(p, point) < .001:
                        break
                if len(short) < 2 or not forecast.route_safe(short, now, hold=8.):
                    diagnostics["forecast_rejected"] += 1
                    return None
                if travel[name]+length > config['travel_budget_m']:
                    return None
                return Goal(*short[-1], goal.yaw, goal.gain)
            eligible = {n: p for n, p in positions.items()
                        if not shadows.get(n, (True,))[0] and forecast.hold_safe(p, now, 5.)}
            if not eligible:
                diagnostics.update(status='WAITING_FOR_SHADOW_WINDOW', assigned=0)
                windows = [now+delay for delay in range(2, 242, 2)
                           if any(forecast.hold_safe(p, now+delay, 5.) for p in positions.values())]
                diagnostics['next_sunlit_sim_s'] = min(windows) if windows else None
                return {}, diagnostics, positions
            goals = allocate_exploration(self.method, snapshot, eligible, solver_seconds=.3, diagnostics=diagnostics,
                             min_distance=.5, transform_goal=short_leg, network=network,
                             team_positions=all_positions, active_goals=active, **options)
            if not goals:
                eligible = {n: p for n, p in positions.items()
                            if not shadows.get(n, (True,))[0] and forecast.hold_safe(p, now, 5.)}
                if eligible:
                    snapshot.candidates = observed_reposition_goals(snapshot, eligible,
                                                                  radius=config.get('max_leg_m', 3.))
                    diagnostics['mode'] = 'OBSERVED_REPOSITION'
                    # These are intermediate safe stops, not frontier claims.
                    # All ordinary grid, radio, shadow and separation checks remain.
                    goals = allocate_exploration(self.method, snapshot, eligible, solver_seconds=.3, diagnostics=diagnostics,
                        min_distance=.5, transform_goal=short_leg, network=network,
                        team_positions=all_positions, active_goals=active, **options)
            return goals, diagnostics, positions
        self.planning.submit(context, compute)
        return {}

    def update(self):
        # The base class registers this method as a timer during construction.
        if not hasattr(self, 'enable_pub'):
            return
        ready = self.inputs_safe()
        if ready:
            for n, task in self.bt.active.items():
                position = self.bt.positions()[n]
                previous, stamp, task_id = self.progress.get(n, (position, self.now(), task['task_id']))
                if task_id != task['task_id'] or math.dist(position, previous) > .12:
                    previous, stamp = position, self.now()
                self.progress[n] = (previous, stamp, task['task_id'])
                if self.now()-stamp > 12.:
                    task['cancel_requested'] = True
        requested = self.bt.enabled
        self.bt.enabled = requested and ready
        try:
            super().update()
        finally:
            self.bt.enabled = requested
        retained = {n: route for n, route in self.selected_routes.items() if n in self.bt.active}
        if set(retained) != set(self.selected_routes):
            self.selected_routes = retained
            self.publish_routes(retained)
        if (not self.bt.active and self.diagnostics.get('assigned') == 0):
            self.bt.phase = 'WAITING_FOR_SHADOW_WINDOW'
        allowed = set()
        gates, requested_commands, blocked = {}, {}, []
        if ready and requested and not self.bt.finished:
            positions, yaws, commands = self.bt.positions(), {}, {}
            for n in ROBOTS:
                position = positions[n]
                if n in self.last_position:
                    self.travel[n] += math.dist(position, self.last_position[n])
                self.last_position[n] = position
                state = self.bt.states[n][0]
                yaws[n] = state.get('yaw')
                v, w, stamp = self.commands.get(n, (0., 0., -100.))
                route, plan_stamp = self.plans.get(n, ([], -100.))
                upcoming = [position]
                if route:
                    i = min(range(len(route)), key=lambda j: math.dist(position, route[j]))
                    for p in route[i:]:
                        if math.dist(position, p) > 1.:
                            break
                        upcoming.append(p)
                safe = (not self.shadow[n][0] and -.5 <= self.now()-stamp < .5
                        and -.5 <= self.now()-plan_stamp < 3. and len(upcoming) > 1
                        and self.forecast.route_safe(upcoming, self.now(), turn_delay=0., hold=2.)
                        and self.travel[n] < self.config['travel_budget_m'])
                commands[n] = (v, w) if safe and n in self.bt.active else (0., 0.)
                requested_commands[n] = [v, w]
                gates[n] = {'shadow': self.shadow[n][0], 'command_age_s': self.now()-stamp,
                            'plan_age_s': self.now()-plan_stamp, 'upcoming_points': len(upcoming),
                            'route_gate': bool(safe), 'has_task': n in self.bt.active}
                if not safe and n in self.bt.active and self.shadow[n][0]:
                    self.bt.active[n]['cancel_requested'] = True
            if all(yaw is not None for yaw in yaws.values()):
                granted, blocked = select_physical_commands(self.bt.radio, positions, yaws, commands,
                                                       ROBOTS, horizon=1.2, clearance=.85, recovery_floor=.65)
                allowed = {n for n, command in granted.items() if any(command)}
        # Recheck expiry after the solver; never enable on the old input snapshot.
        ready = ready and self.inputs_safe()
        for n, pub in self.permits.items():
            pub.publish(Bool(data=ready and n in allowed))
        self.enable_pub.publish(Bool(data=not (ready and requested)))
        if time.monotonic()-self.last_report > 3.:
            self.last_report = time.monotonic()
            report = {'phase': self.bt.phase, 'inputs_ready': ready, 'sim_time': self.now(),
                      'allocator': self.method, 'path_planner': self.path_planner, 'coverage': self.coverage,
                      'positions': self.bt.positions() if self.bt.ready() else {},
                      'travel_m': self.travel, 'motion_permits': sorted(allowed),
                      'shadow': self.shadow, 'solver': self.diagnostics,
                      'motion_gates': gates, 'requested_commands': requested_commands,
                      'physical_blocked': blocked,
                      'planning_pending': self.planning.future is not None,
                      'active': self.bt.active, 'history': self.bt.history,
                      'mission_complete': self.bt.finished, 'scope': 'Live simulation only'}
            self.report_path.parent.mkdir(parents=True, exist_ok=True)
            encoded = json.dumps(report, indent=2)+'\n'
            self.report_path.write_text(encoded)
            method_path = self.report_path.with_name(f'live_status_{self.method}.json')
            temporary = method_path.with_suffix('.tmp')
            temporary.write_text(encoded)
            temporary.replace(method_path)

    def hold(self):
        for pub in self.permits.values():
            pub.publish(Bool(data=False))
        self.enable_pub.publish(Bool(data=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--legacy-repo', required=True, type=Path)
    parser.add_argument('--path-planner', choices=('astar', 'distance_field'), default='astar')
    parser.add_argument('--method', choices=('cp_sat', 'space_astar'), default='cp_sat')
    parser.add_argument('--resume-status', type=Path)
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    team = LunarTeam(args.legacy_repo.resolve(), args.path_planner, args.method, args.resume_status)
    robots = [ExplorationRobot(namespace=n, parameter_overrides=[Parameter('use_sim_time', value=True)])
              for n in ROBOTS]
    for node in [team, *robots]:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            team.bt.enabled = False
            team.hold()
            for robot in robots:
                robot.nav.cancel()
            stop_deadline = time.monotonic()+2.
            while time.monotonic() < stop_deadline and rclpy.ok():
                executor.spin_once(timeout_sec=.05)
        team.planning.close()
        for node in [team, *robots]:
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
