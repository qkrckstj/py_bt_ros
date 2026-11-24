"""Path reservations for the exploration experiment, without lidar classification.

Robot positions come from the robot BTs' localized poses. Planned paths come
from Nav2. Reservation grids are PRIVATE planning layers, never SLAM input.
This is conservative priority coordination, not optimal multi-agent planning.
"""
import array
import json
import math
import time

import numpy as np
from scipy import ndimage
from .exploration_radio import RadioNetwork


def sample_route(points, position, step=.1):
    """Trim a completed path prefix and densely sample the remaining segments."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points) or not np.isfinite(points).all():
        return None
    nearest = int(np.argmin(np.linalg.norm(points-np.asarray(position), axis=1)))
    if math.dist(points[nearest], position) > .8:
        return None
    route = np.vstack([position, points[nearest:]])
    result = [route[0]]
    for start, end in zip(route, route[1:]):
        count = max(1, math.ceil(math.dist(start, end)/step))
        result.extend(start+(end-start)*(i/count) for i in range(1, count+1))
    return np.asarray(result)


def routes_conflict(a, b, clearance):
    # Chunk the distance calculation to bound memory for long map-scale paths.
    return any(np.any(np.sum((a[i:i+64, None, :]-b[None, :, :])**2, axis=2) < clearance**2)
               for i in range(0, len(a), 64))


def route_window(route, distance):
    """Arc-length prefix including an interpolated final point."""
    result = [route[0]]
    remaining = distance
    for a, b in zip(route, route[1:]):
        length = float(np.linalg.norm(b-a))
        if length > remaining:
            result.append(a+(b-a)*remaining/length)
            break
        result.append(b)
        remaining -= length
    return np.asarray(result)


def route_prediction(route, speed=.4, horizon=3.):
    """Same time samples for every peer; stop at the end of a short path."""
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    distance = np.minimum(np.arange(0., horizon+.001, .2)*speed, arc[-1])
    return np.column_stack([np.interp(distance, arc, route[:, i]) for i in (0, 1)])


def reserve_routes(positions, routes, order, clearance=.85, previous=()):
    """Reserve a rolling 3 s window, comparing peers at matching times.

    Accepted movers can follow each other. A waiting/missing-path robot remains
    a stationary obstacle. Actual velocities are checked separately before any
    command is forwarded; constant-speed path predictions are not a safety proof.
    """
    accepted, reasons, predicted = {}, {}, {}
    for name in order:
        route = routes.get(name)
        if route is None:
            reasons[name] = 'Waiting for a fresh path to the current goal'
            continue
        prediction = route_prediction(route)
        # A slightly larger restart gap prevents repeated release/stop at the
        # same boundary, without reducing the minimum separation of movers.
        gap = clearance if name in previous else clearance+.1
        peer = next((n for n, p in positions.items() if n != name
                     and np.any(np.sum((prediction-predicted.get(n, np.asarray(p)))**2,
                                       axis=1) < gap**2)), None)
        if peer:
            reasons[name] = f'Near-term conflict with {peer}; yield/replan'
            continue
        accepted[name] = route_window(route, 1.2)
        predicted[name] = prediction
        reasons[name] = 'Rolling reservation'
    return accepted, reasons


def constrain_motion(positions, yaws, commands, order, clearance=.85, radio=None):
    """Check combined commanded motion, treating every held peer as stopped.

    In addition to path intent, predict actual unicycle commands for 1.2 s.
    Grant in priority order and check all pairs at 0.1 s intervals. This guards
    against a leader stopping or turning while a follower's path remains clear.
    """
    if radio and not radio.graph(positions)['all_connected']:
        return {n: (0., 0.) for n in positions}, {n: 'radio' for n in positions}
    # Keep the safety margin when the current graph has it. If localization
    # drift consumes that margin, allow connected recovery instead of freezing
    # even inward commands whose first prediction is still in the margin band.
    # The actual radio radius remains a hard limit at every predicted sample.
    use_radio_margin = bool(radio and radio.graph(positions, safe=True)['all_connected'])
    accepted = {n: (0., commands.get(n, (0., 0.))[1]) for n in positions}
    blocked = {}
    for name in order:
        if name not in positions:
            continue
        trial = {**accepted, name: commands.get(name, (0., 0.))}
        reason = None
        for t in np.arange(.1, 1.201, .1):
            future = {n: RadioNetwork.project(p, yaws.get(n, 0.), *trial[n], float(t))
                      for n, p in positions.items()}
            for peer in positions:
                if peer != name and math.dist(future[name], future[peer]) < clearance:
                    reason = 'collision'
                    break
            if reason:
                break
            if radio and not radio.graph(future, safe=use_radio_margin)['all_connected']:
                reason = 'radio'
                break
        if reason is None:
            accepted[name] = trial[name]
        elif abs(trial[name][0]) > 1e-6:
            blocked[name] = reason
    return accepted, blocked


def reservation_cells(shape, resolution, origin, blocked_points, clearance=.85):
    """Unknown means no contribution: unreserved cells cannot erase SLAM walls
    or make unexplored cells free when this maximum-combined layer is applied.
    """
    seeds = np.zeros(shape, dtype=bool)
    for points in blocked_points:
        points = np.asarray(points)
        if not len(points):
            continue
        cols = np.floor((points[:, 0]-origin[0])/resolution).astype(int)
        rows = np.floor((points[:, 1]-origin[1])/resolution).astype(int)
        valid = (rows >= 0) & (cols >= 0) & (rows < shape[0]) & (cols < shape[1])
        seeds[rows[valid], cols[valid]] = True
    cells = np.full(shape, -1, dtype=np.int8)
    if seeds.any():
        cells[ndimage.distance_transform_edt(~seeds)*resolution <= clearance+resolution*.71] = 100
    return cells


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import Point
    from nav_msgs.msg import OccupancyGrid, Path
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    class Traffic(Node):
        def __init__(self):
            super().__init__('exploration_traffic')
            self.names = self.declare_parameter('robots', ['robot1', 'robot2', 'robot3', 'robot4']).value
            self.clearance = float(self.declare_parameter('route_clearance', .85).value)
            self.radio = RadioNetwork(list(self.declare_parameter('base_position', [0., 0.]).value),
                float(self.declare_parameter('radio_range', 7.).value),
                float(self.declare_parameter('radio_margin', .5).value)) if self.declare_parameter('radio_enabled', False).value else None
            self.team, self.team_received, self.states, self.paths, self.commands = {}, -math.inf, {}, {}, {}
            self.grid, self.grid_received, self.map_sent = None, -math.inf, -math.inf
            self.granted, self.reasons = {}, {}
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self.map, qos)
            self.create_subscription(String, '/exploration/team_status', self.team_state, qos)
            self.status = self.create_publisher(String, '/exploration/traffic_status', qos)
            # Estimated poses remain planning inputs. Actual simulated link
            # status and final /cmd_vel are owned by the radio environment.
            self.radio_status = self.create_publisher(String, '/exploration/radio_estimate', 10)
            self.radio_markers = self.create_publisher(MarkerArray, '/exploration/radio_estimate_markers', qos)
            self.mask_pubs = {n: self.create_publisher(OccupancyGrid, f'/{n}/reservation_map', qos) for n in self.names}
            topic = 'cmd_vel_candidate' if self.radio else 'cmd_vel'
            self.vel_pubs = {n: self.create_publisher(Twist, f'/{n}/{topic}', 10) for n in self.names}
            for name in self.names:
                self.create_subscription(String, f'/{name}/exploration/state', lambda m, n=name: self.state(n, m), 10)
                self.create_subscription(Path, f'/{name}/plan', lambda m, n=name: self.path(n, m), 1)
                self.create_subscription(Twist, f'/{name}/cmd_vel_nav', lambda m, n=name: self.velocity(n, m), 1)
            self.create_timer(.1, self.update)

        def map(self, msg):
            if msg.header.frame_id == 'map' and msg.info.width*msg.info.height == len(msg.data):
                self.grid, self.grid_received = msg, time.monotonic()

        def team_state(self, msg):
            try:
                self.team, self.team_received = json.loads(msg.data), time.monotonic()
            except ValueError:
                pass

        def state(self, name, msg):
            try:
                self.states[name] = json.loads(msg.data), time.monotonic()
            except ValueError:
                pass

        def path(self, name, msg):
            points = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
            if msg.header.frame_id == 'map' and points:
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                self.paths[name] = points, time.monotonic(), stamp

        def velocity(self, name, msg):
            if all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
                # Predict the same saturated commands the physical driver uses.
                msg.linear.x = max(-.12, min(.4, msg.linear.x))
                msg.angular.z = max(-.9, min(.9, msg.angular.z))
                self.commands[name] = msg, time.monotonic()

        def update(self):
            now = time.monotonic()
            sim = self.get_clock().now().nanoseconds*1e-9
            positions = {}
            for name in self.names:
                state, received = self.states.get(name, ({}, -math.inf))
                p = state.get('position')
                if now-received <= 2. and p and len(p) == 2 and all(math.isfinite(v) for v in p):
                    positions[name] = p
            radio_ready = len(positions) == len(self.names)
            ready = (radio_ready and all(self.states[n][0].get('ready') for n in self.names)
                     and self.grid is not None and now-self.grid_received <= 3.)
            active = self.team.get('active', {}) if now-self.team_received <= 2. else {}
            routes = {}
            order = sorted(active, key=lambda n: (active[n].get('created_at', 0.), n))
            if ready:
                for name in order:
                    points, received, stamp = self.paths.get(name, ([], -math.inf, -math.inf))
                    state = self.states[name][0]
                    if (now-received <= 3. and -.25 <= sim-stamp <= 3. and points
                            and math.dist(points[-1], active[name]['goal'][:2]) <= .5
                            and state.get('task_id') == active[name].get('task_id')
                            and state.get('session') == self.team.get('session')):
                        route = sample_route(points, positions[name])
                        if route is not None:
                            routes[name] = route
                self.granted, self.reasons = reserve_routes(positions, routes, order, self.clearance,
                                                           previous=self.granted)
            else:
                self.granted, self.reasons = {}, {n: 'Waiting for all localized peer poses and shared map' for n in self.names}
            # Seed the costmap layer before Nav2 is ready. Waiting for every
            # action server here would make costmap startup depend on itself.
            if self.grid is not None and now-self.map_sent >= .5:
                self.publish_masks(positions, order)
                self.map_sent = now
            velocities = {}
            for name in self.names:
                raw, received = self.commands.get(name, (Twist(), -math.inf))
                permitted = ready and name in self.granted and now-received < .5
                # Circular footprints may turn in place while awaiting a route;
                # translational motion always requires a non-conflicting path.
                turning = (ready and name in active and now-received < .5 and abs(raw.linear.x) < 1e-5
                           and all(math.dist(positions[name], p) >= self.clearance
                                   for n, p in positions.items() if n != name))
                velocities[name] = raw if permitted or turning else Twist()
            radio_blocked = []
            motion_blocked = []
            if ready:
                yaws = {n: self.states[n][0].get('yaw') or 0. for n in self.names}
                desired = {n: (v.linear.x, v.angular.z) for n, v in velocities.items()}
                accepted, blocked = constrain_motion(positions, yaws, desired,
                    [*order, *[n for n in self.names if n not in order]], self.clearance, self.radio)
                motion_blocked = [n for n, reason in blocked.items() if reason == 'collision']
                radio_blocked = [n for n, reason in blocked.items() if reason == 'radio']
                for name, (v, w) in accepted.items():
                    command = Twist()
                    command.linear.x, command.angular.z = float(v), float(w)
                    velocities[name] = command
            if self.radio and radio_ready:
                self.publish_radio(positions, radio_ready, radio_blocked)
            elif self.radio:
                self.radio_status.publish(String(data=json.dumps({'ready': False, 'connected': {}})))
            for name, velocity in velocities.items():
                self.vel_pubs[name].publish(velocity)
            self.status.publish(String(data=json.dumps({'allowed': sorted(self.granted),
                'reasons': self.reasons, 'clearance': self.clearance, 'ready': ready,
                'radio_blocked': radio_blocked, 'motion_blocked': motion_blocked})))

        def publish_radio(self, positions, ready, blocked):
            state = {**self.radio.graph(positions), 'ready': ready, 'blocked_motion': blocked}
            self.radio_status.publish(String(data=json.dumps(state)))
            markers, points = [], {'base': self.radio.base, **positions}
            palette = [(1., 1., 1.), (.95, .25, .25), (.25, .9, .4), (.3, .55, 1.), (1., .75, .2)]
            for i, (name, xy) in enumerate(points.items()):
                m = Marker()
                m.header.frame_id, m.header.stamp = 'map', self.get_clock().now().to_msg()
                m.ns, m.id, m.type, m.action = 'radio_range', i, Marker.LINE_STRIP, Marker.ADD
                m.pose.orientation.w, m.scale.x = 1., .035
                m.color.r, m.color.g, m.color.b = palette[i]
                m.color.a = .8
                m.points = [Point(x=float(xy[0]+self.radio.radius*math.cos(j*math.tau/96)),
                                  y=float(xy[1]+self.radio.radius*math.sin(j*math.tau/96)), z=.12)
                            for j in range(97)]
                markers.append(m)
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'radio_links', 0, Marker.LINE_LIST, Marker.ADD
            m.pose.orientation.w, m.scale.x = 1., .06
            m.color.g, m.color.a = 1., 1.
            m.points = [Point(x=float(points[n][0]), y=float(points[n][1]), z=.22)
                        for link in state['links'] for n in link]
            markers.append(m)
            self.radio_markers.publish(MarkerArray(markers=markers))

        def publish_masks(self, positions, order):
            grid = self.grid
            p, q = grid.info.origin.position, grid.info.origin.orientation
            if abs(q.x)+abs(q.y)+abs(q.z) > 1e-6:
                return  # This simulation's shared fusion grid is axis aligned.
            for name in self.names:
                blocked = [np.asarray([xy]) for peer, xy in positions.items() if peer != name]
                for peer in order:
                    if peer == name:
                        break
                    if peer in self.granted:
                        blocked.append(self.granted[peer])
                msg = OccupancyGrid()
                msg.header.frame_id, msg.header.stamp = 'map', self.get_clock().now().to_msg()
                msg.info = grid.info
                cells = reservation_cells((grid.info.height, grid.info.width), grid.info.resolution,
                                          (p.x, p.y), blocked, self.clearance)
                msg.data = array.array('b', cells.tobytes())
                self.mask_pubs[name].publish(msg)

    rclpy.init(args=args)
    node = Traffic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for pub in node.vel_pubs.values():
            pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
