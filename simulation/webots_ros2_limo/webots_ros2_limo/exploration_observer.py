"""Webots diagnostics, visuals and simulated physical radio environment.

World poses model the radio environment and a final collision veto when the
interlock stops selected peers. Navigation/SLAM/allocation still use wheel/IMU
and laser localization; world poses are not injected into them.
"""
import json
import importlib.util
import math
from pathlib import Path
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import DurabilityPolicy, QoSProfile
from .exploration_radio import RadioNetwork, physical_radio_permit, select_physical_commands


class FlatObserver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('flat_experiment_observer')
        self.publisher = self.node.create_publisher(String, '/exploration/ground_truth', 10)
        self.nodes = {f'robot{i}': self.robot.getFromDef(f'ROBOT{i}') for i in range(1, 5)}
        self.starts = json.loads(properties.get('starts_json', '{}'))
        self.base = json.loads(properties.get('base_json', '[0,0]'))
        self.radio_range = float(properties.get('radio_range', '0'))
        self.display_radio = RadioNetwork(self.base, self.radio_range) if self.radio_range else None
        self.radio_state = {}
        self.commands, self.next_radio = {}, 0.
        if self.display_radio:
            self.radio_publisher = self.node.create_publisher(String, '/exploration/radio_status', 10)
            self.marker_publisher = self.node.create_publisher(MarkerArray, '/exploration/radio_markers',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.velocity_publishers = {n: self.node.create_publisher(Twist, f'/{n}/cmd_vel', 1)
                                        for n in self.nodes}
            for name in self.nodes:
                self.node.create_subscription(Twist, f'/{name}/cmd_vel_candidate',
                    lambda m, n=name: self.receive_command(n, m), 1)
        self.rings = {n: self.robot.getFromDef('RADIO_'+n.upper()) for n in ['base', *self.nodes]}
        self.links = self.robot.getFromDef('RADIO_LINKS')
        # Visual-only aids must not become laser obstacles. Webots lidar also
        # sees render geometry that has no boundingObject (including lines).
        overlays = [*self.rings.values(), self.links,
                    self.robot.getFromDef('BASE_MARKER'), self.robot.getFromDef('SUN_MARKER')]
        laser_count = 0
        for robot in self.nodes.values():
            laser = robot.getFromProtoDef('EXPLORATION_LIDAR') if robot else None
            if laser:
                laser_count += 1
                for overlay in overlays:
                    if overlay:
                        overlay.setVisibility(laser, False)
        if laser_count != 4:
            raise RuntimeError('Cannot isolate visual overlays from all four lidar sensors')
        self.node.get_logger().info('Visual overlays hidden from all four lidar sensors; main view unchanged')
        self.link_pairs = [(a, b) for i, a in enumerate(['base', *self.nodes])
                           for b in ['base', *self.nodes][i+1:]]
        if self.links:
            geometry = self.links.getField('geometry').getSFNode()
            self.link_coordinates = geometry.getField('coord').getSFNode().getField('point')
            indices = geometry.getField('coordIndex')
            # Allocate once. Removing/reinserting every frame makes Webots
            # rebuild its geometry and visibly blink between empty updates.
            while self.link_coordinates.getCount() < 2*len(self.link_pairs):
                self.link_coordinates.insertMFVec3f(-1, [0., 0., 0.])
            if indices.getCount() != 3*len(self.link_pairs):
                raise ValueError('Radio segments must be allocated in the generated world')
        self.sun = None
        if properties.get('sun_path'):
            root = Path(properties['sun_path'])
            spec = importlib.util.spec_from_file_location('imported_lunar_sun', root/'lunar_sun.py')
            self.sun_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.sun_module)
            self.sun_config = json.loads((root/'sun_config.json').read_text())
            self.sun = self.robot.getFromDef('LUNAR_SUN')
            view = self.robot.getFromDef('DEMO_VIEW')
            height = float(properties.get('overview_height', '0'))
            if height > 0:
                # Apply once at scene startup: Webots may restore a previous
                # close-up viewpoint even when the generated world is larger.
                view.getField('position').setSFVec3f([0., 0., height])
                view.getField('orientation').setSFRotation([-.577350269, .577350269, .577350269, 2.094395102])
            for side in ('WEST', 'EAST', 'SOUTH', 'NORTH'):
                boundary = self.robot.getFromDef('BOUNDARY_'+side)
                if boundary:
                    boundary.setVisibility(view, False)
            for i in range(96):
                boundary = self.robot.getFromDef(f'BOUNDARY_RING_{i}')
                if boundary:
                    boundary.setVisibility(view, False)
        self.next, self.next_image = 0., 10.
        self.directory = Path.home() / 'limo_exploration_output'
        self.directory.mkdir(parents=True, exist_ok=True)

    def receive_command(self, name, msg):
        if all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            self.commands[name] = (max(-.12, min(.4, msg.linear.x)),
                                   max(-.9, min(.9, msg.angular.z))), self.robot.getTime()

    def radio_update(self, now):
        positions = {n: v.getPosition()[:2] for n, v in self.nodes.items() if v}
        yaws = {n: math.atan2(v.getOrientation()[3], v.getOrientation()[0])
                for n, v in self.nodes.items() if v}
        fresh = all(n in self.commands and 0. <= now-self.commands[n][1] < .3 for n in self.nodes)
        commands = {n: c if 0. <= now-t < .3 else (0., 0.) for n, (c, t) in self.commands.items()}
        # World positions add a final simulated radio/collision veto only. The
        # mapper, allocator and Nav2 continue using their own localized poses.
        granted, blocked = select_physical_commands(self.display_radio, positions, yaws,
            commands, self.nodes, inputs_fresh=fresh)
        moving = [n for n, c in granted.items() if any(c)]
        allowed = (fresh and (not blocked or bool(moving)) and physical_radio_permit(
            self.display_radio, positions, yaws, granted, self.nodes, clearance=.85))
        graph = self.display_radio.graph(positions)
        # Publish measured links/connectivity, not perfect position coordinates.
        graph.pop('positions')
        self.radio_state = {**graph, 'ready': len(positions) == len(self.nodes),
                            'source': 'simulated_physical_radio', 'motion_allowed': allowed,
                            'blocked_motion': blocked, 'allowed_motion': moving,
                            'commands_fresh': fresh,
                            'hold_reason': 'stale_commands' if not fresh else
                                           'radio_or_robot_clearance' if blocked else ''}
        self.radio_publisher.publish(String(data=json.dumps(self.radio_state)))
        for name, pub in self.velocity_publishers.items():
            msg = Twist()
            if allowed:
                msg.linear.x, msg.angular.z = granted[name]
            pub.publish(msg)

    def visuals(self, now):
        if self.sun:
            c = self.sun_config
            vector, azimuth = self.sun_module.sun_direction(now, c['elevation_deg'],
                c['initial_azimuth_deg'], c['azimuth_period_sim_seconds'])
            self.sun.getField('direction').setSFVec3f([-v for v in vector])
            self.robot.getFromDef('SUN_MARKER').getField('translation').setSFVec3f([30*v for v in vector])
            self.robot.setLabel(0, f'LUNAR EXPLORATION | Sun azimuth {azimuth:.0f} deg', .02, .02, .035, 0xFFFFFF, 0, 'Arial')
        if not self.radio_range:
            return
        points = {'base': [*self.base, .08], **{n: v.getPosition() for n, v in self.nodes.items() if v}}
        # Physical range visualization is independent of Nav2/SLAM startup.
        # The same physical-range model drives radio status and final interlock.
        display_graph = self.display_radio.graph({n: p[:2] for n, p in points.items() if n != 'base'})
        self.radio_markers(points, display_graph)
        for name, point in points.items():
            if self.rings.get(name):
                self.rings[name].getField('translation').setSFVec3f([point[0], point[1], point[2]+.04])
        if self.links:
            visible = {frozenset(link) for link in display_graph['links']}
            for i, (a, b) in enumerate(self.link_pairs):
                if frozenset((a, b)) in visible and a in points and b in points:
                    endpoints = [points[a], points[b]]
                else:
                    # A zero-length segment hides a genuinely absent link
                    # without destroying any render object or its indices.
                    endpoints = [[*self.base, .08]]*2
                for j, p in enumerate(endpoints):
                    self.link_coordinates.setMFVec3f(2*i+j, [p[0], p[1], p[2]+.08])
        connected = sum(display_graph['connected'].values())
        control = ('PARTIAL HOLD' if self.radio_state.get('blocked_motion') else 'MOTION ENABLED') \
            if self.radio_state.get('motion_allowed') else 'MOTION HOLD'
        self.robot.setLabel(1, f'RADIO R={self.radio_range:g} m | BASE REACHABLE {connected}/4 (MULTIHOP) | {control}',
                            .02, .07, .032, 0x88FFBB, 0, 'Arial')

    def radio_markers(self, points, graph):
        """RViz displays the same measured radio scene as Webots."""
        markers = []
        colors = [(1., 1., 1.), (.95, .25, .25), (.25, .9, .4), (.3, .55, 1.), (1., .75, .2)]
        for i, (name, p) in enumerate(points.items()):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type, m.action = 'radio_range', i, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w, m.scale.x, m.color.a = 1., .035, .8
            m.color.r, m.color.g, m.color.b = colors[i]
            m.points = [Point(x=float(p[0]+self.radio_range*math.cos(j*math.tau/96)),
                              y=float(p[1]+self.radio_range*math.sin(j*math.tau/96)), z=.12)
                        for j in range(97)]
            markers.append(m)
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id, m.type, m.action = 'radio_links', 0, Marker.LINE_LIST, Marker.ADD
        m.pose.orientation.w, m.scale.x, m.color.g, m.color.a = 1., .06, 1., 1.
        m.points = [Point(x=float(points[n][0]), y=float(points[n][1]), z=.22)
                    for link in graph['links'] for n in link]
        self.marker_publisher.publish(MarkerArray(markers=[*markers, m]))

    def step(self):
        now = self.robot.getTime()
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.display_radio and now >= self.next_radio:
            self.next_radio = now+.1
            self.radio_update(now)
        if now >= self.next:
            self.next = now + .5
            self.visuals(now)
            self.publisher.publish(String(data=json.dumps({'sim_time': now,
                'starts': self.starts,
                'poses': {name: node.getPosition() for name, node in self.nodes.items() if node},
                'orientations': {name: node.getOrientation() for name, node in self.nodes.items() if node}})))
        if now >= self.next_image:
            self.next_image = now + 30.
            self.robot.exportImage(str(self.directory/'webots_latest.jpg'), 85)
