"""ROS adapters for the four-robot allocation prototype (no simulator launcher)."""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .exploration_policy import FrontierGrid
from .exploration_runtime import RobotBT, TeamBT
from .exploration_radio import RadioNetwork


def decode(message):
    try:
        value = json.loads(message.data)
        return value if isinstance(value, dict) else None
    except (ValueError, TypeError):
        return None


class Navigation:
    def __init__(self, node, action_name, frame):
        self.node, self.frame = node, frame
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.state, self.handle, self.cancel_requested = 'IDLE', None, False
        self.cancel_future = None
        self.pose_ready = False

    def ready(self):
        return self.pose_ready and self.client.server_is_ready()

    def start(self, goal):
        if self.state in ('SENDING', 'NAVIGATING', 'CANCELING'):
            raise RuntimeError('Previous navigation action has not terminated')
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = self.frame
        # This is a fixed target in the shared map, not a historical sensor
        # observation. Zero asks TF/Nav2 to use the latest transform even when
        # replanning a long-running goal after its original timestamp expired.
        request.pose.header.stamp = Time().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = goal[:2]
        request.pose.pose.orientation.z = math.sin(goal[2] / 2)
        request.pose.pose.orientation.w = math.cos(goal[2] / 2)
        self.handle, self.cancel_requested, self.cancel_future = None, False, None
        self.state = 'SENDING'
        self.client.send_goal_async(request).add_done_callback(self.accepted)

    def accepted(self, future):
        try:
            self.handle = future.result()
            if not self.handle.accepted:
                self.state = 'CANCELED' if self.cancel_requested else 'FAILED'
                return
            self.state = 'NAVIGATING'
            self.handle.get_result_async().add_done_callback(self.finished)
            if self.cancel_requested:
                self.cancel()
        except Exception as exc:
            self.node.get_logger().error(f'Goal acceptance failed: {exc}')
            self.state = 'FAILED'

    def finished(self, future):
        try:
            response = future.result()
            if self.cancel_requested or response.status == GoalStatus.STATUS_CANCELED:
                self.state = 'CANCELED'
            elif response.status == GoalStatus.STATUS_SUCCEEDED:
                self.state = 'SUCCEEDED'
            else:
                self.state = 'FAILED'
        except Exception as exc:
            self.node.get_logger().error(f'Goal result failed: {exc}')
            self.state = 'FAILED'
        self.handle = None

    def cancel(self):
        if self.state not in ('SENDING', 'NAVIGATING', 'CANCELING'):
            return
        self.cancel_requested = True
        if self.handle and self.cancel_future is None:
            self.state = 'CANCELING'
            self.cancel_future = self.handle.cancel_goal_async()
        # Only the action result certifies termination, not the cancel response.

    def poll(self):
        return self.state


class ExplorationRobot(Node):
    def __init__(self, **kwargs):
        super().__init__('exploration_robot', **kwargs)
        self.frame = self.declare_parameter('map_frame', 'map').value
        ns = self.get_namespace().strip('/')
        self.base = self.declare_parameter('base_frame', f'{ns}/base_link').value
        self.pose_timeout = float(self.declare_parameter('pose_timeout', 3.).value)
        self.buffer = Buffer(node=self, cache_time=Duration(seconds=30.))
        self.listener = TransformListener(self.buffer, self)
        # A single real LIMO can run its driver/Nav2 at the root namespace
        # while this BT keeps robot1-scoped task and status topics.
        action_name = self.declare_parameter('nav_action', 'navigate_to_pose').value
        self.nav = Navigation(self, action_name, self.frame)
        self.bt = RobotBT(self.nav,
                          lease=float(self.declare_parameter('command_lease', 3.).value),
                          observe_seconds=float(self.declare_parameter('observe_seconds', 2.).value),
                          radio_enabled=self.declare_parameter('radio_enabled', False).value)
        self.create_subscription(String, '/exploration/radio_status', self.radio, 10)
        self.publisher = self.create_publisher(String, 'exploration/state', 10)
        self.create_subscription(String, 'exploration/command', self.receive, 10)
        self.create_timer(.2, self.update)
        self.get_logger().info(f'Robot BT: {self.base}; waiting for team allocation')

    def receive(self, message):
        command = decode(message)
        if command:
            self.bt.receive(command, time.monotonic())

    def radio(self, message):
        state = decode(message)
        if state is not None and state.get('ready'):
            self.bt.link_ok = state.get('connected', {}).get(self.get_namespace().strip('/'), False)
            self.bt.link_received = time.monotonic()

    def update(self):
        position, yaw = None, None
        age, tf_error = None, None
        try:
            transform = self.buffer.lookup_transform(self.frame, self.base, Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            p = transform.transform.translation
            if -.25 <= age <= self.pose_timeout and all(math.isfinite(v) for v in (p.x, p.y)):
                position = [p.x, p.y]
                q = transform.transform.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            else:
                tf_error = f'Pose transform is outside freshness limit (age={age:.3f}s)'
        except Exception as exc:
            tf_error = str(exc)
        self.nav.pose_ready = position is not None
        self.bt.tick(time.monotonic())
        self.publisher.publish(String(data=json.dumps({**self.bt.snapshot(), 'position': position, 'yaw': yaw,
                                                      'tf_age': age, 'tf_error': tf_error,
                                                      'ready': self.nav.ready()})))


class ExplorationTeam(Node):
    def __init__(self, **kwargs):
        super().__init__('exploration_team', **kwargs)
        robots = self.declare_parameter('robots', ['robot1', 'robot2', 'robot3', 'robot4']).value
        if any(not r or '/' in r or not r.replace('_', '').isalnum() for r in robots):
            raise ValueError('robots must be simple namespace names')
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.bounds = list(self.declare_parameter('exploration_bounds', [0., 0., 0., 0.]).value)
        if self.bounds == [0., 0., 0., 0.]:
            self.bounds = []
        self.clearance = float(self.declare_parameter('clearance', .32).value)
        self.min_frontier_length = float(self.declare_parameter('min_frontier_length', .5).value)
        enabled = self.declare_parameter('radio_enabled', False).value
        radio = RadioNetwork(list(self.declare_parameter('base_position', [0., 0.]).value),
            float(self.declare_parameter('radio_range', 7.).value),
            float(self.declare_parameter('radio_margin', .5).value)) if enabled else None
        self.command_publishers = {r: self.create_publisher(String, f'/{r}/exploration/command', 10) for r in robots}
        self.bt = TeamBT(robots, self.send,
                         autostart=self.declare_parameter('autostart', False).value,
                         timeout=float(self.declare_parameter('input_timeout', 4.).value),
                         separation=float(self.declare_parameter('goal_separation', 2.).value),
                         min_goal_distance=float(self.declare_parameter('min_goal_distance', .5).value),
                         claim_radius=float(self.declare_parameter('exploration_claim_radius', 0.).value),
                         complete_hold=float(self.declare_parameter('completion_hold', 5.).value),
                         goal_timeout=float(self.declare_parameter('goal_timeout', 120.).value),
                         completion_mode=self.declare_parameter('completion_mode', 'frontier').value,
                         return_home=self.declare_parameter('return_home', True).value,
                         homes=json.loads(self.declare_parameter('homes_json', '{}').value),
                         require_mapping=self.declare_parameter('require_mapping_control', True).value,
                         return_timeout=float(self.declare_parameter('return_timeout', 600.).value),
                         return_retries=int(self.declare_parameter('return_retries', 2).value), radio=radio,
                         require_radio_measurement=self.declare_parameter('require_radio_measurement', False).value,
                         strategy=self.declare_parameter('strategy', 'frontier').value,
                         sweep_base=list(self.declare_parameter('base_position', [0., 0.]).value) if not enabled else list(radio.base),
                         sweep_bounds=list(self.declare_parameter('sweep_bounds', [-29., -29., 29., 29.]).value),
                         sweep_radius=float(self.declare_parameter('sweep_radius', 28.).value),
                         sweep_slot_slack=float(self.declare_parameter('sweep_slot_slack', 0.).value))
        self.create_subscription(String, '/exploration/radio_status', self.radio_measurement, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/exploration/team_status', qos)
        self.map_control = self.create_publisher(String, '/exploration/map_control', qos)
        self.create_subscription(String, '/exploration/mapping_state', self.mapping, qos)
        self.markers = self.create_publisher(MarkerArray, '/exploration/goals', qos)
        self.create_subscription(OccupancyGrid, '/map', self.map, qos)
        for name in robots:
            self.create_subscription(String, f'/{name}/exploration/state',
                                     lambda m, name=name: self.state(name, m), 10)
        self.create_subscription(Empty, '/exploration/start', self.start, 10)
        self.create_subscription(Empty, '/exploration/stop', self.stop, 10)
        self.create_timer(.2, self.update)
        self.get_logger().info(f'Team BT: {len(robots)} robots; explore / freeze / return; autostart={self.bt.enabled}')

    def mapping(self, message):
        state = decode(message)
        if state is not None:
            self.bt.update_mapping(state, time.monotonic())

    def send(self, name, command):
        self.command_publishers[name].publish(String(data=json.dumps(command)))

    def state(self, name, message):
        state = decode(message)
        if state is not None:
            self.bt.update_robot(name, state, time.monotonic())

    def radio_measurement(self, message):
        state = decode(message)
        if state is not None:
            self.bt.radio_measurement, self.bt.radio_received = state, time.monotonic()

    def map(self, message):
        i, q = message.info, message.info.origin.orientation
        try:
            if message.header.frame_id != self.frame or i.width*i.height != len(message.data):
                raise ValueError('Shared map frame/shape mismatch')
            if abs(q.x) + abs(q.y) > 1e-6 or abs(q.z*q.z+q.w*q.w-1) > 1e-4:
                raise ValueError('Shared map must have a normalized planar orientation')
            yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
            grid = FrontierGrid(np.asarray(message.data).reshape(i.height, i.width), i.resolution,
                                (i.origin.position.x, i.origin.position.y, yaw),
                                clearance=self.clearance, bounds=self.bounds,
                                min_frontier_length=self.min_frontier_length, cache_distances=True)
            self.bt.update_map(grid, time.monotonic())
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f'Ignoring map: {exc}')

    def start(self, _message):
        self.bt.enabled, self.bt.complete_since = True, None
        # A failed return may be retried, but start never unfreezes a completed
        # mission or overwrites the original home positions.
        for name in self.bt.return_failed:
            self.bt.return_attempts[name] = 0
        self.bt.return_failed.clear()

    def stop(self, _message):
        self.bt.enabled = False
        self.update()

    def update(self):
        started = time.perf_counter()
        self.bt.tick(time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)
        snapshot = self.bt.snapshot()
        snapshot['team_tick_ms'] = (time.perf_counter()-started)*1000.
        self.map_control.publish(String(data=json.dumps({'session': self.bt.session, 'mode': self.bt.mapping_mode})))
        self.status_pub.publish(String(data=json.dumps(snapshot)))
        markers = [Marker(action=Marker.DELETEALL)]
        if self.bt.sweep and self.bt.sweep.stage != 'CLEANUP':
            line = Marker()
            line.header.frame_id, line.header.stamp = self.frame, self.get_clock().now().to_msg()
            line.ns, line.id, line.type, line.action = 'preferred_radar_formation', 0, Marker.LINE_STRIP, Marker.ADD
            line.pose.orientation.w, line.scale.x = 1., .08
            line.color.r, line.color.g, line.color.b, line.color.a = .2, .8, 1., .65
            line.points = [Point(x=float(p[0]), y=float(p[1]), z=.1)
                           for p in [self.bt.sweep.base, *self.bt.sweep.slots().values()]]
            markers.append(line)
        palette = [(1., .2, .2), (.2, 1., .2), (.2, .4, 1.), (1., .7, .1)]
        for index, name in enumerate(self.bt.robots):
            task = self.bt.active.get(name)
            if task is None:
                continue
            marker = Marker()
            marker.header.frame_id = self.frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type, marker.action = 'frontier_goals', index, Marker.SPHERE, Marker.ADD
            marker.pose.position = Point(x=float(task['goal'][0]), y=float(task['goal'][1]), z=.2)
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .3
            marker.color.r, marker.color.g, marker.color.b = palette[index % len(palette)]
            marker.color.a = 1.
            marker.text = name
            markers.append(marker)
            claim = task.get('claim')
            if claim:
                a, b, radius = claim['start'], claim['end'], claim['radius']
                angle = math.atan2(b[1]-a[1], b[0]-a[0])
                outline = []
                for center, offset in ((b, -math.pi/2), (a, math.pi/2)):
                    outline += [Point(x=float(center[0]+radius*math.cos(angle+offset+j*math.pi/24)),
                                      y=float(center[1]+radius*math.sin(angle+offset+j*math.pi/24)), z=.08)
                                for j in range(25)]
                outline.append(outline[0])
                area = Marker()
                area.header = marker.header
                area.ns, area.id = 'exploration_workspaces', index
                area.type, area.action = Marker.LINE_STRIP, Marker.ADD
                area.pose.orientation.w, area.scale.x = 1., .045
                area.color.r, area.color.g, area.color.b = palette[index % len(palette)]
                area.color.a, area.points = .65, outline
                markers.append(area)
        self.markers.publish(MarkerArray(markers=markers))


def run(factory, args=None):
    rclpy.init(args=args)
    node = factory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if isinstance(node, ExplorationRobot):
            node.nav.cancel()
            until = time.monotonic() + 2.
            while rclpy.ok() and node.nav.poll() in ('SENDING', 'NAVIGATING', 'CANCELING') and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def team_main(args=None):
    run(ExplorationTeam, args)


def robot_main(args=None):
    run(ExplorationRobot, args)
