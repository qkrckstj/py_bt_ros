"""One-way real ROS -> UDP -> isolated simulation ROS velocity bridge.

Run ``python3 -m webots_ros2_limo.twin_bridge source --target PC_IP`` on
the real robot after sourcing its normal ROS environment. This side creates
subscriptions and a TF listener only; it never publishes a motion command.
The sink must run in the isolated simulation ROS domain.
"""

import argparse
import ipaddress
import json
import math
import socket
import sys
import time

from .twin_protocol import (
    MAX_PACKET_BYTES, MirrorGate, PacketError, Pose, SourceState, Velocity, finite,
)


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return value


def udp_port(value):
    value = int(value)
    if not 1 <= value <= 65535:
        raise argparse.ArgumentTypeError('must be in 1..65535')
    return value


def ipv4(value):
    try:
        return str(ipaddress.IPv4Address(value))
    except ipaddress.AddressValueError as exc:
        raise argparse.ArgumentTypeError('use an IPv4 address') from exc


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='role', required=True)
    source = sub.add_parser('source', help='read real robot topics; send UDP only')
    source.add_argument('--target', type=ipv4, required=True, help='simulation PC IPv4')
    source.add_argument('--port', type=udp_port, default=17440)
    source.add_argument('--velocity-source', choices=('command', 'odometry'),
                        default='odometry', help='odometry mirrors both RC and ROS driving')
    source.add_argument('--odom-topic', default='/odom')
    source.add_argument('--cmd-topic', default='/cmd_vel')
    source.add_argument('--cmd-type', choices=('twist', 'stamped'), default='twist')
    source.add_argument('--map-frame', default='map')
    source.add_argument('--base-frame', default='base_link')
    source.add_argument('--rate', type=positive, default=20.0)
    source.add_argument('--max-age', type=positive, default=1.0)
    sink = sub.add_parser('sink', help='publish commands only into simulation')
    sink.add_argument('--robot-ip', type=ipv4, required=True)
    sink.add_argument('--listen', type=ipv4, default='0.0.0.0')
    sink.add_argument('--port', type=udp_port, default=17440)
    sink.add_argument('--map-frame', default='map')
    sink.add_argument('--rate', type=positive, default=20.0)
    sink.add_argument('--watchdog', type=positive, default=0.5)
    sink.add_argument('--max-age', type=positive, default=1.0)
    sink.add_argument('--pose-max-age', type=positive, default=1.0)
    sink.add_argument('--position-tolerance', type=positive, default=0.25)
    sink.add_argument('--yaw-tolerance', type=positive, default=0.26)
    sink.add_argument('--max-linear', type=positive, default=1.0)
    sink.add_argument('--max-angular', type=positive, default=4.0)
    return parser.parse_args(argv)


def quaternion_yaw(q):
    values = [finite(getattr(q, key), f'orientation {key}') for key in ('x', 'y', 'z', 'w')]
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9 or abs(norm - 1.0) > 0.05:
        raise PacketError('invalid orientation quaternion')
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_from_message(message):
    return Pose(finite(message.position.x, 'pose x'),
                finite(message.position.y, 'pose y'), quaternion_yaw(message.orientation))


def velocity_from_message(message):
    # Out-of-plane motion cannot be represented by this planar LIMO twin.
    for value in (message.linear.z, message.angular.x, message.angular.y):
        if abs(finite(value, 'out-of-plane velocity')) > 1e-4:
            raise PacketError('out-of-plane velocity is unsupported')
    return Velocity(finite(message.linear.x, 'linear x'),
                    finite(message.linear.y, 'linear y'),
                    finite(message.angular.z, 'angular z'))


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def odometry_velocity(message, base_frame, wall_time, max_age):
    # Odometry twist is expressed in child_frame_id, not header.frame_id.
    if message.child_frame_id != base_frame:
        raise PacketError('odometry velocity must be in the configured base frame')
    age = wall_time - stamp_seconds(message.header.stamp)
    if not math.isfinite(age) or age < -0.1 or age > max_age:
        raise PacketError('odometry is stale or clock differs')
    velocity = velocity_from_message(message.twist.twist)
    # Chassis feedback is quantized to 0.001; suppress tiny standstill noise.
    velocity = Velocity(*(0.0 if abs(v) <= .002 else v
                          for v in (velocity.x, velocity.y, velocity.yaw)))
    return velocity, max(0.0, age)


def accept_datagram(gate, data, sender_ip, expected_ip, now, wall_time):
    # IP filtering avoids mixing multiple robots, but is not authentication.
    # Use this protocol only on the robot's trusted LAN or a private VPN.
    if sender_ip != expected_ip:
        return False
    return gate.receive(data, now, wall_time)


def make_source_node(args):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener, TransformException

    class SourceNode(Node):
        def __init__(self):
            super().__init__('digital_twin_source')
            if self.get_parameter('use_sim_time').value:
                raise RuntimeError('The real source must use use_sim_time=false')
            self.state = SourceState()
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setblocking(False)
            self.destination = (args.target, args.port)
            self.tf_buffer = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf_buffer, self)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            if args.velocity_source == 'odometry':
                topic = args.odom_topic
                self.subscription = self.create_subscription(
                    Odometry, topic, self.on_odometry, qos)
            else:
                topic = args.cmd_topic
                msg_type = Twist if args.cmd_type == 'twist' else TwistStamped
                self.subscription = self.create_subscription(msg_type, topic,
                                                             self.on_command, qos)
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.heartbeat = self.create_timer(1.0 / args.rate, self.send,
                                               clock=self.steady_clock)
            self.last_warning = 0.0
            self.get_logger().info(
                f'Read-only source ({args.velocity_source}): {topic} -> UDP {args.target}:{args.port}; '
                f'TF {args.map_frame} -> {args.base_frame}. Synchronize both PC clocks.')

        def warn_throttled(self, message):
            now = time.monotonic()
            if now - self.last_warning > 5.0:
                self.last_warning = now
                self.get_logger().warning(message)

        def on_command(self, message):
            try:
                age = 0.0
                if args.cmd_type == 'stamped':
                    age = time.time() - stamp_seconds(message.header.stamp)
                    if age < -0.1 or age > args.max_age:
                        raise PacketError('stamped command is stale or clock differs')
                    message = message.twist
                self.state.command(velocity_from_message(message), time.monotonic(),
                                   max(0.0, age))
            except PacketError as exc:
                self.state.invalidate_command()
                self.warn_throttled(f'Ignoring real command: {exc}')

        def on_odometry(self, message):
            try:
                velocity, age = odometry_velocity(message, args.base_frame,
                                                  time.time(), args.max_age)
                self.state.command(velocity, time.monotonic(), age)
            except PacketError as exc:
                self.state.invalidate_command()
                self.warn_throttled(f'Ignoring real velocity feedback: {exc}')

        def send(self):
            pose, age = None, None
            try:
                transform = self.tf_buffer.lookup_transform(
                    args.map_frame, args.base_frame, rclpy.time.Time())
                age = time.time() - stamp_seconds(transform.header.stamp)
                if age < -0.1 or age > args.max_age:
                    raise PacketError('real map pose is stale or clock differs')
                t = transform.transform
                pose = Pose(finite(t.translation.x, 'TF x'),
                            finite(t.translation.y, 'TF y'), quaternion_yaw(t.rotation))
                age = max(0.0, age)
            except (TransformException, PacketError) as exc:
                self.warn_throttled(f'Real pose unavailable; simulation will stay stopped: {exc}')
                pose, age = None, None
            data = self.state.encode(time.monotonic(), time.time(), pose, age, args.map_frame)
            try:
                self.socket.sendto(data, self.destination)
            except (BlockingIOError, OSError) as exc:
                self.warn_throttled(f'UDP send failed: {exc}')

        def destroy_node(self):
            self.socket.close()
            return super().destroy_node()

    return SourceNode()


def make_sink_node(args):
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist, PoseStamped
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock as ClockMessage

    class SinkNode(Node):
        def __init__(self):
            super().__init__('digital_twin_sink')
            # The explicit steady timer below is independent of use_sim_time.
            self.gate = MirrorGate(watchdog=args.watchdog, max_age=args.max_age,
                                   pose_max_age=args.pose_max_age,
                                   position_tolerance=args.position_tolerance,
                                   yaw_tolerance=args.yaw_tolerance,
                                   max_linear=args.max_linear, max_angular=args.max_angular,
                                   frame=args.map_frame)
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((args.listen, args.port))
            self.socket.setblocking(False)
            self.command_pub = self.create_publisher(Twist, '/cmd_vel', 1)
            self.pose_pub = self.create_publisher(PoseStamped, '/digital_twin/real_pose', 1)
            self.status_pub = self.create_publisher(String, '/digital_twin/status', 1)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.pose_sub = self.create_subscription(PoseStamped, '/digital_twin/sim_pose',
                                                     self.on_sim_pose, qos)
            self.clock_sub = self.create_subscription(ClockMessage, '/clock', self.on_clock, qos)
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.loop_timer = self.create_timer(1.0 / args.rate, self.tick,
                                                clock=self.steady_clock)
            self.last_status = None
            self.last_pose_key = None
            self.get_logger().info(
                f'Simulation-only sink listening UDP {args.listen}:{args.port} '
                f'from {args.robot_ip}; waiting for clock, matching poses and a fresh zero command.')

        def on_sim_pose(self, message):
            try:
                if message.header.frame_id != args.map_frame:
                    raise PacketError('sim_pose must be in the map frame')
                self.gate.update_sim_pose(pose_from_message(message.pose), time.monotonic())
            except PacketError as exc:
                self.gate.sim_pose_received = None
                self.gate.disarm(f'invalid_sim_pose: {exc}')

        def on_clock(self, message):
            self.gate.update_clock(stamp_seconds(message.clock), time.monotonic())

        def tick(self):
            # Drain bounded batches without executing queued historical commands.
            # evaluate/publish runs once after the newest accepted packet.
            for _ in range(256):
                try:
                    data, sender = self.socket.recvfrom(MAX_PACKET_BYTES + 1)
                except BlockingIOError:
                    break
                except OSError as exc:
                    self.gate.reject(f'udp_receive_error: {exc}')
                    break
                accept_datagram(self.gate, data, sender[0], args.robot_ip,
                                time.monotonic(), time.time())
            else:
                self.gate.reject('udp_backlog_overflow')
            velocity, status = self.gate.evaluate(time.monotonic(), time.time())
            command = Twist()
            command.linear.x, command.linear.y = velocity.x, velocity.y
            command.angular.z = velocity.yaw
            self.command_pub.publish(command)
            packet = self.gate.packet
            if (packet and packet.pose and status['real_pose_age'] is not None and
                    status['real_pose_age'] <= args.pose_max_age):
                key = (packet.session, packet.sequence)
                if key != self.last_pose_key:
                    pose = PoseStamped()
                    pose.header.frame_id = args.map_frame
                    observed_at = max(0.0, packet.sent_at - packet.pose_age)
                    pose.header.stamp.sec = int(observed_at)
                    pose.header.stamp.nanosec = min(999999999, int((observed_at % 1) * 1e9))
                    pose.pose.position.x, pose.pose.position.y = packet.pose.x, packet.pose.y
                    pose.pose.orientation.z = math.sin(packet.pose.yaw / 2)
                    pose.pose.orientation.w = math.cos(packet.pose.yaw / 2)
                    self.pose_pub.publish(pose)
                    self.last_pose_key = key
            message = String()
            message.data = json.dumps(status, allow_nan=False, separators=(',', ':'))
            self.status_pub.publish(message)
            if status['state'] != self.last_status:
                self.get_logger().info(f"Digital twin: {status['state']}")
                self.last_status = status['state']

        def destroy_node(self):
            # Best effort zero on normal exit; the simulation drive also needs
            # its own cmd_vel timeout for process kill or machine failure.
            try:
                if self.context.ok():
                    self.command_pub.publish(Twist())
            finally:
                self.socket.close()
            return super().destroy_node()

    return SinkNode()


def main(argv=None):
    import rclpy
    from rclpy.utilities import remove_ros_args
    from rclpy.executors import ExternalShutdownException

    raw = sys.argv if argv is None else ['twin_bridge'] + list(argv)
    args = parse_args(remove_ros_args(args=raw)[1:])
    rclpy.init(args=raw[1:])
    node = None
    try:
        node = make_source_node(args) if args.role == 'source' else make_sink_node(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
