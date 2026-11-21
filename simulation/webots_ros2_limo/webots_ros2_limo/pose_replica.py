"""Mirror real map poses; never integrate velocity or use simulated sensors."""
import json
import math
import socket
import time
from .twin_protocol import MAX_PACKET_BYTES, PacketError, decode_packet


class PoseFeed:
    def __init__(self, robot_ip, max_age=.5):
        self.robot_ip, self.max_age = robot_ip, max_age
        self.packet = None
        self.session = None
        self.sequence = -1
        self.retired = set()
        self.received = None
        self.reason = 'waiting_for_real_pose'
        self.rejected = 0

    def receive(self, data, sender, now, wall):
        try:
            if sender != self.robot_ip:
                raise PacketError('unexpected_source')
            packet = decode_packet(data)
            transport_age = wall - packet.sent_at
            if not -.1 <= transport_age <= self.max_age:
                raise PacketError('packet_stale_or_clock_skew')
            if packet.session in self.retired:
                raise PacketError('retired_session')
            if packet.session == self.session and packet.sequence <= self.sequence:
                raise PacketError('out_of_order')
            if self.session and packet.session != self.session:
                self.retired.add(self.session)
            self.session, self.sequence = packet.session, packet.sequence
            self.received = now
            if packet.pose is None or packet.pose_age + max(0., transport_age) > self.max_age:
                self.packet = None
                self.reason = 'real_pose_unavailable'
                return False
            self.packet = packet
            self.reason = 'following_real_pose'
            return True
        except PacketError as exc:
            self.rejected += 1
            # A bad/reordered packet must not erase a newer valid pose.
            return False

    def latest(self, now, wall):
        if self.received is None or now - self.received > self.max_age:
            self.reason = 'waiting_for_real_pose' if self.received is None else 'connection_lost'
            return None
        if self.packet and self.packet.pose_age + max(0., wall - self.packet.sent_at) <= self.max_age:
            return self.packet
        self.reason = 'real_pose_stale'
        return None


class PoseReplica:
    def init(self, webots_node, properties):
        import rclpy
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from std_msgs.msg import String
        from tf2_ros import TransformBroadcaster
        self.robot = webots_node.robot
        # The PROTO has no devices, so neither Webots nor ROS can produce a
        # synthetic sensor measurement for this robot.
        if self.robot.getNumberOfDevices() != 0:
            raise RuntimeError('Pose replica must have zero Webots devices')
        self.body = self.robot.getSelf()
        self.translation = self.body.getField('translation')
        self.rotation = self.body.getField('rotation')
        self.height = self.translation.getSFVec3f()[2]
        self.feed = PoseFeed(properties['robot_ip'])
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', int(properties.get('port', 17440))))
        self.sock.setblocking(False)
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('pose_replica', namespace='robot1', use_global_arguments=False)
        self.pose_pub = self.node.create_publisher(PoseStamped, '/robot1/pose', 1)
        self.status_pub = self.node.create_publisher(String, '/digital_twin/status', 1)
        self.tf = TransformBroadcaster(self.node)
        self.PoseStamped, self.TransformStamped, self.String = PoseStamped, TransformStamped, String
        self.last_status = 0.
        self.applied = None
        self.node.get_logger().info('robot1: real pose replica, zero simulated devices, no cmd_vel input')

    def step(self):
        now, wall = time.monotonic(), time.time()
        for _ in range(256):
            try:
                data, address = self.sock.recvfrom(MAX_PACKET_BYTES + 1)
            except BlockingIOError:
                break
            self.feed.receive(data, address[0], now, wall)
        packet = self.feed.latest(now, wall)
        if packet:
            pose = packet.pose
            self.translation.setSFVec3f([pose.x, pose.y, self.height])
            self.rotation.setSFRotation([0., 0., 1., pose.yaw])
            self.applied = (pose.x, pose.y, pose.yaw)
            msg = self.PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(self.robot.getTime() * 1e9), 10**9)
            msg.pose.position.x, msg.pose.position.y = pose.x, pose.y
            msg.pose.position.z = self.height
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(pose.yaw / 2), math.cos(pose.yaw / 2)
            self.pose_pub.publish(msg)
            tf = self.TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = 'robot1/base_link'
            tf.transform.translation.x, tf.transform.translation.y = pose.x, pose.y
            tf.transform.translation.z = self.height
            tf.transform.rotation = msg.pose.orientation
            self.tf.sendTransform(tf)
        if now - self.last_status >= .2:
            self.last_status = now
            msg = self.String()
            actual = self.body.getPosition()
            msg.data = json.dumps({
                'mode': 'pose_replica', 'state': self.feed.reason,
                'sensor_devices': self.robot.getNumberOfDevices(),
                'real_pose': self.applied, 'webots_position': actual,
                'pose_age': None if not packet else packet.pose_age + max(0., wall - packet.sent_at),
                'packet_sequence': self.feed.sequence, 'rejected_packets': self.feed.rejected,
            })
            self.status_pub.publish(msg)
