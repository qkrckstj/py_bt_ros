"""Allowlisted simulated peer data -> /sim/... on the real robot.

CDR keeps the original simulated timestamps. /sim/clock and /sim/tf stay separate
from the real robot's clock/TF. No drive topic or action is relayed.
"""
import argparse
import base64
import json
import socket
import struct
import threading
import time

MAX_FRAME = 4 * 1024 * 1024
TOPICS = {'/clock': 'rosgraph_msgs/msg/Clock',
          '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage'}
for _robot in ('robot2', 'robot3'):
    for _suffix, _type in {
        'odom': 'nav_msgs/msg/Odometry', 'scan': 'sensor_msgs/msg/LaserScan',
        'camera/color/image_raw/compressed': 'sensor_msgs/msg/CompressedImage',
        'camera/color/camera_info': 'sensor_msgs/msg/CameraInfo',
        'human/detected': 'std_msgs/msg/Bool',
        'human/position_valid': 'std_msgs/msg/Bool',
        'human/position': 'geometry_msgs/msg/PointStamped',
    }.items():
        TOPICS[f'/{_robot}/{_suffix}'] = _type


def encode(topic, data, now):
    if topic not in TOPICS:
        raise ValueError('topic not permitted')
    body = json.dumps({'topic': topic, 'sent_at': now,
                       'data': base64.b64encode(data).decode('ascii')}).encode()
    if len(body) > MAX_FRAME:
        raise ValueError('peer frame too large')
    return struct.pack('!I', len(body)) + body


def decode(body, now):
    if len(body) > MAX_FRAME:
        raise ValueError('peer frame too large')
    obj = json.loads(body)
    if obj['topic'] not in TOPICS:
        raise ValueError('topic not permitted')
    if not -.1 <= now - obj['sent_at'] <= 1.:
        raise ValueError('peer frame stale or clock differs')
    return obj['topic'], base64.b64decode(obj['data'], validate=True)


def filter_tf(message):
    message.transforms = [tf for tf in message.transforms
                          if tf.child_frame_id.startswith(('robot2/', 'robot3/'))
                          and (tf.header.frame_id == 'map' or
                               tf.header.frame_id.startswith(('robot2/', 'robot3/')))]
    return message


def main(argv=None):
    import rclpy
    from rclpy.utilities import remove_ros_args
    from rclpy.serialization import serialize_message, deserialize_message
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import String
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=['send', 'receive'])
    parser.add_argument('--robot-ip', default='192.168.0.131')
    parser.add_argument('--peer-ip', help='Allowed laptop IP, required for receiver')
    parser.add_argument('--port', type=int, default=17441)
    args = parser.parse_args(remove_ros_args(args=argv)[1:] if argv is None else argv)
    if args.role == 'receive' and not args.peer_ip:
        parser.error('--peer-ip is required')
    rclpy.init()
    node = rclpy.create_node('peer_link', namespace='robot1' if args.role == 'send' else 'sim')
    types = {topic: get_message(kind) for topic, kind in TOPICS.items()}
    lock, stopped = threading.Lock(), threading.Event()
    pending, received, counts, last = {}, {}, {}, {}
    tf_cache = {'/tf': {}, '/tf_static': {}}
    stats = {'connected': False, 'error': '', 'rejected': 0}
    publishers, subscriptions = {}, []
    for topic in TOPICS:
        qos = QoSProfile(depth=1)
        if topic == '/tf_static':
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        publishers[topic] = node.create_publisher(types[topic], '/sim' + topic, qos)
    status = node.create_publisher(String, 'status', 1)

    def callback(topic, message):
        now = time.time()
        if topic in tf_cache:
            for tf in filter_tf(message).transforms:
                tf_cache[topic][tf.child_frame_id] = tf
            message.transforms = list(tf_cache[topic].values())
            if not message.transforms:
                return
        # Bounded latest-only queues; camera traffic cannot accumulate history.
        interval = .2 if 'compressed' in topic else .05
        if now - last.get(topic, 0.) < interval:
            return
        last[topic] = now
        publishers[topic].publish(message)  # Replica-side peer inbox.
        with lock:
            pending[topic] = (serialize_message(message), now)

    if args.role == 'send':
        for topic in TOPICS:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            if topic == '/tf_static':
                qos.reliability = ReliabilityPolicy.RELIABLE
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
                qos.depth = 100  # Several independent latched mount publishers.
            subscriptions.append(node.create_subscription(
                types[topic], topic, lambda msg, topic=topic: callback(topic, msg), qos))

    def sender():
        while not stopped.is_set():
            try:
                with socket.create_connection((args.robot_ip, args.port), timeout=.5) as sock:
                    stats.update(connected=True, error='')
                    while not stopped.is_set():
                        with lock:
                            batch = list(pending.items())
                            pending.clear()
                        for topic, (data, stamp) in batch:
                            if time.time() - stamp < .5:
                                sock.sendall(encode(topic, data, stamp))
                        stopped.wait(.02)
            except (OSError, ValueError) as exc:
                stats.update(connected=False, error=str(exc))
                stopped.wait(.5)

    def receiver():
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', args.port))
            server.listen(2)
            server.settimeout(.2)
            while not stopped.is_set():
                try:
                    sock, address = server.accept()
                except socket.timeout:
                    continue
                with sock:
                    if address[0] != args.peer_ip:
                        continue
                    sock.settimeout(.2)
                    stats.update(connected=True, error='')
                    buffer = bytearray()
                    try:
                        while not stopped.is_set():
                            try:
                                data = sock.recv(65536)
                            except socket.timeout:
                                continue
                            if not data:
                                break
                            buffer.extend(data)
                            while len(buffer) >= 4:
                                size = struct.unpack('!I', buffer[:4])[0]
                                if not 0 < size <= MAX_FRAME:
                                    raise ValueError('invalid frame length')
                                if len(buffer) < size + 4:
                                    break
                                body = bytes(buffer[4:size + 4])
                                del buffer[:size + 4]
                                topic, cdr = decode(body, time.time())
                                message = deserialize_message(cdr, types[topic])
                                if topic in ('/tf', '/tf_static'):
                                    message = filter_tf(message)
                                with lock:
                                    received[topic] = (message, time.time())
                    except Exception as exc:
                        stats.update(error=str(exc), rejected=stats['rejected'] + 1)
                    finally:
                        stats['connected'] = False

    thread = threading.Thread(target=sender if args.role == 'send' else receiver, daemon=True)
    thread.start()
    last_status = [0.]

    def tick():
        if args.role == 'send' and tf_cache['/tf_static'] and time.time() - last.get('/tf_static', 0.) >= .5:
            # Retain all static mounts and resend after a network reconnect.
            message = types['/tf_static']()
            message.transforms = list(tf_cache['/tf_static'].values())
            callback('/tf_static', message)
        with lock:
            batch = list(received.items())
            received.clear()
        for topic, (message, stamp) in batch:
            if time.time() - stamp < 1.:
                publishers[topic].publish(message)
                counts[topic] = counts.get(topic, 0) + 1
        if time.monotonic() - last_status[0] >= 1.:
            last_status[0] = time.monotonic()
            message = String()
            message.data = json.dumps({**stats, 'received_counts': counts})
            status.publish(message)
    node.create_timer(.02, tick)
    node.get_logger().info(f'Peer relay {args.role}: only robot2/robot3 data into /sim/*; no real drive output')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stopped.set()
        thread.join(timeout=1.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
