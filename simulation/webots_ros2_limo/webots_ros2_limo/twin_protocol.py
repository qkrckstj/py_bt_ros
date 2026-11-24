"""Small, ROS-independent wire protocol and fail-closed digital-twin gate.

Only planar velocities cross the wire. The receiver never sends commands back
to the source. Wall time detects delayed UDP packets; monotonic time controls
all local watchdogs, even when Webots is paused or its clock is reset.
"""

from collections import deque
from dataclasses import dataclass
import json
import math
import uuid


VERSION = 1
MAX_PACKET_BYTES = 4096


class PacketError(ValueError):
    pass


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PacketError(f'{name} must be a number')
    value = float(value)
    if not math.isfinite(value):
        raise PacketError(f'{name} must be finite')
    return value


def counter(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise PacketError(f'{name} must be a nonnegative integer')
    return value


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


@dataclass(frozen=True)
class Velocity:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    @property
    def stopped(self):
        return max(abs(self.x), abs(self.y), abs(self.yaw)) <= 1e-4


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Packet:
    session: str
    sequence: int
    sent_at: float
    command_sequence: int
    command_age: float | None
    velocity: Velocity
    pose: Pose | None
    pose_age: float | None
    frame: str


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise PacketError(f'duplicate key: {key}')
        result[key] = value
    return result


def decode_packet(data, expected_frame='map'):
    if not isinstance(data, bytes) or len(data) > MAX_PACKET_BYTES:
        raise PacketError('invalid packet size/type')
    try:
        obj = json.loads(data.decode('utf-8'), object_pairs_hook=_unique_object)
        if obj['version'] != VERSION or isinstance(obj['version'], bool):
            raise PacketError('unsupported version')
        session = obj['session']
        if not isinstance(session, str) or uuid.UUID(session).hex != session:
            raise PacketError('session must be a UUID hex string')
        command = obj['command']
        active = command['active']
        if not isinstance(active, bool):
            raise PacketError('active must be boolean')
        velocity = Velocity(finite(command['linear_x'], 'linear_x'),
                            finite(command['linear_y'], 'linear_y'),
                            finite(command['angular_z'], 'angular_z'))
        age = finite(command['age'], 'command age') if active else None
        if age is not None and age < 0:
            raise PacketError('negative command age')
        if not active and (not velocity.stopped or command['age'] is not None):
            raise PacketError('inactive command must be zero with null age')
        pose_obj = obj['pose']
        pose, pose_age, frame = None, None, expected_frame
        if pose_obj is not None:
            frame = pose_obj['frame']
            if frame != expected_frame:
                raise PacketError(f'pose frame must be {expected_frame}')
            pose = Pose(finite(pose_obj['x'], 'pose x'),
                        finite(pose_obj['y'], 'pose y'),
                        finite(pose_obj['yaw'], 'pose yaw'))
            pose_age = finite(pose_obj['age'], 'pose age')
            if pose_age < 0:
                raise PacketError('negative pose age')
        return Packet(session, counter(obj['sequence'], 'sequence'),
                      finite(obj['sent_at'], 'sent_at'),
                      counter(command['sequence'], 'command sequence'),
                      age, velocity, pose, pose_age, frame)
    except (KeyError, TypeError, ValueError, UnicodeError, AttributeError,
            OverflowError, RecursionError) as exc:
        if isinstance(exc, PacketError):
            raise
        raise PacketError(f'malformed packet: {type(exc).__name__}') from exc


class SourceState:
    """Cache command receipt time; heartbeat transmission never refreshes it."""

    def __init__(self, session=None):
        self.session = session or uuid.uuid4().hex
        self.sequence = 0
        self.command_sequence = 0
        self.velocity = Velocity()
        self.command_received = None
        self.initial_command_age = 0.0

    def command(self, velocity, now, initial_age=0.0):
        for value in (velocity.x, velocity.y, velocity.yaw, now, initial_age):
            finite(value, 'command value')
        if initial_age < 0:
            raise PacketError('negative initial age')
        self.command_sequence += 1
        self.velocity = velocity
        self.command_received = now
        self.initial_command_age = initial_age

    def invalidate_command(self):
        self.command_sequence += 1
        self.velocity = Velocity()
        self.command_received = None

    def encode(self, now, wall_time, pose=None, pose_age=None, frame='map'):
        self.sequence += 1
        active = self.command_received is not None
        obj = {
            'version': VERSION, 'session': self.session,
            'sequence': self.sequence, 'sent_at': wall_time,
            'command': {
                'sequence': self.command_sequence, 'active': active,
                'age': max(0.0, now - self.command_received) + self.initial_command_age
                if active else None,
                'linear_x': self.velocity.x, 'linear_y': self.velocity.y,
                'angular_z': self.velocity.yaw,
            },
            'pose': None if pose is None else {
                'frame': frame, 'x': pose.x, 'y': pose.y,
                'yaw': pose.yaw, 'age': pose_age,
            },
        }
        return json.dumps(obj, allow_nan=False, separators=(',', ':')).encode()


class MirrorGate:
    """Require fresh streams, aligned start and a new zero command to arm.

    After arming, pose drift is reported rather than corrected. A reset, stale
    feed, invalid datagram or clock pause disarms and requires another zero
    command and alignment. A heartbeat of an old zero command cannot rearm.
    """

    def __init__(self, watchdog=0.5, max_age=1.0, pose_max_age=1.0,
                 position_tolerance=0.25, yaw_tolerance=0.26,
                 clock_skew_tolerance=0.1, max_linear=1.0, max_angular=4.0,
                 frame='map'):
        for value in (watchdog, max_age, pose_max_age, position_tolerance,
                      yaw_tolerance, clock_skew_tolerance, max_linear, max_angular):
            if finite(value, 'gate parameter') <= 0:
                raise ValueError('gate parameters must be positive')
        self.watchdog, self.max_age, self.pose_max_age = watchdog, max_age, pose_max_age
        self.position_tolerance, self.yaw_tolerance = position_tolerance, yaw_tolerance
        self.clock_skew_tolerance = clock_skew_tolerance
        self.max_linear, self.max_angular, self.frame = max_linear, max_angular, frame
        self.packet = None
        self.packet_received = None
        self.packet_delay = 0.0
        self.session = None
        self.retired_sessions = set()
        self.last_sequence = -1
        self.last_command_sequence = -1
        self.armed = False
        self.zero_after = 0
        self.reason = 'waiting_for_packet'
        self.sim_pose = None
        self.sim_pose_received = None
        self.clock_value = None
        self.clock_received = None
        self.clock_progress = None
        self.clock_samples = deque()
        self.last_wall = None
        self.last_monotonic = None
        self.rejected_packets = 0

    def disarm(self, reason):
        if self.armed:
            self.zero_after = max(self.zero_after, self.last_command_sequence)
        self.armed = False
        self.reason = reason

    def reject(self, reason):
        self.rejected_packets += 1
        self.disarm(reason)
        self.zero_after = max(self.zero_after, self.last_command_sequence)
        self.packet_received = None
        return False

    def receive(self, data, now, wall_time):
        try:
            packet = decode_packet(data, self.frame)
        except PacketError as exc:
            return self.reject(f'invalid_packet: {exc}')
        delay = wall_time - packet.sent_at
        if delay < -self.clock_skew_tolerance:
            return self.reject('clock_skew: source clock is ahead; synchronize clocks')
        if delay > self.max_age:
            return self.reject('packet_too_old_or_clock_skew: synchronize clocks')
        if packet.session in self.retired_sessions:
            return self.reject('retired_session_replay')
        if packet.session == self.session:
            if packet.sequence <= self.last_sequence:
                return self.reject('out_of_order_or_replayed_packet')
            if packet.command_sequence < self.last_command_sequence:
                return self.reject('command_sequence_regressed')
            if (self.packet is not None and
                    packet.command_sequence == self.last_command_sequence and
                    (packet.velocity != self.packet.velocity or
                     (packet.command_age is None) != (self.packet.command_age is None))):
                return self.reject('command_changed_without_new_sequence')
        else:
            if self.session is not None:
                self.retired_sessions.add(self.session)
            self.disarm('new_source_session')
            self.session = packet.session
            self.zero_after = 0
            self.last_sequence = self.last_command_sequence = -1
        if math.hypot(packet.velocity.x, packet.velocity.y) > self.max_linear:
            return self.reject('linear_velocity_exceeds_limit')
        if abs(packet.velocity.yaw) > self.max_angular:
            return self.reject('angular_velocity_exceeds_limit')
        self.packet = packet
        self.packet_received = now
        self.packet_delay = max(0.0, delay)
        self.last_sequence = packet.sequence
        self.last_command_sequence = packet.command_sequence
        return True

    def update_sim_pose(self, pose, now):
        try:
            for value in (pose.x, pose.y, pose.yaw):
                finite(value, 'sim pose')
        except PacketError:
            self.sim_pose_received = None
            self.disarm('invalid_sim_pose')
            return
        self.sim_pose, self.sim_pose_received = pose, now

    def update_clock(self, sim_time, now):
        try:
            finite(sim_time, 'clock')
            if sim_time < 0:
                raise PacketError('negative clock')
        except PacketError:
            self.clock_progress = None
            self.disarm('invalid_sim_clock')
            return
        previous, previous_time = self.clock_value, self.clock_received
        self.clock_value, self.clock_received = sim_time, now
        if previous is None:
            return
        delta, elapsed = sim_time - previous, max(0.0, now - previous_time)
        if delta < -1e-6 or delta > max(2.0, elapsed * 3.0 + 0.2):
            self.disarm('sim_clock_reset_or_jump')
            self.zero_after = max(self.zero_after, self.last_command_sequence)
            self.clock_progress = None
            self.clock_samples.clear()
        elif delta > 0 and sim_time > 0:
            self.clock_progress = now
            self.clock_samples.append((now, sim_time))
            while len(self.clock_samples) > 2 and self.clock_samples[0][0] < now - 2.0:
                self.clock_samples.popleft()

    def _age(self, source_age, now):
        if source_age is None or self.packet_received is None:
            return None
        return source_age + self.packet_delay + max(0.0, now - self.packet_received)

    def evaluate(self, now, wall_time):
        if self.last_wall is not None:
            jump = (wall_time - self.last_wall) - (now - self.last_monotonic)
            if abs(jump) > self.clock_skew_tolerance:
                self.reject('local_wall_clock_jump: synchronize clocks')
        self.last_wall, self.last_monotonic = wall_time, now
        packet = self.packet
        command_age = self._age(packet.command_age, now) if packet else None
        pose_age = self._age(packet.pose_age, now) if packet else None
        distance = yaw_error = None
        if packet and packet.pose and self.sim_pose:
            distance = math.hypot(packet.pose.x - self.sim_pose.x,
                                  packet.pose.y - self.sim_pose.y)
            yaw_error = abs(angle_difference(packet.pose.yaw, self.sim_pose.yaw))
        failure = None
        if self.packet_received is None:
            failure = self.reason if self.rejected_packets else 'waiting_for_packet'
        elif now - self.packet_received > self.watchdog:
            failure = 'source_watchdog_timeout'
        elif command_age is None:
            failure = 'waiting_for_real_command'
        elif command_age > self.max_age:
            failure = 'real_command_stale'
        elif self.clock_progress is None or now - self.clock_progress > self.watchdog:
            failure = 'sim_clock_not_running'
        elif self.sim_pose_received is None or now - self.sim_pose_received > self.pose_max_age:
            failure = 'sim_pose_stale'
        elif packet.pose is None or pose_age is None or pose_age > self.pose_max_age:
            failure = 'real_pose_unavailable_or_stale'
        if failure:
            self.disarm(failure)
        elif not self.armed:
            if distance > self.position_tolerance or yaw_error > self.yaw_tolerance:
                self.reason = 'waiting_for_initial_alignment'
            elif not packet.velocity.stopped or packet.command_sequence <= self.zero_after:
                self.reason = 'waiting_for_new_zero_command'
            else:
                self.armed, self.reason = True, 'armed'
        velocity = packet.velocity if self.armed else Velocity()
        rtf = None
        if len(self.clock_samples) >= 2:
            first, last = self.clock_samples[0], self.clock_samples[-1]
            if last[0] > first[0]:
                rtf = (last[1] - first[1]) / (last[0] - first[0])
        status = {
            'armed': self.armed, 'state': self.reason, 'session': self.session,
            'packet_sequence': self.last_sequence,
            'command_age': command_age, 'real_pose_age': pose_age,
            'position_drift_m': distance, 'yaw_drift_rad': yaw_error,
            'real_time_factor': rtf, 'rejected_packets': self.rejected_packets,
            'velocity': {'linear_x': velocity.x, 'linear_y': velocity.y,
                         'angular_z': velocity.yaw},
        }
        return velocity, status
