import json
import math

import pytest

from webots_ros2_limo.twin_protocol import MirrorGate, PacketError, Pose, SourceState, Velocity, decode_packet
from webots_ros2_limo.twin_bridge import accept_datagram, parse_args


def ready():
    gate, source = MirrorGate(), SourceState()
    gate.update_clock(1., 10.)
    gate.update_clock(1.1, 10.1)
    gate.update_sim_pose(Pose(2., 3., .4), 10.1)
    source.command(Velocity(), 10.1)
    assert gate.receive(source.encode(10.1, 100., Pose(2., 3., .4), 0.), 10.1, 100.)
    assert gate.evaluate(10.1, 100.)[1]['armed']
    return gate, source


def test_aligned_zero_arms_and_velocity_is_preserved():
    gate, source = ready()
    source.command(Velocity(.2, 0., -.3), 10.2)
    gate.receive(source.encode(10.2, 100.1, Pose(2., 3., .4), 0.), 10.2, 100.1)
    velocity, status = gate.evaluate(10.2, 100.1)
    assert velocity == Velocity(.2, 0., -.3)
    assert status['position_drift_m'] == 0.


def test_real_pose_drift_is_reported_not_corrected():
    gate, source = ready()
    source.command(Velocity(.2), 10.2)
    gate.receive(source.encode(10.2, 100.1, Pose(5., 7., .4), 0.), 10.2, 100.1)
    velocity, status = gate.evaluate(10.2, 100.1)
    assert status['armed'] and status['position_drift_m'] == 5.
    assert velocity.x == .2


def test_heartbeat_cannot_refresh_old_command():
    gate, source = ready()
    for now in (10.5, 11., 11.5):
        gate.update_clock(now - 9., now)
        gate.update_sim_pose(Pose(2., 3., .4), now)
        gate.receive(source.encode(now, now + 89.9, Pose(2., 3., .4), 0.), now, now + 89.9)
        velocity, status = gate.evaluate(now, now + 89.9)
    assert velocity.stopped
    assert status['state'] == 'real_command_stale'


def test_disconnect_stops():
    gate, _ = ready()
    velocity, status = gate.evaluate(10.7, 100.6)
    assert velocity.stopped and status['state'] == 'source_watchdog_timeout'


def test_pause_requires_new_zero_and_alignment():
    gate, source = ready()
    gate.receive(source.encode(10.7, 100.6, Pose(2., 3., .4), 0.), 10.7, 100.6)
    assert gate.evaluate(10.7, 100.6)[1]['state'] == 'sim_clock_not_running'
    gate.update_clock(1.2, 10.71)
    gate.update_sim_pose(Pose(2., 3., .4), 10.71)
    assert not gate.evaluate(10.71, 100.61)[1]['armed']
    source.command(Velocity(), 10.72)
    gate.receive(source.encode(10.72, 100.62, Pose(2., 3., .4), 0.), 10.72, 100.62)
    assert gate.evaluate(10.72, 100.62)[1]['armed']


def test_reset_disarms():
    gate, _ = ready()
    gate.update_clock(0., 10.2)
    assert not gate.evaluate(10.2, 100.1)[1]['armed']


@pytest.mark.parametrize('wall', [98., 103.])
def test_clock_skew_or_delayed_packet_stops(wall):
    gate, source = ready()
    assert not gate.receive(source.encode(10.2, 100.1, Pose(2., 3., .4), 0.), 10.2, wall)
    assert not gate.armed


def test_replay_rejected():
    gate, source = ready()
    data = source.encode(10.2, 100.1, Pose(2., 3., .4), 0.)
    assert gate.receive(data, 10.2, 100.1)
    assert not gate.receive(data, 10.21, 100.11)


def test_wrong_sender_ignored():
    gate, source = ready()
    assert not accept_datagram(gate, source.encode(10.2, 100.1),
                               '192.168.0.2', '192.168.0.131', 10.2, 100.1)
    assert gate.armed


@pytest.mark.parametrize('velocity', [Velocity(2.), Velocity(0., 0., 5.)])
def test_excess_speed_rejected(velocity):
    gate, source = ready()
    source.command(velocity, 10.2)
    assert not gate.receive(source.encode(10.2, 100.1, Pose(2., 3., .4), 0.), 10.2, 100.1)
    assert gate.evaluate(10.2, 100.1)[0].stopped


def test_new_session_requires_zero_again():
    gate, _ = ready()
    source = SourceState()
    source.command(Velocity(.2), 10.2)
    gate.receive(source.encode(10.2, 100.1, Pose(2., 3., .4), 0.), 10.2, 100.1)
    assert gate.evaluate(10.2, 100.1)[1]['state'] == 'waiting_for_new_zero_command'


def test_unknown_real_pose_never_arms():
    gate, source = ready()
    gate.receive(source.encode(10.2, 100.1), 10.2, 100.1)
    assert gate.evaluate(10.2, 100.1)[1]['state'] == 'real_pose_unavailable_or_stale'


def test_bad_pose_alignment_never_arms():
    gate, source = ready()
    gate.disarm('test')
    source.command(Velocity(), 10.2)
    gate.receive(source.encode(10.2, 100.1, Pose(20., 30., .4), 0.), 10.2, 100.1)
    assert gate.evaluate(10.2, 100.1)[1]['state'] == 'waiting_for_initial_alignment'


def test_corrupt_and_nonfinite_packet():
    with pytest.raises(PacketError):
        decode_packet(b'not json')
    source = SourceState()
    obj = json.loads(source.encode(1., 1.))
    obj['sent_at'] = math.nan
    with pytest.raises(PacketError):
        decode_packet(json.dumps(obj).encode())


def test_source_initially_has_no_command():
    assert decode_packet(SourceState().encode(1., 1.)).command_age is None


def test_cli():
    assert parse_args(['source', '--target', '192.168.0.23']).cmd_type == 'twist'
    assert parse_args(['sink', '--robot-ip', '192.168.0.131']).watchdog == .5
