from pathlib import Path
import json
import pytest
from webots_ros2_limo.pose_replica import PoseFeed
from webots_ros2_limo.twin_protocol import SourceState, Pose
from webots_ros2_limo.peer_relay import encode, decode, TOPICS


def test_pose_works_without_velocity_and_new_pose_corrects_any_distance():
    source = SourceState()
    feed = PoseFeed('192.168.0.131')
    for seq, pose in enumerate([Pose(0., 0., 0.), Pose(3., -4., 1.2)]):
        stamp = 100. + seq * .02
        data = source.encode(stamp, stamp, pose, .01)
        assert feed.receive(data, feed.robot_ip, stamp, stamp)
        assert feed.latest(stamp, stamp).pose == pose
    # Arrival freshness does not refresh an old pose.
    assert feed.latest(101., 101.) is None
    assert feed.reason == 'connection_lost'


def test_replay_wrong_source_missing_pose_and_old_pose():
    source = SourceState()
    feed = PoseFeed('192.168.0.131')
    data = source.encode(1., 100., Pose(0., 0., 0.), .01)
    assert not feed.receive(data, '192.168.0.9', 1., 100.)
    assert feed.receive(data, feed.robot_ip, 1., 100.)
    assert not feed.receive(data, feed.robot_ip, 1.1, 100.1)
    assert not feed.receive(source.encode(1.1, 100.1, None, None), feed.robot_ip, 1.1, 100.1)
    assert feed.latest(1.1, 100.1) is None
    assert not feed.receive(source.encode(1.2, 100.2, Pose(0., 0., 0.), 2.), feed.robot_ip, 1.2, 100.2)


def test_replica_proto_contains_no_devices_or_physics():
    proto = (Path(__file__).parents[1] / 'protos/LimoReplica.proto').read_text()
    for kind in ('Camera {', 'Lidar {', 'RangeFinder {', 'Motor {', 'GPS {',
                 'InertialUnit {', 'Physics {', 'boundingObject'):
        assert kind not in proto


def test_only_peer_data_can_cross_to_real_robot():
    for topic in ('/cmd_vel', '/robot1/scan', '/initialpose', '/clock_override', '/robot2/cmd_vel'):
        with pytest.raises(ValueError):
            encode(topic, b'data', 100.)
        payload = json.dumps({'topic': topic, 'sent_at': 100., 'data': ''}).encode()
        with pytest.raises(ValueError):
            decode(payload, 100.)
    wire = encode('/robot2/scan', b'\x00\x01\xff', 100.)
    assert decode(wire[4:], 100.1) == ('/robot2/scan', b'\x00\x01\xff')
    with pytest.raises(ValueError):
        decode(wire[4:], 102.)
    assert all('robot1' not in topic and 'cmd_vel' not in topic for topic in TOPICS)
