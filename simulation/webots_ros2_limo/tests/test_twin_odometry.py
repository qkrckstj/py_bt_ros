from types import SimpleNamespace as NS

import pytest

from webots_ros2_limo.twin_bridge import odometry_velocity, parse_args
from webots_ros2_limo.twin_protocol import PacketError, Velocity
from test_twin_protocol import ready


def odom(x=0., yaw=0., stamp=100., child='base_link'):
    return NS(child_frame_id=child,
              header=NS(stamp=NS(sec=int(stamp), nanosec=round((stamp % 1) * 1e9))),
              twist=NS(twist=NS(linear=NS(x=x, y=0., z=0.),
                                angular=NS(x=0., y=0., z=yaw))))


def test_default_supports_chassis_feedback_and_command_is_explicit():
    assert parse_args(['source', '--target', '127.0.0.1']).velocity_source == 'odometry'
    assert parse_args(['source', '--target', '127.0.0.1', '--velocity-source',
                       'command']).velocity_source == 'command'


def test_feedback_mirrors_rc_then_ros_without_adding_commands_and_stops_on_loss():
    gate, source = ready()
    # Both input modes produce the same chassis feedback interface.
    for now, message in [(10.2, odom(.15, -.4, 100.1)),
                         (10.3, odom(.3, .2, 100.2))]:
        wall = now + 89.9
        velocity, age = odometry_velocity(message, 'base_link', wall, 1.)
        source.command(velocity, now, age)
        gate.update_clock(now - 9., now)
        gate.update_sim_pose(gate.sim_pose, now)
        gate.receive(source.encode(now, wall, gate.sim_pose, 0.), now, wall)
        assert gate.evaluate(now, wall)[0] == velocity
    # Heartbeats/TF continuing must not keep a lost odometry sample alive.
    gate.update_clock(2.5, 11.5)
    gate.update_sim_pose(gate.sim_pose, 11.5)
    gate.receive(source.encode(11.5, 101.4, gate.sim_pose, 0.), 11.5, 101.4)
    assert gate.evaluate(11.5, 101.4)[0].stopped


@pytest.mark.parametrize('message,now', [
    (odom(child='odom'), 100.), (odom(stamp=98.), 100.),
    (odom(stamp=101.), 100.), (odom(x=float('nan')), 100.),
])
def test_bad_feedback_is_rejected(message, now):
    with pytest.raises(PacketError):
        odometry_velocity(message, 'base_link', now, 1.)


def test_stationary_noise_is_zero_but_normal_slow_motion_is_retained():
    assert odometry_velocity(odom(.001, -.002), 'base_link', 100., 1.)[0] == Velocity()
    assert odometry_velocity(odom(.01, .02), 'base_link', 100., 1.)[0] == Velocity(.01, 0., .02)
