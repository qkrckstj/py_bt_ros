"""Real TF2 cache behavior for long-running map-frame navigation targets."""
import copy

import pytest
import tf2_geometry_msgs  # Registers PoseStamped conversion with real TF2.
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from tf2_ros import Buffer, ExtrapolationException

from webots_ros2_limo.exploration_ros import Navigation


def test_long_running_map_goal_survives_expiry_of_its_original_tf_timestamp():
    class Pending:
        def add_done_callback(self, callback):
            pass

    class Client:
        def send_goal_async(self, request):
            self.request = request
            return Pending()

    nav = object.__new__(Navigation)
    nav.frame, nav.state, nav.client = 'map', 'IDLE', Client()
    nav.start([10., 0., 0.])
    target = nav.client.request.pose
    buffer = Buffer(cache_time=Duration(seconds=2.))
    for second in range(100, 105):
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'robot/base_link'
        tf.header.stamp.sec = second
        tf.transform.rotation.w = 1.
        tf.transform.translation.x = float(second-100)
        buffer.set_transform(tf, 'test_odometry')
    # A timestamp fixed at goal dispatch can no longer be transformed, even
    # though current localization and the target's map coordinates are valid.
    historical = copy.deepcopy(target)
    historical.header.stamp.sec = 100
    with pytest.raises(ExtrapolationException):
        buffer.transform(historical, 'robot/base_link')
    current = buffer.transform(target, 'robot/base_link')
    assert current.pose.position.x == pytest.approx(6.)
    assert current.header.stamp.sec == 104
