"""ROS message/action integration. Opt in on an isolated DDS domain.

RUN_EXPLORATION_ROS_TESTS=1 ROS_DOMAIN_ID=217 python3 -m pytest ...
The four action servers below are mocks; this test is not a physics validation.
"""
import os
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('RUN_EXPLORATION_ROS_TESTS') != '1',
                                reason='ROS integration uses an explicitly isolated domain')


def test_four_robot_ros_assignment_result_and_stop():
    import numpy as np
    import rclpy
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from tf2_ros import TransformBroadcaster
    from webots_ros2_limo.exploration_ros import ExplorationRobot, ExplorationTeam

    rclpy.init()
    fake = Node('exploration_mock_simulation')
    team = ExplorationTeam(parameter_overrides=[Parameter('autostart', value=True),
        Parameter('require_mapping_control', value=False)])
    workers = [ExplorationRobot(namespace=f'robot{i}', parameter_overrides=[
        Parameter('observe_seconds', value=.1)]) for i in range(1, 5)]
    received, canceled, servers = {}, [], []
    group = ReentrantCallbackGroup()

    def execute(handle, name):
        received[name] = received.get(name, 0) + 1
        # Second-round actions remain active until the operator stop arrives.
        duration = .3 if received[name] == 1 else 8.
        until = time.monotonic() + duration
        while time.monotonic() < until:
            if handle.is_cancel_requested:
                canceled.append(name)
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(.02)
        handle.succeed()
        return NavigateToPose.Result()

    for i in range(1, 5):
        name = f'robot{i}'
        servers.append(ActionServer(fake, NavigateToPose, f'/{name}/navigate_to_pose',
            execute_callback=lambda h, name=name: execute(h, name),
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group))
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = fake.create_publisher(OccupancyGrid, '/map', qos)
    broadcaster = TransformBroadcaster(fake)
    cells = np.full((80, 80), -1, dtype=np.int8)
    cells[8:72, 8:72] = 0
    positions = [(3., 3.), (12., 3.), (3., 12.), (12., 12.)]

    def inputs():
        transforms = []
        for i, (x, y) in enumerate(positions, 1):
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'map', f'robot{i}/base_link'
            t.header.stamp = fake.get_clock().now().to_msg()
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation.w = 1.
            transforms.append(t)
        broadcaster.sendTransform(transforms)
        m = OccupancyGrid()
        m.header.frame_id, m.header.stamp = 'map', fake.get_clock().now().to_msg()
        m.info.width = m.info.height = 80
        m.info.resolution, m.info.origin.orientation.w = .2, 1.
        m.data = cells.ravel().tolist()
        publisher.publish(m)

    timer = fake.create_timer(.2, inputs)
    executor = MultiThreadedExecutor(num_threads=8)
    for node in [fake, team, *workers]:
        executor.add_node(node)

    def wait_until(predicate, seconds=12.):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if predicate():
                return
        raise AssertionError({'received': received, 'history': team.bt.history,
                              'phase': team.bt.phase, 'reason': team.bt.reason})

    try:
        wait_until(lambda: len(team.bt.history) >= 4)
        assert set(received) == {f'robot{i}' for i in range(1, 5)}
        assert {entry['result'] for entry in team.bt.history} == {'VISITED'}
        wait_until(lambda: all(received.get(f'robot{i}', 0) >= 2 for i in range(1, 5)))
        team.bt.enabled = False
        wait_until(lambda: len(canceled) == 4 and not team.bt.active)
        assert not team.bt.finished and team.bt.phase == 'STOPPED'
    finally:
        fake.destroy_timer(timer)
        executor.shutdown(timeout_sec=10.)
        for server in servers:
            server.destroy()
        for node in [*workers, team, fake]:
            node.destroy_node()
        rclpy.shutdown()


def test_ros_freeze_localization_handshake_and_four_home_arrivals():
    """Real ROS adapters/fusion with mocked SLAM services and navigation."""
    import hashlib
    import json
    import numpy as np
    import rclpy
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from std_srvs.srv import SetBool
    from tf2_ros import TransformBroadcaster
    from webots_ros2_limo.exploration_ros import ExplorationRobot, ExplorationTeam
    from webots_ros2_limo.exploration_flat_map import FlatMap

    rclpy.init()
    names = [f'robot{i}' for i in range(1, 5)]
    homes = {n: [3.+2*i, 3., 0.] for i, n in enumerate(names)}
    positions = {n: [p[0], 10.] for n, p in homes.items()}
    fake = Node('return_mock_slam_navigation')
    fusion = FlatMap(parameter_overrides=[Parameter('starts_json', value=json.dumps({n: [0., 0., 0.] for n in names}))])
    team = ExplorationTeam(parameter_overrides=[Parameter('autostart', value=True),
        Parameter('homes_json', value=json.dumps(homes))])
    workers = [ExplorationRobot(namespace=n) for n in names]
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers = {n: fake.create_publisher(OccupancyGrid, f'/{n}/map', qos) for n in names}
    broadcaster = TransformBroadcaster(fake)
    group = ReentrantCallbackGroup()
    calls, active, max_active, services, servers, frozen_hashes = {}, set(), [0], [], [], []

    def mode(request, response, name):
        calls[name] = request.data
        response.success = True
        return response

    def execute(handle, name):
        # The coordinator must receive every freeze acknowledgement first.
        assert all(calls.get(n) is True for n in names)
        active.add(name)
        max_active[0] = max(max_active[0], len(active))
        until = time.monotonic()+.3
        while time.monotonic() < until:
            time.sleep(.02)
        p = handle.request.pose.pose.position
        positions[name] = [p.x, p.y]
        # TF delivery precedes Nav2's arrival result on a moving physical robot.
        time.sleep(.35)
        active.remove(name)
        handle.succeed()
        return NavigateToPose.Result()

    for name in names:
        services.append(fake.create_service(SetBool, f'/{name}/slam_toolbox/set_localization_mode',
            lambda req, res, name=name: mode(req, res, name), callback_group=group))
        servers.append(ActionServer(fake, NavigateToPose, f'/{name}/navigate_to_pose',
            execute_callback=lambda h, name=name: execute(h, name),
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group))
    cells = np.full((80, 80), 100, dtype=np.int8)
    cells[1:-1, 1:-1] = 0

    def inputs():
        transforms = []
        for name in names:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'map', name+'/base_link'
            t.header.stamp = fake.get_clock().now().to_msg()
            t.transform.translation.x, t.transform.translation.y = positions[name]
            t.transform.rotation.w = 1.
            transforms.append(t)
            m = OccupancyGrid()
            m.header.frame_id, m.header.stamp = name+'/map', fake.get_clock().now().to_msg()
            m.info.width = m.info.height = 80
            m.info.resolution, m.info.origin.orientation.w = .2, 1.
            # A deliberately contradictory late source must not change /map.
            m.data = (np.full_like(cells, 100) if fusion.mapping.frozen_map is not None else cells).ravel().tolist()
            publishers[name].publish(m)
        broadcaster.sendTransform(transforms)

    def check_map(m):
        if fusion.mapping.frozen_map is not None:
            frozen_hashes.append((hashlib.sha256(bytes(v+1 for v in m.data)).hexdigest(),
                                  m.info.width, m.info.height, m.info.origin.position.x, m.info.origin.position.y))

    sub = fake.create_subscription(OccupancyGrid, '/map', check_map, qos)
    timer = fake.create_timer(.1, inputs)
    executor = MultiThreadedExecutor(num_threads=8)
    nodes = [fake, fusion, team, *workers]
    for node in nodes:
        executor.add_node(node)
    deadline = time.monotonic()+24.
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if team.bt.finished and len(frozen_hashes) >= 3:
                break
        assert team.bt.finished, (team.bt.phase, team.bt.reason, team.bt.return_failed,
                                  team.bt.mapping_state, team.bt.history)
        assert team.bt.phase == 'COMPLETE'
        assert team.bt.returned == set(names)
        assert max_active[0] == 1
        assert len(set(frozen_hashes)) == 1 and len(frozen_hashes) >= 3
        assert all(calls[n] for n in names)
    finally:
        fake.destroy_timer(timer)
        executor.shutdown(timeout_sec=5.)
        for server in servers:
            server.destroy()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
