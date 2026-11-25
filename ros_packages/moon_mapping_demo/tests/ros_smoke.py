#!/usr/bin/env python3
"""합성 /clock·4대 odom으로 검사하는 ROS 메시지 연결 smoke test.

실제 Webots/물리 주행 검증이 아니다. 설치한 Jazzy overlay를 source한 뒤:
  ROS_DOMAIN_ID=187 python3 ros_packages/moon_mapping_demo/tests/ros_smoke.py
  ROS_DOMAIN_ID=188 python3 ros_packages/moon_mapping_demo/tests/ros_smoke.py --execute

기본 모드는 VISUALIZED와 늦게 구독한 경로/marker의 transient 수신 및 정지
명령을 확인한다. --execute는 별도의 두 실행에서 대기 robot4의 odom 중단과
/clock 역행, stop 서비스, cmd_vel 발행자 충돌의 안전 정지와 runtime 기록을
검사한다. 로봇 위치는 움직이지 않는다.
"""
import argparse
import json
from pathlib import Path
import tempfile
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from moon_mapping_demo.shadow_mission import ShadowMission, stamp


class Probe(Node):
    def __init__(self, robots):
        super().__init__('shadow_smoke_probe')
        self.all_robots = tuple(robots)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pubs = {r: self.create_publisher(Odometry, f'/{r}/odom', 10)
                          for r in self.all_robots}
        self.statuses, self.commands, self.nonzero = [], {}, 0
        self.command_counts = {r: 0 for r in self.all_robots}
        self.heartbeat = None
        self.paths, self.markers = {}, None
        self.late_subscriptions = []
        self.stop_client = self.create_client(Trigger, '/shadow_mission/stop')
        self.transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/shadow_mission/status', self.status, self.transient)
        self.create_subscription(Bool, '/multi_bt/emergency_stop',
                                 lambda msg: setattr(self, 'heartbeat', msg.data), self.transient)
        self.command_subscriptions = [
            self.create_subscription(Twist, f'/{r}/cmd_vel',
                                     lambda msg, r=r: self.velocity(r, msg), 10)
            for r in self.all_robots]

    def status(self, message):
        self.statuses.append(json.loads(message.data))

    def velocity(self, robot, message):
        self.commands[robot] = (message.linear.x, message.angular.z)
        self.command_counts[robot] += 1
        if abs(message.linear.x) > 1e-10 or abs(message.angular.z) > 1e-10:
            self.nonzero += 1

    def publish_inputs(self, sim_time, suppress=()):
        self.clock_pub.publish(Clock(clock=stamp(sim_time)))
        for robot, publisher in self.odom_pubs.items():
            if robot in suppress:
                continue
            message = Odometry()
            message.header.stamp = stamp(sim_time)
            message.header.frame_id = f'{robot}/odom'
            message.child_frame_id = f'{robot}/base_link'
            message.pose.pose.orientation.w = 1.
            publisher.publish(message)

    def subscribe_late(self, selected):
        self.late_subscriptions = [
            self.create_subscription(PathMsg, f'/{robot}/shadow_path',
                                     lambda msg, robot=robot: self.paths.update({robot: msg}),
                                     self.transient)
            for robot in selected]
        self.late_subscriptions.append(self.create_subscription(
            MarkerArray, '/shadow_mission/markers',
            lambda msg: setattr(self, 'markers', msg), self.transient))

    def latest_phase(self):
        return self.statuses[-1]['phase'] if self.statuses else None

    def stopped(self, selected):
        return self.heartbeat is True and all(
            robot in self.commands and self.commands[robot] == (0., 0.) for robot in selected)


def run_case(case, timeout, output_root=None):
    execute = case != 'visualization'
    output = (Path(output_root).resolve() / case if output_root else
              Path(tempfile.mkdtemp(prefix=f'lunar_ros_smoke_{case}_')))
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true',
                     '-p', f'execute:={str(execute).lower()}', '-p', f'output:={output}'])
    mission, probe, executor = None, None, None
    try:
        mission = ShadowMission()
        probe = Probe(mission.all_robots)
        executor = SingleThreadedExecutor()
        executor.add_node(mission)
        executor.add_node(probe)
        started = time.monotonic()
        last_publish = 0.
        injected = None
        stop_future = None
        conflicting_publisher = None
        late = False
        success_since = None
        while time.monotonic()-started < timeout:
            wall = time.monotonic()
            sim_time = 1.+wall-started
            if injected is not None and case == 'clock_backward':
                sim_time = .25+wall-injected
            suppress = ('robot4',) if injected is not None and case == 'standby_odom_stale' else ()
            if wall-last_publish >= .03:
                probe.publish_inputs(sim_time, suppress)
                last_publish = wall
            executor.spin_once(timeout_sec=.01)
            phase = probe.latest_phase()
            target_phase = 'EXECUTING' if execute else 'VISUALIZED'
            if phase == target_phase and not late:
                # 경로/marker가 발행된 뒤 새 구독자를 만들어 durability를 검증한다.
                probe.subscribe_late(mission.robots)
                late = True
            if phase == 'ERROR' and injected is None:
                raise AssertionError(f'입력 교란 전 ERROR: {probe.statuses[-1]}')
            ready = (set(probe.paths) == set(mission.robots) and probe.markers is not None)
            if execute and injected is None and phase == 'EXECUTING' and ready and probe.heartbeat is False:
                injected = wall
                if case == 'user_stop':
                    assert probe.stop_client.service_is_ready()
                    stop_future = probe.stop_client.call_async(Trigger.Request())
                elif case == 'command_conflict':
                    conflicting_publisher = probe.create_publisher(Twist, f'/{mission.robots[0]}/cmd_vel', 10)
            if not execute:
                success = phase == 'VISUALIZED' and ready and probe.stopped(mission.robots)
            elif case == 'standby_odom_stale':
                success = (injected is not None and phase == 'ERROR'
                           and 'odom' in probe.statuses[-1]['reason'] and probe.stopped(mission.robots))
            elif case == 'clock_backward':
                success = (injected is not None and phase == 'STOPPED'
                           and '/clock' in probe.statuses[-1]['reason'] and probe.stopped(mission.robots))
            elif case == 'user_stop':
                success = (stop_future is not None and stop_future.done() and stop_future.result().success
                           and phase == 'STOPPED' and probe.stopped(mission.robots))
            else:
                success = (injected is not None and phase == 'ERROR'
                           and 'cmd_vel' in probe.statuses[-1]['reason'] and probe.stopped(mission.robots))
            if success:
                if success_since is None:
                    success_since = wall
                if wall-success_since >= .2:
                    break
            else:
                success_since = None
        else:
            raise AssertionError(f'{case} 시간 초과: status={probe.statuses[-3:]}, '
                                 f'paths={list(probe.paths)}, heartbeat={probe.heartbeat}, '
                                 f'commands={probe.commands}')
        if not execute:
            assert probe.nonzero == 0, f'시각화 모드에서 nonzero 명령 {probe.nonzero}개'
        for robot, path in probe.paths.items():
            assert path.header.frame_id == 'map' and len(path.poses) >= 2, robot
            times = [p.header.stamp.sec+p.header.stamp.nanosec/1e9 for p in path.poses]
            assert all(p.header.frame_id == 'map' for p in path.poses)
            assert all(b > a for a, b in zip(times, times[1:])), (robot, times)
        text_markers = [m for m in probe.markers.markers if m.action == Marker.ADD
                        and m.type == Marker.TEXT_VIEW_FACING]
        assert text_markers and all('s' in m.text for m in text_markers)
        runtime = json.loads((output / 'runtime.json').read_text())
        assert runtime['phase'] == probe.latest_phase(), runtime
        return {'case': case, 'result': 'passed', 'simulation': '합성 clock/odom; 물리 주행 아님',
                'phase': probe.latest_phase(), 'late_path_topics': sorted(probe.paths),
                'time_markers': len(text_markers), 'nonzero_command_messages': probe.nonzero,
                'selected_command_counts': {r: probe.command_counts[r] for r in mission.robots},
                'final_emergency_stop': probe.heartbeat, 'output': str(output)}
    finally:
        if mission is not None:
            mission.halt()
            mission.pool.shutdown(wait=True, cancel_futures=True)
        if executor is not None:
            executor.shutdown()
        if probe is not None:
            probe.destroy_node()
        if mission is not None:
            mission.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='합성 입력의 실행 안전 정지 검사')
    parser.add_argument('--timeout', type=float, default=15., help='각 case의 실제 시간 제한(초)')
    parser.add_argument('--output-root', type=Path, help='runtime과 검사 요약을 저장할 폴더')
    args = parser.parse_args()
    cases = ['standby_odom_stale', 'clock_backward', 'user_stop', 'command_conflict'] if args.execute else ['visualization']
    results = [run_case(case, args.timeout, args.output_root) for case in cases]
    summary = json.dumps({'ros_smoke': results}, ensure_ascii=False, indent=2)
    if args.output_root:
        args.output_root.mkdir(parents=True, exist_ok=True)
        (args.output_root / 'ros_validation.json').write_text(summary + '\n')
    print(summary)


if __name__ == '__main__':
    main()
