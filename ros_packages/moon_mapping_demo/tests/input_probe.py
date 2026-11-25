#!/usr/bin/env python3
"""실행 중인 Webots의 clock·odom 수신 간격을 관측한다. 속도를 발행하지 않는다."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--output', type=Path, default=Path('results/gui_input_probe.json'))
    args = parser.parse_args()
    if not 0 < args.seconds <= 120:
        parser.error('--seconds는 0보다 크고 120 이하여야 합니다.')
    rclpy.init()
    node = Node('shadow_input_probe')
    started = time.monotonic()
    streams = {}
    latest_clock = None

    def observe(name, stamp):
        wall = time.monotonic()
        entry = streams.setdefault(name, {'count': 0, 'first_stamp': stamp,
                                         'first_wall': wall, 'max_wall_gap_seconds': 0.,
                                         'max_stamp_gap_seconds': 0.,
                                         'max_observed_odom_lead_seconds': 0.,
                                         'max_observed_odom_lag_seconds': 0.})
        if entry['count']:
            entry['max_wall_gap_seconds'] = max(entry['max_wall_gap_seconds'], wall - entry['last_wall'])
            entry['max_stamp_gap_seconds'] = max(entry['max_stamp_gap_seconds'], stamp - entry['last_stamp'])
        entry['count'] += 1
        entry['last_wall'], entry['last_stamp'] = wall, stamp
        if name != 'clock' and latest_clock is not None:
            entry['max_observed_odom_lead_seconds'] = max(entry['max_observed_odom_lead_seconds'], stamp - latest_clock)
            entry['max_observed_odom_lag_seconds'] = max(entry['max_observed_odom_lag_seconds'], latest_clock - stamp)

    def clock(msg):
        nonlocal latest_clock
        latest_clock = msg.clock.sec + msg.clock.nanosec / 1e9
        observe('clock', latest_clock)

    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    for robot in ('robot1', 'robot2', 'robot3', 'robot4'):
        node.create_subscription(Odometry, f'/{robot}/odom',
                                 lambda msg, r=robot: observe(r, msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9),
                                 qos_profile_sensor_data)
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
        ended = time.monotonic()
        for entry in streams.values():
            entry['final_wall_age_seconds'] = ended - entry['last_wall']
            entry['initial_discovery_seconds'] = entry['first_wall'] - started
            elapsed = entry['last_wall'] - entry['first_wall']
            entry['sim_seconds_per_wall_second'] = ((entry['last_stamp'] - entry['first_stamp']) / elapsed
                                                     if elapsed else None)
        report = {'wall_seconds': ended - started, 'streams': streams,
                  'all_inputs_received': set(streams) == {'clock', 'robot1', 'robot2', 'robot3', 'robot4'},
                  'note': '독립 구독자의 수신 간격. 실행기 내부 큐 지연과 다를 수 있음.'}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report['all_inputs_received'] else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
