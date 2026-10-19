"""Wait for an advancing simulation clock before activating ROS controllers."""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=60.0)
    options, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('wait_for_sim_clock')
    first = None
    ready = False

    def receive(message):
        nonlocal first, ready
        stamp = message.clock.sec * 10**9 + message.clock.nanosec
        if stamp <= 0:
            return
        if first is None or stamp < first:
            first = stamp
        elif stamp > first:
            ready = True

    subscription = node.create_subscription(Clock, '/clock', receive, qos_profile_sensor_data)
    node.get_logger().info('Waiting for /clock to advance; Webots must be playing.')
    deadline = time.monotonic() + options.timeout
    try:
        while rclpy.ok() and not ready and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if ready:
            node.get_logger().info('Simulation clock is advancing; controllers can start.')
            return 0
        node.get_logger().error('Simulation clock did not advance. Start Webots playback and relaunch.')
        return 1
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
