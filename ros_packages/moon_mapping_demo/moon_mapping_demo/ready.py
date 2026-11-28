"""Wait for odometry from the spawned robots and scans from the navigating robots."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def main():
    rclpy.init();node=Node('multi_robot_ready');seen=set();subs=[]
    robots = node.declare_parameter('robots', ['robot1', 'robot2', 'robot3']).value
    scan_robots = node.declare_parameter('scan_robots', robots).value
    for name, typ, sources in [('scan', LaserScan, scan_robots), ('odom', Odometry, robots)]:
        for robot in sources:
            key=robot+'/'+name
            subs.append(node.create_subscription(typ,'/'+key,lambda msg,k=key:seen.add(k),qos_profile_sensor_data))
    end=time.monotonic()+100
    try:
        while rclpy.ok() and len(seen)<len(subs) and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
        ok=len(seen)==len(subs)
        node.get_logger().info(f'Sensors ready: {sorted(seen)}')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(0 if ok else 1)
