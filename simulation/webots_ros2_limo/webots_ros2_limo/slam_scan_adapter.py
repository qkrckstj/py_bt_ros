"""Normalize scan geometry for Karto without changing the real-LIMO-compatible /scan."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def normalize(scan):
    count = len(scan.ranges)
    if count < 2:
        return None
    # Webots samples the complete FOV; the real driver metadata's nominal step
    # is slightly different and Karto otherwise expects 249 instead of 248 rays.
    scan.angle_increment = (scan.angle_max - scan.angle_min) / (count - 1)
    return scan


def main():
    rclpy.init()
    node = Node('slam_scan_adapter')
    pub = node.create_publisher(LaserScan, '/scan_slam', qos_profile_sensor_data)
    def receive(message):
        message = normalize(message)
        if message is not None:
            pub.publish(message)
    node.create_subscription(LaserScan, '/scan', receive, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
