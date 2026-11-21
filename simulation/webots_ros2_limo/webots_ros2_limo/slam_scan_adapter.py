"""Normalize Karto scans while leaving the hardware /scan unchanged."""
import math

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


def real_no_return_ranges(ranges, physical_maximum, mapping_maximum=12.):
    """Clear only convincing no-return sectors; never turn isolated bad rays free.

    The real YDLidar publishes 0 for invalid measurements. A run bounded on
    both sides by distant real hits is treated as open space. Karto's private
    sentinel lies above its raster threshold, so it clears without creating
    a false occupied endpoint. Raw /scan still reaches Nav2 unchanged.
    """
    maximum = min(float(physical_maximum), float(mapping_maximum))
    sentinel = maximum + .01
    output = [float(v) if math.isfinite(v) and 0. < v < maximum else math.nan
              for v in ranges]
    i = 0
    while i < len(ranges):
        if ranges[i] != 0.:
            i += 1
            continue
        end = i
        while end < len(ranges) and ranges[end] == 0.:
            end += 1
        left = float(ranges[i-1]) if i else math.nan
        right = float(ranges[end]) if end < len(ranges) else math.nan
        if end-i >= 2 and all(math.isfinite(v) and 3. <= v < maximum
                              for v in (left, right)):
            output[i:end] = [sentinel] * (end-i)
        i = end
    return output, maximum+.02


def main():
    rclpy.init()
    node = Node('slam_scan_adapter')
    real_no_return = bool(node.declare_parameter('real_no_return', False).value)
    min_period = max(0., float(node.declare_parameter('min_period', 0.).value))
    last_stamp = None
    pub = node.create_publisher(LaserScan, '/scan_slam', qos_profile_sensor_data)
    def receive(message):
        nonlocal last_stamp
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if min_period and last_stamp is not None and 0. <= stamp-last_stamp < min_period:
            return
        last_stamp = stamp
        message = normalize(message)
        if message is not None:
            if real_no_return:
                message.ranges, message.range_max = real_no_return_ranges(
                    message.ranges, message.range_max)
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
