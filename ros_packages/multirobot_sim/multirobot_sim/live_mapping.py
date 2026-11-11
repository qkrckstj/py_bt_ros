"""Adapt Webots scans for SLAM Toolbox and publish the shared live map."""

import array
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from .map_grid import Grid, align_grid


class LiveMapping(Node):
    def __init__(self):
        super().__init__('live_mapping')
        share = Path(get_package_share_directory('multirobot_sim'))
        environment = yaml.safe_load((share / 'config/environment.yaml').read_text())
        self.robot = self.declare_parameter('robot', 'robot1').value
        self.start = environment['starts'][self.robot]
        self.pending_grid = None
        self.map_load_time = None

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.static_tf = StaticTransformBroadcaster(self)
        anchor = TransformStamped()
        anchor.header.stamp = self.get_clock().now().to_msg()
        anchor.header.frame_id = 'map'
        anchor.child_frame_id = f'{self.robot}/map'
        anchor.transform.translation.x = float(self.start['x'])
        anchor.transform.translation.y = float(self.start['y'])
        anchor.transform.rotation.z = math.sin(self.start['yaw'] / 2)
        anchor.transform.rotation.w = math.cos(self.start['yaw'] / 2)
        self.static_tf.sendTransform([anchor])

        self.scan_pub = self.create_publisher(
            LaserScan, f'/{self.robot}/scan_slam', qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, f'/{self.robot}/scan', self.relay_scan, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, f'/{self.robot}/map', self.receive_grid, map_qos)
        self.create_timer(1.0, self.publish_map)

    def relay_scan(self, scan):
        count = len(scan.ranges)
        if count < 2:
            return
        # Webots samples both ends of the FOV. Keep the original /scan metadata
        # for Nav2 and provide Karto with the matching angular step.
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (count - 1)
        self.scan_pub.publish(scan)

    def receive_grid(self, msg):
        info = msg.info
        position, orientation = info.origin.position, info.origin.orientation
        origin = (position.x, position.y, orientation.x, orientation.y, orientation.z, orientation.w)
        if (msg.header.frame_id != f'{self.robot}/map'
                or info.width == 0 or info.height == 0
                or not math.isfinite(info.resolution) or info.resolution <= 0
                or not all(map(math.isfinite, origin))
                or len(msg.data) != info.width * info.height):
            self.get_logger().warning(f'Ignoring invalid SLAM grid from {self.robot}')
            return

        start = self.start
        cosine, sine = math.cos(start['yaw']), math.sin(start['yaw'])
        origin_yaw = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y ** 2 + orientation.z ** 2))
        x = start['x'] + cosine * position.x - sine * position.y
        y = start['y'] + sine * position.x + cosine * position.y
        cells = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        self.pending_grid = Grid(cells, info.resolution, x, y, start['yaw'] + origin_yaw)

    def publish_map(self):
        if self.pending_grid is None:
            return
        grid = align_grid(self.pending_grid)
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.map_load_time is None:
            self.map_load_time = msg.header.stamp
            self.get_logger().info(f'Shared live map is ready from {self.robot}')
        msg.info.map_load_time = self.map_load_time
        msg.info.resolution = grid.resolution
        msg.info.height, msg.info.width = grid.cells.shape
        msg.info.origin.position.x = grid.x
        msg.info.origin.position.y = grid.y
        msg.info.origin.orientation.w = 1.0
        msg.data = array.array('b', grid.cells.tobytes())
        self.map_pub.publish(msg)
        self.pending_grid = None


def main():
    rclpy.init()
    node = LiveMapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
