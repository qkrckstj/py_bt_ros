"""Four independent SLAM grids aligned by known simulation starting transforms.

This is grid fusion, not joint multi-robot SLAM. No world geometry is used to
populate the map. The grid resampler is bundled with this package.
"""
import array
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from rclpy.time import Time
from .exploration_map_grid import Grid, align_grid
from .exploration_scan import mapping_ranges
from .exploration_mapping import MappingMode


def merge_native_grids(grids):
    """Align on a fixed lattice at the finest actual source resolution.

    The four mappers currently produce 0.1 m cells. Upsampling to 0.05 m only
    adds interpolation edges and cost; it does not add measured information.
    Rebuild from latest snapshots, keeping occupied-wins safety semantics.
    """
    grids = list(grids)
    r = min(g.resolution for g in grids)
    aligned = [align_grid(g, r) for g in grids]
    offsets = [(round(g.x/r), round(g.y/r)) for g in aligned]
    x0, y0 = min(x for x, y in offsets), min(y for x, y in offsets)
    x1 = max(x+g.cells.shape[1] for g, (x, y) in zip(aligned, offsets))
    y1 = max(y+g.cells.shape[0] for g, (x, y) in zip(aligned, offsets))
    if (x1-x0)*(y1-y0) > 4_000_000:
        raise ValueError('Merged map exceeds limit; check SLAM alignment')
    cells = np.full((y1-y0, x1-x0), -1, dtype=np.int8)
    for g, (x, y) in zip(aligned, offsets):
        patch = cells[y-y0:y-y0+g.cells.shape[0], x-x0:x-x0+g.cells.shape[1]]
        np.maximum(patch, g.cells, out=patch)
    return Grid(cells, r, x0*r, y0*r)


def include_localized_footprints(grid, positions, radius=.30):
    """Keep fresh localized robot footprints inside the shared grid extent.

    SLAM bounds can exclude a stationary robot whose nearby rays have no return.
    Extend the existing lattice with unknown cells; only the physical footprint
    is known free. Never create a free corridor between it and measured space.
    """
    positions = list(positions)
    if not positions:
        return grid
    if (grid.yaw != 0. or radius <= 0. or not math.isfinite(radius)
            or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in positions)):
        raise ValueError('Invalid localized footprint geometry')
    r = grid.resolution
    h, w = grid.cells.shape
    x0 = min(0, *(math.floor((p[0]-radius-grid.x)/r+1e-9) for p in positions))
    y0 = min(0, *(math.floor((p[1]-radius-grid.y)/r+1e-9) for p in positions))
    x1 = max(w, *(math.ceil((p[0]+radius-grid.x)/r-1e-9) for p in positions))
    y1 = max(h, *(math.ceil((p[1]+radius-grid.y)/r-1e-9) for p in positions))
    if (x1-x0)*(y1-y0) > 4_000_000:
        raise ValueError('Merged map exceeds limit; check localized robot poses')
    cells = np.full((y1-y0, x1-x0), -1, dtype=np.int8)
    cells[-y0:h-y0, -x0:w-x0] = grid.cells
    x, y = grid.x+x0*r, grid.y+y0*r
    yy, xx = np.ogrid[:cells.shape[0], :cells.shape[1]]
    for px, py in positions:
        footprint = (x+(xx+.5)*r-px)**2+(y+(yy+.5)*r-py)**2 <= radius**2
        cells[footprint] = 0
    return Grid(cells, r, x, y)


class FlatMap(Node):
    def __init__(self, **kwargs):
        super().__init__('flat_shared_map', **kwargs)
        self.starts = json.loads(self.declare_parameter('starts_json', '{}').value)
        self.grids, self.received = {}, {}
        self.latest_map = None
        self.mapping = MappingMode(self.starts)
        self.pending = {}
        self.mode_clients = {n: self.create_client(SetBool, f'/{n}/slam_toolbox/set_localization_mode')
                             for n in self.starts}
        self.qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, '/map', self.qos)
        self.status = self.create_publisher(String, '/exploration/map_sources', self.qos)
        self.mode_status = self.create_publisher(String, '/exploration/mapping_state', self.qos)
        self.create_subscription(String, '/exploration/map_control', self.control, self.qos)
        self.scans = {}
        anchors = []
        self.broadcaster = StaticTransformBroadcaster(self)
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        for name, (x, y, yaw) in self.starts.items():
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'map', name + '/map'
            tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
            tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
            anchors.append(tf)
            self.scans[name] = self.create_publisher(LaserScan, f'/{name}/scan_slam', qos_profile_sensor_data)
            self.create_subscription(LaserScan, f'/{name}/scan_mapping_input', lambda m, n=name: self.scan(n, m), qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, f'/{name}/map', lambda m, n=name: self.receive(n, m), self.qos)
        self.broadcaster.sendTransform(anchors)
        self.create_timer(1., self.publish)

    def control(self, msg):
        try:
            request = json.loads(msg.data)
            self.mapping.request(request.get('session'), request.get('mode'), self.latest_map)
        except (ValueError, AttributeError):
            return

    def update_mode(self):
        session, mode = self.mapping.session, self.mapping.mode
        if not session:
            return
        now = time.monotonic()
        for name, client in self.mode_clients.items():
            if name in self.pending:
                future, started, requested_session, requested_mode = self.pending[name]
                if future.done():
                    try:
                        self.mapping.acknowledge(name, requested_session, requested_mode, future.result().success)
                    except Exception as exc:
                        self.get_logger().warning(f'{name} localization-mode request failed: {exc}')
                    del self.pending[name]
                elif now-started > 5.:
                    future.cancel()
                    del self.pending[name]
                else:
                    continue
            if name not in self.mapping.confirmed and client.service_is_ready():
                self.pending[name] = (client.call_async(SetBool.Request(data=mode == 'LOCALIZATION')),
                                      now, session, mode)

    def publish_status(self):
        self.update_mode()
        fresh = (set(self.grids) == set(self.starts)
                 and all(time.monotonic()-t <= 10. for t in self.received.values()))
        self.mode_status.publish(String(data=json.dumps(self.mapping.snapshot(fresh))))
        self.status.publish(String(data=json.dumps({name: {'age': time.monotonic()-self.received[name],
            'known': int((g.cells >= 0).sum())} for name, g in self.grids.items()})))

    def scan(self, name, scan):
        if len(scan.ranges) < 2:
            return
        scan.angle_increment = (scan.angle_max-scan.angle_min)/(len(scan.ranges)-1)
        scan.range_min = .1
        scan.ranges, scan.range_max = mapping_ranges(scan.ranges)
        self.scans[name].publish(scan)

    def receive(self, name, msg):
        if self.mapping.frozen_map is not None:
            return
        info = msg.info
        if not info.width or not info.height or len(msg.data) != info.width*info.height:
            return
        x, y, yaw = self.starts[name]
        p, q = info.origin.position, info.origin.orientation
        c, s = math.cos(yaw), math.sin(yaw)
        angle = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        self.grids[name] = Grid(np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width),
                               info.resolution, x+c*p.x-s*p.y, y+s*p.x+c*p.y, yaw+angle)
        self.received[name] = time.monotonic()

    def publish(self):
        self.publish_status()
        if self.mapping.frozen_map is not None:
            # Publish an unchanged geometry/data snapshot. Header time remains a
            # live heartbeat; no moving-footprint clearing or re-fusion here.
            msg = self.mapping.frozen_map
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            return
        if not self.grids:
            return
        grid = merge_native_grids(self.grids.values())
        positions = []
        for name in self.starts:
            try:
                tf = self.buffer.lookup_transform('map', name+'/base_link', Time())
                age = self.get_clock().now().nanoseconds*1e-9-tf.header.stamp.sec-tf.header.stamp.nanosec*1e-9
                if not -.25 <= age < 2.:
                    continue
                p = tf.transform.translation
                if math.isfinite(p.x) and math.isfinite(p.y):
                    positions.append((p.x, p.y))
            except Exception:
                pass
        grid = include_localized_footprints(grid, positions)
        msg = OccupancyGrid()
        msg.header.frame_id, msg.header.stamp = 'map', self.get_clock().now().to_msg()
        msg.info.resolution = grid.resolution
        msg.info.height, msg.info.width = grid.cells.shape
        msg.info.origin.position.x, msg.info.origin.position.y = grid.x, grid.y
        msg.info.origin.orientation.w = 1.
        msg.data = array.array('b', grid.cells.tobytes())
        self.latest_map = msg
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FlatMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
