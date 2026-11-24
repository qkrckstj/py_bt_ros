#!/usr/bin/env python3
"""Live occupancy/robot markers for RViz, independent of its indexed-map shader."""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
COLORS = [(1., .15, .15), (.1, .8, .15), (1., .6, .05), (.1, .4, 1.)]


def row_spans(mask):
    """Exact horizontal runs; compact map geometry without downsampling cells."""
    edges = np.diff(np.pad(mask.astype(np.int8), ((0, 0), (1, 1))), axis=1)
    rows, starts = np.nonzero(edges == 1)
    _, ends = np.nonzero(edges == -1)
    return zip(rows, starts, ends)


class LunarMapView(Node):
    def __init__(self):
        super().__init__('lunar_map_visualizer')
        self.publisher = self.create_publisher(MarkerArray, '/exploration/obstacle_map', QOS)
        self.robots = self.create_publisher(MarkerArray, '/exploration/robot_markers', QOS)
        self.states = {}
        self.create_subscription(OccupancyGrid, '/lunar/robot1/team_map', self.map, QOS)
        for i in range(1, 5):
            name = f'robot{i}'
            self.create_subscription(String, f'/{name}/exploration/state',
                                     lambda m, n=name: self.state(n, m), 10)
        self.create_timer(.5, self.robot_markers)

    def marker(self, name, kind, color, index=0):
        m = Marker()
        m.header.frame_id, m.header.stamp = 'map', self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = name, index, kind, Marker.ADD
        m.pose.orientation.w = 1.
        m.color.r, m.color.g, m.color.b, m.color.a = *color, 1.
        return m

    def map(self, message):
        i, q = message.info, message.info.origin.orientation
        if message.header.frame_id != 'map' or i.width*i.height != len(message.data):
            return
        cells = np.asarray(message.data).reshape(i.height, i.width)
        background = self.marker('unknown_mission_area', Marker.CUBE, (.5, .5, .5))
        background.pose.position.z = -.08
        background.scale.x, background.scale.y, background.scale.z = 60., 60., .02
        markers = [background]
        for name, mask, color in [
            ('observed_free', (cells >= 0) & (cells < 25), (.92, .92, .92)),
            ('uncertain_obstacle', (cells >= 25) & (cells < 65), (.3, .3, .3)),
            ('occupied', cells >= 65, (.02, .02, .02)),
        ]:
            # Each horizontal run becomes two flat triangles. This retains
            # every occupied cell while avoiding tens of thousands of cubes.
            m = self.marker(name, Marker.TRIANGLE_LIST, color)
            m.scale.x = m.scale.y = m.scale.z = 1.
            m.pose.orientation.z, m.pose.orientation.w = q.z, q.w
            m.pose.position = message.info.origin.position
            # Points are local to the map origin; marker pose applies rotation.
            for row, start, end in row_spans(mask):
                x0, x1 = float(start*i.resolution), float(end*i.resolution)
                y0, y1 = float(row*i.resolution), float((row+1)*i.resolution)
                m.points.extend(Point(x=x, y=y, z=-.03) for x,y in
                                [(x0,y0),(x1,y0),(x1,y1),(x0,y0),(x1,y1),(x0,y1)])
            markers.append(m)
        outline = self.marker('mission_boundary', Marker.LINE_STRIP, (.1, .65, .9))
        outline.scale.x = .08
        outline.points = [Point(x=x, y=y, z=.03) for x,y in
                          [(-30.,-30.), (30.,-30.), (30.,30.), (-30.,30.), (-30.,-30.)]]
        markers.append(outline)
        legend = self.marker('map_legend', Marker.TEXT_VIEW_FACING, (.1, .2, .3))
        legend.pose.position.x, legend.pose.position.y, legend.pose.position.z = -23., -16., .2
        legend.scale.z = .4
        legend.text = 'LIVE LUNAR MAP\nBlack: obstacle | White: free | Gray: unknown'
        markers.append(legend)
        self.publisher.publish(MarkerArray(markers=markers))

    def state(self, name, message):
        try:
            self.states[name] = json.loads(message.data)
        except (ValueError, TypeError):
            pass

    def robot_markers(self):
        markers = []
        for i, (name, state) in enumerate(sorted(self.states.items())):
            position = state.get('position')
            if not position:
                continue
            arrow = self.marker(name, Marker.ARROW, COLORS[i % 4])
            arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = *position[:2], .15
            yaw = state.get('yaw') or 0.
            arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = .8, .25, .25
            label = self.marker(name, Marker.TEXT_VIEW_FACING, COLORS[i % 4], 1)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = *position[:2], .7
            label.scale.z, label.text = .35, name
            markers.extend([arrow, label])
        self.robots.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = LunarMapView()
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
