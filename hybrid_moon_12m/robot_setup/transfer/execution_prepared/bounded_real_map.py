"""Publish a 4 m physical Nav2 map with occupied cells outside the real region."""
import math,json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from ownership import Layout,project_grid
from limits import local_pose

class BoundedMap(Node):
    def __init__(self):
        super().__init__('hybrid_bounded_real_map')
        self.reference=Path(__file__).with_name('real_reference.json');self.latest=None
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(OccupancyGrid,'/hybrid/physical_nav_map',q)
        self.sub=self.create_subscription(OccupancyGrid,'/map',self.receive,q)
        self.create_timer(.5,self.publish)
    def receive(self,m):self.latest=m
    def publish(self):
        if self.latest is None or not self.reference.is_file():return
        anchor=json.loads(self.reference.read_text())['anchor'];m=self.latest
        # Axis-aligned output avoids rotated OccupancyGrid origins in Nav2.
        resolution=.05;extent=3.0
        bounds=(anchor[0]-extent,anchor[1]-extent,anchor[0]+extent,anchor[1]+extent)
        l=Layout(bounds=bounds,real_bounds=bounds,resolution=resolution)
        q=m.info.origin.orientation;p=m.info.origin.position
        origin=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        cells=project_grid(np.array(m.data).reshape(m.info.height,m.info.width),m.info.resolution,origin,l)
        rows,cols=np.indices(l.shape);dx=bounds[0]+(cols+.5)*resolution-anchor[0];dy=bounds[1]+(rows+.5)*resolution-anchor[1]
        c,s=math.cos(anchor[2]),math.sin(anchor[2]);x=c*dx+s*dy;y=-s*dx+c*dy
        cells[(np.abs(x)>2.)|(np.abs(y)>2.)]=100
        out=OccupancyGrid();out.header.frame_id='map';out.header.stamp=self.get_clock().now().to_msg()
        out.info.resolution=resolution;out.info.width=l.shape[1];out.info.height=l.shape[0];out.info.origin.position.x=bounds[0];out.info.origin.position.y=bounds[1];out.info.origin.orientation.w=1.;out.data=cells.ravel().astype(int).tolist()
        self.pub.publish(out)

def main():
    rclpy.init();n=BoundedMap()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
