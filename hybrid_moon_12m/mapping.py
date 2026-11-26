"""Stationary hybrid mapping preparation; owns no motion/action interface."""
import os,json,math,socket,time,uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Bool,String,ColorRGBA
from visualization_msgs.msg import Marker
from tf2_ros import Buffer,TransformListener
from ownership import Layout,project_grid,compose,clear_virtual_bodies
from limits import observation_offset
from map_wire import decode_map,MAX_PACKET

class Mapping(Node):
    def __init__(self):
        super().__init__('hybrid_mapping_prepare')
        config=json.loads((Path(__file__).parent/'config/environment.json').read_text())
        self.preparation_only=bool(self.declare_parameter('preparation_only',True).value)
        self.layout=Layout(bounds=tuple(config['bounds']),real_bounds=tuple(config['real_bounds']),resolution=config['resolution'],real_offset=tuple(config['real_offset']))
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.socket.bind(('0.0.0.0',config['real_map_udp_port']));self.socket.setblocking(False)
        self.display_anchor=None;self.display_reference_id=None;self.display_alignment_id=uuid.uuid4().hex;self.display_offset=None
        self.ip=os.environ.get('HYBRID_ROBOT_IP', config['real_robot_ip']);self.real=None;self.last_real=None;self.sequence=-1;self.session=None
        self.virtual={};self.scores={};self.scan_times={};self.subscriptions_keep=[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.visual=self.create_publisher(Marker,'/hybrid/map_cells',qos)
        self.output=self.create_publisher(OccupancyGrid,'/hybrid/map',qos)
        self.nav_sim=self.create_publisher(OccupancyGrid,'/hybrid/sim_nav_map',qos)
        self.nav_real=self.create_publisher(OccupancyGrid,'/hybrid/real_nav_map',qos)
        self.metadata=self.create_publisher(String,'/hybrid/real_metadata',qos)
        self.pose=self.create_publisher(PoseStamped,'/hybrid/real_pose',10)
        self.stop=self.create_publisher(Bool,'/multi_bt/emergency_stop',qos)
        self.state=self.create_publisher(String,'/hybrid/preparation_status',qos)
        for name in config['sim_starts']:
            self.scores[name]=np.zeros(self.layout.shape,dtype=np.int16)
            self.virtual[name]=np.full(self.layout.shape,-1,dtype=np.int16)
            self.subscriptions_keep.append(self.create_subscription(LaserScan,f'/{name}/scan_mapping',lambda m,n=name:self.scan(n,m),qos_profile_sensor_data))
        self.create_timer(.1,self.receive)
        self.create_timer(.5,self.publish)

    def scan(self,name,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if stamp-self.scan_times.get(name,-100.)<.5:return
        try:
            tf=self.buffer.lookup_transform('map',msg.header.frame_id,rclpy.time.Time())
        except Exception:return
        p,q=tf.transform.translation,tf.transform.rotation
        # Reject mapping while tilted; 2-D projection is not slope-aware SLAM.
        roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
        pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        if max(abs(roll),abs(pitch))>math.radians(5):return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.scan_times[name]=stamp
        score=self.scores[name];grid=self.virtual[name];l=self.layout
        try:
            base=self.buffer.lookup_transform('map',name+'/base_link',rclpy.time.Time()).transform.translation
            rr,cc=np.indices(l.shape)
            under_robot=(l.bounds[0]+(cc+.5)*l.resolution-base.x)**2+(l.bounds[1]+(rr+.5)*l.resolution-base.y)**2<=.30**2
            grid[under_robot]=0;score[under_robot]=-15
        except Exception:pass
        for i in range(0,len(msg.ranges),3):
            value=float(msg.ranges[i])
            # Zero/NaN are invalid, not proof of an empty ray.
            if math.isnan(value) or value<msg.range_min:continue
            hit=math.isfinite(value) and value<min(msg.range_max,6.)
            length=min(value,msg.range_max,6.)
            if length<=0:continue
            angle=yaw+msg.angle_min+i*msg.angle_increment
            ds=np.arange(0.,length,l.resolution*.7)
            if hit:ds=np.append(ds,length)
            cols=np.floor((p.x+ds*math.cos(angle)-l.bounds[0])/l.resolution).astype(int)
            rows=np.floor((p.y+ds*math.sin(angle)-l.bounds[1])/l.resolution).astype(int)
            valid=(cols>=0)&(cols<l.shape[1])&(rows>=0)&(rows<l.shape[0])
            cells=list(dict.fromkeys(zip(rows[valid].tolist(),cols[valid].tolist())))
            for k,(row,col) in enumerate(cells):
                endpoint=hit and k==len(cells)-1 and bool(valid[-1])
                score[row,col]=np.clip(score[row,col]+(5 if endpoint else -1),-15,15)
                grid[row,col]=100 if score[row,col]>1 else 0

    def receive(self):
        for _ in range(8):
            try:raw,address=self.socket.recvfrom(MAX_PACKET+1)
            except BlockingIOError:return
            if address[0]!=self.ip or len(raw)>MAX_PACKET:continue
            try:
                packet=json.loads(raw)
                if packet['version']!=1:continue
                session=packet['session'];seq=packet['sequence']
                if not isinstance(session,str) or not isinstance(seq,int):continue
                if session==self.session and seq<=self.sequence:continue
                w,h,res,origin,data=decode_map(packet['map'])
                reference=packet.get('reference')
                pose=packet.get('pose')
                pose_valid=isinstance(pose,list) and len(pose)==3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in pose)
                reference_id=reference.get('id') if isinstance(reference,dict) else None
                if self.display_anchor is None:
                    anchor = reference.get('anchor') if reference_id else None
                    if not isinstance(anchor, list) or len(anchor) != 3:
                        continue
                    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in anchor):
                        continue
                    self.display_anchor = tuple(anchor)
                    self.display_reference_id = reference_id
                elif reference_id!=self.display_reference_id:
                    self.get_logger().warning('Real reference changed; restart while stationary to realign')
                    continue
                offset=observation_offset(self.layout.real_offset,self.display_anchor)
                self.display_offset=offset
                real=project_grid(np.array(data).reshape(h,w),res,origin,self.layout,offset)
                self.real=real;self.sequence=seq;self.session=session;self.last_real=time.monotonic()
                if pose_valid:
                    x,y,a=pose;dx,dy,theta=offset
                    m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
                    m.pose.position.x=dx+math.cos(theta)*x-math.sin(theta)*y
                    m.pose.position.y=dy+math.sin(theta)*x+math.cos(theta)*y
                    m.pose.orientation.z=math.sin((a+theta)/2);m.pose.orientation.w=math.cos((a+theta)/2)
                    self.pose.publish(m)
                self.metadata.publish(String(data=json.dumps({'reference':reference,'display_offset':self.display_offset,'display_alignment_id':self.display_alignment_id,'pose_received':pose_valid,'receive_monotonic':self.last_real})))
            except (ValueError,KeyError,TypeError,OverflowError):continue

    def message(self,cells):
        l=self.layout;m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.info.resolution=l.resolution;m.info.width=l.shape[1];m.info.height=l.shape[0]
        m.info.origin.position.x=l.bounds[0];m.info.origin.position.y=l.bounds[1];m.info.origin.orientation.w=1.
        m.data=cells.ravel().astype(int).tolist();return m

    def publish(self):
        # Preparation mode is intentionally incapable of enabling motion.
        if self.preparation_only:self.stop.publish(Bool(data=True))
        combined=compose(self.layout,self.real,self.virtual.values())
        bodies=[]
        for name in self.virtual:
            try:
                t=self.buffer.lookup_transform('map',name+'/base_link',rclpy.time.Time())
                age=(self.get_clock().now().nanoseconds-(t.header.stamp.sec*10**9+t.header.stamp.nanosec))/1e9
                if -.25<=age<=.7:bodies.append((t.transform.translation.x,t.transform.translation.y))
            except Exception:pass
        combined=clear_virtual_bodies(combined,self.layout,bodies)
        self.output.publish(self.message(combined))
        visible=Marker();visible.header.frame_id='map';visible.header.stamp=self.get_clock().now().to_msg()
        visible.ns='observed_map';visible.id=0;visible.type=Marker.CUBE_LIST;visible.action=Marker.ADD
        visible.pose.orientation.w=1.;visible.scale.x=self.layout.resolution;visible.scale.y=self.layout.resolution;visible.scale.z=.008
        rows,cols=np.nonzero(combined>=0)
        for row,col in zip(rows,cols):
            visible.points.append(Point(x=self.layout.bounds[0]+(float(col)+.5)*self.layout.resolution,y=self.layout.bounds[1]+(float(row)+.5)*self.layout.resolution,z=.005))
            shade=1.-float(combined[row,col])/100.
            visible.colors.append(ColorRGBA(r=shade,g=shade,b=shade,a=1.))
        self.visual.publish(visible)
        region=Marker();region.header=visible.header;region.ns='real_region';region.id=1;region.type=Marker.LINE_STRIP;region.action=Marker.ADD;region.pose.orientation.w=1.;region.scale.x=.04;region.color=ColorRGBA(r=1.,g=.25,b=.1,a=1.)
        a,b,c,d=self.layout.real_bounds
        region.points=[Point(x=float(x),y=float(y),z=.04) for x,y in [(a,b),(c,b),(c,d),(a,d),(a,b)]]
        self.visual.publish(region)
        mask=self.layout.real_mask();sim=combined.copy();sim[mask]=100
        real=combined.copy();real[~mask]=100
        self.nav_sim.publish(self.message(sim));self.nav_real.publish(self.message(real))
        age=None if self.last_real is None else time.monotonic()-self.last_real
        self.state.publish(String(data=json.dumps({'mode':'PREPARATION_ONLY','real_motion_enabled':False,'real_map_received':self.real is not None,'real_receive_age_s':age,'sim_scans_received':list(self.scan_times),'map_observed_cells':int((combined>=0).sum()),'all_sim_sensors_ready':len(self.scan_times)==3,'requires_user_approval_before_real_motion':True})))

def main():
    rclpy.init();n=Mapping()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.socket.close();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
