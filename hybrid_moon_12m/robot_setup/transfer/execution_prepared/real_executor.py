"""Physical Nav2 relay/gate. Disabled by default; requires explicit operator arming."""
import argparse,json,math,os,socket,time,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path as NavPath, OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from limits import PhysicalLimits,local_pose,map_pose
from command_wire import pack,Reader,MAX_FRAME
from nav_adapter import Navigation
from scan_evidence import ScanEvidence
from swept_map_guard import swept_clear
from readiness import HoldWindow
from rclpy.qos import QoSProfile,DurabilityPolicy

class Executor(Node):
    def __init__(self,args):
        super().__init__('hybrid_physical_executor')
        self.allowed=bool(args.allow_motion and os.environ.get('LIMO_USER_APPROVED')=='YES')
        self.key=Path(args.key_file).read_bytes()
        if len(self.key)<32:raise ValueError('Pairing key must contain at least 32 bytes')
        self.reference_path=Path(args.reference)
        self.anchor=None;self.reference=None;self.peer_ip=args.peer_ip
        self.limits=PhysicalLimits(bounds=(-2.,-2.,2.,2.),max_linear=.20,max_angular=.70);self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        self.nav=Navigation(self,'/navigate_to_pose','map');self.pose=None;self.pose_age=math.inf
        self.last_output = (0., 0.)
        self.latest_cmd=Twist();self.cmd_time=-math.inf;self.last_packet=-math.inf;self.path_time=-math.inf
        self.nav_path=[];self.scan=None;self.scan_time=-math.inf;self.task_id=None;self.command_active=False
        self.nav_map=None;self.nav_map_time=-math.inf;self.world_pose=None
        self.map_sub=self.create_subscription(OccupancyGrid,'/hybrid/physical_nav_map',self.map_cb,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan_evidence=ScanEvidence();self.scan_reason='NOT_CHECKED';self.guard_details={}
        self.remote_hold=False;self.remote_hold_window=HoldWindow();self.input_hold_window=HoldWindow()
        self.reason='DISARMED';self.reference_fault=False;self.last_pose=None;self.last_pose_at=None
        self.base_node = args.dashboard_base
        self.output = self.create_publisher(Twist, args.output_topic, 10)
        self.cmd_sub=self.create_subscription(Twist,'/hybrid/nav_request',self.cmd,10)
        self.path_sub=self.create_subscription(NavPath,'/plan',self.path,10)
        self.scan_sub=self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.server=socket.socket();self.server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);self.server.bind(('0.0.0.0',args.port));self.server.listen(1);self.server.setblocking(False)
        self.peer=None;self.rx=b'';self.tx=b'';self.reader=None;self.sequence=0;self.session=uuid.uuid4().hex
        self.create_timer(.05,self.tick)
        self.get_logger().info('Physical executor '+('OPERATOR-ARMED' if self.allowed else 'DISARMED'))
    def map_cb(self,m):self.nav_map=m;self.nav_map_time=time.monotonic()
    def cmd(self,m):self.latest_cmd=m;self.cmd_time=time.monotonic()
    def scan_cb(self,m):
        self.scan=m;self.scan_time=time.monotonic()
        self.scan_evidence.update(m,self.pose,self.scan_time)
    def path(self,m):
        if self.anchor is None or m.header.frame_id!='map':return
        self.nav_path=[local_pose((p.pose.position.x,p.pose.position.y,0.),self.anchor)[:2] for p in m.poses]
        self.path_time=time.monotonic()
    def zero(self,reason):
        self.output.publish(Twist())
        self.last_output = (0., 0.)
        self.reason = reason
    def cancel(self,reason):
        self.command_active=False;self.nav.cancel();self.zero(reason)
    def disconnect(self):
        self.cancel('LINK_LOST')
        if self.peer:self.peer.close()
        self.peer=None;self.rx=b'';self.tx=b'';self.reader=None
    def pose_update(self,now):
        if self.anchor is None and self.reference_path.is_file():
            data=json.loads(self.reference_path.read_text());self.anchor=data['anchor'];self.reference=data['id']
            if len(self.anchor)!=3 or not all(math.isfinite(x) for x in self.anchor):raise ValueError('Invalid reference')
        try:
            tf=self.buffer.lookup_transform('map','base_link',rclpy.time.Time())
            stamp=tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
            self.pose_age=self.get_clock().now().nanoseconds/1e9-stamp
            q=tf.transform.rotation;p=tf.transform.translation
            world=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            if self.anchor is None:return
            self.world_pose=world
            current=local_pose(world,self.anchor)
            if self.last_pose is not None and self.last_pose_at is not None:
                dt=now-self.last_pose_at
                if dt>0 and math.dist(current[:2],self.last_pose[:2])>.08+dt*.12:self.reference_fault=True
            self.last_pose,self.last_pose_at=current,now;self.pose=current
        except Exception:self.pose_age=math.inf
        self.nav.pose_ready=bool(self.pose is not None and -.25<=self.pose_age<=.5 and not self.reference_fault)
    def receive_command(self,payload,now):
        kind=payload.get('kind')
        if kind=='stop':self.last_packet=now;self.cancel('OPERATOR_STOP');return
        if kind=='hold':
            if self.allowed and self.command_active and payload.get('task_id')==self.task_id and payload.get('reference')==self.reference:
                self.last_packet=now;self.remote_hold=True;self.remote_hold_window.waiting(now)
            return
        if kind!='goal':return
        if not self.allowed or self.reference_fault:self.cancel('DISARMED_OR_REFERENCE_FAULT');return
        if payload.get('reference')!=self.reference:self.cancel('REFERENCE_MISMATCH');return
        goal=payload.get('goal');route=payload.get('path')
        if not isinstance(goal,list) or len(goal)!=3 or not self.limits.path_safe(route) or not self.limits.pose_safe(goal,self.limits.reserve):
            self.cancel('OUTSIDE_REAL_REGION');return
        task=payload.get('task_id')
        if not isinstance(task,str) or len(task)>200:return
        # A same-task heartbeat must not cancel the goal during a short TF gap.
        if task==self.task_id:
            self.last_packet=now;self.remote_hold=False;self.remote_hold_window.reset();return
        if not self.nav.ready():self.cancel('POSE_OR_NAV2_UNAVAILABLE');return
        self.last_packet=now;self.remote_hold=False;self.remote_hold_window.reset()
        if self.nav.poll() in ('SENDING','NAVIGATING','CANCELING'):
            self.cancel('WAITING_PREVIOUS_GOAL_CANCEL');return
        self.task_id=task;self.nav_path=[];self.path_time=-math.inf
        self.last_output = (0., 0.)
        self.latest_cmd=Twist();self.cmd_time=-math.inf
        self.command_active=True;self.nav.start(map_pose(goal,self.anchor))
    def networking(self,now):
        if self.peer is None:
            try:
                peer,address=self.server.accept()
                if address[0]!=self.peer_ip:peer.close();return
                self.peer=peer;peer.setblocking(False);self.reader=Reader(self.key);self.rx=b''
            except BlockingIOError:return
        try:
            try:
                block=self.peer.recv(MAX_FRAME)
                if not block:self.disconnect();return
                self.rx+=block
            except BlockingIOError:pass
            if len(self.rx)>MAX_FRAME:raise ValueError('Overlong frame')
            while b'\n' in self.rx:
                frame,self.rx=self.rx.split(b'\n',1)
                self.receive_command(self.reader.read(frame),now)
            self.sequence+=1
            if not self.tx:
                status={'reference':self.reference,'ready':self.allowed and self.nav.ready() and self.limits.pose_safe(self.pose or (math.inf,math.inf),self.limits.reserve),'state':self.nav.poll(),'task_id':self.task_id,'reason':self.reason,'pose':self.pose,'armed':self.allowed,'guard_version':'stable_readiness_v5','pose_ready':self.nav.pose_ready,'action_server_ready':self.nav.client.server_is_ready(),'reference_fault':self.reference_fault,'dashboard_route_ready':self.dashboard_route_ready() if self.base_node else None,'remote_hold':self.remote_hold,'guard_details':self.guard_details,'scan_guard':self.scan_reason,'pose_age':self.pose_age if math.isfinite(self.pose_age) else None,'cmd_age':now-self.cmd_time if math.isfinite(self.cmd_time) else None,'path_age':now-self.path_time if math.isfinite(self.path_time) else None,'scan_age':now-self.scan_time if math.isfinite(self.scan_time) else None}
                requested = (self.latest_cmd.linear.x, self.latest_cmd.angular.z)
                status['requested_velocity'] = requested if all(map(math.isfinite, requested)) else None
                status['published_velocity'] = self.last_output
                self.tx=pack(status,self.key,self.sequence,self.session)
            try:
                sent=self.peer.send(self.tx);self.tx=self.tx[sent:]
            except BlockingIOError:pass
        except (OSError,ValueError,KeyError,TypeError):self.disconnect()
    def scan_safe(self,v,w,now):
        if self.scan is None or now-self.scan_time>.3:
            self.scan_reason='LIDAR_STREAM_STALE';return False
        ok,self.scan_reason=self.scan_evidence.check(self.scan,self.pose,now,v,w,self.limits.radius)
        if ok:return True
        if self.scan_reason not in ('LIDAR_UNOBSERVED_RAY','LIDAR_EVIDENCE_EXPIRED'):return False
        # A missing ray cannot hide a known close return elsewhere in the scan.
        valid=0
        for i,value in enumerate(self.scan.ranges):
            d=float(value)
            if d==0:continue
            if not math.isfinite(d) or not self.scan.range_min<=d<=self.scan.range_max:
                self.scan_reason='LIDAR_INVALID_NONZERO';return False
            valid+=1;a=self.scan.angle_min+i*self.scan.angle_increment
            if math.hypot(.103+d*math.cos(a),d*math.sin(a))<self.limits.radius+.04:
                self.scan_reason='LIDAR_OBSTACLE_CLOSE';return False
        if not valid:self.scan_reason='LIDAR_NO_VALID_RETURNS';return False
        if self.world_pose is None or now-self.nav_map_time>.8:
            self.scan_reason='SWEPT_MAP_OR_POSE_STALE';return False
        self.guard_details={}
        ok,self.scan_reason=swept_clear(self.nav_map,self.world_pose,v,w,self.limits.radius,self.guard_details)
        return ok
    def dashboard_route_ready(self):
        topic = self.output.topic_name
        subscribers = self.get_subscriptions_info_by_topic(topic)
        publishers = self.get_publishers_info_by_topic(topic)
        return (
            len(subscribers) == 1
            and subscribers[0].node_namespace.rstrip('/') + '/' + subscribers[0].node_name == self.base_node
            and subscribers[0].topic_type == 'geometry_msgs/msg/Twist'
            and len(publishers) == 1
            and publishers[0].node_name == self.get_name()
            and self.output.get_subscription_count() == 1
        )

    def tick(self):
        now=time.monotonic()
        try:self.pose_update(now);self.networking(now)
        except Exception as e:self.cancel('INPUT_ERROR');self.get_logger().warning(str(e));return
        if not self.allowed:self.cancel('DISARMED');return
        if now-self.last_packet>.6:self.cancel('COMMAND_LEASE_EXPIRED');return
        if not self.command_active or self.nav.poll() not in ('SENDING','NAVIGATING'):
            self.zero('IDLE');return
        if self.reference_fault:self.cancel('REFERENCE_FAULT');return
        if self.remote_hold:
            if self.remote_hold_window.waiting(now):self.zero('HOLDING_GOAL_FOR_REMOTE_INPUT')
            else:self.cancel('REMOTE_INPUT_HOLD_TIMEOUT')
            return
        if not self.nav.pose_ready or self.pose is None:
            if self.input_hold_window.waiting(now):self.zero('HOLDING_GOAL_FOR_POSE')
            else:self.cancel('POSE_HOLD_TIMEOUT')
            return
        self.input_hold_window.reset()
        if not self.limits.pose_safe(self.pose,self.limits.reserve):self.cancel('BOUNDARY_GUARD');return
        if now-self.path_time>1.5 or not self.limits.path_safe(self.nav_path):self.zero('WAITING_VALID_NAV2_PATH');return
        if now-self.cmd_time>.3:self.zero('WAITING_FRESH_VELOCITY');return
        v,w=self.latest_cmd.linear.x,self.latest_cmd.angular.z
        if not self.limits.velocity_safe(self.pose,v,w):self.zero('VELOCITY_OR_PREDICTED_BOUNDARY_GUARD');return
        if not self.scan_safe(v,w,now):self.zero(self.scan_reason);return
        if self.base_node and not self.dashboard_route_ready():
            self.cancel('DASHBOARD_VELOCITY_ROUTE_UNAVAILABLE');return
        self.output.publish(self.latest_cmd)
        self.last_output = (v, w)
        self.reason = 'FOLLOWING_APPROVED_GOAL'

def main():
    p=argparse.ArgumentParser();p.add_argument('--key-file',required=True);p.add_argument('--reference',default=str(Path(__file__).with_name('real_reference.json')));p.add_argument('--peer-ip',default='172.20.10.2');p.add_argument('--port',type=int,default=17444);p.add_argument('--allow-motion',action='store_true')
    p.add_argument('--dashboard-base', default='')
    p.add_argument('--output-topic', default='/hybrid/preparation_disabled_cmd_vel')
    a, ros = p.parse_known_args()
    if a.dashboard_base and (not a.dashboard_base.startswith('/') or not a.output_topic.startswith('/')
                             or a.output_topic.startswith('/hybrid/') or not a.output_topic.endswith('/cmd_vel')):
        p.error('Use the dashboard base node and its full cmd_vel input topic')
    rclpy.init(args=ros);n=Executor(a)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.cancel('SHUTDOWN');n.output.publish(Twist());n.server.close()
        if n.peer:n.peer.close()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
