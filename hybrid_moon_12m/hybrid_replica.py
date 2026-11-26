"""Visual-only replica using locally republished real observations."""
import math,time,json
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseStamped,TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

class Replica:
    def init(self,webots_node,properties):
        self.robot=webots_node.robot;self.body=self.robot.getSelf()
        self.translation=self.body.getField('translation');self.rotation=self.body.getField('rotation')
        initial=json.loads(Path(__file__).with_name('config').joinpath('environment.json').read_text())['real_offset']
        self.translation.setSFVec3f([initial[0],initial[1],.1482])
        self.rotation.setSFRotation([0.,0.,1.,initial[2]])
        if not rclpy.ok():rclpy.init()
        self.node=rclpy.create_node('hybrid_real_replica',use_global_arguments=False)
        self.last=None;self.pose=None;self.last_status=0.;self.snapshot_due=None;self.snapshot_done=False
        self.sub=self.node.create_subscription(PoseStamped,'/hybrid/real_pose',self.receive,10)
        self.pub=self.node.create_publisher(String,'/digital_twin/status',10)
        self.tf=TransformBroadcaster(self.node)
    def receive(self,msg):self.last=time.monotonic();self.pose=msg
    def step(self):
        rclpy.spin_once(self.node,timeout_sec=0.)
        now=time.monotonic();fresh=self.last is not None and now-self.last<1.5
        if fresh:
            p=self.pose.pose.position;q=self.pose.pose.orientation
            self.translation.setSFVec3f([p.x,p.y,.1482]);self.rotation.setSFRotation([0.,0.,1.,2*math.atan2(q.z,q.w)])
            if self.snapshot_due is None:self.snapshot_due=now+.5
            if not self.snapshot_done and now>=self.snapshot_due:
                self.robot.exportImage('/tmp/replica_webots_direction.png',100);self.snapshot_done=True
            t=TransformStamped();t.header=self.pose.header;t.child_frame_id='robot1/base_link'
            t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.rotation=q;self.tf.sendTransform(t)
        if now-self.last_status>.5:
            self.last_status=now
            self.pub.publish(String(data='following_real_pose' if fresh else 'waiting_for_readonly_real_observer'))
