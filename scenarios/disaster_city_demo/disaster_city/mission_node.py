"""Team BT, per-robot asynchronous Nav2 executors and public state service."""
import os
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker,MarkerArray
from webots_ros2_limo.exploration_bt import Leaf,sequence,Status
from .engine import Mission
from .grid import CityGrid
from .policy import Baseline

ROOT=Path(__file__).resolve().parent


class Runtime(Node):
    def __init__(self):
        super().__init__('city_task_manager',parameter_overrides=[])
        profile=Path(os.environ.get('CITY_SCENARIO_DIR',str(ROOT/'profiles/junyong_10_damage')))
        self.cfg=json.loads((profile/'scenario.json').read_text());self.grid=CityGrid(profile/'city.pgm')
        from .plugins import load_policy
        self.engine=Mission(self.cfg,self.grid)
        self.policy=load_policy(self.grid,self.cfg,
            allocator=os.environ.get('CITY_ALLOCATOR'), model=os.environ.get('CITY_PPO_MODEL'))
        self.engine.policy_name=getattr(self.policy,'policy_name','BASELINE | nearest feasible team')
        self.nav_clients=[ActionClient(self,NavigateToPose,f'/robot{i+1}/navigate_to_pose') for i in range(len(self.cfg["starts"]))]
        self.nav_states=[False]*len(self.cfg["starts"])
        self.nav_state_clients=[self.create_client(GetState,f'/robot{i+1}/bt_navigator/get_state') for i in range(len(self.cfg['starts']))]
        self.nav_checks={};self.nav_check_at=-1e9
        for i in range(len(self.cfg["starts"])):
            self.create_subscription(TransitionEvent,f'/robot{i+1}/bt_navigator/transition_event',
                lambda m,i=i:self.nav_states.__setitem__(i,m.goal_state.id==3),10)
        self.stops=[self.create_publisher(Twist,f'/robot{i+1}/cmd_vel',10) for i in range(len(self.cfg["starts"]))]
        self.handles={};self.versions={};self.sending=set();self.progress={};self.last_pose=-1e9
        self.last_time=None;self.plan_at=-1e9;self.pub_at=-1e9;self.started=False;self.event_index=0
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher=self.create_publisher(String,'/disaster/observation',qos)
        self.markers=self.create_publisher(MarkerArray,'/disaster/markers',qos)
        self.create_subscription(String,'/disaster/sim_poses',self.pose,10)
        self.team=sequence('TeamBT',Leaf('FreshFleet',self.fresh),Leaf('AssignFromPublicState',self.plan))
        self.robot_trees=[sequence(f'Robot{i+1}BT',Leaf('Nav2Ready',lambda i=i:Status.SUCCESS if self.nav_clients[i].server_is_ready() else Status.RUNNING),
                          Leaf('ExecuteTaskOrCharge',lambda i=i:self.execute(i))) for i in range(len(self.cfg["starts"]))]
        self.create_timer(.2,self.tick)

    def pose(self,m):
        data=json.loads(m.data);self.last_pose=time.monotonic()
        for i,r in enumerate(data['robots']):self.engine.pose(i,r['xy'],r['yaw'],r['speed'])

    def fresh(self):
        return Status.SUCCESS if time.monotonic()-self.last_pose<1. and all(r['ready'] for r in self.engine.robots) else Status.RUNNING

    def plan(self):
        if self.engine.time<self.plan_at+self.cfg.get("decision_period_s",1.):return Status.SUCCESS
        self.plan_at=self.engine.time
        proposals=self.policy.decide(self.engine.observe())
        if hasattr(self.policy,'last_action'):
            self.engine.event('policy_action',action=self.policy.last_action)
        for i,role,target,task,slot in proposals:
            if role=='release_team':self.engine.release_team(task)
            elif role=='charge_request':self.engine.charge(i)
            else:self.engine.assign(i,role,target,task,slot)
        return Status.SUCCESS

    def execute(self,i):
        r=self.engine.robots[i];version=r['version']
        if i in self.sending:return Status.RUNNING
        if self.versions.get(i)!=version:
            if i in self.handles:
                handle=self.handles.pop(i);self.sending.add(i)
                def cancelled(_):self.sending.discard(i)
                handle.cancel_goal_async().add_done_callback(cancelled)
                return Status.RUNNING
            self.versions[i]=version
            if r['target'] is None:self.stops[i].publish(Twist());return Status.SUCCESS
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
            goal.pose.pose.position.x=float(r['target'][0]);goal.pose.pose.position.y=float(r['target'][1])
            yaw=math.atan2(r['target'][1]-r['xy'][1],r['target'][0]-r['xy'][0])
            goal.pose.pose.orientation.z=math.sin(yaw/2);goal.pose.pose.orientation.w=math.cos(yaw/2)
            self.sending.add(i);self.progress[i]=(self.engine.time,list(r['xy']))
            def accepted(f):
                self.sending.discard(i)
                try:h=f.result()
                except Exception:self.engine.failed(i,version);return
                if not h.accepted:self.engine.failed(i,version);return
                self.handles[i]=h;r['nav_status']='navigating'
                def done(f):
                    if self.handles.get(i) is h:self.handles.pop(i,None)
                    if r['version']!=version:return
                    try:status=f.result().status
                    except Exception:status=6
                    if status==4:r['nav_status']='arrived'
                    elif status!=5:self.engine.failed(i,version)
                h.get_result_async().add_done_callback(done)
            self.nav_clients[i].send_goal_async(goal).add_done_callback(accepted)
        # Applies ONLY while actually navigating, never gathering/working/charging.
        if r['nav_status']=='navigating' and r['state']=='moving' and i in self.progress:
            since,p=self.progress[i]
            if math.dist(p,r['xy'])>.15:self.progress[i]=(self.engine.time,list(r['xy']))
            elif self.engine.time-since>45.:
                self.engine.failed(i,version)
        return Status.RUNNING if r['target'] is not None else Status.SUCCESS

    def tick(self):
        # Transition events may precede discovery while a PPO model is loading.
        # Verify actual lifecycle state until startup, without changing that state.
        wall=time.monotonic()
        if not self.started and wall-self.nav_check_at>=2.:
            self.nav_check_at=wall
            for i,client in enumerate(self.nav_state_clients):
                if i in self.nav_checks or not client.service_is_ready():continue
                future=client.call_async(GetState.Request());self.nav_checks[i]=(future,wall)
                def checked(f,i=i):
                    self.nav_checks.pop(i,None)
                    try:self.nav_states[i]=f.result().current_state.id==3
                    except Exception:pass
                future.add_done_callback(checked)
            for i,(future,since) in list(self.nav_checks.items()):
                if wall-since>5.:
                    future.cancel();self.nav_checks.pop(i,None)
        now=self.get_clock().now().nanoseconds/1e9
        dt=0. if self.last_time is None else max(0.,min(1.,now-self.last_time));self.last_time=now
        if not self.started and self.fresh()==Status.SUCCESS and all(self.nav_states) and all(c.server_is_ready() for c in self.nav_clients):
            self.started=True;self.engine.phase='RUNNING';self.get_logger().info('All configured Nav2 servers ready; city mission started')
        if self.started and self.engine.phase=='RUNNING':
            if self.fresh()==Status.SUCCESS:
                self.engine.advance(dt);self.team.tick()
                for tree in self.robot_trees:tree.tick()
            else:
                for pub in self.stops:pub.publish(Twist())
        elif self.started:
            for i in range(len(self.cfg["starts"])):
                self.engine.assign(i,'idle');self.robot_trees[i].tick();self.stops[i].publish(Twist())
        if now-self.pub_at<.5:return
        self.pub_at=now;obs=self.engine.observe();obs['team_bt']=self.team.snapshot()
        obs['readiness']=dict(nav_active=self.nav_states,pose_age_s=time.monotonic()-self.last_pose,
                              action_servers=[c.server_is_ready() for c in self.nav_clients])
        obs['robot_bts']=[t.snapshot() for t in self.robot_trees]
        self.publisher.publish(String(data=json.dumps(obs)))
        (ROOT/'runtime/latest.json').write_text(json.dumps(obs,indent=2))
        if self.engine.phase in ('COMPLETE','TIME_LIMIT','BATTERY_FAILURE'):
            (ROOT/'runtime/result.json').write_text(json.dumps(dict(backend='webots',
                phase=self.engine.phase,time_s=self.engine.time,completed=sum(j['state']=='done' for j in self.engine.jobs.values()),
                total=len(self.cfg['tasks']),seed=self.cfg.get('seed'),robots=len(self.cfg['starts']),
                policy=self.engine.policy_name,model=getattr(self.policy,'model_path',None),
                model_sha256=getattr(self.policy,'model_sha256',None),
                fire_growth_per_s=self.cfg.get('fire_growth_per_s',0.)),indent=2))
        if self.event_index<len(self.engine.events):
            with (ROOT/'runtime/events.jsonl').open('a') as f:
                for e in self.engine.events[self.event_index:]:f.write(json.dumps(e)+'\n')
            self.event_index=len(self.engine.events)
        markers=MarkerArray()
        for r in obs['robots']:
            m=Marker();m.header.frame_id='map';m.ns='robots';m.id=r['id'];m.type=Marker.ARROW;m.action=Marker.ADD
            m.pose.position.x=float(r['xy'][0]);m.pose.position.y=float(r['xy'][1]);m.pose.position.z=.2
            m.pose.orientation.z=math.sin(r['yaw']/2);m.pose.orientation.w=math.cos(r['yaw']/2)
            m.scale.x=.6;m.scale.y=.25;m.scale.z=.2;m.color.r=.3;m.color.g=.65;m.color.b=.9;m.color.a=1.;markers.markers.append(m)
        for key,j in obs['tasks'].items():
            m=Marker();m.header.frame_id='map';m.ns='tasks';m.id=int(key);m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
            m.pose.position.x=float(j['xy'][0]);m.pose.position.y=float(j['xy'][1]);m.pose.position.z=.8;m.pose.orientation.w=1.
            m.scale.z=.4;m.color.r=.2;m.color.g=.6;m.color.b=.2;m.color.a=1.
            m.text=f"{j['kind']} {100*j['progress_s']/j['seconds']:.0f}% ({j['working']}/{j['minimum']})";markers.markers.append(m)
        self.markers.publish(markers)


def main():
    rclpy.init();node=Runtime()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        for pub in node.stops:pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
