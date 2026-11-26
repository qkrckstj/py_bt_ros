"""Owned frontier mission. This program is never started by run_prepare.sh."""
import argparse,fcntl,json,math,os,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid,Path as NavPath
from std_msgs.msg import Bool,Empty,String
from geometry_msgs.msg import Twist
from tf2_ros import Buffer,TransformListener
from chanseo_exploration.exploration_policy import FrontierGrid
from chanseo_exploration.exploration_policy import allocate
from chanseo_exploration.exploration_runtime import TeamBT,RobotBT
from nav_adapter import Navigation
from ownership import Layout
from limits import PhysicalLimits,display_to_reference,map_pose
from remote_nav import LinkService,RemoteNavigation
from readiness import StableNavigation
from hybrid_team import HybridTeamBT

class Coordinator(Node):
    def __init__(self,args):
        super().__init__('hybrid_mission')
        self.physical=not args.virtual_only
        self.trial_seconds=float(os.environ.get('HYBRID_TRIAL_SECONDS','0'))
        self.trial_deadline=math.inf;self.trial_started=False
        if self.physical and (not args.allow_physical or os.environ.get('LIMO_USER_APPROVED')!='YES'):
            raise RuntimeError('Physical mission requires explicit user approval and --allow-physical')
        config = json.loads(Path(__file__).with_name("config").joinpath("environment.json").read_text())
        self.layout = Layout(bounds=tuple(config["bounds"]), real_bounds=tuple(config["real_bounds"]),
                             resolution=config["resolution"], real_offset=tuple(config["real_offset"]))
        self.grid=None;self.grid_at=-math.inf;self.reference=None;self.reference_at=-math.inf;self.display_offset=None;self.alignment_id=None
        self.names=['robot1','robot2','robot3','robot4'] if self.physical else ['robot2','robot3','robot4']
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.stop_pub=self.create_publisher(Bool,'/multi_bt/emergency_stop',q)
        self.status=self.create_publisher(String,'/hybrid/mission_status',q)
        self.velocity={};self.paths={};self.positions={};self.yaws={};self.progress={};self.last_report=0.
        self.cmd_pubs={n:self.create_publisher(Twist,f'/{n}/cmd_vel',10) for n in self.names if n!='robot1'}
        self.subs=[self.create_subscription(OccupancyGrid,'/hybrid/map',self.map,q),self.create_subscription(String,'/hybrid/real_metadata',self.metadata,q),self.create_subscription(Empty,'/hybrid/mission/start',self.start,10),self.create_subscription(Empty,'/hybrid/mission/stop',self.stop,10)]
        for name in self.cmd_pubs:
            self.subs+=[self.create_subscription(Twist,f'/{name}/cmd_vel_requested',lambda m,n=name:self.velocity.__setitem__(n,(m,time.monotonic())),10),self.create_subscription(NavPath,f'/{name}/plan',lambda m,n=name:self.paths.__setitem__(n,(m,time.monotonic())),10)]
        self.nav={n:Navigation(self,f'/{n}/navigate_to_pose','map') for n in self.names if n!='robot1'}
        if self.physical:
            key=Path(args.key_file).read_bytes()
            if len(key)<32:raise ValueError('Pairing key must contain at least 32 bytes')
            self.link=LinkService(os.environ.get('HYBRID_ROBOT_IP', config['real_robot_ip']),17444,key);self.nav['robot1']=RemoteNavigation(self.link,self.real_route)
        self.nav={name:StableNavigation(nav) for name,nav in self.nav.items()}
        self.unobserved_task = None
        self.unobserved_since = 0.
        self.deferred_real_goals = {}
        self.last_poses={};self.readiness_diagnostics={}
        self.robots={n:RobotBT(self.nav[n],lease=1.,observe_seconds=.5) for n in self.names}
        self.bt=HybridTeamBT(self.names,self.command,autostart=False,require_mapping=False,return_home=False,completion_mode='bounds',complete_hold=10.,separation=.8,min_goal_distance=.3,timeout=2.,goal_timeout=45.,allocator=self.allocate)
        self.create_timer(.1,self.tick)
    def map(self,m):
        self.grid=FrontierGrid(np.array(m.data).reshape(m.info.height,m.info.width),m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y,0.),clearance=.3,spacing=.2,bounds=self.layout.bounds,min_frontier_length=.2)
        self.grid_at=time.monotonic()
    def metadata(self,m):
        try:
            value=json.loads(m.data);reference=value.get('reference')
            if reference:
                if self.reference and self.reference['id']!=reference['id']:self.stop(None)
                alignment=value.get('display_alignment_id')
                if self.alignment_id is not None and alignment!=self.alignment_id:
                    self.stop(None)
                    self.deferred_real_goals.clear()
                self.alignment_id=alignment;self.display_offset=value.get('display_offset')
                self.reference=reference;self.reference_at=time.monotonic()
        except (ValueError,KeyError,TypeError):pass
    def real_route(self,goal):
        if self.reference is None or self.grid is None or 'robot1' not in self.positions:raise ValueError('Fresh real pose, reference or map unavailable')
        route=self.grid.path(self.positions['robot1'],goal[:2],self.grid.distances(self.positions['robot1']))
        if not self.layout.path_allowed('robot1',route,footprint=.34):raise ValueError('Physical route outside region')
        if self.display_offset is not None:
            target=display_to_reference(goal,self.display_offset,self.reference['anchor'])
            local_path=[display_to_reference((p[0],p[1],0.),self.display_offset,self.reference['anchor'])[:2] for p in route]
            return self.reference['id'],list(target),local_path
        raise ValueError('Display alignment unavailable; refusing an unaligned physical goal')
    def allocate(self,grid,positions,**kwargs):
        # The 1 m physical area may legitimately have no frontier at startup.
        # Do not invent a motion goal just to make the physical robot move.
        kwargs.pop('min_distance',None)
        original=kwargs.get('validate_goal')
        distances={n:grid.distances(p) for n,p in positions.items()}
        checked={}
        def valid(name,goal,chosen):
            footprint=.34 if name=='robot1' else .3
            key=(name,tuple(goal.xy))
            if key not in checked:
                route=grid.path(positions[name],goal.xy,distances[name])
                checked[key]=self.layout.path_allowed(name,route,footprint) and (
                    name != "robot1" or self.retry_route_allowed(grid, goal.xy, route))
            return checked[key] and (original is None or original(name,goal,chosen))
        kwargs['validate_goal']=valid
        return allocate(grid,positions,min_distance=.06,**kwargs)
    def command(self,name,message):
        now=time.monotonic()
        if message.get('mode')=='goal':
            try:
                if not self.layout.goal_allowed(name,message['goal'],.34 if name=='robot1' else .3):raise ValueError('Invalid goal')
                self.robots[name].receive(message,now)
            except (ValueError,KeyError,TypeError):self.robots[name].receive({'mode':'stop','session':self.bt.session},now)
        else:self.robots[name].receive(message,now)
    def start_error(self, now):
        if self.physical:
            if not self.link.fresh():
                return '리모 주행 실행기 응답 없음. 리모의 Nav2 실행 창에서 종료 오류를 확인하세요.'
            status = self.link.status
            if not status.get('armed'):
                return '실제 리모 주행 잠금 상태. 안내 3단계의 주행 허용 명령으로 실행하세요.'
            if status.get('dashboard_route_ready') is False:
                return '차체 /cmd_vel 연결 실패. 안내 3단계의 입력과 중복 발행기를 확인하세요.'
            if self.reference is None or now-self.reference_at > 1. or status.get('reference') != self.reference.get('id'):
                return '실제 지도 기준이 없거나 다릅니다. 리모 매핑을 유지하고 노트북 탐사 실행을 다시 시작하세요.'
        if self.grid is None or now-self.grid_at >= 2.:
            return '지도 수신 대기 중입니다. 지도 수신기와 Webots 실행 창을 확인하세요.'
        unavailable = [name for name in self.names if not self.nav[name].motion_ready()]
        if unavailable:
            return '준비 대기 중: ' + ', '.join(unavailable) + '. Nav2 활성화와 위치 수신을 확인한 뒤 다시 시작하세요.'
        return ''

    def start(self,_):
        if self.bt.enabled or (self.trial_seconds>0 and self.trial_started):return
        error = self.start_error(time.monotonic())
        if error:
            self.get_logger().warning('탐사 시작 보류. ' + error)
            return
        if self.trial_seconds>0:
            self.trial_deadline=time.monotonic()+self.trial_seconds;self.trial_started=True
            if self.physical:self.link.set_deadline(self.trial_deadline)
        self.bt.enabled=True
    def stop(self,_):
        self.bt.enabled=False
        if self.physical:self.nav['robot1'].cancel()
        for nav in self.nav.values():nav.cancel()
        self.stop_pub.publish(Bool(data=True))
        for pub in self.cmd_pubs.values():pub.publish(Twist())
    def virtual_safe(self,name,cmd,now):
        msg,stamp=cmd
        if now-stamp>.3 or name not in self.positions:return False
        plan,at=self.paths.get(name,(None,-math.inf))
        if plan is None or now-at>1.5 or plan.header.frame_id!='map':return False
        path=[(p.pose.position.x,p.pose.position.y) for p in plan.poses]
        if not self.layout.path_allowed(name,path,.3):return False
        if not all(math.isfinite(v) for v in (msg.linear.x,msg.angular.z)):return False
        if not (0.<=msg.linear.x<=.18 and abs(msg.angular.z)<=.65):return False
        x,y=self.positions[name];yaw=self.yaws[name]
        for dt in (.2,.5,.8):
            w=msg.angular.z;v=msg.linear.x
            if abs(w)<1e-5:px=x+v*math.cos(yaw)*dt;py=y+v*math.sin(yaw)*dt
            else:px=x+v/w*(math.sin(yaw+w*dt)-math.sin(yaw));py=y-v/w*(math.cos(yaw+w*dt)-math.cos(yaw))
            if not self.layout.goal_allowed(name,(px,py),.3):return False
            if any(peer!=name and math.dist((px,py),p)<.65 for peer,p in self.positions.items()):return False
        return True
    def retry_route_allowed(self, grid, goal, route):
        for target, (cell, clearance) in self.deferred_real_goals.items():
            if math.dist(goal, target) >= .25:
                continue
            if cell is None:
                return False
            x, y = cell
            ox, oy = self.layout.bounds[:2]
            col = math.floor((x - ox) / grid.resolution)
            row = math.floor((y - oy) / grid.resolution)
            height, width = grid.cells.shape
            if 0 <= row < height and 0 <= col < width and 0 <= grid.cells[row, col] < 25:
                continue
            # Sample the candidate route at half a cell. Include the sample
            # spacing in the clearance so the intervening segments are covered.
            for start, end in zip(route, route[1:] or route):
                count = max(1, math.ceil(math.dist(start, end) / (grid.resolution / 2)))
                for fraction in np.linspace(0., 1., count + 1):
                    point = tuple(a + fraction * (b - a) for a, b in zip(start, end))
                    if math.dist(point, cell) <= clearance + grid.resolution / 2:
                        return False
        return True

    def recover_unobserved(self, now):
        task = self.bt.active.get('robot1')
        status = self.link.status if self.link.fresh() else {}
        if (not self.bt.enabled or not task or task.get('cancel_requested')
                or status.get('task_id') != self.nav['robot1'].id
                or status.get('reason') != 'SWEPT_PATH_UNOBSERVED'):
            self.unobserved_task = None
            return
        if self.unobserved_task != task['task_id']:
            self.unobserved_task = task['task_id']
            self.unobserved_since = now
        elif now - self.unobserved_since >= 2.:
            details = status.get('guard_details') or {}
            xy = details.get('cell_xy')
            cell = None
            if xy and self.display_offset is not None:
                cell = map_pose((*xy, 0.), self.display_offset)[:2]
            clearance = .34 + float(details.get('resolution', .05)) / math.sqrt(2.)
            self.deferred_real_goals[tuple(task['goal'][:2])] = (cell, clearance)
            task['cancel_requested'] = True
            task['cancel_reason'] = 'SWEPT_PATH_UNOBSERVED'
            self.get_logger().info('미관측 경로 지속: 실제 리모 목표를 보류하고 다른 목표를 탐색합니다.')

    def tick(self):
        now=time.monotonic()
        if self.bt.enabled and now>=self.trial_deadline:self.stop(None)
        self.positions={};self.yaws={};ready_names=[];motion_ready_names=[]
        for name in self.names:
            nav=self.nav[name];ready=False;position=None;yaw=0.;pose_reason='TF_UNAVAILABLE'
            if name!='robot1':nav.pose_ready=False
            else:nav.local_ready=False
            try:
                t=self.buffer.lookup_transform('map',name+'/base_link',rclpy.time.Time())
                age=(self.get_clock().now().nanoseconds-(t.header.stamp.sec*10**9+t.header.stamp.nanosec))/1e9
                if -.25<=age<=.7:
                    p=t.transform.translation;q=t.transform.rotation;position=[p.x,p.y];yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    self.positions[name]=position;self.yaws[name]=yaw
                    if name!='robot1':nav.pose_ready=True
                    else:nav.local_ready=True
                    self.last_poses[name]=(position,yaw,now)
                    pose_reason='TF_CURRENT'
                else:pose_reason='TF_STALE'
                ready=nav.ready()
            except Exception:
                if name!='robot1':nav.pose_ready=False
            if position is None and name in self.last_poses:
                position,yaw,at=self.last_poses[name]
                # Retain the last stopped peer location as a collision exclusion;
                # it does not grant that peer permission to move.
                self.positions[name]=position;self.yaws[name]=yaw
            ready=nav.ready()
            motion_ready=nav.motion_ready()
            if motion_ready:motion_ready_names.append(name)
            self.readiness_diagnostics[name]={**nav.diagnostic(),'tf_reason':pose_reason}
            if ready:ready_names.append(name)
            self.bt.update_robot(name,{'position':position,'ready':ready,'yaw':yaw},now)
        for name in self.names:
            self.robots[name].tick(time.monotonic())
            state=self.robots[name].snapshot()
            state.update(position=self.positions.get(name),ready=name in ready_names,yaw=self.yaws.get(name,0.))
            self.bt.update_robot(name,state,time.monotonic())
        if self.grid is not None and now-self.grid_at<2.:self.bt.update_map(self.grid,now)
        for name,task in self.bt.active.items():
            if name not in self.positions:continue
            position = self.positions[name]
            yaw = self.yaws[name]
            previous, previous_yaw, at, tid = self.progress.get(
                name, (position, yaw, now, task['task_id'])
            )
            turned = abs(math.remainder(yaw - previous_yaw, math.tau)) >= math.radians(5)
            if tid != task['task_id'] or math.dist(previous, position) > .04 or (
                name == 'robot1' and name in motion_ready_names and turned
            ):
                previous, previous_yaw, at, tid = position, yaw, now, task['task_id']
            self.progress[name] = (previous, previous_yaw, at, tid)
            if now-at>15.:task['cancel_requested']=True
        if self.physical:
            self.recover_unobserved(now)
        self.bt.tick(now)
        running=self.bt.enabled and self.bt.phase in ('EXPLORING','PARTIAL_WAIT') and self.grid is not None and now-self.grid_at<2.
        if self.physical:
            physical=self.nav['robot1'];payload=physical.payload
            if running and not physical.motion_ready() and physical.poll() in ('SENDING','NAVIGATING') and payload.get('kind')=='goal':
                payload={'kind':'hold','task_id':payload.get('task_id'),'reference':payload.get('reference')}
            self.link.tick(payload if running else {'kind':'stop'})
        self.stop_pub.publish(Bool(data=not running))
        for name,pub in self.cmd_pubs.items():
            requested,stamp=self.velocity.get(name,(Twist(),-math.inf))
            bounded=Twist();bounded.linear.x=max(0.,min(.18,requested.linear.x));bounded.angular.z=max(-.65,min(.65,requested.angular.z))
            cmd=(bounded,stamp)
            allowed=running and name in motion_ready_names and name in self.bt.active and not self.bt.active[name].get('cancel_requested') and self.virtual_safe(name,cmd,now)
            pub.publish(cmd[0] if allowed else Twist())
        if now-self.last_report>.5:
            self.last_report=now
            self.status.publish(String(data=json.dumps({'exploration_backend':'dev_chanseo_greedy','physical_mode':self.physical,'physical_connection':self.link.status if self.physical else None,'real_region_m':self.layout.real_bounds[2]-self.layout.real_bounds[0],'ready_robots':ready_names,'motion_ready_robots':motion_ready_names,'readiness':self.readiness_diagnostics,'robot_positions':self.positions,'navigation_states':{n:self.nav[n].poll() for n in self.names},'reference_ready':self.reference is not None,'bt':self.bt.snapshot(),'motion_started':self.bt.enabled})))

def main():
    p=argparse.ArgumentParser();p.add_argument('--virtual-only',action='store_true');p.add_argument('--allow-physical',action='store_true');p.add_argument('--key-file',default='');a,ros=p.parse_known_args()
    # Keep this descriptor open for the lifetime of the coordinator.
    instance_lock = Path(__file__).with_name('.mission.lock').open('a')
    try:
        fcntl.flock(instance_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('탐사 조정 노드가 이미 실행 중입니다. 기존 실행을 먼저 종료하세요.')
    rclpy.init(args=ros);n=Coordinator(a)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():n.stop(None)
        if n.physical:n.link.tick({'kind':'stop'});n.link.shutdown()
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
