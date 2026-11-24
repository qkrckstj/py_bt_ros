"""Distance-gated ROS message forwarding and per-robot shared sensor maps.

This emulates radio links in software; ROS DDS itself is not an RF simulator.
Each agent sends local scan-grid observations and acknowledges packets. Only
the router can deliver to a robot's rx topic, and only along an in-range hop.
"""
import base64
import argparse
import json
import math
import signal
import time
import zlib
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from mission import settings

ROBOTS = ('robot1','robot2','robot3','robot4')
QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def neighbors(positions, sender, radius):
    return [r for r,p in positions.items() if r != sender and math.dist(positions[sender],p) <= radius]


def connected(positions, radius):
    if len(positions) != 4:
        return False
    seen = {'robot1'}
    while True:
        more = {r for s in seen for r in neighbors(positions,s,radius)}-seen
        if not more:
            return len(seen) == 4
        seen.update(more)


def routes_connected(plans, positions, radius):
    """Search progress order rather than assuming identical progress fractions.

    At most 9^4 joint states. Runtime velocity permits enforce short-step link
    margins while agents follow their individual long Nav2 goals.
    """
    samples = {}
    for r in ROBOTS:
        route = [positions[r]]+plans.get(r,[])
        points = [route_point(route,fraction=f) for f in np.linspace(0,1,9)]
        samples[r] = [points[0]]
        for p in points[1:]:
            if math.dist(p,samples[r][-1])>.02:samples[r].append(p)
    goal = tuple(len(samples[r])-1 for r in ROBOTS)
    start = (0,0,0,0)
    queue = deque([start])
    seen = {start}
    while queue:
        state = queue.popleft()
        if state == goal:return True
        for i,r in enumerate(ROBOTS):
            if state[i] == goal[i]:continue
            successor = list(state)
            successor[i] += 1
            successor = tuple(successor)
            if successor in seen:continue
            sample = {s:samples[s][successor[j]] for j,s in enumerate(ROBOTS)}
            if connected(sample,radius-.15):
                seen.add(successor)
                queue.append(successor)
    return False


def route_point(route, distance=None, fraction=None):
    lengths = [math.dist(a,b) for a,b in zip(route,route[1:])]
    remaining = (fraction*sum(lengths)) if fraction is not None else distance
    for a,b,length in zip(route,route[1:],lengths):
        if length and remaining <= length:
            return (a[0]+(b[0]-a[0])*remaining/length,a[1]+(b[1]-a[1])*remaining/length)
        remaining -= length
    return tuple(route[-1])


def movement_permits(positions, projections, radius):
    """Select robots that may advance while every short-step corner stays linked."""
    best = set()
    for mask in range(1,16):
        allowed = [r for i,r in enumerate(ROBOTS) if mask&(1<<i)]
        if len(allowed)<=len(best):
            continue
        edges = {r:set() for r in ROBOTS}
        for i,r in enumerate(ROBOTS):
            a = [positions[r], projections[r]] if r in allowed else [positions[r]]
            for s in ROBOTS[i+1:]:
                b = [positions[s],projections[s]] if s in allowed else [positions[s]]
                current = math.dist(positions[r],positions[s])
                bound = max(radius-.3,current) if current<=radius else radius-.3
                if max(math.dist(p,q) for p in a for q in b)<=bound+1e-8:
                    edges[r].add(s)
                    edges[s].add(r)
        seen = {'robot1'}
        while True:
            more = set().union(*(edges[r] for r in seen))-seen
            if not more:break
            seen.update(more)
        if len(seen)==4:best=set(allowed)
    return best


class RadioRouter(Node):
    def __init__(self, radius, status_topic='/lunar/radio/status'):
        super().__init__('lunar_radio_router')
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.radius = radius
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer,self)
        self.receivers = {r:self.create_publisher(String,f'/lunar/{r}/rx',20) for r in ROBOTS}
        self.subs = [self.create_subscription(String,f'/lunar/{r}/tx',lambda m,r=r:self.forward(r,m),50) for r in ROBOTS]
        self.status = self.create_publisher(String,status_topic,QOS)
        self.sent = self.dropped = self.multihop = 0
        self.create_timer(2.,self.report)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def positions(self):
        result = {}
        for r in ROBOTS:
            try:
                t = self.buffer.lookup_transform('map',r+'/base_link',Time())
                if not -.5 <= self.now()-(t.header.stamp.sec+t.header.stamp.nanosec/1e9) < 3:
                    continue
                result[r] = (t.transform.translation.x,t.transform.translation.y)
            except Exception:
                pass
        return result

    def forward(self, sender, msg):
        try:
            packet = json.loads(msg.data)
            positions = self.positions()
            if (sender not in positions or packet['sender'] != sender or
                    packet['route'][-1] != sender or packet['ttl'] <= 0 or
                    not -.5 <= self.now()-packet['stamp'] < 12):
                self.dropped += 1
                return
            delivered = False
            for receiver in neighbors(positions,sender,self.radius):
                if receiver in packet['route']:
                    continue
                outgoing = {**packet,'route':packet['route']+[receiver],'ttl':packet['ttl']-1}
                self.receivers[receiver].publish(String(data=json.dumps(outgoing)))
                self.sent += 1
                self.multihop += len(outgoing['route']) > 2
                delivered = True
            if not delivered:
                self.dropped += 1
        except (ValueError,KeyError,TypeError):
            self.dropped += 1

    def report(self):
        positions = self.positions()
        self.status.publish(String(data=json.dumps({'stamp':self.now(),'connected':connected(positions,self.radius),
                            'delivered_hops':self.sent,'dropped_or_terminal':self.dropped,
                            'multihop_deliveries':self.multihop,'radius_m':self.radius})))


class Agent(Node):
    def __init__(self, robot, config):
        super().__init__(f'{robot}_radio_agent')
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.robot = robot
        self.bounds = config['bounds_m']
        self.resolution = .1
        self.width = round((self.bounds[2]-self.bounds[0])/self.resolution)
        self.height = round((self.bounds[3]-self.bounds[1])/self.resolution)
        self.local = np.full((self.height,self.width),-1,dtype=np.int8)
        self.sources = {}
        self.source_times = {}
        self.source_positions = {}
        self.acks = {}
        self.ack_routes = {}
        self.seen = {}
        self.sequence = 0
        self.assigned_round = 0
        self.assignment_acks = {}
        self.assigned_task = None
        self.last_scan = -100.
        self.command = Twist()
        self.command_time = self.permit_time = -100.
        self.permitted = False
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer,self)
        self.tx = self.create_publisher(String,f'/lunar/{robot}/tx',50)
        self.map_pub = self.create_publisher(OccupancyGrid,f'/lunar/{robot}/team_map',QOS)
        self.health = self.create_publisher(String,f'/lunar/{robot}/radio_status',QOS)
        self.velocity_pub = self.create_publisher(Twist,f'/{robot}/cmd_vel',10)
        self.subs = [self.create_subscription(String,f'/lunar/{robot}/rx',self.receive,50),
                     self.create_subscription(LaserScan,f'/{robot}/scan',self.scan,qos_profile_sensor_data)]
        self.subs.append(self.create_subscription(Twist,f'/{robot}/cmd_vel_requested',self.velocity,10))
        self.subs.append(self.create_subscription(Bool,f'/lunar/{robot}/motion_permit',self.permit,QOS))
        if robot == 'robot1':
            self.subs.append(self.create_subscription(String,'/lunar/robot1/assignment',self.assignment,10))
        self.create_timer(2.,self.broadcast)
        self.create_timer(.5,self.publish_map)
        self.create_timer(.1,self.velocity_tick)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def velocity(self,msg):
        self.command = msg
        self.command_time = self.now()

    def permit(self,msg):
        self.permitted = msg.data
        self.permit_time = self.now()

    def velocity_tick(self):
        allowed = self.permitted and self.now()-self.permit_time<.6 and self.now()-self.command_time<.5
        self.velocity_pub.publish(self.command if allowed else Twist())

    def send(self, kind, payload, destination='*'):
        self.sequence += 1
        packet = {'id':f'{self.robot}:{self.sequence}','kind':kind,'origin':self.robot,
                  'sender':self.robot,'route':[self.robot],'ttl':3,'stamp':self.now(),
                  'destination':destination,'payload':payload}
        self.seen[packet['id']] = self.now()
        self.tx.publish(String(data=json.dumps(packet)))

    def scan(self, msg):
        if self.now()-self.last_scan < .5 or len(msg.ranges)<2:
            return
        try:
            t = self.buffer.lookup_transform('map',msg.header.frame_id,Time()).transform
        except Exception:
            return
        self.last_scan = self.now()
        q = t.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        ranges = np.asarray(msg.ranges)
        hits = np.isfinite(ranges)&(ranges>=.1)&(ranges<12.)
        ranges = np.where(hits,ranges,12.)
        angles = np.linspace(msg.angle_min,msg.angle_max,len(ranges))+yaw
        steps = np.arange(0.,12.,self.resolution*.5)[None,:]
        xs = t.translation.x+np.cos(angles)[:,None]*steps
        ys = t.translation.y+np.sin(angles)[:,None]*steps
        cols = np.floor((xs-self.bounds[0])/self.resolution).astype(int)
        rows = np.floor((ys-self.bounds[1])/self.resolution).astype(int)
        valid = (rows>=0)&(rows<self.height)&(cols>=0)&(cols<self.width)&(steps<ranges[:,None]-.08)
        self.local[rows[valid],cols[valid]] = 0
        cols = np.floor((t.translation.x+np.cos(angles)*ranges-self.bounds[0])/self.resolution).astype(int)
        rows = np.floor((t.translation.y+np.sin(angles)*ranges-self.bounds[1])/self.resolution).astype(int)
        valid = hits&(rows>=0)&(rows<self.height)&(cols>=0)&(cols<self.width)
        self.local[rows[valid],cols[valid]] = 100

    def broadcast(self):
        if self.now()-self.last_scan > 3:
            return
        try:
            t = self.buffer.lookup_transform('map',self.robot+'/base_link',Time()).transform.translation
        except Exception:
            return
        self.source_positions[self.robot] = (t.x,t.y)
        self.sources[self.robot] = self.local.copy()
        self.source_times[self.robot] = self.now()
        self.send('map',{'grid':base64.b64encode(zlib.compress(self.local.tobytes())).decode(),
                         'position':[t.x,t.y]})
        self.seen = {k:v for k,v in self.seen.items() if self.now()-v<30}

    def assignment(self, msg):
        data = json.loads(msg.data)
        self.assigned_round = data['round']
        self.assigned_task = data['tasks'][self.robot]
        self.assignment_acks.clear()
        self.send('assignment',data)

    def receive(self, msg):
        try:
            packet = json.loads(msg.data)
            if packet['id'] in self.seen or packet['route'][-1] != self.robot:
                return
            self.seen[packet['id']] = self.now()
            origin = packet['origin']
            if packet['kind'] == 'map':
                grid = np.frombuffer(zlib.decompress(base64.b64decode(packet['payload']['grid'])),dtype=np.int8).reshape(self.local.shape)
                if packet['stamp'] >= self.source_times.get(origin,-1):
                    self.sources[origin] = grid.copy()
                    self.source_times[origin] = packet['stamp']
                    self.source_positions[origin] = packet['payload']['position']
                self.send('ack',{'for':packet['id'],'received_route':packet['route']},origin)
            elif packet['kind'] == 'assignment':
                if packet['origin'] != 'robot1':
                    return
                self.assigned_round = max(self.assigned_round,packet['payload']['round'])
                self.assigned_task = packet['payload']['tasks'][self.robot]
                self.send('ack',{'for':packet['id'],'round':self.assigned_round,'received_route':packet['route']},origin)
            elif packet['kind'] == 'ack' and packet['destination'] == self.robot:
                self.acks[origin] = self.now()
                self.ack_routes[origin] = packet['route']
                if 'round' in packet['payload']:
                    self.assignment_acks[origin] = packet['payload']['round']
            if packet['ttl'] > 0 and packet['destination'] != self.robot:
                packet['sender'] = self.robot
                self.tx.publish(String(data=json.dumps(packet)))
        except (ValueError,KeyError,TypeError,zlib.error):
            pass

    def publish_map(self):
        if self.robot not in self.sources:
            return
        merged = self.local.copy()
        for robot,grid in self.sources.items():
            if robot != self.robot and self.now()-self.source_times[robot]<12:
                merged = np.maximum(merged,grid)
        rows, cols = np.mgrid[:self.height,:self.width]
        for position in self.source_positions.values():
            xs = self.bounds[0]+(cols+.5)*self.resolution-position[0]
            ys = self.bounds[1]+(rows+.5)*self.resolution-position[1]
            merged[xs*xs+ys*ys < .30**2] = 0
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width,msg.info.height = self.width,self.height
        msg.info.origin.position.x,msg.info.origin.position.y = self.bounds[:2]
        msg.info.origin.orientation.w = 1.
        msg.data = merged.ravel().tolist()
        self.map_pub.publish(msg)
        others = [r for r in ROBOTS if r != self.robot]
        ok = all(self.now()-self.source_times.get(r,-100)<10 and self.now()-self.acks.get(r,-100)<10 for r in others)
        self.health.publish(String(data=json.dumps({'robot':self.robot,'stamp':self.now(),
            'network_ok':ok,'source_stamps':self.source_times,'ack_stamps':self.acks,
            'ack_routes':self.ack_routes,'assigned_round':self.assigned_round,
            'assigned_task':self.assigned_task,'assignment_acks':self.assignment_acks})))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--router-only',action='store_true')
    parser.add_argument('--radius',type=float)
    parser.add_argument('--status-topic',default='/lunar/radio/status')
    args,ros_args = parser.parse_known_args()
    config = settings()
    radius = config['radio_range_m'] if args.radius is None else args.radius
    if not math.isfinite(radius) or radius<=0:
        raise ValueError('Radio radius must be finite and positive')
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    nodes = [RadioRouter(radius,args.status_topic)]
    if not args.router_only:
        nodes += [Agent(r,config) for r in ROBOTS]
    nodes[0].get_logger().info(f'Distance-gated communication radius: {radius:.1f}m')
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    stopping = [False]
    signal.signal(signal.SIGINT,lambda *_:stopping.__setitem__(0,True))
    signal.signal(signal.SIGTERM,lambda *_:stopping.__setitem__(0,True))
    try:
        while rclpy.ok() and not stopping[0]:
            executor.spin_once(timeout_sec=.1)
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
