"""전용 통합 노드: 시간 경로 명령 소유권·취득 시각 TF·관측 지도·BT 상태."""
from collections import deque
import csv
import json
import math
import time
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, Empty
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from .exploration import Exploration
from .risk import RiskPolicy
from .shadow_mission import ShadowMission, stamp


class ExplorationMission(ShadowMission):
    def __init__(self):
        super().__init__()
        mode = self.declare_parameter('mode', 'explore').value
        departure = self.declare_parameter('departure_policy', '').value
        if departure:
            self.cfg['departure']={**self.cfg.get('departure',{}),'mode':departure}
        risk_stage = self.declare_parameter('risk_stage', '').value
        if risk_stage:
            self.cfg['risk']=RiskPolicy.stage(self.cfg.get('risk'),risk_stage)
        self.engine = Exploration(self.cfg, self.field, self.pcfg, self.output, mode, pool=self.pool)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending_scans = deque(maxlen=120)
        self.scan_drops = {'queue': 0, 'expired': 0, 'frame': 0, 'pre_session': 0}
        for robot in self.robots:
            self.create_subscription(LaserScan, f'/{robot}/scan_observation',
                                     lambda msg, r=robot: self.scan(r, msg), qos_profile_sensor_data)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.observed_map = self.create_publisher(OccupancyGrid, '/exploration/observations', q)
        self.observation_markers = self.create_publisher(MarkerArray, '/exploration/observation_markers', q)
        self.bt_markers = self.create_publisher(MarkerArray, '/exploration/bt_markers', q)
        self.team_status = self.create_publisher(String, '/exploration/team_status', q)
        self.robot_status = {r: self.create_publisher(String, f'/{r}/exploration/state', q) for r in self.robots}
        self.command_status = {r: self.create_publisher(String, f'/{r}/exploration/command', 10) for r in self.robots}
        # 원본의 Empty 시작·정지 토픽도 수용한다. 로봇 명령 토픽은 모니터링용 단방향이다.
        self.create_subscription(Empty, '/exploration/start', lambda msg: self.start(None, Trigger.Response()), 10)
        self.create_subscription(Empty, '/exploration/stop', lambda msg: self.stop(None, Trigger.Response()), 10)
        self.last_path_token = None
        self.plot_session = None
        self.last_observation_marker_token = None
        self.last_shadow_sim_time = -math.inf
        self.last_shadow_session = None

    def odom(self, robot, msg):
        super().odom(robot, msg)
        if hasattr(self, 'engine') and self.engine.active and robot in self.poses:
            self.engine.metrics.pose(robot, self.odoms[robot][0], self.poses[robot][:2])

    def scan(self, robot, msg):
        if msg.header.frame_id != f'{robot}/laser_link':
            self.scan_drops['frame'] += 1
            return
        if len(self.pending_scans) == self.pending_scans.maxlen:
            self.scan_drops['queue'] += 1
        self.pending_scans.append((robot, msg, time.monotonic()))

    def fuse_scans(self, now):
        pending, self.pending_scans = self.pending_scans, deque(maxlen=120)
        for robot, msg, received in pending:
            acquired = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if not self.engine.active or acquired < self.engine.metrics.started_sim:
                self.scan_drops['pre_session'] += 1
                continue
            if now - acquired > 1. or acquired > now + .05 or time.monotonic() - received > 3.:
                self.scan_drops['expired'] += 1
                continue
            try:
                transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            except TransformException:
                self.pending_scans.append((robot, msg, received))
                continue
            p, q = transform.transform.translation, transform.transform.rotation
            # 본 실험의 면적은 2D 투영이며 기울어진 센서 자세는 잘못된 평면 광선으로 쓰지 않는다.
            roll = math.atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
            pitch = math.asin(max(-1., min(1., 2*(q.w*q.y - q.z*q.x))))
            if abs(roll) > .05 or abs(pitch) > .05:
                self.scan_drops['frame'] += 1
                continue
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            observation = self.engine.observation
            revision = observation.revision
            new_area = observation.integrate(robot, acquired, (p.x, p.y, yaw), msg.ranges,
                msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
            if observation.revision != revision:
                path = self.engine.output / 'scan_poses.csv'
                header = not path.exists()
                with path.open('a', newline='') as f:
                    writer = csv.writer(f)
                    if header:
                        writer.writerow(['robot','acquisition_sim_seconds','sensor_x','sensor_y','sensor_yaw',
                            'angle_min','angle_increment','rays','device_range_min','device_range_max','new_area_m2'])
                    writer.writerow([robot,acquired,p.x,p.y,yaw,msg.angle_min,msg.angle_increment,
                                     len(msg.ranges),msg.range_min,msg.range_max,new_area])

    def start(self, request, response):
        if not hasattr(self, 'engine') or self.engine.active:
            response.success, response.message = False, '진행 중이거나 초기화 중입니다.'
        elif not self.execute:
            response.success, response.message = False, 'execute:=true가 필요합니다.'
        elif not self.fresh(self.now()) or time.monotonic() - self.clock_wall > self.wall_timeout:
            response.success, response.message = False, '전체 odom·clock의 최신 입력이 필요합니다.'
        elif any(self.count_publishers(f'/{r}/cmd_vel') != 1 for r in self.robots):
            response.success, response.message = False, 'cmd_vel 발행자 충돌: 기존 launch를 종료하세요.'
        else:
            self.engine.start(self.now(), time.monotonic(), self.poses)
            self.pending_scans.clear()
            response.success, response.message = True, '새 BT session 시작: ' + self.engine.team.session
        return response

    def stop(self, request, response):
        self.auto_plan = False
        self.engine.terminate('user_stop')
        self.halt()
        response.success, response.message = True, '전체 BT·시간 경로를 취소하고 정지했습니다.'
        return response

    def write_events(self):
        if hasattr(self, 'engine'):
            self.engine.persist()

    def tick(self):
        if not hasattr(self, 'engine'):
            return
        e = self.engine
        now, wall = self.now(), time.monotonic()
        if now < self.last_clock - .001:
            e.terminate('clock_reversed', 'ERROR')
            self.auto_plan = False
            self.pending_scans.clear()
        if now != self.last_clock:
            self.clock_wall = wall
        self.last_clock = now
        valid = self.fresh(now) and wall - self.clock_wall <= self.wall_timeout
        exclusive = all(self.count_publishers(f'/{r}/cmd_vel') == 1 for r in self.robots)
        if e.active and not exclusive:
            e.terminate('cmd_vel 발행자 충돌: 기존 launch 동시 실행', 'ERROR')
        if e.phase == 'WAITING' and self.auto_plan and self.execute:
            if self.input_stability.ready(now, wall, valid and exclusive):
                e.start(now, wall, self.poses)
                self.pending_scans.clear()
            elif not exclusive:
                e.reason='로봇별 명령 발행자 확인 대기'
            elif not valid:
                e.reason='전체 로봇 위치와 시계의 최신 입력 대기'
            else:
                e.reason='최신 입력을 확인했으며 실제 시간 기준으로 안정화 중'
        self.fuse_scans(now)
        e.runtime_inputs = {'input_health': self.input_health(now), 'scan_drops': dict(self.scan_drops),
                            'exclusive_cmd_vel_publishers': exclusive,
                            'startup':self.input_stability.progress(now,wall) if e.phase=='WAITING' else None}
        controls = e.step(now, wall, self.poses, valid and exclusive)
        for command in e.commands:
            if command['sim_time'] == now:
                self.command_status[command['robot']].publish(String(data=json.dumps(command)))
        if e.active and valid and exclusive and self.execute:
            self.heartbeat.publish(Bool(data=False))
            for robot, control in controls.items():
                msg = Twist()
                msg.linear.x, msg.angular.z = control.linear, control.angular
                self.commands[robot].publish(msg)
        else:
            self.halt()
        token = e.team.session, e.plan_id, bool(e.paths)
        if token != self.last_path_token:
            self.paths = e.paths
            self.publish_paths()
            for r in self.robots:
                if r not in e.paths:
                    msg = Path()
                    msg.header.frame_id, msg.header.stamp = 'map', stamp(now)
                    self.path_pubs[r].publish(msg)
            self.last_path_token = token
        if wall - self.last_report >= 1.:
            state = e.snapshot()
            state['input_health'], state['scan_drops'] = self.input_health(now), self.scan_drops
            msg = String(data=json.dumps(state, ensure_ascii=False))
            self.status.publish(msg)
            self.team_status.publish(msg)
            for r, robot_state in state['robots'].items():
                self.robot_status[r].publish(String(data=json.dumps(robot_state)))
            self.publish_observations(now)
            e.persist()
            if (e.phase, e.reason) != self.last_reported_state:
                self.get_logger().info(f'{e.phase}: {e.reason}')
                self.last_reported_state = e.phase, e.reason
            self.last_report = wall
        # 정지한 뒤 별도 worker에서만 그래프를 생성한다. 제어 콜백에서 그리지 않는다.
        if e.metrics and not e.active and self.plot_session != e.team.session:
            from .metrics import plot_metrics
            self.pool.submit(plot_metrics, e.metrics.payload(e.observation), e.output)
            self.plot_session = e.team.session

    def publish_observations(self, now):
        e = self.engine
        if self.observed_map.get_subscription_count():
            msg=OccupancyGrid()
            msg.header.frame_id, msg.header.stamp = 'map', stamp(now)
            msg.info.resolution = e.area.resolution
            msg.info.height, msg.info.width = e.area.shape
            msg.info.origin.position.x, msg.info.origin.position.y = e.area.xmin, e.area.ymin
            msg.info.origin.orientation.w = 1.
            msg.data = e.observation.cells.ravel().tolist()
            self.observed_map.publish(msg)
        if self.observation_markers.get_subscription_count():
            self.publish_observation_markers(now)
        if not self.bt_markers.get_subscription_count():
            return
        array = MarkerArray()
        texts = [f'관측 {100*e.observation.coverage_fraction:.1f}% ({e.observation.observed_area_m2:.1f}/{e.area.denominator_m2:.1f} m²)\nTeamBT: {e.team.phase} / {e.phase}']
        for r,bt in e.robot_bt.items():
            state='출발 대기' if bt.status=='WAITING_DEPARTURE' else bt.status
            record=e.metrics.active_departures.get(r,{}) if e.metrics else {}
            role={'lead':'선발','follow':'후발'}.get(record.get('role'),'')
            when=record.get('planned_translation_start_sim_seconds')
            suffix=f' / 예정 이동 {when:.1f}s' if when is not None else ''
            texts.append(f'{r} {role}: {state} / {bt.navigation.poll()}{suffix}')
        for index, text in enumerate(texts):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns, marker.id, marker.type, marker.action = 'bt', index, Marker.TEXT_VIEW_FACING, Marker.ADD
            marker.pose.position = Point(x=e.area.xmin+3., y=e.area.ymin-1.-index*.6, z=1.)
            marker.pose.orientation.w, marker.scale.z, marker.color.a = 1., .4, 1.
            marker.color.r, marker.color.g, marker.color.b = 1., 1., .1
            marker.text = text
            array.markers.append(marker)
        colors=[(.9,.3,.2),(.2,.8,.3),(.3,.5,1.),(.9,.7,.2)]
        if (e.options.get('show_shadow_mesh',False) and getattr(e.field,'geometry_mode',None)=='mesh'
                and (self.last_shadow_session!=e.team.session or now-self.last_shadow_sim_time>=.5)):
            shadow=Marker()
            shadow.header.frame_id,shadow.header.stamp='map',stamp(now)
            shadow.ns,shadow.id='terrain_shadow_mesh',0
            shadow.type,shadow.action=Marker.TRIANGLE_LIST,Marker.ADD
            shadow.pose.orientation.w=1.
            shadow.scale.x=shadow.scale.y=shadow.scale.z=1.
            shadow.color.r,shadow.color.g,shadow.color.b,shadow.color.a=.12,.18,.35,.3
            shadow.points=[Point(x=float(p[0]),y=float(p[1]),z=.025)
                           for triangle in e.field.shadow_triangles(now) for p in triangle]
            array.markers.append(shadow)
            self.last_shadow_sim_time,self.last_shadow_session=now,e.team.session
        for index,(robot,region) in enumerate(sorted(e.work_regions.items())):
            supporting=region.get('activity')=='support'
            xy=region['target_xy'] if supporting else region['frontier_xy']
            marker=Marker()
            marker.header.frame_id='map'
            marker.ns,marker.id='work_regions',index
            marker.type,marker.action=Marker.TEXT_VIEW_FACING,Marker.ADD
            marker.pose.position=Point(x=float(xy[0]),y=float(xy[1]),z=.8)
            marker.pose.orientation.w=1.
            marker.scale.z=.45
            marker.color.r,marker.color.g,marker.color.b=colors[index%len(colors)]
            marker.color.a=1.
            marker.text=robot+(' 보호 이동' if supporting else ' 작업 구역')
            marker.lifetime=Duration(seconds=2.).to_msg()
            array.markers.append(marker)
            arrow=Marker()
            arrow.header.frame_id='map'
            arrow.ns,arrow.id='work_region_directions',index
            arrow.type,arrow.action=Marker.ARROW,Marker.ADD
            arrow.pose.orientation.w=1.
            pose=e.poses.get(robot)
            if pose:
                arrow.points=[Point(x=float(pose[0]),y=float(pose[1]),z=.3),
                              Point(x=float(xy[0]),y=float(xy[1]),z=.3)]
                arrow.scale.x,arrow.scale.y,arrow.scale.z=.04,.12,.16
                arrow.color=marker.color
                arrow.lifetime=marker.lifetime
                array.markers.append(arrow)
        self.bt_markers.publish(array)

    def publish_observation_markers(self, now):
        """지도 텍스처 셰이더 대신 배경과 관측 셀 점으로 같은 격자를 표시한다."""
        e=self.engine
        token=e.team.session,e.observation.revision
        if token==self.last_observation_marker_token:return
        markers=MarkerArray()
        background=Marker()
        background.header.frame_id,background.header.stamp='map',stamp(now)
        background.ns,background.id='observations',0
        background.type,background.action=Marker.CUBE,Marker.ADD
        background.pose.orientation.w=1.
        background.pose.position=Point(x=(e.area.xmin+e.area.xmax)/2.,y=(e.area.ymin+e.area.ymax)/2.,z=.005)
        background.scale.x,background.scale.y,background.scale.z=float(e.area.xmax-e.area.xmin),float(e.area.ymax-e.area.ymin),.005
        background.color.r=background.color.g=background.color.b=.35
        background.color.a=.45
        markers.markers.append(background)
        for number,value,shade in [(1,0,1.),(2,100,.05)]:
            marker=Marker()
            marker.header=background.header
            marker.ns,marker.id='observations',number
            marker.type,marker.action=Marker.POINTS,Marker.ADD
            marker.pose.orientation.w=1.
            marker.scale.x=marker.scale.y=float(e.area.resolution)
            marker.color.r=marker.color.g=marker.color.b=shade
            marker.color.a=.95
            rows,cols=np.nonzero(e.observation.cells==value)
            marker.points=[Point(x=float(e.area.xmin+(col+.5)*e.area.resolution),
                y=float(e.area.ymin+(row+.5)*e.area.resolution),z=.03) for row,col in zip(rows,cols)]
            if not marker.points:marker.action=Marker.DELETE
            markers.markers.append(marker)
        self.observation_markers.publish(markers)
        self.last_observation_marker_token=token


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExplorationMission()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if rclpy.ok():
                node.halt()
            node.engine.close()
            node.pool.shutdown(wait=True, cancel_futures=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
