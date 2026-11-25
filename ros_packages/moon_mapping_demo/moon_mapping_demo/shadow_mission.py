"""사전 지형 기반 시간 경로의 ROS 시각화·실행·지연 재계획 어댑터."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .coordination import coordinate
from .execution import InputStability, control_step, inputs_fresh
from .experiment import load_experiment, make_field, path_data, plan_experiment


def stamp(seconds):
    ns = round(seconds * 1e9)
    return TimeMsg(sec=ns // 1000000000, nanosec=ns % 1000000000)


class ShadowMission(Node):
    def __init__(self):
        super().__init__('shadow_mission')
        share = Path(get_package_share_directory('moon_mapping_demo'))
        config = self.declare_parameter('config', str(share / 'config/shadow_experiment.yaml')).value
        self.execute = self.declare_parameter('execute', False).value
        self.auto_plan = self.declare_parameter('auto_plan', True).value
        self.output = Path(self.declare_parameter('output', 'results/shadow_hungarian_ros').value)
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('이 실험은 use_sim_time:=true와 Webots /clock이 필요합니다.')
        self.cfg, self.env, self.pcfg, self.share = load_experiment(config, share)
        self.field = make_field(self.cfg, self.env, self.pcfg, self.share)
        self.settings = self.cfg['execution']
        self.wall_timeout = self.settings.get('input_wall_timeout', self.settings['odom_timeout'])
        self.robots = self.cfg['robots']
        self.all_robots = tuple(self.env['starts'])
        self.odoms, self.poses = {}, {}
        self.paths, self.goals, self.events = {}, {}, []
        self.result, self.future = None, None
        self.replans = 0
        self.actual_arrivals = {}
        self.phase, self.reason = 'WAITING', '전체 로봇의 최신 odom 및 /clock 대기'
        self.pool = ThreadPoolExecutor(max_workers=1)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/shadow_mission/status', q)
        # 기존 드라이버/BT의 비상정지·heartbeat 계약 재사용. 다른 BT와 동시 실행 금지.
        self.heartbeat = self.create_publisher(Bool, '/multi_bt/emergency_stop', q)
        self.markers = self.create_publisher(MarkerArray, '/shadow_mission/markers', q)
        self.path_pubs = {r: self.create_publisher(PathMsg, f'/{r}/shadow_path', q) for r in self.robots}
        self.commands = {r: self.create_publisher(Twist, f'/{r}/cmd_vel', 10) for r in self.robots}
        for r in self.all_robots:
            self.create_subscription(Odometry, f'/{r}/odom', lambda msg, r=r: self.odom(r, msg), qos_profile_sensor_data)
        self.create_service(Trigger, '/shadow_mission/start', self.start)
        self.create_service(Trigger, '/shadow_mission/stop', self.stop)
        # /clock이 멈춰도 stale odom·정지 감시가 작동하도록 제어 콜백은 steady timer.
        self.timer = self.create_timer(self.settings['control_period'], self.tick,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_clock, self.clock_wall = 0., time.monotonic()
        self.last_report = 0.
        self.last_reported_state = None
        self.input_stability = InputStability(self.settings.get('startup_stable_seconds', 3.))
        self.halt()

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def odom(self, robot, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if msg.header.frame_id != f'{robot}/odom' or msg.child_frame_id != f'{robot}/base_link':
            return
        values = (p.x, p.y, q.x, q.y, q.z, q.w, t)
        if not all(math.isfinite(v) for v in values):
            return
        if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.) > .01:
            return
        s = self.env['starts'][robot]
        c, sn = math.cos(s['yaw']), math.sin(s['yaw'])
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.poses[robot] = (s['x'] + c * p.x - sn * p.y, s['y'] + sn * p.x + c * p.y, s['yaw'] + yaw)
        self.odoms[robot] = (t, time.monotonic())

    def fresh(self, now):
        return inputs_fresh(self.odoms, self.all_robots, now, time.monotonic(),
                            self.settings['odom_timeout'], self.wall_timeout)

    def input_health(self, now):
        wall = time.monotonic()
        return {'sim_time': now, 'clock_wall_age_seconds': wall - self.clock_wall,
                'odom_timeout_seconds': self.settings['odom_timeout'],
                'input_wall_timeout_seconds': self.wall_timeout,
                'robots': {r: {'stamp_sim_seconds': self.odoms[r][0],
                               'sim_age_seconds': now - self.odoms[r][0],
                               'wall_age_seconds': wall - self.odoms[r][1]}
                           if r in self.odoms else None for r in self.all_robots}}

    def halt(self):
        self.heartbeat.publish(Bool(data=True))
        for pub in self.commands.values():
            pub.publish(Twist())

    def start(self, request, response):
        if self.future or self.phase in ('EXECUTING', 'PLANNING'):
            response.success, response.message = False, '진행 중입니다. stop 후 다시 시작하세요.'
        elif not self.fresh(self.now()):
            response.success, response.message = False, '대기 로봇을 포함한 최신 odom이 필요합니다.'
        else:
            self.replans = 0
            self.actual_arrivals = {}
            self.queue_plan()
            response.success, response.message = True, '계획 생성을 시작했습니다.'
        return response

    def stop(self, request, response):
        self.phase, self.reason, self.paths = 'STOPPED', '사용자 정지', {}
        self.auto_plan = False
        self.halt()
        self.write_events()
        response.success, response.message = True, self.reason
        return response

    def queue_plan(self, replan=False):
        self.halt()
        self.paths = {}
        self.phase, self.reason = 'PLANNING', '남은 경로 재계획' if replan else '비용 행렬·배정·공동 경로 계산'
        starts = {r: tuple(p[:2]) for r, p in self.poses.items()}
        t0 = self.now() + self.settings['lead_seconds']
        standby = [(p[0], p[1], self.pcfg.robot_radius) for r, p in starts.items() if r not in self.robots]
        if replan:
            self.future = self.pool.submit(coordinate, self.field, self.pcfg, starts,
                                           dict(self.goals), t0, standby, self.settings['goal_hold_seconds'])
        else:
            self.future = self.pool.submit(plan_experiment, self.cfg, self.env, self.pcfg,
                                           self.field, t0, starts, self.output)
        self.replanning = replan
        self.plan_starts = starts

    def request_replan(self, reason):
        self.halt()
        self.events.append({'sim_time': self.now(), 'reason': reason, 'kind': 'replan'})
        if self.replans >= self.settings['max_replans']:
            self.phase, self.reason, self.paths = 'ERROR', '재계획 횟수 초과: ' + reason, {}
            self.write_events()
            return
        self.replans += 1
        self.queue_plan(replan=bool(self.goals))

    def write_events(self):
        self.output.mkdir(parents=True, exist_ok=True)
        payload = {'phase': self.phase, 'reason': self.reason, 'events': self.events,
                   'replans': self.replans, 'current_paths': path_data(self.paths),
                   'original_assignment_total_ticks': self.result['assignment'].total_cost if self.result and self.result['assignment'] else None,
                   'current_remaining_total_ticks': sum(p.cost_ticks for p in self.paths.values())}
        payload['actual_arrival_sim_seconds'] = self.actual_arrivals
        payload['observed_poses'] = self.poses
        payload['input_health'] = self.input_health(self.now())
        if self.result:
            payload['initial_t0_sim_seconds'] = self.result['t0']
            payload['actual_elapsed_seconds_by_robot'] = {
                r: t - self.result['t0'] for r, t in self.actual_arrivals.items()}
        (self.output / 'runtime.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')

    def finish_plan(self, now):
        future, self.future = self.future, None
        if self.phase == 'STOPPED':
            return
        try:
            value = future.result()
            if self.replanning:
                self.paths = value
            else:
                self.result = value
                if not value['paths']:
                    raise ValueError(value['error'] or '완전 배정 또는 공동 실행 경로 없음')
                self.paths, self.goals = value['paths'], value['assigned_goals']
            t0 = min(p.points[0].t for p in self.paths.values())
            self.publish_paths()
            if self.execute and now >= t0:
                self.request_replan('계산 완료 전에 출발 시각 경과')
                return
            self.phase = 'EXECUTING' if self.execute else 'VISUALIZED'
            self.reason = '시간 경로 실행' if self.execute else '시각화 전용: 속도 명령 정지 유지'
            self.events.append({'kind': 'plan', 'sim_time': now, 't0': t0,
                                'replanned': self.replanning, 'paths': path_data(self.paths)})
        except Exception as exc:
            self.phase, self.reason, self.paths = 'ERROR', str(exc), {}
            self.get_logger().error(self.reason)
        self.write_events()

    def tick(self):
        now, wall = self.now(), time.monotonic()
        if now < self.last_clock - .001:
            self.phase, self.reason, self.paths = 'STOPPED', '/clock 역행: start로 새 실험 필요', {}
            self.auto_plan = False
            self.halt()
            self.write_events()
        if now != self.last_clock:
            self.clock_wall = wall
        self.last_clock = now
        if wall - self.last_report > 1.:
            state = self.phase, self.reason
            if state != self.last_reported_state:
                if self.phase == 'ERROR':
                    self.get_logger().error(f'{self.phase}: {self.reason}')
                else:
                    self.get_logger().info(f'{self.phase}: {self.reason}')
                self.last_reported_state = state
            self.status.publish(String(data=json.dumps({'phase': self.phase, 'reason': self.reason,
                                                         'sim_time': now, 'replans': self.replans,
                                                         'input_health': self.input_health(now)}, ensure_ascii=False)))
            self.last_report = wall
        if self.future:
            self.halt()
            if self.future.done():
                self.finish_plan(now)
            return
        if self.phase == 'WAITING' and self.auto_plan:
            valid = self.fresh(now) and wall - self.clock_wall <= self.wall_timeout
            if self.input_stability.ready(now, wall, valid):
                self.queue_plan()
            else:
                self.reason = '전체 로봇 odom·clock 안정화 대기'
        if self.phase != 'EXECUTING':
            self.halt()
            return
        if not self.fresh(now) or wall - self.clock_wall > self.wall_timeout:
            self.phase, self.reason = 'ERROR', '전체 odom 또는 /clock 갱신 중단'
            self.halt()
            self.write_events()
            return
        if any(self.count_publishers(f'/{r}/cmd_vel') > 1 for r in self.robots):
            self.phase, self.reason = 'ERROR', '다른 cmd_vel 발행자와 실행 권한 충돌'
            self.halt()
            self.write_events()
            return
        controls, reason = control_step(self.field, self.paths, self.poses, now, self.settings,
                                        min(.22, self.pcfg.speed * 1.2), self.pcfg.robot_radius)
        if reason:
            self.request_replan(reason)
            return
        for r, c in controls.items():
            if c.arrived and r not in self.actual_arrivals:
                self.actual_arrivals[r] = now
        end = max(p.points[-1].t for p in self.paths.values()) + self.settings['goal_hold_seconds']
        if all(c.arrived for c in controls.values()) and now >= end:
            self.phase, self.reason = 'COMPLETE', '모든 목표 도착 및 설정된 대기 완료'
            self.halt()
            self.write_events()
            return
        self.heartbeat.publish(Bool(data=False))
        for r, c in controls.items():
            msg = Twist()
            msg.linear.x, msg.angular.z = c.linear, c.angular
            self.commands[r].publish(msg)

    def publish_paths(self):
        array = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        array.markers.append(clear)
        palette = ((1., .2, .2), (.2, 1., .3), (.3, .5, 1.), (1., .7, .2))
        for index, (r, path) in enumerate(self.paths.items()):
            msg = PathMsg()
            msg.header.frame_id, msg.header.stamp = 'map', stamp(path.points[0].t)
            for p in path.points:
                pose = PoseStamped()
                pose.header.frame_id, pose.header.stamp = 'map', stamp(p.t)
                pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.w = p.x, p.y, 1.
                msg.poses.append(pose)
            self.path_pubs[r].publish(msg)
            for number, p in enumerate(path.points):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.ns, marker.id, marker.type, marker.action = r, number, Marker.TEXT_VIEW_FACING, Marker.ADD
                marker.pose.position = Point(x=p.x, y=p.y, z=.25 + index * .1)
                marker.pose.orientation.w, marker.scale.z, marker.color.a = 1., .22, 1.
                marker.color.r, marker.color.g, marker.color.b = palette[index % len(palette)]
                prior = path.points[max(0, number - 1)]
                wait = f' 대기 {p.t-prior.t:.1f}s' if number and p.x == prior.x and p.y == prior.y else ''
                marker.text = f'{r} {p.t:.1f}s{wait}'
                array.markers.append(marker)
        self.markers.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ShadowMission()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if rclpy.ok():
                node.halt()
            node.pool.shutdown(wait=False, cancel_futures=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
