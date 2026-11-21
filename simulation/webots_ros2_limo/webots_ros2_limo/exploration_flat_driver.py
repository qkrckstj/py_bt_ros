"""Camera-free Webots differential drive for the flat exploration test.

Publish physical lidar and wheel/IMU odometry. No GPS or world truth is used.
"""
import math
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from .exploration_localization import MappingScanGate, integrate, scan_geometry
from .exploration_scan import PersistentMappingHits


class FlatDriver:
    def init(self, webots_node, properties):
        self.robot, self.ns = webots_node.robot, properties['namespace']
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('flat_driver', namespace=self.ns)
        self.motors = [self.robot.getDevice(s+'_wheel') for s in
                       ('front_left', 'rear_left', 'front_right', 'rear_right')]
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.)
        self.encoders = [self.robot.getDevice(s+'_wheel_sensor') for s in
                         ('front_left', 'rear_left', 'front_right', 'rear_right')]
        self.imu = self.robot.getDevice('imu inertial')
        self.lidar = self.robot.getDevice('laser')
        step = int(self.robot.getBasicTimeStep())
        for device in [*self.encoders, self.imu]:
            device.enable(step)
        self.lidar.enable(100)
        self.scan_pub = self.node.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.mapping_scan_pub = self.node.create_publisher(LaserScan, 'scan_mapping_input', qos_profile_sensor_data)
        from std_msgs.msg import String
        self.quality_type = String
        self.quality_pub = self.node.create_publisher(String, 'scan_quality', 1)
        self.scan_gate = MappingScanGate()
        self.persistent_hits = PersistentMappingHits()
        self.scan_counts = {'accepted': 0, 'rejected': 0}
        self.odom_pub = self.node.create_publisher(Odometry, 'odom', 10)
        self.broadcaster = TransformBroadcaster(self.node)
        self.node.create_subscription(Twist, 'cmd_vel', self.command, 10)
        self.velocity, self.received = (0., 0.), -100.
        self.applied_v = 0.
        self.previous, self.yaw0, self.x, self.y = None, None, 0., 0.
        self.step_ms = step
        self.scan_steps = max(1, round(100/step))
        self.previous_heading, self.previous_time = 0., None

    def command(self, message):
        v, w = message.linear.x, message.angular.z
        if math.isfinite(v) and math.isfinite(w):
            self.velocity = max(-.12, min(.40, v)), max(-.9, min(.9, w))
            self.received = self.robot.getTime()

    def stamp(self, now):
        from builtin_interfaces.msg import Time
        sec, nanosec = divmod(round(now*1e9), 1_000_000_000)
        return Time(sec=sec, nanosec=nanosec)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        now = self.robot.getTime()
        v, w = self.velocity if now-self.received < .5 else (0., 0.)
        # Ramp acceleration to reduce wheel slip at the higher cruise speed.
        # A stop from the path coordinator still brakes immediately.
        if v and abs(v) > abs(self.applied_v):
            dv = .6*self.step_ms/1000.
            v = max(self.applied_v-dv, min(self.applied_v+dv, v))
        self.applied_v = v
        left, right = (v-.086*2.414*w)/.045, (v+.086*2.414*w)/.045
        # Preserve curvature when combined forward/turn commands exceed a
        # wheel's actual limit, instead of requesting impossible motor speeds.
        scale = max(1., abs(left)/min(m.getMaxVelocity() for m in self.motors),
                    abs(right)/min(m.getMaxVelocity() for m in self.motors))
        left, right = left/scale, right/scale
        for motor, speed in zip(self.motors, (left, left, right, right)):
            motor.setVelocity(speed)
        wheels = [sensor.getValue() for sensor in self.encoders]
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        if not all(math.isfinite(a) for a in [yaw, *wheels]):
            return
        if self.previous is None:
            self.previous, self.yaw0 = wheels, yaw
            self.previous_time = now
            return
        heading = math.atan2(math.sin(yaw-self.yaw0), math.cos(yaw-self.yaw0))
        dx = sum((a-b)*.045 for a,b in zip(wheels, self.previous))/4
        self.previous = wheels
        self.x, self.y, dh = integrate(self.x, self.y, dx, self.previous_heading, heading)
        dt = now-self.previous_time
        self.previous_heading, self.previous_time = heading, now
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = self.ns+'/odom', self.ns+'/base_link'
        odom.header.stamp = self.stamp(now)
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = dx/dt, dh/dt
        for index, value in ((0, 1e-3), (7, 1e-3), (14, 1e6), (21, 1e6), (28, 1e6), (35, 1e-3)):
            odom.pose.covariance[index] = odom.twist.covariance[index] = value
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = odom.header, odom.child_frame_id
        transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.broadcaster.sendTransform(transform)
        if round(now*1000/self.step_ms) % self.scan_steps == 0:
            ranges = list(self.lidar.getRangeImage())[::-1]
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = self.ns+'/laser_link', self.stamp(now)
            scan.angle_min, scan.angle_max, scan.angle_increment = scan_geometry(self.lidar.getFov(), len(ranges))
            scan.range_min, scan.range_max, scan.scan_time = self.lidar.getMinRange(), self.lidar.getMaxRange(), .1
            scan.ranges = [float(r) for r in ranges]
            self.scan_pub.publish(scan)
            # Only the SLAM input is gated. Live /scan for Nav2 is untouched.
            stable = self.scan_gate.accept(now, roll, pitch, dh/dt)
            if stable:
                # Odom/IMU motion compensation; no simulator truth or semantic
                # peer labels. Keep raw scan above intact for local avoidance.
                scan.ranges = self.persistent_hits.filter(now, scan.ranges,
                    scan.angle_min, scan.angle_increment,
                    (self.x+.103*math.cos(heading), self.y+.103*math.sin(heading)), heading)
                # A cold/settling persistence filter can have no confirmed
                # endpoints. Karto still sees a full-sized scan vector then
                # attempts correlation against an empty feature set. Do not
                # let no-return-only scans seed or move its pose graph.
                stable = sum(math.isfinite(r) and .1 < r < 12. for r in scan.ranges) >= 12
                if stable:
                    self.mapping_scan_pub.publish(scan)
                else:
                    self.scan_gate.reason = 'insufficient_confirmed_hits'
            self.scan_counts['accepted' if stable else 'rejected'] += 1
            if round(now*10) % 10 == 0:
                import json
                self.quality_pub.publish(self.quality_type(data=json.dumps({**self.scan_counts,
                    'reason': self.scan_gate.reason, 'roll': roll, 'pitch': pitch,
                    'yaw_rate': dh/dt})))
