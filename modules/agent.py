"""Receive goals into the blackboard and run the homework behavior tree."""
from copy import deepcopy
import json
import math

from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from modules.base_bt_nodes import Status
from modules.utils import config
from modules.bt_constructor import build_behavior_tree
from modules.ros_bridge import ROSBridge


class Agent:
    def __init__(self):
        self.config = config
        self.ros_bridge = ROSBridge.get(**config.get('ros', {}))
        self.node = self.ros_bridge.node
        self.options = config['photo_mission']
        self.blackboard = dict(active=False, error='', image_path='',
                               mission_id=0, arrival_checks={})
        self.tf = Buffer(node=self.node)
        self.listener = TransformListener(self.tf, self.node)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.node.create_publisher(String, '/bt/mission_status', latched)
        self.home_pub = self.node.create_publisher(PoseStamped, '/bt/home_pose', latched)
        self.goal_sub = self.node.create_subscription(
            PoseStamped, self.options['goal_topic'], self.receive_goal, 10)
        self.publish_status('IDLE')

    def publish_status(self, stage):
        payload = {key: self.blackboard[key] for key in ('active', 'error', 'image_path', 'mission_id')}
        payload.update(stage=stage, arrival_checks=self.blackboard['arrival_checks'])
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.node.get_logger().info(f'{stage}: {self.blackboard["error"] or self.blackboard["image_path"]}')

    def current_pose(self):
        """Read a map-frame pose; reject a missing clock or stale odometry."""
        now = self.node.get_clock().now()
        if now.nanoseconds == 0:
            raise ValueError('Waiting for simulation /clock')
        robot = self.options['robot_frame']
        transform = self.tf.lookup_transform(self.options['global_frame'], robot, Time())
        odom = self.tf.lookup_transform(self.options['odom_frame'], robot, Time())
        age = abs((now - Time.from_msg(odom.header.stamp)).nanoseconds) / 1e9
        if age > self.options['tf_max_age']:
            raise ValueError('Odometry TF is stale: check Webots and use_sim_time')
        pose = PoseStamped()
        pose.header.frame_id = self.options['global_frame']
        pose.header.stamp = now.to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.orientation = deepcopy(transform.transform.rotation)
        return pose

    def receive_goal(self, message):
        if self.blackboard['active']:
            self.node.get_logger().warning('Goal ignored: mission is active')
            return
        try:
            if message.header.frame_id != self.options['global_frame']:
                raise ValueError(f'RViz Fixed Frame must be {self.options["global_frame"]}')
            goal = deepcopy(message)
            p, q = goal.pose.position, goal.pose.orientation
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                raise ValueError('Goal contains non-finite coordinates')
            norm = math.hypot(q.x, q.y, q.z, q.w)
            if norm < 1e-6:
                raise ValueError('Goal quaternion has zero length')
            q.x, q.y, q.z, q.w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
            home = self.current_pose()
        except (ValueError, TransformException) as error:
            self.blackboard['error'] = str(error)
            self.publish_status('REJECTED')
            return
        self.tree.halt()
        self.tree.reset()
        self.blackboard.update(
            active=True, goal_pose=goal, home_pose=home,
            error='', image_path='', arrival_checks={},
            mission_id=self.blackboard['mission_id'] + 1)
        self.home_pub.publish(home)
        self.publish_status('STARTING')

    def create_behavior_tree(self, behavior_tree_xml):
        self.tree = build_behavior_tree(self, behavior_tree_xml, self.config['scenario'])

    async def run_tree(self):
        self.ros_bridge.spin_once()
        if not self.blackboard['active']:
            return

        self.tree.reset()
        result = await self.tree.run(self, self.blackboard)
        if result in (Status.SUCCESS, Status.FAILURE):
            self.blackboard['active'] = False
            self.publish_status('COMPLETE' if result == Status.SUCCESS else 'FAILED')
        return result

    def close(self):
        def halt(node):
            for child in getattr(node, 'children', []):
                halt(child)
            node.halt()

        halt(self.tree)
        self.ros_bridge.shutdown()
