"""Webots plugin: publish complete URDF and reliably initialize the TF publisher.

Recent controller_manager versions require ros2_control tags on the description
topic. Retry robot_state_publisher initialization until its service responds;
the driver's one-shot startup service call can run before discovery completes.
"""
from copy import deepcopy
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters


def merge_description(geometry, hardware):
    root = ET.fromstring(geometry)
    # Ignore the placeholder published before Webots connects.
    if not root.findall('joint'):
        return None
    for old in root.findall('ros2_control'):
        root.remove(old)
    for control in ET.fromstring(hardware).findall('ros2_control'):
        root.append(deepcopy(control))
    return ET.tostring(root, encoding='unicode')


class ControlDescriptionPlugin:
    def init(self, webots_node, properties):
        if not rclpy.ok():
            rclpy.init(args=[])
        self.node = Node('webots_control_description')
        path = Path(get_package_share_directory('webots_ros2_turtlebot')) / 'resource/turtlebot_webots.urdf'
        self.description = merge_description(webots_node.robot.getUrdf(), path.read_text())
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.node.create_publisher(String, '/control_robot_description', qos)
        self.publisher.publish(String(data=self.description))
        self.client = self.node.create_client(SetParameters, '/robot_state_publisher/set_parameters')
        self.future = None
        self.sent_at = 0.0
        self.initialized = False

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.initialized:
            return
        if self.future is not None and self.future.done():
            response = self.future.result()
            self.initialized = all(result.successful for result in response.results)
            if self.initialized:
                self.node.get_logger().info('Robot description and TF publisher initialized')
                return
        if self.client.service_is_ready() and time.monotonic() - self.sent_at > 2.0:
            if self.future is not None and not self.future.done():
                self.client.remove_pending_request(self.future)
            request = SetParameters.Request(parameters=[Parameter(
                name='robot_description', value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=self.description))])
            self.future = self.client.call_async(request)
            self.sent_at = time.monotonic()
