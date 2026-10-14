"""Save a fresh frame only when an external image subscriber is present."""
from datetime import datetime
from pathlib import Path
import threading
import time
from uuid import uuid4

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class CaptureImageServer(Node):
    def __init__(self):
        super().__init__('capture_image_server')
        self.declare_parameter('image_topic', '/TurtleBot3Burger/front_camera/image_color')
        self.declare_parameter('output_dir', str(Path(__file__).resolve().parents[2] / 'output' / 'captures'))
        self.declare_parameter('frame_timeout', 3.0)
        self.directory = Path(self.get_parameter('output_dir').value).expanduser()
        self.bridge = CvBridge()
        self.condition = threading.Condition()
        self.request_lock = threading.Lock()
        self.frame = None
        self.frame_number = 0
        self.callback_group = ReentrantCallbackGroup()
        self.service = self.create_service(
            Trigger, '/bt/capture_image', self.capture, callback_group=self.callback_group)
        self.get_logger().info(f'Capture service ready; output: {self.directory}')

    def receive_image(self, message):
        with self.condition:
            self.frame = message
            self.frame_number += 1
            self.condition.notify_all()

    def require_external_subscriber(self):
        topic = self.get_parameter('image_topic').value
        endpoints = self.get_subscriptions_info_by_topic(topic)
        if not any(
            (endpoint.node_name, endpoint.node_namespace)
            != (self.get_name(), self.get_namespace())
            for endpoint in endpoints
        ):
            raise RuntimeError(
                f'Camera requires manual subscription: ros2 topic echo {topic} '
                '--qos-reliability best_effort --no-arr')

    def capture(self, request, response):
        response.success = False
        if not self.request_lock.acquire(blocking=False):
            response.message = 'Another capture is in progress'
            return response
        subscription = None
        try:
            # Never become the first subscriber and activate the camera ourselves.
            self.require_external_subscriber()
            requested_at = self.get_clock().now().nanoseconds
            if requested_at == 0:
                raise RuntimeError('No simulation clock received')
            deadline = time.monotonic() + float(self.get_parameter('frame_timeout').value)
            with self.condition:
                self.frame = None
                before = self.frame_number
                # Receive a frame from the camera the user has already enabled.
                subscription = self.create_subscription(
                    Image, self.get_parameter('image_topic').value, self.receive_image,
                    qos_profile_sensor_data, callback_group=self.callback_group)
                while True:
                    self.require_external_subscriber()
                    frame = self.frame
                    stamp = (frame.header.stamp.sec * 10**9 + frame.header.stamp.nanosec) if frame is not None else 0
                    if self.frame_number > before and stamp > requested_at:
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('No new camera frame after capture request; check camera topic and simulation clock')
                    # Stop waiting and unsubscribe if the user's subscriber exits.
                    self.condition.wait(min(remaining, 0.1))
            image = self.bridge.imgmsg_to_cv2(frame, desired_encoding='bgr8')
            self.directory.mkdir(parents=True, exist_ok=True)
            filename = f'{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:8]}.png'
            path = (self.directory / filename).resolve()
            if not cv2.imwrite(str(path), image):
                raise OSError(f'Could not write {path}')
            response.success = True
            response.message = str(path)
            self.get_logger().info(f'Saved {path}')
        except Exception as error:
            response.message = str(error)
            self.get_logger().error(response.message)
        finally:
            try:
                if subscription is not None:
                    self.destroy_subscription(subscription)
                with self.condition:
                    self.frame = None
            finally:
                self.request_lock.release()
        return response


def main():
    rclpy.init()
    node = CaptureImageServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
