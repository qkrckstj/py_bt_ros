import math
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class Navigation:
    def __init__(self, node, action_name, frame):
        self.node, self.frame = node, frame
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.state, self.handle, self.cancel_requested = 'IDLE', None, False
        self.cancel_future = None
        self.pose_ready = False

    def ready(self):
        return self.pose_ready and self.client.server_is_ready()

    def start(self, goal):
        if self.state in ('SENDING', 'NAVIGATING', 'CANCELING'):
            raise RuntimeError('Previous navigation action has not terminated')
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = self.frame
        # This is a fixed target in the shared map, not a historical sensor
        # observation. Zero asks TF/Nav2 to use the latest transform even when
        # replanning a long-running goal after its original timestamp expired.
        request.pose.header.stamp = Time().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = goal[:2]
        request.pose.pose.orientation.z = math.sin(goal[2] / 2)
        request.pose.pose.orientation.w = math.cos(goal[2] / 2)
        self.handle, self.cancel_requested, self.cancel_future = None, False, None
        self.state = 'SENDING'
        self.client.send_goal_async(request).add_done_callback(self.accepted)

    def accepted(self, future):
        try:
            self.handle = future.result()
            if not self.handle.accepted:
                self.state = 'CANCELED' if self.cancel_requested else 'FAILED'
                return
            self.state = 'NAVIGATING'
            self.handle.get_result_async().add_done_callback(self.finished)
            if self.cancel_requested:
                self.cancel()
        except Exception as exc:
            self.node.get_logger().error(f'Goal acceptance failed: {exc}')
            self.state = 'FAILED'

    def finished(self, future):
        try:
            response = future.result()
            if self.cancel_requested or response.status == GoalStatus.STATUS_CANCELED:
                self.state = 'CANCELED'
            elif response.status == GoalStatus.STATUS_SUCCEEDED:
                self.state = 'SUCCEEDED'
            else:
                self.state = 'FAILED'
        except Exception as exc:
            self.node.get_logger().error(f'Goal result failed: {exc}')
            self.state = 'FAILED'
        self.handle = None

    def cancel(self):
        if self.state not in ('SENDING', 'NAVIGATING', 'CANCELING'):
            return
        self.cancel_requested = True
        if self.handle and self.cancel_future is None:
            self.state = 'CANCELING'
            self.cancel_future = self.handle.cancel_goal_async()
        # Only the action result certifies termination, not the cancel response.

    def poll(self):
        return self.state


