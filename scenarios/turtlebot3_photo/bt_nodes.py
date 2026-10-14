"""Photo mission: goal check, navigation, arrival check, capture and return."""
from copy import deepcopy
import math
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from modules.base_bt_nodes import BTNodeList, Node, Sequence, Status
from modules.base_bt_nodes_ros import ActionWithROSAction, ActionWithROSService

CUSTOM_ACTION_NODES = ['MoveToGoal', 'CaptureImage', 'Return']
CUSTOM_CONDITION_NODES = ['HasGoal', 'IsAtGoal', 'IsAtHome']
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)


class HasGoal(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.type = 'Condition'

    async def run(self, agent, blackboard):
        self.status = Status.SUCCESS if blackboard.get('active') else Status.RUNNING
        return self.status


class IsAtGoal(Node):
    """Separate arrival condition: compare current TF position AND heading.

    Navigation completion alone cannot pass this condition. It returns only
    SUCCESS/FAILURE, and a failed check stops the Sequence before the next action.
    """
    pose_key = 'goal_pose'

    def __init__(self, name, agent):
        super().__init__(name)
        self.type = 'Condition'

    @staticmethod
    def yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))

    async def run(self, agent, blackboard):
        try:
            current = agent.current_pose().pose
            target = blackboard[self.pose_key].pose
            distance = math.hypot(current.position.x - target.position.x,
                                  current.position.y - target.position.y)
            angle = self.yaw(current.orientation) - self.yaw(target.orientation)
            heading_error = abs(math.atan2(math.sin(angle), math.cos(angle)))
            options = agent.config['photo_mission']
            passed = (distance <= options['arrival_position_tolerance'] and
                      heading_error <= options['arrival_heading_tolerance'])
            blackboard['arrival_checks'][self.name] = {
                'passed': passed, 'distance_m': distance, 'heading_error_rad': heading_error}
            self.status = Status.SUCCESS if passed else Status.FAILURE
            if not passed:
                blackboard['error'] = (
                    f'{self.name}: not at target '
                    f'(distance={distance:.3f} m, heading error={heading_error:.3f} rad)')
        except (ValueError, TransformException) as error:
            blackboard['arrival_checks'][self.name] = {'passed': False, 'error': str(error)}
            blackboard['error'] = f'{self.name}: {error}'
            self.status = Status.FAILURE
        agent.publish_status(self.name)
        return self.status


class IsAtHome(IsAtGoal):
    pose_key = 'home_pose'


class MoveToGoal(ActionWithROSAction):
    """Send one blackboard pose to Nav2 and wait for its result."""
    pose_key = 'goal_pose'

    def __init__(self, name, agent):
        super().__init__(name, agent, (NavigateToPose, agent.config['photo_mission']['navigation_action']))
        self.goal_future = None
        self.started_at = None

    async def run(self, agent, blackboard):
        try:
            if self.started_at is None:
                self.started_at = time.monotonic()
                agent.publish_status(self.name)
            if self.goal_future is None:
                if not self.client.server_is_ready():
                    if time.monotonic() - self.started_at > agent.config['photo_mission']['server_timeout']:
                        raise TimeoutError('Nav2 server unavailable')
                    self.status = Status.RUNNING
                    return self.status
                goal = NavigateToPose.Goal()
                goal.pose = deepcopy(blackboard[self.pose_key])
                goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
                self.goal_future = self.client.send_goal_async(goal)

            if self.goal_future.done() and self._goal_handle is None:
                self._goal_handle = self.goal_future.result()
                if not self._goal_handle.accepted:
                    raise RuntimeError('Nav2 rejected the goal')
                self._result_future = self._goal_handle.get_result_async()

            if self._result_future is not None and self._result_future.done():
                response = self._result_future.result()
                if response.status != GoalStatus.STATUS_SUCCEEDED or getattr(response.result, 'error_code', 0):
                    raise RuntimeError(f'Nav2 failed: status={response.status}, {response.result}')
                self.status = Status.SUCCESS
            else:
                self.status = Status.RUNNING
        except Exception as error:
            blackboard['error'] = f'{self.name}: {error}'
            self.status = Status.FAILURE
        return self.status

    def halt(self):
        # Cancel an unfinished goal, including one accepted after shutdown begins.
        if self.goal_future is not None and self._goal_handle is None:
            self.goal_future.add_done_callback(self.cancel_accepted_goal)
        elif self._goal_handle is not None and self._goal_handle.accepted:
            if self._result_future is None or not self._result_future.done():
                self._goal_handle.cancel_goal_async()
        self.goal_future = None
        self._goal_handle = None
        self._result_future = None
        self.started_at = None

    @staticmethod
    def cancel_accepted_goal(future):
        try:
            handle = future.result()
            if handle.accepted:
                handle.cancel_goal_async()
        except Exception:
            pass  # The ROS context may already be shutting down.


class Return(MoveToGoal):
    pose_key = 'home_pose'


class CaptureImage(ActionWithROSService):
    """Request one photo and wait for the service's save result."""
    def __init__(self, name, agent):
        super().__init__(name, agent, (Trigger, agent.config['photo_mission']['capture_service']))
        self.started_at = None

    async def run(self, agent, blackboard):
        try:
            if self.started_at is None:
                self.started_at = time.monotonic()
                agent.publish_status(self.name)
            if self._future is None:
                if not self.client.service_is_ready():
                    if time.monotonic() - self.started_at > agent.config['photo_mission']['server_timeout']:
                        raise TimeoutError('Capture service unavailable')
                    self.status = Status.RUNNING
                    return self.status
                self._future = self.client.call_async(Trigger.Request())
            if self._future.done():
                response = self._future.result()
                if not response.success:
                    raise RuntimeError(response.message)
                blackboard['image_path'] = response.message
                self.status = Status.SUCCESS
            else:
                self.status = Status.RUNNING
        except Exception as error:
            blackboard['error'] = f'{self.name}: {error}'
            self.status = Status.FAILURE
        return self.status

    def halt(self):
        self._future = None
        self.started_at = None
