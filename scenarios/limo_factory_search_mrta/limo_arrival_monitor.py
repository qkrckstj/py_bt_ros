import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus

class ArrivalMonitor(Node):
    def __init__(self):
        super().__init__('limo_arrival_monitor')

        self.zones = {
            'robot1': '1구역',
            'robot2': '3구역',
            'robot3': '2구역'
        }
        self.moving = set()
        self.completed = set()

        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        for robot in self.zones:
            self.create_subscription(
                GoalStatus,
                '/unused',
                lambda msg: None,
                10
            ) if False else None

        from action_msgs.msg import GoalStatusArray

        for robot in self.zones:
            self.create_subscription(
                GoalStatusArray,
                f'/{robot}/navigate_to_pose/_action/status',
                lambda msg, name=robot: self.check(msg, name),
                qos
            )

        print('\n===== LIMO 도착 확인 =====', flush=True)
        print('로봇 이동 상태 확인 중...\n', flush=True)

    def check(self, msg, robot):
        for goal in msg.status_list:
            goal_id = (robot, bytes(goal.goal_info.goal_id.uuid))

            if goal.status == GoalStatus.STATUS_EXECUTING and robot not in self.completed:
                if goal_id not in self.moving:
                    self.moving.add(goal_id)
                    print(f'{robot} → {self.zones[robot]} 이동 중...', flush=True)

            elif goal.status == GoalStatus.STATUS_SUCCEEDED:
                if goal_id in self.moving and robot not in self.completed:
                    self.completed.add(robot)
                    print(f'{robot} → {self.zones[robot]} 도착 완료!', flush=True)

        if len(self.completed) == 3:
            print('\n모든 로봇 도착 완료!', flush=True)
            raise SystemExit

rclpy.init()
node = ArrivalMonitor()

try:
    rclpy.spin(node)
except (KeyboardInterrupt, SystemExit):
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
