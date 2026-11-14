import json
import math
import rclpy

from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


tasks = {
    1: (1.0, 8.6, -1.570796327),
    2: (1.2, -8.6, 1.570796327),
    3: (4.85, 0.65, -1.570796327)
}


def assign_tasks(robots, tasks):
    assignments = {}
    remaining = tasks.copy()

    for robot, position in robots.items():
        if not remaining:
            break

        task_id = min(
            remaining,
            key=lambda task: math.dist(
                position, remaining[task][:2]
            )
        )

        assignments[robot] = [list(remaining[task_id])]
        del remaining[task_id]

    return assignments


class PatrolPublisher(Node):
    def __init__(self):
        super().__init__('patrol_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(
            String, '/multi_bt/patrol_tasks', qos
        )

        self.published = False
        self.timer = self.create_timer(2.0, self.publish_tasks)

    def get_robot_positions(self):
        robots = {}

        for robot in ['robot1', 'robot2', 'robot3']:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    f'{robot}/base_link',
                    Time()
                )

                p = transform.transform.translation
                robots[robot] = (p.x, p.y)

            except TransformException:
                return None

        return robots

    def publish_tasks(self):
        if self.published:
            return

        robots = self.get_robot_positions()

        if robots is None:
            self.get_logger().info(
                'Waiting for robot positions...'
            )
            return

        assignments = assign_tasks(robots, tasks)

        msg = String()
        msg.data = json.dumps(assignments)
        self.publisher.publish(msg)

        self.get_logger().info(
            f'Robot positions: {robots}'
        )
        self.get_logger().info(
            f'Patrol tasks: {msg.data}'
        )

        self.get_logger().info('===== MRTA 업무 할당 결과 =====')

        zones = {
            (1.0, 8.6): '1구역',
            (1.2, -8.6): '2구역',
            (4.85, 0.65): '3구역'
        }

        for robot, goals in assignments.items():
            if goals:
                x, y = goals[0][:2]
                zone = zones.get((x, y), '알 수 없는 구역')
                self.get_logger().info(f'{robot} → {zone}')

        self.get_logger().info('============================')

        self.published = True
        self.timer.cancel()


def main():
    rclpy.init()
    node = PatrolPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
