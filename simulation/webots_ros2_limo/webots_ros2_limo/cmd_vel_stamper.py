"""Bridge the LIMO Twist interface to Jazzy's stamped diff-drive controller."""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node('limo_cmd_vel_stamper')
    publisher = node.create_publisher(TwistStamped, '/diffdrive_controller/cmd_vel', 10)

    def forward(message):
        stamped = TwistStamped()
        stamped.header.stamp = node.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = message
        publisher.publish(stamped)

    node.create_subscription(Twist, '/cmd_vel', forward, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
