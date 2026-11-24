#!/usr/bin/env python3
"""Operator-run keyboard control with periodic Twist and a key timeout.

This is the ONLY helper in this integration that commands real robot motion.
It must be started explicitly in an interactive terminal by the operator.
"""
import argparse
import math
import select
import sys
import termios
import time
import tty


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=.15)
    parser.add_argument('--turn', type=float, default=.5)
    parser.add_argument('--key-timeout', type=float, default=.4)
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('An interactive terminal is required')
    if not (math.isfinite(args.speed) and 0 < args.speed <= .4 and
            math.isfinite(args.turn) and 0 < args.turn <= 1. and
            math.isfinite(args.key_timeout) and 0 < args.key_timeout <= 1.):
        parser.error('speed: (0,.4], turn: (0,1], key-timeout: (0,1]')
    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init()
    node = rclpy.create_node('twin_operator_keyboard')
    publisher = node.create_publisher(Twist, '/cmd_vel', 1)
    settings = termios.tcgetattr(sys.stdin)
    keys = {'i': (1, 0), ',': (-1, 0), 'j': (0, 1), 'l': (0, -1),
            'u': (1, 1), 'o': (1, -1), 'm': (-1, -1), '.': (-1, 1)}
    x = yaw = 0
    last_key = 0.
    print('REAL LIMO: hold i=forward, ,=back, j/l=turn, k/space=stop, Ctrl-C=exit.', flush=True)
    print(f'Speed {args.speed:.2f} m/s; no key for {args.key_timeout:.1f}s -> stop.', flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], .05)[0]:
                key = sys.stdin.read(1)
                if key in ('\x03', ''):
                    break
                x, yaw = keys.get(key, (0, 0))
                last_key = time.monotonic()
            if time.monotonic() - last_key > args.key_timeout:
                x = yaw = 0
            message = Twist()
            message.linear.x = x * args.speed
            message.angular.z = yaw * args.turn
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        for _ in range(3):
            publisher.publish(Twist())
            time.sleep(.03)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
