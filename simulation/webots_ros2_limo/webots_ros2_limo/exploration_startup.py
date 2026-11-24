"""One-shot recovery of dropped lifecycle startup replies in the local fleet.

Verify the actual state after every reply/timeout instead of blindly repeating
a transition. Exit once all nodes are active; never reactivate a later user pause.
Normal Nav2 lifecycle managers remain configured and running.
"""
import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState


class StartupRecovery(Node):
    def __init__(self):
        super().__init__('exploration_startup_recovery')
        robots = self.declare_parameter('robots', ['robot1', 'robot2', 'robot3', 'robot4']).value
        nodes = ('slam_toolbox', 'controller_server', 'smoother_server', 'planner_server',
                 'behavior_server', 'bt_navigator')
        self.targets = [f'/{robot}/{node}' for robot in robots for node in nodes]
        self.index, self.pending, self.done = 0, None, False
        self.deadline = time.monotonic()+150.
        self.lifecycle_clients = {target: (self.create_client(GetState, target+'/get_state'),
                                 self.create_client(ChangeState, target+'/change_state'))
                        for target in self.targets}
        self.create_timer(.15, self.update)

    def update(self):
        now = time.monotonic()
        if self.done:
            return
        if now > self.deadline:
            self.get_logger().error(f'Startup incomplete at {self.targets[self.index]}; team input guards remain active')
            self.done = True
            return
        if self.pending:
            future, kind, deadline = self.pending
            if not future.done():
                if now > deadline:
                    future.cancel()
                    self.pending = None
                return
            self.pending = None
            try:
                response = future.result()
            except Exception:
                return
            if kind == 'state' and response:
                state = response.current_state.id
                if state == 3:
                    self.index += 1
                    if self.index == len(self.targets):
                        self.get_logger().info('All 24 fleet lifecycle nodes verified active')
                        self.done = True
                    return
                if state in (1, 2):
                    target = self.targets[self.index]
                    client = self.lifecycle_clients[target][1]
                    if client.service_is_ready():
                        self.get_logger().warning(f'Recovering startup: {target} is {response.current_state.label}')
                        request = ChangeState.Request(transition=Transition(id=1 if state == 1 else 3))
                        self.pending = (client.call_async(request), 'transition', now+3.)
                return
            # Whether the transition reply succeeded or was lost, re-read state.
            return
        client = self.lifecycle_clients[self.targets[self.index]][0]
        if client.service_is_ready():
            self.pending = (client.call_async(GetState.Request()), 'state', now+3.)


def main(args=None):
    rclpy.init(args=args)
    node = StartupRecovery()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
