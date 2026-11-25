"""Small synchronous BT controls for the allocation prototype.

Leaves must be non-blocking: ROS actions are started once and polled on later
ticks. Reactive controls halt a previously running branch when it is deselected.
This module has no ROS dependency, so the same trees run in the offline demo.
"""
from enum import Enum


class Status(str, Enum):
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    RUNNING = 'RUNNING'


class Leaf:
    def __init__(self, name, tick, halt=lambda: None):
        self.name, self.callback, self.on_halt = name, tick, halt
        self.status = None

    def tick(self):
        self.status = self.callback()
        return self.status

    def halt(self):
        if self.status == Status.RUNNING:
            self.on_halt()
        self.status = None

    def snapshot(self):
        return {'name': self.name, 'status': self.status}


class ReactiveControl:
    """Sequence continues on SUCCESS; fallback continues on FAILURE."""
    def __init__(self, name, children, *, fallback=False):
        self.name, self.children, self.fallback = name, list(children), fallback
        self.status = None

    def tick(self):
        advance = Status.FAILURE if self.fallback else Status.SUCCESS
        self.status = advance
        for index, child in enumerate(self.children):
            self.status = child.tick()
            if self.status != advance:
                for unused in self.children[index + 1:]:
                    unused.halt()
                return self.status
        return self.status

    def halt(self):
        for child in self.children:
            child.halt()
        self.status = None

    def snapshot(self):
        return {'name': self.name, 'status': self.status,
                'children': [child.snapshot() for child in self.children]}


def sequence(name, *children):
    return ReactiveControl(name, children)


def fallback(name, *children):
    return ReactiveControl(name, children, fallback=True)
