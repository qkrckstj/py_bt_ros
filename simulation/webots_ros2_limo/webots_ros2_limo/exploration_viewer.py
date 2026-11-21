"""Read BT snapshots for pygame and publish robot markers for RViz.

Closing this viewer never sends a stop or goal to the exploration system.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import time

import numpy as np
import pygame
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from .exploration_bt import Status


PALETTE = [(224, 74, 74), (62, 176, 111), (75, 135, 230), (226, 166, 48)]


class Viewer(Node):
    def __init__(self):
        super().__init__('exploration_viewer')
        self.team, self.robots, self.received = {}, {}, {}
        self.known_area = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/exploration/team_status',
                                 lambda m: self.receive('team', m), qos)
        for i in range(4):
            name = f'robot{i+1}'
            self.create_subscription(String, f'/{name}/exploration/state',
                                     lambda m, n=name: self.receive(n, m), 1)
        self.create_subscription(OccupancyGrid, '/map', self.grid, qos)
        self.markers = self.create_publisher(MarkerArray, '/exploration/robots', qos)
        self.create_timer(.2, self.publish_markers)

    def receive(self, name, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                return
        except (ValueError, TypeError):
            return
        if name == 'team':
            self.team = data
        else:
            self.robots[name] = data
        self.received[name] = time.monotonic()

    def grid(self, message):
        cells = np.asarray(message.data, dtype=np.int8)
        self.known_area = np.count_nonzero(cells >= 0) * message.info.resolution**2

    def publish_markers(self):
        markers = []
        for i, name in enumerate(f'robot{n+1}' for n in range(4)):
            state = self.robots.get(name, {})
            p = state.get('position')
            if not p or time.monotonic()-self.received.get(name, 0) > 3:
                continue
            for text in (False, True):
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns, m.id = name, int(text)
                m.type = Marker.TEXT_VIEW_FACING if text else Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y = float(p[0]), float(p[1])
                m.pose.position.z = .65 if text else .15
                m.pose.orientation.w = 1.
                m.scale.x, m.scale.y, m.scale.z = .5, .4, .3
                if text:
                    m.scale.z = .4
                    m.text = name
                m.color.r, m.color.g, m.color.b = [c/255 for c in PALETTE[i]]
                m.color.a = 1.
                m.lifetime.sec = 1
                markers.append(m)
        self.markers.publish(MarkerArray(markers=markers))



CONDITIONS = {'Disabled', 'MissingInputs', 'MissionFinished', 'AllCellsKnown',
              'UnavailableOrStopped', 'ExplorationEnded', 'NoExplorationGoals', 'ReturnGoalAssigned',
              'BaseDisconnected', 'BaseLinkUnavailable'}


class SnapshotNode:
    """Adapt real ROS snapshots to the existing repository renderer interface."""
    def __init__(self, data):
        self.name = data.get('name', '?')
        self.type = data.get('type', 'Condition' if self.name in CONDITIONS else 'Action')
        self.status = Status(data['status']) if data.get('status') in Status._value2member_map_ else None
        self.children = [SnapshotNode(child) for child in data.get('children', [])]


def load_renderer(repo):
    path = repo / 'modules/bt_visualiser.py'
    if not path.is_file():
        raise RuntimeError(f'Original BT renderer missing: {path}. Set bt_repo to your py_bt_ros checkout.')
    spec = importlib.util.spec_from_file_location('exploration_original_bt_renderer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Fit the current leaf names; all drawing and interaction use the original renderer.
    module.NODE_WIDTH = 210
    return module


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--snapshot', type=lambda value: Path(value) if value else None)
    parser.add_argument('--repo', type=Path, default=Path.home()/'ros2_ws/src/py_bt_ros')
    options, ros_args = parser.parse_known_args(args)
    renderer = load_renderer(options.repo)
    rclpy.init(args=ros_args)
    node = Viewer()
    pygame.init()
    screen = pygame.display.set_mode((1280, 850), pygame.RESIZABLE)
    pygame.display.set_caption('Exploration BT | Original py_bt_ros renderer')
    font = pygame.font.SysFont(renderer.FONT_NAME, 18)
    small = pygame.font.SysFont(renderer.FONT_NAME, 15)
    viewers = [renderer.BTViewer(direction='Horizontal') for _ in range(5)]
    names = ['team', 'robot1', 'robot2', 'robot3', 'robot4']
    clock = pygame.time.Clock()
    running, saved, selected = True, False, 0
    try:
        while running and rclpy.ok():
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0)
            events = pygame.event.get()
            tree_events = []
            for event in events:
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and pygame.K_0 <= event.key <= pygame.K_4:
                    selected = event.key-pygame.K_0
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and event.pos[1] < 45:
                    selected = min(4, max(0, event.pos[0]//150))
                else:
                    tree_events.append(event)
            running = running and viewers[selected].handle_events(tree_events)
            name = names[selected]
            state = node.team if selected == 0 else node.robots.get(name, {})
            screen.fill(renderer.BG_COLOR)
            for i, label in enumerate(['0  Team BT', '1  Robot 1', '2  Robot 2', '3  Robot 3', '4  Robot 4']):
                rect = pygame.Rect(i*150+5, 5, 143, 34)
                pygame.draw.rect(screen, (215, 230, 245) if i == selected else (238, 238, 238), rect)
                screen.blit(font.render(label, True, (20, 20, 20)), (rect.x+10, 12))
            age = '--' if name not in node.received else f'{time.monotonic()-node.received[name]:.1f}s'
            status = state.get('phase', state.get('status', 'Waiting for status'))
            info = f'{name.upper()} | {status} | Data age: {age}'
            if selected:
                p = state.get('position')
                if p:
                    info += f' | Position: ({p[0]:.2f}, {p[1]:.2f})'
            elif state.get('sweep'):
                sweep = state['sweep']
                info += f" | RADAR {sweep['stage']} {sweep['angle_degrees']:.0f} deg | Radius {sweep['radius']:.1f} m"
            screen.blit(small.render(info, True, (30, 30, 30)), (15, 49))
            area = screen.subsurface(pygame.Rect(0, 80, screen.get_width(), max(1, screen.get_height()-115)))
            data = state.get('tree')
            if data:
                viewers[selected].render_tree(area, SnapshotNode(data))
            else:
                area.fill(renderer.BG_COLOR)
                area.blit(font.render('Waiting for live BT snapshot...', True, (80, 80, 80)), (30, 30))
            footer = '0-4: select tree | Drag: pan | Wheel / +/-: zoom | R: reset view | Esc: close viewer only'
            screen.blit(small.render(footer, True, (65, 65, 65)), (15, screen.get_height()-26))
            pygame.display.flip()
            if options.snapshot and not saved and data and len(node.robots) == 4:
                options.snapshot.parent.mkdir(parents=True, exist_ok=True)
                pygame.image.save(screen, str(options.snapshot))
                saved = True
            clock.tick(10)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
