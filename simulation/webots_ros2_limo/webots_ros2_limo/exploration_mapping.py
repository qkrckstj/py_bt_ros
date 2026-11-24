"""Idempotent map-freeze handshake, independent of ROS service timing."""
import copy


class MappingMode:
    def __init__(self, robots):
        self.robots = tuple(robots)
        self.session, self.mode, self.retired = None, 'MAPPING', set()
        self.confirmed, self.frozen_map = set(), None

    def request(self, session, mode, latest_map):
        if not isinstance(session, str) or not session or session in self.retired:
            return False
        if mode not in ('MAPPING', 'LOCALIZATION'):
            return False
        if mode == 'LOCALIZATION' and latest_map is None:
            return False
        if session == self.session:
            # A delayed exploration command must never reopen a frozen mission.
            if self.mode == 'LOCALIZATION' and mode == 'MAPPING':
                return False
            if mode == self.mode:
                return True
        elif self.session:
            self.retired.add(self.session)
        self.session, self.mode, self.confirmed = session, mode, set()
        self.frozen_map = copy.deepcopy(latest_map) if mode == 'LOCALIZATION' else None
        return True

    def acknowledge(self, robot, session, mode, success):
        if success and robot in self.robots and (session, mode) == (self.session, self.mode):
            self.confirmed.add(robot)

    def snapshot(self, sources_fresh):
        frozen = self.mode == 'LOCALIZATION' and self.frozen_map is not None
        return {'session': self.session, 'mode': self.mode, 'frozen': frozen,
                'confirmed': sorted(self.confirmed), 'sources_fresh': sources_fresh,
                'ready': bool(self.session) and set(self.robots) == self.confirmed
                and (frozen or sources_fresh)}
