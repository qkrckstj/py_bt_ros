"""실제 거리 센서의 시각·자세·가림을 반영한 고정 평가 격자.

사전 지형 또는 목표 도착 정보는 받지 않는다. 광선이 통과한 셀과 첫
반사점 셀만 관측하며 광선 사이의 부채꼴을 임의로 채우지 않는다.
"""
from collections import deque
from dataclasses import dataclass
import math
import os
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class EvaluationArea:
    xmin: float
    ymin: float
    xmax: float
    ymax: float
    resolution: float = .25

    def __post_init__(self):
        values = (self.xmin, self.ymin, self.xmax, self.ymax, self.resolution)
        if not all(math.isfinite(v) for v in values) or self.resolution <= 0:
            raise ValueError('평가 영역은 유한한 좌표와 양의 해상도가 필요합니다.')
        if self.xmax <= self.xmin or self.ymax <= self.ymin:
            raise ValueError('평가 영역의 너비와 높이는 양수여야 합니다.')
        for length in (self.xmax - self.xmin, self.ymax - self.ymin):
            if abs(length / self.resolution - round(length / self.resolution)) > 1e-7:
                raise ValueError('고정 분모를 위해 평가 영역은 해상도의 정수 배여야 합니다.')

    @property
    def shape(self):
        return (round((self.ymax - self.ymin) / self.resolution),
                round((self.xmax - self.xmin) / self.resolution))

    @property
    def denominator_m2(self):
        return self.shape[0] * self.shape[1] * self.resolution ** 2

    def index(self, xy):
        return (math.floor((xy[1] - self.ymin) / self.resolution),
                math.floor((xy[0] - self.xmin) / self.resolution))

    def contains(self, xy):
        return self.xmin <= xy[0] < self.xmax and self.ymin <= xy[1] < self.ymax


def ray_cells(area, origin, end):
    """ROI로 자른 선분이 양의 길이로 통과하는 셀을 DDA로 순회한다."""
    x, y = origin
    dx, dy = end[0] - x, end[1] - y
    lo, hi = 0., 1.
    for p, d, low, high in ((x, dx, area.xmin, area.xmax),
                             (y, dy, area.ymin, area.ymax)):
        if abs(d) < 1e-14:
            if not low <= p < high:
                return
        else:
            a, b = sorted(((low - p) / d, (high - p) / d))
            lo, hi = max(lo, a), min(hi, b)
    if lo > hi or hi < 0 or lo > 1:
        return
    a = (min(math.nextafter(area.xmax, area.xmin), max(area.xmin, x + lo * dx)),
         min(math.nextafter(area.ymax, area.ymin), max(area.ymin, y + lo * dy)))
    b = (min(math.nextafter(area.xmax, area.xmin), max(area.xmin, x + hi * dx)),
         min(math.nextafter(area.ymax, area.ymin), max(area.ymin, y + hi * dy)))
    row, col = area.index(a)
    last_row, last_col = area.index(b)
    vx, vy = b[0] - a[0], b[1] - a[1]
    sx, sy = (1 if vx > 0 else -1), (1 if vy > 0 else -1)
    tx = (((area.xmin + (col + (vx > 0)) * area.resolution) - a[0]) / vx
          if abs(vx) > 1e-14 else math.inf)
    ty = (((area.ymin + (row + (vy > 0)) * area.resolution) - a[1]) / vy
          if abs(vy) > 1e-14 else math.inf)
    step_x = area.resolution / abs(vx) if abs(vx) > 1e-14 else math.inf
    step_y = area.resolution / abs(vy) if abs(vy) > 1e-14 else math.inf
    height, width = area.shape
    for _ in range(height + width + 3):
        if not (0 <= row < height and 0 <= col < width):
            return
        yield row, col
        if (row, col) == (last_row, last_col):
            return
        if abs(tx - ty) < 1e-12:
            col, row, tx, ty = col + sx, row + sy, tx + step_x, ty + step_y
        elif tx < ty:
            col, tx = col + sx, tx + step_x
        else:
            row, ty = row + sy, ty + step_y


class ObservationGrid:
    """중복 없는 팀 면적과 최초 융합 기여도를 누적한다.

    로봇별 신규 면적은 성공적으로 융합한 처리 순서의 최초 기여도다.
    늦은 스캔이 최신 점유 상태를 덮지 않도록 셀별 취득 시각을 보존한다.
    """
    def __init__(self, area, robots, sensor_range=6.):
        self.area, self.robots = area, tuple(robots)
        if not self.robots or len(set(self.robots)) != len(self.robots):
            raise ValueError('고유한 로봇 ID가 필요합니다.')
        if not math.isfinite(sensor_range) or sensor_range <= 0:
            raise ValueError('센서 관측 상한은 양의 유한 수여야 합니다.')
        self.sensor_range = sensor_range
        self.cells = np.full(area.shape, -1, dtype=np.int8)
        self.first_seen = np.full(area.shape, np.nan)
        self.last_seen = np.full(area.shape, -math.inf)
        self.observed_count = 0
        self.revision = 0
        self.new_cells = dict.fromkeys(self.robots, 0)
        self.valid_frames = dict.fromkeys(self.robots, 0)
        self.last_received_stamp = dict.fromkeys(self.robots, -math.inf)
        self.last_valid_stamp = dict.fromkeys(self.robots, -math.inf)
        self.recent_stamps = {r: deque(maxlen=1024) for r in self.robots}
        self.rejected_frames = 0
        self.valid_rays = 0
        self.last_pose = dict.fromkeys(self.robots)

    @property
    def observed_area_m2(self):
        return self.observed_count * self.area.resolution ** 2

    @property
    def coverage_fraction(self):
        return self.observed_area_m2 / self.area.denominator_m2

    def integrate(self, robot, stamp, pose, ranges, angle_min, angle_increment,
                  range_min, range_max):
        if robot not in self.new_cells:
            raise ValueError('등록되지 않은 로봇의 관측입니다.')
        geometry = (*pose, stamp, angle_min, angle_increment, range_min, range_max)
        if (len(pose) != 3 or not all(math.isfinite(v) for v in geometry)
                or stamp < 0 or angle_increment <= 0 or range_min < 0 or range_max <= range_min):
            self.rejected_frames += 1
            return 0
        if stamp <= self.last_received_stamp[robot]:
            self.rejected_frames += 1
            return 0
        self.last_received_stamp[robot] = stamp
        free, occupied = set(), set()
        x, y, yaw = pose
        valid = 0
        limit = min(self.sensor_range, range_max)
        for index, distance in enumerate(ranges):
            if math.isnan(distance) or distance == -math.inf or distance < range_min or distance == 0:
                continue
            if math.isfinite(distance) and distance > range_max:
                continue
            valid += 1
            length = min(distance, limit)
            angle = yaw + angle_min + index * angle_increment
            end = (x + length * math.cos(angle), y + length * math.sin(angle))
            free.update(ray_cells(self.area, (x, y), end))
            if math.isfinite(distance) and distance < range_max and distance <= self.sensor_range:
                if self.area.contains(end):
                    occupied.add(self.area.index(end))
        free.difference_update(occupied)
        new = 0
        for value, points in ((0, free), (100, occupied)):
            if not points:
                continue
            rows, cols = np.asarray(tuple(points), dtype=np.intp).T
            unseen = self.cells[rows, cols] == -1
            new += int(np.count_nonzero(unseen))
            # 로봇 간 지연 수신은 허용한다. 기여도는 최초 융합 순서지만
            # 셀의 최초 취득 시각은 융합한 스캔 중 가장 이른 시각이다.
            self.first_seen[rows, cols] = np.fmin(self.first_seen[rows, cols], stamp)
            current = stamp >= self.last_seen[rows, cols]
            self.cells[rows[current], cols[current]] = value
            self.last_seen[rows[current], cols[current]] = stamp
        self.observed_count += new
        self.new_cells[robot] += new
        self.valid_rays += valid
        if valid:
            self.last_pose[robot] = tuple(pose)
            self.valid_frames[robot] += 1
            self.last_valid_stamp[robot] = stamp
            self.recent_stamps[robot].append(stamp)
            self.revision += 1
        return new * self.area.resolution ** 2

    def observed_since(self, robot, stamp, minimum_frames=3):
        return sum(t >= stamp for t in self.recent_stamps[robot]) >= minimum_frames

    def potential_gain(self, xy):
        """목표 후보용 상한 휴리스틱. 실제 관측 면적으로 누적하지 않는다."""
        rows, cols = np.ogrid[:self.area.shape[0], :self.area.shape[1]]
        x = self.area.xmin + (cols + .5) * self.area.resolution
        y = self.area.ymin + (rows + .5) * self.area.resolution
        disk = (x - xy[0]) ** 2 + (y - xy[1]) ** 2 <= self.sensor_range ** 2
        return float(np.count_nonzero((self.cells == -1) & disk)) * self.area.resolution ** 2

    def snapshot(self):
        return {'denominator_m2': self.area.denominator_m2,
                'observed_area_m2': self.observed_area_m2,
                'coverage_fraction': self.coverage_fraction,
                'new_area_m2_by_robot': {r: n * self.area.resolution ** 2 for r, n in self.new_cells.items()},
                'valid_frames_by_robot': self.valid_frames.copy(),
                'valid_rays': self.valid_rays, 'rejected_frames': self.rejected_frames,
                'revision': self.revision,
                'last_scan_stamp_sim_seconds': {r: t if math.isfinite(t) else None for r,t in self.last_valid_stamp.items()},
                'last_sensor_pose_xy_yaw': self.last_pose.copy(),
                'area_definition': 'ROI 내 광선으로 관측한 점유·자유 2D 셀 면적의 합집합',
                'robot_credit_definition': '융합 처리 순서의 최초 신규 팀 셀 기여도'}

    def save(self, output):
        """미관측 셀과 셀별 최초·최신 취득 시각을 독립 분석용으로 보존한다."""
        path = Path(output) / 'observations.npz'
        temporary = path.with_suffix('.npz.tmp')
        with temporary.open('wb') as f:
            np.savez_compressed(f, cells=self.cells, first_seen_sim_seconds=self.first_seen,
                                last_seen_sim_seconds=self.last_seen,
                                origin_xy=[self.area.xmin,self.area.ymin], resolution=self.area.resolution)
        os.replace(temporary,path)
