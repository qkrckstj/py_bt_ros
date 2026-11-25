"""WBT 원본 삼각형의 평지 통로와 같은 시각의 투영 그림자를 검사한다.

기본 mesh 모델은 사각 기둥/구간 AABB를 충돌 판정에 쓰지 않는다.
비교용 legacy_columns와 기존 shadow_boxes API는 이전 결과 재현용이다.
"""
from collections import OrderedDict
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np

from .mesh_geometry import MeshShadow, TriangleIndex


def sun_direction(elapsed, elevation_deg, initial_azimuth_deg, period):
    """태양 컨트롤러와 동일한 절대 시뮬레이션 시간/ENU 정의."""
    az = math.radians((initial_azimuth_deg + 360.*elapsed/period) % 360.)
    el = math.radians(elevation_deg)
    return (math.cos(el)*math.sin(az), math.cos(el)*math.cos(az), math.sin(el))


def _rotation(values):
    x, y, z, angle = values
    norm = math.sqrt(x*x+y*y+z*z)
    if norm == 0:
        return np.eye(3)
    v = np.array([x, y, z])/norm
    cross = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return math.cos(angle)*np.eye(3)+(1-math.cos(angle))*np.outer(v, v)+math.sin(angle)*cross


def _numbers(text):
    return np.fromstring(text.replace(',', ' '), sep=' ')


def _blocks(text):
    """이 월드의 최상위 DEF Solid/LimoFourDiff를 중괄호 균형으로 읽는다."""
    for match in re.finditer(r'^DEF\s+(\w+)\s+(Solid|LimoFourDiff)\s*\{', text, re.M):
        depth, i = 1, match.end()
        while depth and i < len(text):
            if text[i] == '{':
                depth += 1
            elif text[i] == '}':
                depth -= 1
            i += 1
        if depth:
            raise ValueError('WBT 중괄호가 닫히지 않았습니다')
        yield match.group(1), match.group(2), text[match.end():i-1]


def _segment_hits_boxes(a, b, boxes, radius):
    if not len(boxes):
        return False
    lo = boxes[:, :2]-radius
    hi = boxes[:, 2:]+radius
    enter, leave = np.zeros(len(boxes)), np.ones(len(boxes))
    valid = np.ones(len(boxes), dtype=bool)
    for axis in (0, 1):
        delta = b[axis]-a[axis]
        if abs(delta) < 1e-12:
            valid &= (a[axis] >= lo[:, axis]) & (a[axis] <= hi[:, axis])
        else:
            u, v = (lo[:, axis]-a[axis])/delta, (hi[:, axis]-a[axis])/delta
            enter = np.maximum(enter, np.minimum(u, v))
            leave = np.minimum(leave, np.maximum(u, v))
    return bool(np.any(valid & (enter <= leave)))


def _trig_range(a, b, cosine=False):
    if b-a >= 2*math.pi:
        return -1., 1.
    fn = math.cos if cosine else math.sin
    values = [fn(a), fn(b)]
    offset = 0. if cosine else math.pi/2
    first = math.ceil((a-offset)/math.pi)
    last = math.floor((b-offset)/math.pi)
    values.extend(fn(offset+k*math.pi) for k in range(first, last+1))
    return min(values), max(values)


class TerrainModel:
    def __init__(self):
        self._shadow_regions = {}
        self.geometry_mode='legacy_columns'

    @classmethod
    def from_world(cls, world_path, sun_config_path, safety_time_step=.25,
                   terrain_height_threshold=.001, static_robots=(), column_resolution=.5,
                   geometry_mode='mesh',shadow_plane_height=.02):
        text = Path(world_path).read_text()
        config_text = Path(sun_config_path).read_text()
        sun = json.loads(config_text)
        if sun.get('mode') != 'constant_elevation':
            raise ValueError('constant_elevation 태양만 지원합니다')
        elevation = float(sun['elevation_deg'])
        initial = float(sun['initial_azimuth_deg'])
        period = float(sun['azimuth_period_sim_seconds'])
        values = (elevation, initial, period, safety_time_step, terrain_height_threshold, column_resolution)
        if not all(math.isfinite(v) for v in values) or not 0 < elevation < 90 or min(values[2:]) <= 0:
            raise ValueError('잘못된 지형/태양 설정입니다')
        # 평지 통로 모델의 기준은 z=0이다. 임계값으로 경사 주행을 활성화하지 않는다.
        if terrain_height_threshold > .001:
            raise ValueError('평지 모델의 terrain_height_threshold는 0.001m 이하이어야 합니다')
        if geometry_mode not in ('mesh','legacy_columns'):
            raise ValueError('terrain_geometry는 mesh 또는 legacy_columns여야 합니다')
        if not math.isfinite(shadow_plane_height) or shadow_plane_height<0:
            raise ValueError('그림자 수신 높이는 0 이상의 유한 값이어야 합니다')
        triangles = []
        robot_starts = {}
        boundary_boxes = []
        ground_bounds = None
        for name, kind, block in _blocks(text):
            translation_match = re.search(r'^\s*translation\s+([^\n]+)', block)
            rotation_match = re.search(r'^\s*rotation\s+([^\n]+)', block)
            translation = _numbers(translation_match.group(1)) if translation_match else np.zeros(3)
            rotation_values = _numbers(rotation_match.group(1)) if rotation_match else np.array([0, 0, 1, 0])
            rotation = _rotation(rotation_values)
            if kind == 'LimoFourDiff':
                robot_name = re.search(r'\bname\s+"([^"]+)"', block)
                if robot_name:
                    robot_starts[robot_name.group(1)] = {'x': float(translation[0]), 'y': float(translation[1]),
                                                       'yaw': float(math.atan2(rotation[1, 0], rotation[0, 0]))}
                continue
            point_match = re.search(r'\bpoint\s*\[([^]]*)\]', block, re.S)
            index_match = re.search(r'\bcoordIndex\s*\[([^]]*)\]', block, re.S)
            if point_match is None:
                if name.startswith('BOUNDARY_'):
                    size_match = re.search(r'\bBox\s*\{\s*size\s+([^}]+)', block)
                    if not size_match:
                        raise ValueError(f'지원되지 않는 경계벽 형상: {name}')
                    size = _numbers(size_match.group(1))
                    corners = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5)
                                        for z in (-.5, .5)])*size
                    corners = corners @ rotation.T+translation
                    low, high = corners.min(axis=0), corners.max(axis=0)
                    boundary_boxes.append((low[0], low[1], high[0], high[1]))
                    continue
                raise ValueError(f'지원되지 않는 Solid 형상: {name}')
            if index_match is None:
                raise ValueError(f'삼각형 인덱스 누락: {name}')
            vertices = _numbers(point_match.group(1)).reshape(-1, 3) @ rotation.T + translation
            faces, face = [], []
            for value in _numbers(index_match.group(1)).astype(int):
                if value == -1:
                    faces.extend((face[0], face[k], face[k+1]) for k in range(1, len(face)-1))
                    face = []
                else:
                    face.append(value)
            if face:
                raise ValueError(f'닫히지 않은 coordIndex face: {name}')
            mesh = vertices[np.asarray(faces)]
            triangles.append(mesh)
            if name == 'LUNAR_GROUND':
                ground_bounds = (float(vertices[:, 0].min()), float(vertices[:, 0].max()),
                                 float(vertices[:, 1].min()), float(vertices[:, 1].max()))
        if ground_bounds is None or not triangles:
            raise ValueError('LUNAR_GROUND 삼각형 지형이 필요합니다')
        result = cls()
        result.bounds = ground_bounds
        result.robot_starts = robot_starts
        result.geometry_mode=geometry_mode
        result.shadow_plane_height=shadow_plane_height
        result.identifier = hashlib.sha256((text+'\n'+config_text+'\n'+geometry_mode+
                                            str(shadow_plane_height)).encode()).hexdigest()
        result.elevation, result.initial_azimuth, result.period = elevation, initial, period
        result.safety_time_step = safety_time_step
        result.static_robots = tuple(static_robots)
        result.boundary_boxes = np.asarray(boundary_boxes, dtype=float).reshape(-1, 4)
        result._shadow_cache = OrderedDict()
        result._static_cache = OrderedDict()
        all_triangles = np.concatenate(triangles)
        result.triangle_count = len(all_triangles)
        result.mesh_triangles=all_triangles
        nonflat=all_triangles[np.any(np.abs(all_triangles[:,:,2])>terrain_height_threshold,axis=1)]
        if geometry_mode=='mesh':
            result._terrain_index=TriangleIndex(nonflat[:,:,:2])
            result._mesh_shadow=MeshShadow(all_triangles,elevation,initial,period,
                                           safety_time_step,shadow_plane_height)
        # 각 비평탄 삼각형 XY AABB와 교차하는 셀을 모두 채운다.
        columns = {}
        xmin, xmax, ymin, ymax = ground_bounds
        for triangle in nonflat:
            low, high = triangle.min(axis=0), triangle.max(axis=0)
            ia, ib = math.floor((low[0]-xmin)/column_resolution), math.floor((high[0]-xmin)/column_resolution)
            ja, jb = math.floor((low[1]-ymin)/column_resolution), math.floor((high[1]-ymin)/column_resolution)
            for i in range(ia, ib+1):
                for j in range(ja, jb+1):
                    columns[i, j] = max(columns.get((i, j), 0.), float(high[2]))
        boxes, heights = [], []
        for (i, j), height in sorted(columns.items()):
            boxes.append((xmin+i*column_resolution, ymin+j*column_resolution,
                          xmin+(i+1)*column_resolution, ymin+(j+1)*column_resolution))
            heights.append(height)
        result.obstacle_boxes = np.asarray(boxes, dtype=float).reshape(-1, 4)
        heights = np.asarray(heights)
        positive = heights > terrain_height_threshold
        result._caster_boxes = result.obstacle_boxes[positive]
        result._caster_heights = heights[positive]
        return result

    def with_static_robots(self, static_robots):
        import copy
        other = copy.copy(self)
        other.static_robots = tuple(static_robots)
        other._static_cache = OrderedDict()
        return other

    def shadow_boxes(self, ta, tb=None):
        """과거 기둥 모델의 구간 AABB. mesh 충돌 판정에는 사용하지 않는다."""
        tb = ta if tb is None else tb
        step = self.safety_time_step
        key = (math.floor(ta/step), math.ceil(tb/step))
        if key in self._shadow_cache:
            self._shadow_cache.move_to_end(key)
            return self._shadow_cache[key]
        a = math.radians(self.initial_azimuth)+key[0]*step*2*math.pi/self.period
        b = math.radians(self.initial_azimuth)+key[1]*step*2*math.pi/self.period
        sinlo, sinhi = _trig_range(a, b)
        coslo, coshi = _trig_range(a, b, True)
        reach = self._caster_heights/math.tan(math.radians(self.elevation))
        boxes = self._caster_boxes.copy()
        boxes[:, 0] += np.minimum(0., -reach*sinhi)
        boxes[:, 1] += np.minimum(0., -reach*coshi)
        boxes[:, 2] += np.maximum(0., -reach*sinlo)
        boxes[:, 3] += np.maximum(0., -reach*coslo)
        self._shadow_cache[key] = boxes
        if len(self._shadow_cache) > 128:
            self._shadow_cache.popitem(last=False)
        return boxes

    def shadow_triangles(self,t):
        """그림자 수신 평면에 투영한 원래 삼각형. 표시와 검증에 같은 형상을 쓴다."""
        return self._mesh_shadow.projected(t)

    def static_blocked_segment(self, a, b, radius):
        """그림자를 제외한 지형/실제 경계벽/대기 로봇의 swept 충돌 검사."""
        xmin, xmax, ymin, ymax = self.bounds
        if any(not (xmin+radius <= p[0] <= xmax-radius and ymin+radius <= p[1] <= ymax-radius) for p in (a, b)):
            return True
        key = (tuple(a), tuple(b), radius)
        cached=self._static_cache.get(key)
        if cached is not None:return cached
        terrain_hit=(self._terrain_index.blocked(a,b,radius) if self.geometry_mode=='mesh' else
                     _segment_hits_boxes(a,b,self.obstacle_boxes,radius))
        hit = terrain_hit or _segment_hits_boxes(a,b,self.boundary_boxes,radius)
        if not hit:
            for x,y,robot_radius in self.static_robots:
                dx,dy=b[0]-a[0],b[1]-a[1]
                length=dx*dx+dy*dy
                u=min(1.,max(0.,((x-a[0])*dx+(y-a[1])*dy)/length)) if length else 0.
                if math.hypot(a[0]+u*dx-x,a[1]+u*dy-y)<=radius+robot_radius:
                    hit=True
                    break
        self._static_cache[key]=hit
        if len(self._static_cache)>32768:self._static_cache.popitem(last=False)
        return hit

    def blocked_segment(self, a, b, ta, tb, radius):
        if (not all(math.isfinite(v) for v in (*a, *b, ta, tb, radius)) or ta < 0 or tb < ta or radius < 0):
            raise ValueError('잘못된 구간 충돌 검사 인수입니다')
        if self.static_blocked_segment(a, b, radius):
            return True
        if self.geometry_mode=='mesh':
            return self._mesh_shadow.blocked(a,b,ta,tb,radius)
        # 더 넓은 시공간 상자와도 만나지 않는 기둥만 제외한다.
        # 태양·지형이 같은 인스턴스 안에서만 후보 인덱스를 공유한다.
        xmin, xmax = sorted((math.floor(a[0]), math.floor(b[0])))
        ymin, ymax = sorted((math.floor(a[1]), math.floor(b[1])))
        first, last = math.floor(ta / 16), math.ceil(tb / 16)
        key = (xmin, xmax, ymin, ymax, first, last, radius)
        indices = self._shadow_regions.get(key)
        if indices is None:
            boxes = self.shadow_boxes(16 * first, 16 * last)
            indices = np.flatnonzero((boxes[:, 0] - radius <= xmax + 1) &
                                     (boxes[:, 2] + radius >= xmin) &
                                     (boxes[:, 1] - radius <= ymax + 1) &
                                     (boxes[:, 3] + radius >= ymin))
            if len(self._shadow_regions) >= 8192:
                self._shadow_regions.clear()
            self._shadow_regions[key] = indices
        if not len(indices):
            return False
        return _segment_hits_boxes(a, b, self.shadow_boxes(ta, tb)[indices], radius)
