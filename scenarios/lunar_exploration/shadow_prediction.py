"""Conservative simulator-oracle shadow envelopes, not sensor inference.

Project terrain vertices onto z=.02 and conservatively take each mesh's convex
hull. Inflate for the nine-sample footprint used by the original shadow monitor,
pose uncertainty, rasterization and angular motion between temporal samples.
Bounding boxes are available as a more conservative fallback for unit fixtures.
"""
import base64
import json
import math
import zlib

import numpy as np


def triangle_envelope(meshes, direction, origin, resolution, size, margin=.65):
    """Union of triangle footprints and ground-plane shadow projections.

    Unlike a mesh-wide hull, a large ground mesh cannot hide the entire map.
    Raised terrain footprints are excluded too: this is conservative planar
    navigation, not a claim of traversability on slopes or crater surfaces.
    """
    import cv2
    mask = np.zeros((size, size), dtype=np.uint8)
    direction = np.asarray(direction, dtype=float)
    if direction[2] <= 0:
        raise ValueError('Sun must be above the horizon')
    for _, _, v0, e1, e2 in meshes:
        triangles = np.stack((v0, v0+e1, v0+e2), axis=1)
        triangles = triangles[np.max(triangles[:, :, 2], axis=1) > .021]
        projected = triangles[:, :, :2] - (
            np.maximum(triangles[:, :, 2]-.02, 0)/direction[2]
        )[:, :, None]*direction[:2]
        polygons = np.concatenate((triangles[:, :, :2], projected), axis=1)
        pixels = ((polygons-np.asarray(origin))/resolution).astype(np.float32)
        for polygon in pixels:
            hull = cv2.convexHull(polygon).reshape(-1, 2)
            # Separate fills preserve the union even when projections overlap.
            cv2.fillConvexPoly(mask, np.rint(hull).astype(np.int32), 1)
    radius = math.ceil(margin/resolution)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*radius+1, 2*radius+1))
    return cv2.dilate(mask, kernel).astype(bool)


def hull(points):
    points = sorted(set(map(tuple, points)))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.asarray(lower[:-1]+upper[:-1])


def envelope(boxes, direction, origin=(-11., -11.), resolution=.15,
             size=147, margin=.65, clouds=None):
    """Cell-center mask; margin includes <=.11m motion over half a 2s bin."""
    y, x = np.mgrid[:size, :size]
    x = origin[0]+(x+.5)*resolution
    y = origin[1]+(y+.5)*resolution
    blocked = np.zeros(x.shape, dtype=bool)
    direction = np.asarray(direction)
    for i, (low, high) in enumerate(boxes):
        vertices = (clouds[i].copy() if clouds is not None else
                    np.asarray([(a, b, c) for a in (low[0], high[0])
                                for b in (low[1], high[1]) for c in (low[2], high[2])]))
        vertices[:, :2] -= (np.maximum(vertices[:, 2]-.02, 0)/direction[2])[:, None]*direction[:2]
        if clouds is not None:
            # Native hull avoids repeated Python sorting of terrain vertices.
            import cv2
            polygon = cv2.convexHull(vertices[:, :2].astype(np.float32)).reshape(-1, 2)
        else:
            polygon = hull(vertices[:, :2])
        inside = np.ones(x.shape, dtype=bool)
        # Expanded polygon half-planes conservatively cover its radius-margin
        # dilation, including corners. Larger envelopes may reject valid paths.
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
            dx, dy = b-a
            inside &= dx*(y-a[1])-dy*(x-a[0]) >= -margin*math.hypot(dx, dy)
        blocked |= inside
    return blocked


def navigation_mask(masks):
    """Current plus next 2s shadow bin for reactive costmaps.

    Farther-future shadows stay in Forecast. Putting an 8s union in Nav2's
    timeless grid can trap a robot whose time-checked route leaves before
    the shadow arrives.
    """
    return np.any(np.asarray(masks)[:2],axis=0).astype(np.int8)*100


class Forecast:
    def __init__(self, payload, navigation_clearance_m=0.):
        self.payload = json.loads(payload) if isinstance(payload, str) else payload
        p = self.payload
        self.stamp = p['stamp']
        self.dt = p['dt']
        self.origin = p['origin']
        self.resolution = p['resolution']
        self.masks = np.frombuffer(zlib.decompress(base64.b64decode(p['masks'])), dtype=np.uint8).reshape(p['shape'])
        if navigation_clearance_m:
            import cv2
            # Nav2 checks the entire .295m footprint, while the original
            # allocator checked only the path center. Match that footprint
            # plus a raster reserve without changing the oracle's margins.
            radius = math.ceil(navigation_clearance_m/self.resolution)
            ys,xs = np.mgrid[-radius:radius+1,-radius:radius+1]
            kernel = (np.hypot(xs,ys)*self.resolution<=navigation_clearance_m+1e-9).astype(np.uint8)
            self.masks = np.stack([cv2.dilate(mask,kernel) for mask in self.masks])
        self.prefix = np.cumsum(self.masks,axis=0,dtype=np.int16)
        self.route_samples = {}

    def safe(self, point, at):
        age = at-self.stamp
        if age < -.5 or age > (len(self.masks)-1)*self.dt:
            return False
        row = math.floor((point[1]-self.origin[1])/self.resolution)
        col = math.floor((point[0]-self.origin[0])/self.resolution)
        if not (0 <= row < self.masks.shape[1] and 0 <= col < self.masks.shape[2]):
            return False
        index = max(0, min(len(self.masks)-1, int(round(age/self.dt))))
        return not bool(self.masks[index, row, col])

    def hold_safe(self, point, start, duration):
        if start < self.stamp-.5 or start+duration > self.stamp+(len(self.masks)-1)*self.dt:
            return False
        row = math.floor((point[1]-self.origin[1])/self.resolution)
        col = math.floor((point[0]-self.origin[0])/self.resolution)
        if not (0<=row<self.masks.shape[1] and 0<=col<self.masks.shape[2]):
            return False
        lo = max(0,math.floor((start-self.stamp)/self.dt))
        hi = min(len(self.masks)-1,math.ceil((start+duration-self.stamp)/self.dt))
        count = self.prefix[hi,row,col]-(self.prefix[lo-1,row,col] if lo else 0)
        return count == 0

    def route_safe(self, route, start, speed=.10, turn_delay=5., hold=10., team_horizon=0.):
        if not route or not self.hold_safe(route[0], start, turn_delay):
            return False
        # Departure-window searches reuse the same spatial samples at many
        # times. Batch their prefix-sum queries without reducing resolution.
        key = tuple(map(tuple, route))
        if key not in self.route_samples:
            samples = []
            traveled = 0.
            for a, b in zip(route, route[1:]):
                length = math.dist(a, b)
                steps = max(1, math.ceil(length/.10))
                for fraction in np.linspace(0, 1, steps+1)[1:]:
                    samples.append((a[0]+fraction*(b[0]-a[0]),
                                    a[1]+fraction*(b[1]-a[1]),
                                    traveled+fraction*length))
                traveled += length
            if len(self.route_samples) >= 2048:
                self.route_samples.clear()
            self.route_samples[key] = (np.asarray(samples).reshape(-1,3), traveled)
        samples, traveled = self.route_samples[key]
        if len(samples):
            rows = np.floor((samples[:,1]-self.origin[1])/self.resolution).astype(int)
            cols = np.floor((samples[:,0]-self.origin[0])/self.resolution).astype(int)
            earliest = start+samples[:,2]/.22
            latest = start+turn_delay+samples[:,2]/.08
            if (np.any(rows<0) or np.any(cols<0) or
                    np.any(rows>=self.masks.shape[1]) or np.any(cols>=self.masks.shape[2]) or
                    np.any(earliest<self.stamp-.5) or
                    np.any(latest>self.stamp+(len(self.masks)-1)*self.dt)):
                return False
            lo = np.maximum(0,np.floor((earliest-self.stamp)/self.dt).astype(int))
            hi = np.minimum(len(self.masks)-1,np.ceil((latest-self.stamp)/self.dt).astype(int))
            counts = self.prefix[hi,rows,cols]-np.where(lo>0,self.prefix[np.maximum(0,lo-1),rows,cols],0)
            if np.any(counts):
                return False
        earliest = start+traveled/.22
        latest = max(start+turn_delay+traveled/.08+hold,start+team_horizon)
        return self.hold_safe(route[-1], earliest, latest-earliest)

    def departure_delay(self, route, start, hold=15., maximum_wait=None):
        maximum_wait = maximum_wait if maximum_wait is not None else self.payload.get('sun_period_s',240.)
        for delay in np.arange(0,maximum_wait+1,5.):
            if not self.hold_safe(route[0],start,delay+5):
                break
            if self.route_safe(route,start+delay,hold=hold):
                return float(delay)
        return None

    def team_departure(self, routes, positions, start, preferred=None):
        base = math.ceil((start+1)/5)*5
        candidates = ([preferred] if preferred is not None and preferred>=start+1 else [])
        candidates += list(np.arange(base,base+self.payload.get('sun_period_s',240.)+1,5.))
        for departure in dict.fromkeys(candidates):
            if not all(self.hold_safe(p,start,departure-start) for p in positions.values()):
                continue
            horizon = max(5+sum(math.dist(a,b) for a,b in zip(route,route[1:]))/.08+15
                          for route in routes.values())
            if all(self.route_safe(route,departure,hold=15,team_horizon=horizon) for route in routes.values()):
                return departure
        return None

    def departure_windows(self,route,now,base,hold=15.,maximum_wait=None):
        windows = []
        maximum_wait = maximum_wait if maximum_wait is not None else self.payload.get('sun_period_s',240.)
        last_slot = min(48,math.floor((now+maximum_wait-base)/5))
        for slot in range(last_slot+1):
            departure = base+slot*5
            if not self.hold_safe(route[0],now,departure-now+5):
                break
            if self.route_safe(route,departure,hold=hold):
                windows.append(slot)
        return windows

    def departure_constraints(self,route,now,base,hold=15.,maximum_wait=None):
        """Departure slot and bounds on a *shared* team finish slot.

        Each robot must finish its route and stay sunlit until the last robot
        finishes, with a planning reserve. The allocator chooses that common
        finish instead of imposing an arbitrary 90-second stationary wait.
        """
        if not route:
            return []
        length = sum(math.dist(a,b) for a,b in zip(route,route[1:]))
        row = math.floor((route[-1][1]-self.origin[1])/self.resolution)
        col = math.floor((route[-1][0]-self.origin[0])/self.resolution)
        if not (0<=row<self.masks.shape[1] and 0<=col<self.masks.shape[2]):
            return []
        result = []
        for slot in self.departure_windows(route,now,base,hold=hold,maximum_wait=maximum_wait):
            departure = base+slot*5
            arrival = departure+length/.22
            first = max(0,math.floor((arrival-self.stamp)/self.dt))
            shaded = np.flatnonzero(self.masks[first:,row,col])
            last = first+int(shaded[0])-1 if len(shaded) else len(self.masks)-1
            minimum = math.ceil((departure+5+length/.08+hold-base)/5)
            maximum = math.floor((self.stamp+last*self.dt-base)/5)
            if minimum<=maximum:
                result.append((slot,minimum,maximum))
        return result


def encode(masks, stamp, dt=2., origin=(-11., -11.), resolution=.15, **metadata):
    masks = np.asarray(masks, dtype=np.uint8)
    return json.dumps({'source': 'SIMULATOR_TERRAIN_CONVEX_SHADOW_ORACLE',
                       'stamp': stamp, 'dt': dt, 'origin': origin,
                       'resolution': resolution, 'shape': masks.shape,
                       'masks': base64.b64encode(zlib.compress(masks.tobytes())).decode(), **metadata})
