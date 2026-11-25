"""원본 삼각형과 원판 몸체의 충돌. 사각형은 후보 검색에만 사용한다."""
from collections import OrderedDict
import math

import numpy as np
from scipy.spatial import cKDTree


def cross(a, b):
    return a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]


def point_segment_squared(p, a, b):
    edge=b-a
    length=np.sum(edge*edge,axis=-1)
    numerator=np.sum((p-a)*edge,axis=-1)
    ratio=np.divide(numerator,length,out=np.zeros_like(numerator,dtype=float),where=length>1e-24)
    nearest=a+np.clip(ratio,0.,1.)[...,None]*edge
    return np.sum((p-nearest)**2,axis=-1)


def capsule_hits_triangles(a, b, triangles, radius):
    """선분과 삼각형의 정확한 최소 거리. 반경은 스칼라 또는 삼각형별 배열."""
    if not len(triangles):return False
    a,b=np.asarray(a),np.asarray(b)
    edges=np.roll(triangles,-1,axis=1)
    area=cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    side_a=cross(edges-triangles,a-triangles)
    inside=(np.all(side_a>=-1e-12,axis=1)|np.all(side_a<=1e-12,axis=1)) & (np.abs(area)>1e-18)
    if np.any(inside):return True
    if np.array_equal(a,b):
        distance=point_segment_squared(a,triangles,edges).min(axis=1)
    else:
        side_b=cross(edges-triangles,b-triangles)
        inside=(np.all(side_b>=-1e-12,axis=1)|np.all(side_b<=1e-12,axis=1)) & (np.abs(area)>1e-18)
        if np.any(inside):return True
        own_a=cross(b-a,triangles-a)
        own_b=cross(b-a,edges-a)
        overlap=(np.all(np.maximum(np.minimum(triangles,edges),np.minimum(a,b)) <=
                        np.minimum(np.maximum(triangles,edges),np.maximum(a,b))+1e-12,axis=2))
        if np.any((side_a*side_b<=0.) & (own_a*own_b<=0.) & overlap):return True
        distance=np.minimum(point_segment_squared(a,triangles,edges).min(axis=1),
                            point_segment_squared(b,triangles,edges).min(axis=1))
        distance=np.minimum(distance,point_segment_squared(triangles,a,b).min(axis=1))
    return bool(np.any(distance<=np.asarray(radius)**2+1e-12))


def clip_above(triangles, height):
    """그림자 수신 평면보다 위인 삼각형 부분만 남긴다. 경계 면은 삼각 분할한다."""
    triangles=np.asarray(triangles)
    high=triangles[:,:,2]>height
    pieces=[triangles[np.all(high,axis=1)]]
    for triangle in triangles[np.any(high,axis=1)&~np.all(high,axis=1)]:
        polygon=[]
        for a,b in zip(triangle,np.roll(triangle,-1,axis=0)):
            if a[2]>height:polygon.append(a)
            if (a[2]>height)!=(b[2]>height):
                polygon.append(a+(b-a)*(height-a[2])/(b[2]-a[2]))
        pieces.extend(np.asarray([[polygon[0],polygon[i],polygon[i+1]]])
                      for i in range(1,len(polygon)-1))
    return np.concatenate(pieces) if pieces else np.empty((0,3,3))


class TriangleIndex:
    def __init__(self,triangles):
        self.triangles=triangles
        centers=triangles.mean(axis=1)
        self.low=np.minimum(np.minimum(triangles[:,0],triangles[:,1]),triangles[:,2])
        self.high=np.maximum(np.maximum(triangles[:,0],triangles[:,1]),triangles[:,2])
        self.extent=float(np.linalg.norm(triangles-centers[:,None,:],axis=2).max(initial=0.))
        self.tree=cKDTree(centers)
        self.checks=OrderedDict()

    def candidates(self,a,b,radius):
        a,b=np.asarray(a),np.asarray(b)
        return self.tree.query_ball_point((a+b)/2.,np.linalg.norm(b-a)/2.+radius+self.extent+1e-10)

    def blocked(self,a,b,radius):
        key=(tuple(a),tuple(b),radius)
        cached=self.checks.get(key)
        if cached is not None:return cached
        ids=self.candidates(a,b,radius)
        result=capsule_hits_triangles(a,b,self.triangles[ids],radius)
        self.checks[key]=result
        if len(self.checks)>32768:self.checks.popitem(last=False)
        return result


class MeshShadow:
    """평면 투영 그림자와 같은 시간의 몸체를 검사한다.

    각 짧은 구간의 중간 태양각에서 삼각형을 투영한다. 그 구간의 각 꼭짓점
    이동량 상한만 원형으로 더하고, 모호한 충돌은 시간을 나누어 재검사한다.
    따라서 긴 이동·대기 전체의 그림자를 한 사각형으로 합치지 않는다.
    """
    def __init__(self,triangles,elevation,initial,period,step,plane_height=.02):
        self.triangles=clip_above(triangles,plane_height)
        self.xy=self.triangles[:,:,:2]
        self.reach=(self.triangles[:,:,2]-plane_height)/math.tan(math.radians(elevation))
        self.max_reach=self.reach.max(axis=1,initial=0.)
        self.initial,self.period,self.step=math.radians(initial),period,step
        self.frames=OrderedDict()
        self.checks=OrderedDict()
        self.regions=OrderedDict()

    def projected(self,t,ids=None):
        angle=self.initial+2.*math.pi*(t%self.period)/self.period
        xy=self.xy if ids is None else self.xy[ids]
        reach=self.reach if ids is None else self.reach[ids]
        return xy-reach[:,:,None]*np.array([math.sin(angle),math.cos(angle)])

    def candidates(self,a,b,ta,tb,radius):
        middle=(ta+tb)/2.
        bucket=math.floor((middle%self.period)/8.)
        phase=min((bucket+.5)*8.,self.period)
        index=self.frames.get(bucket)
        if index is None:
            index=TriangleIndex(self.projected(phase))
            self.frames[bucket]=index
            if len(self.frames)>32:self.frames.popitem(last=False)
        closest=middle+math.remainder(phase-middle,self.period)
        delta=max(abs(ta-closest),abs(tb-closest))
        # 같은 공간·시간 타일의 넓은 후보 목록만 공유한다. 정확한 전이는
        # 아래 삼각형 검사에서 따로 판정하므로 경로/충돌 결과를 섞지 않는다.
        low=np.floor(np.minimum(a,b));high=np.floor(np.maximum(a,b))+1.
        width=math.ceil(delta/4.)*4.
        key=(bucket,width,*low,*high,radius)
        cached=self.regions.get(key)
        if cached is not None:return cached
        factor=2.*math.sin(min(math.pi/2.,math.pi*width/self.period))
        ids=np.asarray(index.candidates(low,high,radius+float(self.max_reach.max(initial=0.))*factor),dtype=int)
        # 큰 언덕의 최대 이동량을 낮은 모든 삼각형에 적용하지 않는다.
        # 캐시한 사각형은 후보 제외에만 쓰고 최종 판정은 원래 삼각형이다.
        motion=self.max_reach[ids]*factor
        low=low-radius;high=high+radius
        selected=(index.high[ids,0]+motion>=low[0])&(index.high[ids,1]+motion>=low[1])&\
                 (index.low[ids,0]-motion<=high[0])&(index.low[ids,1]-motion<=high[1])
        ids=ids[selected]
        self.regions[key]=ids
        if len(self.regions)>8192:self.regions.popitem(last=False)
        return ids

    def blocked(self,a,b,ta,tb,radius):
        if not len(self.triangles):return False
        key=(tuple(a),tuple(b),ta,tb,radius)
        cached=self.checks.get(key)
        if cached is not None:return cached
        if tuple(a)==tuple(b) and tb-ta>2.:
            # 겹치는 긴 관측 창을 매번 넓게 감싸지 않는다. 절대 2초 구간은
            # 재사용하되 양 끝의 부분 구간도 같은 연속 검사로 확인한다.
            times=[ta,*np.arange((math.floor(ta/2.)+1)*2.,tb,2.).tolist(),tb]
            result=any(self.blocked(a,b,lo,hi,radius) for lo,hi in zip(times,times[1:]))
        else:
            result=self._interval(np.asarray(a),np.asarray(b),ta,tb,radius,
                                  self.candidates(a,b,ta,tb,radius))
        self.checks[key]=result
        if len(self.checks)>32768:self.checks.popitem(last=False)
        return result

    def _interval(self,a,b,ta,tb,radius,ids):
        if not len(ids):return False
        middle=(ta+tb)/2.
        triangles=self.projected(middle,ids)
        motion=2.*self.max_reach[ids]*math.sin(min(math.pi/2.,math.pi*(tb-ta)/(2.*self.period)))
        low=np.minimum(a,b)-radius
        high=np.maximum(a,b)+radius
        tri_high=np.maximum(np.maximum(triangles[:,0],triangles[:,1]),triangles[:,2])
        tri_low=np.minimum(np.minimum(triangles[:,0],triangles[:,1]),triangles[:,2])
        selected=(tri_high[:,0]+motion>=low[0])&(tri_high[:,1]+motion>=low[1])&\
                 (tri_low[:,0]-motion<=high[0])&(tri_low[:,1]-motion<=high[1])
        ids,triangles,motion=ids[selected],triangles[selected],motion[selected]
        if not capsule_hits_triangles(a,b,triangles,radius+motion):return False
        center=(a+b)/2.
        if tb-ta<=self.step or capsule_hits_triangles(center,center,triangles,radius):return True
        return (self._interval(a,center,ta,middle,radius,ids) or
                self._interval(center,b,middle,tb,radius,ids))
