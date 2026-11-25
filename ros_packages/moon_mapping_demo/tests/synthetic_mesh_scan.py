"""원본 삼각형을 라이다 높이에서 자른 합성 광선. 실제 센서 검증은 아니다."""
import math

import numpy as np


def cross(a,b):
    return a[...,0]*b[...,1]-a[...,1]*b[...,0]


def lidar_segments(field,height=.111):
    """현재 PROTO의 지면 위 라이다 높이에서 삼각형과 수평 평면의 교선을 구한다."""
    if hasattr(field,'_synthetic_lidar_segments'):
        return field._synthetic_lidar_segments
    triangles=field.mesh_triangles
    selected=triangles[(triangles[:,:,2].min(axis=1)<=height+1e-12)&
                       (triangles[:,:,2].max(axis=1)>=height-1e-12)]
    segments=[]
    for triangle in selected:
        points=[]
        for a,b in zip(triangle,np.roll(triangle,-1,axis=0)):
            if abs(a[2]-height)<1e-12:points.append(a[:2])
            if (a[2]<height<b[2]) or (b[2]<height<a[2]):
                points.append((a+(b-a)*(height-a[2])/(b[2]-a[2]))[:2])
        unique=[]
        for p in points:
            if all(np.linalg.norm(p-q)>1e-10 for q in unique):unique.append(p)
        if len(unique)==2:segments.append(unique)
        elif len(unique)>2:
            segments.extend(zip(unique,np.roll(np.asarray(unique),-1,axis=0)))
    # 월드 경계 벽도 포함한다. 사각형은 실제 벽 형상이며 지형 근사 기둥은 사용하지 않는다.
    for xmin,ymin,xmax,ymax in field.boundary_boxes:
        corners=np.asarray([(xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax)])
        segments.extend(zip(corners,np.roll(corners,-1,axis=0)))
    result=np.asarray(segments,dtype=float).reshape((-1,2,2))
    field._synthetic_lidar_segments=result
    return result


def simulated_mesh_scan(field,robot,poses,max_range=6.,rays=72):
    x,y,yaw=poses[robot]
    origin=np.asarray([x+.103*math.cos(yaw),y+.103*math.sin(yaw)])
    fov=math.radians(220.)
    angles=yaw-fov/2+np.arange(rays)*fov/(rays-1)
    directions=np.stack((np.cos(angles),np.sin(angles)),axis=1)
    segments=lidar_segments(field)
    selected=segments[np.all(segments.max(axis=1)>=origin-max_range,axis=1)&
                      np.all(segments.min(axis=1)<=origin+max_range,axis=1)]
    edges=selected[:,1]-selected[:,0]
    offsets=selected[:,0]-origin
    denominator=cross(directions[:,None,:],edges[None,:,:])
    with np.errstate(divide='ignore',invalid='ignore'):
        distances=cross(offsets,edges)[None,:]/denominator
        positions=cross(offsets[None,:,:],directions[:,None,:])/denominator
    hits=(np.abs(denominator)>1e-12)&(distances>=0)&(positions>=0)&(positions<=1)
    ranges=np.min(np.where(hits,distances,np.inf),axis=1,initial=np.inf)
    for name,pose in poses.items():
        if name==robot:continue
        offset=np.asarray(pose[:2])-origin
        projected=directions@offset
        discriminant=.26**2-(offset@offset-projected**2)
        hits=np.where((discriminant>=0)&(projected>=0),
            projected-np.sqrt(np.maximum(0,discriminant)),np.inf)
        ranges=np.minimum(ranges,hits)
    ranges=np.where(ranges<=max_range,ranges,np.inf)
    return (*origin,yaw),ranges.tolist(),-fov/2,fov/(rays-1)
