"""Spatial ownership for a hybrid 2-D map; no ROS or motion commands."""
import math
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class Layout:
    bounds: tuple = (-6., -6., 6., 6.)
    real_bounds: tuple = (-6., -6., -4., -4.)
    resolution: float = .05
    real_offset: tuple = (-5., -5., -1.5707963267948966)

    def __post_init__(self):
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('Invalid resolution')
        for box in (self.bounds,self.real_bounds):
            if len(box)!=4 or not all(math.isfinite(v) for v in box) or box[0]>=box[2] or box[1]>=box[3]:
                raise ValueError('Invalid bounds')
        if not (self.bounds[0]<=self.real_bounds[0]<self.real_bounds[2]<=self.bounds[2] and self.bounds[1]<=self.real_bounds[1]<self.real_bounds[3]<=self.bounds[3]):
            raise ValueError('Real region outside map')

    @property
    def shape(self):
        return (round((self.bounds[3]-self.bounds[1])/self.resolution),round((self.bounds[2]-self.bounds[0])/self.resolution))

    def real_mask(self):
        row,col=np.indices(self.shape)
        x=self.bounds[0]+(col+.5)*self.resolution
        y=self.bounds[1]+(row+.5)*self.resolution
        a,b,c,d=self.real_bounds
        return (x>=a)&(x<c)&(y>=b)&(y<d)

    def goal_allowed(self, robot, xy, footprint=.25):
        x,y=xy[:2]
        if not all(math.isfinite(v) for v in (x,y,footprint)) or footprint<0:return False
        a,b,c,d=self.bounds
        if not (a+footprint<=x<=c-footprint and b+footprint<=y<=d-footprint):return False
        a,b,c,d=self.real_bounds
        if robot=='robot1':
            return a+footprint<=x<=c-footprint and b+footprint<=y<=d-footprint
        # Conservative expanded box prevents virtual footprints crossing into the real region.
        return not (a-footprint<=x<=c+footprint and b-footprint<=y<=d+footprint)

    def path_allowed(self,robot,points,footprint=.25):
        """Sample every half map cell, including segment interiors."""
        if not points:return False
        if not self.goal_allowed(robot,points[0],footprint):return False
        for p,q in zip(points,points[1:]):
            steps=max(1,math.ceil(math.dist(p[:2],q[:2])/(self.resolution*.5)))
            for f in np.linspace(0.,1.,steps+1):
                if not self.goal_allowed(robot,(p[0]+f*(q[0]-p[0]),p[1]+f*(q[1]-p[1])),footprint):return False
        return True


def project_grid(cells, resolution, origin, layout, offset=(0.,0.,0.)):
    """Inverse nearest-cell sampling handles rotated input origins and offsets."""
    cells=np.asarray(cells,dtype=np.int16)
    if cells.ndim!=2 or cells.size==0 or resolution<=0 or not math.isfinite(resolution):raise ValueError('Invalid grid')
    if not np.all((cells>=-1)&(cells<=100)):raise ValueError('Invalid occupancy values')
    if not all(math.isfinite(v) for v in (*origin,*offset)):raise ValueError('Invalid transform')
    rows,cols=np.indices(layout.shape)
    x=layout.bounds[0]+(cols+.5)*layout.resolution
    y=layout.bounds[1]+(rows+.5)*layout.resolution
    dx,dy=x-offset[0],y-offset[1];c,s=math.cos(offset[2]),math.sin(offset[2])
    u,v=c*dx+s*dy,-s*dx+c*dy
    dx,dy=u-origin[0],v-origin[1];c,s=math.cos(origin[2]),math.sin(origin[2])
    ix=np.floor((c*dx+s*dy)/resolution).astype(int)
    iy=np.floor((-s*dx+c*dy)/resolution).astype(int)
    valid=(ix>=0)&(ix<cells.shape[1])&(iy>=0)&(iy<cells.shape[0])
    result=np.full(layout.shape,-1,dtype=np.int16)
    result[valid]=cells[iy[valid],ix[valid]]
    return result


def compose(layout, real_grid, virtual_grids):
    """Never use a virtual observation as a fallback inside the real-owned area.

    Full source snapshots replace old source snapshots; no repeated evidence
    accumulation. Virtual conflicts are conservatively obstacle-preferred.
    This is ownership fusion, not probabilistic SLAM alignment.
    """
    output=np.full(layout.shape,-1,dtype=np.int16)
    mask=layout.real_mask()
    if real_grid is not None:
        real=np.asarray(real_grid)
        if real.shape!=layout.shape:raise ValueError('Invalid real shape')
        output[mask]=real[mask]
    for grid in virtual_grids:
        grid=np.asarray(grid)
        if grid.shape!=layout.shape:raise ValueError('Invalid virtual shape')
        valid=(~mask)&(grid>=0)
        output[valid]=np.maximum(output[valid],grid[valid])
    return output

def clear_virtual_bodies(cells,layout,positions,radius=.30):
    """Remove current robot bodies from the terrain map, preserving real ownership."""
    out=cells.copy();rows,cols=np.indices(layout.shape)
    x=layout.bounds[0]+(cols+.5)*layout.resolution
    y=layout.bounds[1]+(rows+.5)*layout.resolution
    virtual=~layout.real_mask()
    for pose in positions:
        if len(pose)<2 or not all(math.isfinite(v) for v in pose[:2]):continue
        out[virtual & ((x-pose[0])**2+(y-pose[1])**2<=radius**2)]=0
    return out
