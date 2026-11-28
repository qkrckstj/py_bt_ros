"""Check newly entered space of a circular robot against observed map cells."""
import math


def cell_enters_new_disk(cx,cy,res,old_x,old_y,new_x,new_y,radius):
    """Does this square intersect future disk outside the current disk?

    Maximise distance to old centre over square ∩ future disk. Its extrema
    occur at square corners, circle/edge intersections, or the farthest
    point on the future circle. No unknown cell is declared globally free.
    """
    dx,dy=new_x-old_x,new_y-old_y
    distance=math.hypot(dx,dy)
    if distance<1e-12:return False
    half=res/2.;xmin,xmax=cx-half,cx+half;ymin,ymax=cy-half,cy+half
    candidates=[];eps=1e-12
    def add(x,y):
        if xmin-eps<=x<=xmax+eps and ymin-eps<=y<=ymax+eps and (x-new_x)**2+(y-new_y)**2<=radius**2+eps:
            candidates.append((x,y))
    for x in (xmin,xmax):
        for y in (ymin,ymax):add(x,y)
        square=radius**2-(x-new_x)**2
        if square>=-eps:
            root=math.sqrt(max(0.,square));add(x,new_y-root);add(x,new_y+root)
    for y in (ymin,ymax):
        square=radius**2-(y-new_y)**2
        if square>=-eps:
            root=math.sqrt(max(0.,square));add(new_x-root,y);add(new_x+root,y)
    add(new_x+radius*dx/distance,new_y+radius*dy/distance)
    return any((x-old_x)**2+(y-old_y)**2>radius**2+eps for x,y in candidates)

def swept_clear(grid,pose,v,w,radius=.30,details=None):
    if grid is None:return False,'SWEPT_MAP_MISSING'
    if abs(v)<1e-6:
        # A circular footprint does not expand during pure rotation.
        return True,'CIRCULAR_FOOTPRINT_ROTATION'
    if v<0:return False,'REVERSE_NOT_ALLOWED'
    x,y,yaw=pose;res=grid.info.resolution;origin=grid.info.origin
    if abs(origin.orientation.z)>1e-6:return False,'ROTATED_GRID_UNSUPPORTED'
    for step in range(1,11):
        dt=step*.1
        if abs(w)<1e-6:px=x+v*math.cos(yaw)*dt;py=y+v*math.sin(yaw)*dt
        else:px=x+v/w*(math.sin(yaw+w*dt)-math.sin(yaw));py=y-v/w*(math.cos(yaw+w*dt)-math.cos(yaw))
        r0=math.floor((py-radius-origin.position.y)/res);r1=math.floor((py+radius-origin.position.y)/res)
        c0=math.floor((px-radius-origin.position.x)/res);c1=math.floor((px+radius-origin.position.x)/res)
        for r in range(r0,r1+1):
            for c in range(c0,c1+1):
                cx=origin.position.x+(c+.5)*res;cy=origin.position.y+(r+.5)*res
                # Inspect only the portion newly swept by the future footprint.
                if not cell_enters_new_disk(cx,cy,res,x,y,px,py,radius):continue
                if not (0<=r<grid.info.height and 0<=c<grid.info.width):return False,'SWEPT_MAP_OUTSIDE'
                value=grid.data[r*grid.info.width+c]
                if value<0 or value>=25:
                    if details is not None:
                        details.update(cell=[r,c],cell_xy=[cx,cy],cell_value=int(value),
                            pose=list(pose),command=[v,w],horizon_s=dt,
                            predicted_center=[px,py],distance_from_current=math.hypot(cx-x,cy-y),
                            distance_from_predicted=math.hypot(cx-px,cy-py),
                            radius=radius,resolution=res,
                            center_inside_current_body=math.hypot(cx-x,cy-y)<=radius,
                            current_body_fully_covers_cell=math.hypot(cx-x,cy-y)+res/math.sqrt(2)<=radius)
                    return False,'SWEPT_PATH_UNOBSERVED' if value<0 else 'SWEPT_PATH_OCCUPIED' 
    return True,'SWEPT_PATH_OBSERVED_FREE'
