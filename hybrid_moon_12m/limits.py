"""Pure safety/coordinate functions shared by laptop and physical robot."""
import math

def local_pose(pose, anchor):
    dx,dy=pose[0]-anchor[0],pose[1]-anchor[1];c,s=math.cos(anchor[2]),math.sin(anchor[2])
    return (c*dx+s*dy,-s*dx+c*dy,math.atan2(math.sin(pose[2]-anchor[2]),math.cos(pose[2]-anchor[2])))

def map_pose(local, anchor):
    c,s=math.cos(anchor[2]),math.sin(anchor[2])
    return (anchor[0]+c*local[0]-s*local[1],anchor[1]+s*local[0]+c*local[1],anchor[2]+local[2])

def observation_offset(display, anchor):
    theta=display[2]-anchor[2];c,s=math.cos(theta),math.sin(theta)
    return (display[0]-c*anchor[0]+s*anchor[1],display[1]-s*anchor[0]-c*anchor[1],theta)

class PhysicalLimits:
    def __init__(self,bounds=(-.5,-.5,.5,.5),radius=.30,reserve=.04,max_linear=.05,max_angular=.2):
        self.bounds,self.radius,self.reserve=bounds,radius,reserve
        self.max_linear,self.max_angular=max_linear,max_angular
    def pose_safe(self,p,margin=0.):
        if len(p)<2 or not all(math.isfinite(v) for v in p):return False
        a,b,c,d=self.bounds;r=self.radius+margin
        return a+r<=p[0]<=c-r and b+r<=p[1]<=d-r
    def path_safe(self,path):
        if not path or len(path)>2000:return False
        if not all(self.pose_safe(p,self.reserve) for p in path):return False
        # A rectangle with an inset is convex: straight segments between valid
        # vertices remain inside it; the physical gate also checks Nav2's path.
        return True
    def velocity_safe(self,pose,v,w):
        if not all(math.isfinite(x) for x in (*pose,v,w)):return False
        if abs(v)>self.max_linear+1e-6 or abs(w)>self.max_angular+1e-6:return False
        if not self.pose_safe(pose,self.reserve):return False
        # 1 s envelope includes stream watchdog, braking, and localization margin.
        for dt in (.2,.5,1.):
            if abs(w)<1e-5:x=pose[0]+v*math.cos(pose[2])*dt;y=pose[1]+v*math.sin(pose[2])*dt
            else:x=pose[0]+v/w*(math.sin(pose[2]+w*dt)-math.sin(pose[2]));y=pose[1]-v/w*(math.cos(pose[2]+w*dt)-math.cos(pose[2]))
            if not self.pose_safe((x,y),self.reserve):return False
        return True

def display_to_reference(pose, display_offset, reference_anchor):
    """Inverse of display alignment, then express goal in the physical anchor."""
    return local_pose(local_pose(pose,display_offset),reference_anchor)

def screen_down_heading(rotation,camera_position,robot_position):
    """Ground heading projecting vertically down in an ENU Webots Viewpoint.

    With zero Viewpoint rotation Webots looks along +X, with +Z up and -Y
    right. Perspective depth must be included for off-center robots.
    """
    axis=rotation[:3];angle=rotation[3];norm=math.sqrt(sum(v*v for v in axis))
    if norm==0:axis=(0.,0.,1.)
    else:axis=tuple(v/norm for v in axis)
    def dot(a,b):return sum(x*y for x,y in zip(a,b))
    def rotate(v):
        a,b,c=axis;x,y,z=v;cross=(b*z-c*y,c*x-a*z,a*y-b*x)
        return tuple(v[i]*math.cos(angle)+cross[i]*math.sin(angle)+axis[i]*dot(axis,v)*(1-math.cos(angle)) for i in range(3))
    right,up,forward=rotate((0.,-1.,0.)),rotate((0.,0.,1.)),rotate((1.,0.,0.))
    relative=tuple(p-c for p,c in zip(robot_position,camera_position))
    depth,x,y=dot(relative,forward),dot(relative,right),dot(relative,up)
    if depth<=0:raise ValueError('Robot is behind the camera')
    wx,wy=(right[i]*depth-x*forward[i] for i in (0,1))
    d=(wy,-wx,0.)
    if math.hypot(d[0],d[1])<1e-9:raise ValueError('Camera cannot define a ground heading')
    if dot(d,up)*depth-y*dot(d,forward)>0:d=tuple(-v for v in d)
    return math.atan2(d[1],d[0])
