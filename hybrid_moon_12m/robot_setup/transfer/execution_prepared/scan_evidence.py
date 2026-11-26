"""Short-lived per-ray evidence for missing (zero) lidar returns.
Unknown directions without recent measured evidence remain blocked.
"""
import math

class ScanEvidence:
    def __init__(self,max_age=.25):
        self.max_age=max_age;self.geometry=None;self.hits={}
    def update(self,scan,pose,now):
        geometry=(len(scan.ranges),float(scan.angle_min),float(scan.angle_increment),float(scan.range_min),float(scan.range_max))
        if geometry!=self.geometry:self.hits={};self.geometry=geometry
        if pose is None:return
        for i,value in enumerate(scan.ranges):
            d=float(value)
            if math.isfinite(d) and scan.range_min<=d<=scan.range_max and d>0:
                self.hits[i]=(d,tuple(pose),now)
    def check(self,scan,pose,now,v,w,radius):
        if v<0:return False,'REVERSE_NOT_ALLOWED'
        if abs(v)<1e-6 and abs(w)<1e-6:return True,'ZERO_COMMAND'
        if pose is None:return False,'LIDAR_POSE_MISSING'
        sector=list(range(len(scan.ranges))) if abs(w)>1e-6 else [i for i in range(len(scan.ranges)) if abs(scan.angle_min+i*scan.angle_increment)<math.radians(40)]
        if not sector:return False,'LIDAR_EMPTY_SECTOR'
        limit=radius+(.04 if abs(w)>1e-6 else .08);reused=0
        for i in sector:
            d=float(scan.ranges[i]);penalty=0.
            if d==0:
                hit=self.hits.get(i)
                if hit is None:return False,'LIDAR_UNOBSERVED_RAY'
                d,old,at=hit;age=now-at
                shift=math.dist(pose[:2],old[:2]);turn=abs(math.atan2(math.sin(pose[2]-old[2]),math.cos(pose[2]-old[2])))
                if age<0 or age>self.max_age or shift>.01 or turn>.03:return False,'LIDAR_EVIDENCE_EXPIRED'
                # Reduce clearance by pose change and an additional age margin.
                penalty=shift+(d+.103)*turn+.1*age;reused+=1
            elif not math.isfinite(d) or not scan.range_min<=d<=scan.range_max:
                return False,'LIDAR_INVALID_NONZERO'
            angle=scan.angle_min+i*scan.angle_increment
            clearance=math.hypot(.103+d*math.cos(angle),d*math.sin(angle))-penalty
            if clearance<limit:return False,'LIDAR_OBSTACLE_CLOSE'
        return True,'LIDAR_OK_RECENT_EVIDENCE' if reused else 'LIDAR_OK_CURRENT'
