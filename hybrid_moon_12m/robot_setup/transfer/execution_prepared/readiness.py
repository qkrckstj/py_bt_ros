"""Separate task continuity from permission to emit motion."""
import time

class StableNavigation:
    def __init__(self,navigation,grace=1.,clock=time.monotonic):
        self.navigation=navigation;self.grace=grace;self.clock=clock
        self.bad_since=None;self.ever_ready=False;self.last_raw=False
    def __getattr__(self,key):return getattr(self.navigation,key)
    @property
    def pose_ready(self):return self.navigation.pose_ready
    @pose_ready.setter
    def pose_ready(self,value):self.navigation.pose_ready=value
    @property
    def local_ready(self):return self.navigation.local_ready
    @local_ready.setter
    def local_ready(self,value):self.navigation.local_ready=value
    def motion_ready(self):
        now=self.clock();self.last_raw=bool(self.navigation.ready())
        if self.last_raw:self.ever_ready=True;self.bad_since=None
        elif self.bad_since is None:self.bad_since=now
        return self.last_raw
    def ready(self):
        if self.motion_ready():return True
        return (self.ever_ready and self.navigation.poll() in ('SENDING','NAVIGATING')
                and self.clock()-self.bad_since<self.grace)
    def diagnostic(self):
        nav=self.navigation;reason='READY' if self.last_raw else 'NAV_UNAVAILABLE'
        if hasattr(nav,'link'):
            if not nav.local_ready:reason='LOCAL_TF_UNAVAILABLE'
            elif not nav.link.fresh():reason='PHYSICAL_LINK_STALE'
            elif not nav.link.status.get('ready'):
                remote=nav.link.status
                reason='PHYSICAL_'+('POSE_UNAVAILABLE' if not remote.get('pose_ready',True) else 'ACTION_SERVER_UNAVAILABLE' if not remote.get('action_server_ready',True) else 'NOT_READY')
        elif not getattr(nav,'pose_ready',False):reason='LOCAL_TF_UNAVAILABLE'
        elif not nav.client.server_is_ready():reason='ACTION_SERVER_UNAVAILABLE'
        return {'reason':reason,'motion_ready':self.last_raw,'retaining_goal':bool(not self.last_raw and self.ever_ready and self.bad_since is not None and self.clock()-self.bad_since<self.grace),
                'unready_seconds':0. if self.bad_since is None else self.clock()-self.bad_since,
                'pose_ready':getattr(self.navigation,'pose_ready',getattr(self.navigation,'local_ready',False))}

class HoldWindow:
    def __init__(self,seconds=1.):self.seconds=seconds;self.since=None
    def waiting(self,now):
        if self.since is None:self.since=now
        return now-self.since<self.seconds
    def reset(self):self.since=None
