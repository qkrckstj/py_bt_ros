"""Continue other robots' existing tasks while one robot is unavailable."""
import math
from chanseo_exploration.exploration_runtime import TeamBT
from chanseo_exploration.exploration_bt import Status

class HybridTeamBT(TeamBT):
    def pause(self,phase):
        if phase!='WAITING' or self.grid is None or self.now-self.map_time>self.timeout:
            return super().pause(phase)
        available=[]
        for name in self.robots:
            state,at=self.states.get(name,({},-math.inf));p=state.get('position')
            if state.get('ready') and p and self.now-at<=self.timeout:available.append(name)
        if not available:return super().pause(phase)
        for name in self.robots:
            self.command(name,self.active.get(name) if name in available else None)
        self.phase='PARTIAL_WAIT'
        # Allocate no new goals from stale peer inputs. Existing healthy tasks
        # retain their identifiers; global safety/map faults still stop all.
        return Status.RUNNING
