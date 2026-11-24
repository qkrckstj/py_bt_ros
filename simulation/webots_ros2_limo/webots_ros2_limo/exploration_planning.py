"""One background allocation at a time; polling never waits for the solver."""
from concurrent.futures import ThreadPoolExecutor


def recent_visit_exclusions(history, now, duration=180., radius=1.25):
    """Avoid immediately reassigning recently reached endpoints; routes remain open."""
    return [(tuple(task['goal'][:2]), radius) for task in history
            if task.get('result') == 'VISITED' and task.get('purpose') == 'explore'
            and 0 <= now-task['finished_at'] < duration]


class PlanningWorker:
    def __init__(self):
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='lunar_cp_sat')
        self.future, self.context = None, None

    def submit(self, context, compute):
        if self.future is not None:
            return False
        self.context = context
        self.future = self.pool.submit(compute)
        return True

    def poll(self, context):
        if self.future is None or not self.future.done():
            return False, None
        future, old_context = self.future, self.context
        self.future, self.context = None, None
        value = future.result()
        return True, value if old_context == context else None

    def close(self):
        self.pool.shutdown(wait=False, cancel_futures=True)


def observed_reposition_goals(grid, positions, radius=3., spacing=.6):
    """Observed safe intermediate goals when no frontier has a shadow-safe stop."""
    import math
    from .exploration_policy import Goal
    goals = {}
    for xy in positions.values():
        steps = math.floor(radius/spacing)
        for step in range(1, steps+1):
            for angle in range(16):
                yaw = angle*math.tau/16
                point = (xy[0]+step*spacing*math.cos(yaw), xy[1]+step*spacing*math.sin(yaw))
                row, col = grid.index(point)
                if 0 <= row < grid.cells.shape[0] and 0 <= col < grid.cells.shape[1] and grid.safe[row, col]:
                    x, y = grid.world(row, col)
                    goals[row, col] = Goal(float(x), float(y), yaw, 0.)
    return list(goals.values())
