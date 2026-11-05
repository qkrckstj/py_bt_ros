"""Confirm a fresh, spatially consistent sequence, never repeated BT ticks."""
class DetectionConfirmation:
    def __init__(self, frames=5, duration=.6, max_gap=.5, min_iou=.3):
        self.frames, self.duration, self.max_gap, self.min_iou = frames, duration, max_gap, min_iou
        self.reset()

    def reset(self):
        self.count = 0
        self.first = self.last = None
        self.box = None

    def update(self, stamp, box):
        if box is None:
            self.reset()
            return False
        if self.last is not None and stamp <= self.last:
            self.reset()
            return False
        overlap = 0.
        if self.box is not None:
            a, b = self.box, box
            intersection = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(0., min(a[3], b[3])-max(a[1], b[1]))
            union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection
            overlap = intersection / union if union > 0 else 0.
        if self.last is None or stamp-self.last > self.max_gap or overlap < self.min_iou:
            self.count, self.first = 0, stamp
        self.count += 1
        self.last, self.box = stamp, list(box)
        return self.count >= self.frames and stamp-self.first >= self.duration
