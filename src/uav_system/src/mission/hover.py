"""Hold the confirmed starting position for a bounded duration, then land."""
from mission_executor.execution import TaskUpdate
from uav_core.validation import positive


class Hover:
    def __init__(self, config):
        positive(config, ('hover_seconds',))
        self.duration = config['hover_seconds']

    def start(self, now, origin):
        self.origin = tuple(origin)
        self.since = now
        return TaskUpdate(self.origin, 'HOVER')

    def step(self, now, local):
        return TaskUpdate(self.origin, 'HOVER', now - self.since >= self.duration)


def create_task(config):
    return Hover(config)
