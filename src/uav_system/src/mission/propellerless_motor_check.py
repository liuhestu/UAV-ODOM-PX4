"""Propellers removed: real-feedback mode/ARM/stream/landing link test."""
from mission_executor.execution import TaskUpdate
from uav_core.validation import positive


class PropellerlessMotorCheck:
    def __init__(self, config):
        positive(config, ('duration_seconds',))
        self.duration = config['duration_seconds']

    def start(self, now, origin):
        self.origin = tuple(origin)
        self.since = now
        return TaskUpdate(self.origin,'LINK_TEST')

    def step(self, now, local):
        return TaskUpdate(self.origin,'LINK_TEST',now-self.since >= self.duration)


def create_task(config):
    return PropellerlessMotorCheck(config)
