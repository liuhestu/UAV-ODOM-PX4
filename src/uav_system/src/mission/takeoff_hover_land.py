"""Ramp to a relative height, hold, then request shared landing."""
from mission_executor.execution import TaskUpdate
from uav_core.validation import positive


class TakeoffHoverLand:
    def __init__(self, config):
        positive(config, ('height', 'climb_rate', 'height_tolerance',
                          'hover_seconds', 'flight_timeout'))
        self.cfg = config

    def start(self, now, origin):
        self.origin = tuple(origin)
        self.since = now
        self.phase = 'TAKEOFF'
        return TaskUpdate(self.origin, self.phase)

    def step(self, now, local):
        x, y, z, yaw = self.origin
        height = z + self.cfg['height']
        target_z = min(height, z + self.cfg['climb_rate'] * (now - self.since)) if self.phase == 'TAKEOFF' else height
        if self.phase == 'TAKEOFF':
            if abs(local[2] - height) < self.cfg['height_tolerance'] and target_z >= height:
                self.phase = 'HOVER'
                self.since = now
            elif now - self.since > self.cfg['flight_timeout']:
                raise ValueError('takeoff timeout')
        done = self.phase == 'HOVER' and now - self.since >= self.cfg['hover_seconds']
        return TaskUpdate((x, y, target_z, yaw), self.phase, done)


def create_task(config):
    return TakeoffHoverLand(config)
