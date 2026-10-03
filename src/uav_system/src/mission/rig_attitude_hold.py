"""Fixed-position roll/pitch rig: level attitude and a bounded thrust envelope."""
import math
from mission_executor.execution import AttitudeCommand, TaskUpdate
from uav_core.validation import positive, attitude_target


class RigAttitudeHold:
    command_kind = 'attitude'

    def __init__(self, config):
        positive(config, ('thrust', 'ramp_up_seconds', 'hold_seconds', 'ramp_down_seconds'))
        attitude_target((0,0,0,1),config['thrust'])
        self.peak_thrust = config['thrust']
        self.up = config['ramp_up_seconds']
        self.hold = config['hold_seconds']
        self.down = config['ramp_down_seconds']

    def start(self, now, origin):
        self.since = now
        yaw = origin[3]
        self.orientation = (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))
        return TaskUpdate(AttitudeCommand(self.orientation,0.0),'RIG_RAMP_UP')

    def step(self, now, local):
        elapsed = max(0.0, now-self.since)
        if elapsed < self.up:
            thrust = self.peak_thrust*elapsed/self.up
            phase = 'RIG_RAMP_UP'
        elif elapsed < self.up+self.hold:
            thrust = self.peak_thrust
            phase = 'RIG_HOLD'
        else:
            thrust = self.peak_thrust*max(0.0,1.0-(elapsed-self.up-self.hold)/self.down)
            phase = 'RIG_RAMP_DOWN'
        done = elapsed >= self.up+self.hold+self.down
        return TaskUpdate(AttitudeCommand(self.orientation,thrust),phase,done)


def create_task(config):
    return RigAttitudeHold(config)
