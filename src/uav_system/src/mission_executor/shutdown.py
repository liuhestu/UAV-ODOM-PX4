"""Shared normal thrust shutdown; emergency stops bypass this profile."""
from uav_core.validation import positive


class ThrustStop:
    def __init__(self, ramp_seconds, zero_seconds):
        positive({'ramp':ramp_seconds,'zero':zero_seconds},('ramp','zero'))
        self.ramp_seconds=ramp_seconds
        self.zero_seconds=zero_seconds

    def start(self, now, thrust):
        self.since=now
        self.initial_thrust=thrust

    def step(self, now):
        elapsed=max(0.0,now-self.since)
        if elapsed<self.ramp_seconds:
            fraction=elapsed/self.ramp_seconds
            thrust=self.initial_thrust*(1.0-3.0*fraction**2+2.0*fraction**3)
            return thrust,'RAMP_DOWN',False
        return 0.0,'ZERO_THRUST',elapsed>=self.ramp_seconds+self.zero_seconds
