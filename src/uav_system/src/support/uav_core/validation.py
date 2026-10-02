"""Shared numeric configuration validation."""
import math

def positive(config, names):
    for name in names:
        value = config[name]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('invalid ' + name)
