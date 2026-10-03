"""Shared numeric configuration validation."""
import math

def positive(config, names):
    for name in names:
        value = config[name]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('invalid ' + name)


def attitude_target(orientation, thrust, max_thrust=1.0):
    """Validate a unit ROS xyzw quaternion and normalized nonnegative thrust."""
    from numbers import Real
    if len(orientation) != 4 or any(not isinstance(v, Real) or isinstance(v, bool)
                                  or not math.isfinite(v) for v in orientation):
        raise ValueError('invalid attitude quaternion')
    if abs(sum(v * v for v in orientation) - 1.0) > 1e-3:
        raise ValueError('attitude quaternion must be normalized')
    if not isinstance(thrust, Real) or isinstance(thrust, bool) or not math.isfinite(thrust) or not 0 <= thrust <= max_thrust:
        raise ValueError('attitude thrust outside configured limit')
