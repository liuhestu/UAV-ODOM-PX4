"""Pure readiness policy: required pose/fusion evidence must be observed."""

DIAGNOSTIC_GATES = frozenset({
    'PX4 critical/unknown status',
    'PX4 sensor health unavailable/failed',
})


class HealthReporter:
    """Report the initial check snapshot and every change, without tick spam."""

    def __init__(self):
        self.previous = None

    def update(self, checks, ready, arm_ready):
        snapshot = (tuple((label, bool(passed)) for label, passed in checks),
                    bool(ready), bool(arm_ready))
        if snapshot == self.previous:
            return None
        self.previous = snapshot
        lines = ['Health checks: ready=%s arm_ready=%s' % (ready, arm_ready)]
        lines.extend('  [%s] %s' % ('PASS' if passed else 'FAIL', label)
                     for label, passed in snapshot[0])
        return '\n'.join(lines)


def estimator_valid(ekf, grounded_ev):
    """Allow the legacy at-rest bit only with real grounded EV fusion proof."""
    return bool(ekf and all(getattr(ekf, name, False) for name in (
        'attitude_status_flag', 'pos_horiz_rel_status_flag', 'pos_vert_abs_status_flag',
        'velocity_horiz_status_flag', 'velocity_vert_status_flag')) and
        not ekf.accel_error_status_flag and
        (not ekf.const_pos_mode_status_flag or grounded_ev))


def evaluate(gates, calibrated, simulated, simulation_transport):
    reasons = [name for name, ok in gates.items() if not ok and name not in DIAGNOSTIC_GATES]
    ready = not reasons
    arm_reasons = []
    if not calibrated:
        arm_reasons.append('extrinsic/world alignment unverified')
    if simulated and not simulation_transport:
        arm_reasons.append('mock source forbidden on hardware transport')
    return ready, ready and not arm_reasons, reasons+arm_reasons
