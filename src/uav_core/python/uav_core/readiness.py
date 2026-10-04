"""Pure readiness policy: unknown PX4 evidence never counts as success."""
def evaluate(gates, calibrated, simulated, simulation_transport):
    reasons = [name for name, ok in gates.items() if not ok]
    ready = not reasons
    arm_reasons = []
    if not calibrated:
        arm_reasons.append('extrinsic/world alignment unverified')
    if simulated and not simulation_transport:
        arm_reasons.append('mock source forbidden on hardware transport')
    return ready, ready and not arm_reasons, reasons+arm_reasons
