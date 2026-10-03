"""ROS-independent health policy and PX4 fusion evidence checks."""
import re

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


TOPICS = ('vehicle_visual_odometry', 'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_hgt', 'vehicle_local_position', 'estimator_status_flags')
RESETS = ('xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter')
SINGLE_EKF_PARAMETERS = {'EKF2_MULTI_IMU': 0, 'SENS_IMU_MODE': 1, 'SENS_MAG_MODE': 1}


def advance_reset_counter(previous, current, received, fused):
    """Establish a new system's baseline only after valid EV fusion.

    Once established, any observed change is a fault, including while fusion
    is unavailable. A failed initial observation may contain retained PX4 data.
    """
    if previous is None and not (received and fused):
        return None, False
    return current, previous is not None and current != previous


def verify_single_ekf_parameters(parameters):
    """Require single EKF and sensor voting; never infer missing parameters.

    SENS_IMU_MODE=1 selects PX4's non-multi EKF startup path. Together with
    SENS_MAG_MODE=1 this does not depend on EKF2_MULTI_MAG, which firmware
    builds without EKF magnetometer support do not expose.
    """
    for name, expected in SINGLE_EKF_PARAMETERS.items():
        value = parameters.get(name)
        if type(value) is not int or value != expected:
            raise ValueError('observer requires single EKF: '+name+' == '+str(expected))


def parse_listener(topic, text):
    text = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', text).replace('\r','')
    # Require topic output, not just echoed command text.
    marker = re.search(r'(?:TOPIC:\s*|^)' + re.escape(topic) + r'(?:\s|$)', text, re.MULTILINE)
    if not marker: raise ValueError('missing topic output: '+topic)
    result={}
    for name, val in re.findall(r'^\s*([a-z_]+):\s*([-+0-9.eE]+|True|False|true|false)\b', text[marker.end():], re.MULTILINE):
        if name in result: raise ValueError('ambiguous repeated field: '+name)
        boolean_fields={'fused','innovation_rejected','xy_valid','z_valid','v_xy_valid','v_z_valid'}
        if name in boolean_fields or val.lower() in ('true','false'):
            if val.lower() not in ('true','false','0','1'):
                raise ValueError('invalid boolean: '+name)
            result[name]=val.lower() in ('true','1')
        else:
            result[name] = float(val)
    return result


def inspect_listener(samples, max_age):
    """Evaluate the existing gates and identify each failed field."""
    vo,pos,hgt,local,flags = [samples[t] for t in TOPICS]
    if local['timestamp']<=0: raise ValueError('invalid PX4 clock')
    # Listener topics are queried sequentially and publish asynchronously.
    # A later topic can legitimately be newer than vehicle_local_position.
    # Use the newest publication in this observation as the shared clock;
    # check local age too, so an old local sample cannot hide stale evidence.
    now=max(s['timestamp'] for s in (vo,pos,hgt,local,flags))
    def fresh(t): return 0<=now-t<=max_age*1e6
    failures=[]
    def check(label, passed, value):
        if not passed:
            failures.append('%s=%s' % (label, value))
        return passed
    received=all([check('vehicle_visual_odometry.'+k, fresh(vo[k]), vo[k])
                  for k in ('timestamp','timestamp_sample')])
    fusion_checks=[]
    for topic,a in zip(TOPICS[1:3],(pos,hgt)):
        for k,passed in [('fused',a['fused'] is True),
                         ('innovation_rejected',a['innovation_rejected'] is False),
                         ('estimator_instance',a['estimator_instance']==0),
                         ('timestamp',fresh(a['timestamp'])),
                         ('time_last_fuse',fresh(a['time_last_fuse']))]:
            fusion_checks.append(check(topic+'.'+k,passed,a[k]))
    valid_checks=[check('vehicle_local_position.timestamp',fresh(local['timestamp']),local['timestamp'])]
    valid=all(valid_checks+[check('vehicle_local_position.'+k,local[k] is True,local[k])
               for k in ('xy_valid','z_valid','v_xy_valid','v_z_valid')])
    # Actual control state distinguishes external aiding from the legacy
    # CONST_POS_MODE bit, which PX4 also sets when the vehicle is at rest.
    aiding_checks=[check('estimator_status_flags.timestamp',fresh(flags['timestamp']),flags['timestamp'])]
    for k in ('cs_ev_pos','cs_ev_hgt','cs_fake_pos','cs_valid_fake_pos','cs_inertial_dead_reckoning'):
        expected=k in ('cs_ev_pos','cs_ev_hgt')
        aiding_checks.append(check('estimator_status_flags.'+k,flags[k] is expected,flags[k]))
    aiding=all(aiding_checks)
    reset=0
    for i,k in enumerate(RESETS):
        value=int(local[k])
        if not 0<=value<=255: raise ValueError('invalid reset counter')
        reset |= value << (i*8)
    return received, all(fusion_checks) and valid and aiding, reset, failures


def evaluate_listener(samples, max_age):
    return inspect_listener(samples, max_age)[:3]
