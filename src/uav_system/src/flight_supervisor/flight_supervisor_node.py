#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus, SysStatus, StatusText
from uav_system.msg import SourceStatus, SystemStatus, EvStatus
from uav_core.geometry import rotation
from flight_supervisor.checks import evaluate, estimator_failures, HealthReporter, advance_reset_counter
from uav_core.runtime import Inbox, xyz, xyzw, wall_loop


def main():
    rospy.init_node('flight_supervisor')
    box = Inbox()
    p = rospy.get_param('~')
    for topic, cls, key in [('/uav/source/status', SourceStatus, 'source'),
        ('/uav/state/health', SourceStatus, 'adapter'), ('/uav/state/odom', Odometry, 'odom'),
        ('/uav/backend/ev_sent', Bool, 'sent'), ('/uav/px4/ev_status', EvStatus, 'ev'),
        ('/mavros/state', State, 'fcu'), ('/mavros/extended_state', ExtendedState, 'extended'),
        ('/mavros/estimator_status', EstimatorStatus, 'ekf'), ('/mavros/sys_status', SysStatus, 'sys'),
        ('/mavros/local_position/odom', Odometry, 'local'), ('/mavros/statustext/recv', StatusText, 'text')]:
        box.subscribe(topic, cls, key)
    pub = rospy.Publisher('/uav/system/status', SystemStatus, queue_size=1)
    ready_pub = rospy.Publisher('/uav/system/ready', Bool, queue_size=1)
    arm_pub = rospy.Publisher('/uav/system/arm_ready', Bool, queue_size=1)
    reset = [None]
    fault = ['']
    session = [None]
    reporter = HealthReporter()
    labels = {
        'source unavailable': 'Source odometry fresh and healthy',
        'adapter unavailable': 'State adapter healthy and source session matches',
        'canonical odometry invalid/stale': 'Canonical odometry fresh and finite',
        'backend not publishing EV': 'Backend publishing external vision',
        'MAVROS disconnected/stale': 'FCU connected with fresh telemetry',
        'PX4 critical/unknown status': 'PX4 system status standby/active (diagnostic only)',
        'PX4 sensor health unavailable/failed': 'PX4 required sensors present/enabled/healthy (diagnostic only)',
        'PX4 local odometry invalid/stale': 'PX4 local odometry fresh and finite',
        'PX4 estimator validity unavailable/failed': 'PX4 estimator attitude/position/velocity valid',
        'PX4 landed-state telemetry unavailable': 'PX4 landed-state telemetry fresh and known',
        'PX4 EV received/fused evidence unavailable': 'PX4 EV received and position/height fused in current session',
        'reset fault': 'No latched source-session or PX4 estimator reset fault',
    }
    def tick():
        with box.lock:
            source, adapter, odom = [box.get(k, p['state_timeout']) for k in ('source', 'adapter', 'odom')]
            fcu, extended, ekf, sys = [box.get(k, p['telemetry_timeout']) for k in ('fcu', 'extended', 'ekf', 'sys')]
            local = box.get('local', p['state_timeout'])
            sent = box.get('sent', p['state_timeout'])
            ev = box.get('ev', p['evidence_timeout'])
            if source and session[0] and source.session_id != session[0]:
                fault[0]='source session changed'
            if source: session[0]=source.session_id
            if ev and ev.reset_valid:
                next_reset,changed=advance_reset_counter(reset[0],ev.reset_counter,ev.received,ev.fused)
                if changed:
                    fault[0]='PX4 estimator reset; restart system'
                    rospy.logwarn('%s; reset counters %s -> %s',fault[0],reset[0],ev.reset_counter)
                reset[0]=next_reset
            def finite(m):
                if not m: return False
                try:
                    rotation(xyzw(m.pose.pose.orientation))
                    return bool(np.isfinite(xyz(m.pose.pose.position)+xyz(m.twist.twist.linear)+xyz(m.twist.twist.angular)).all())
                except ValueError: return False
            required = p['required_sensor_mask']
            health = bool(sys and (sys.sensors_present & required)==required and
                          (sys.sensors_enabled & required)==required and
                          (sys.sensors_health & required)==required)
            ev_fused=bool(ev and ev.received and ev.fused and source and ev.source_session_id==source.session_id)
            ekf_failures=estimator_failures(ekf,ev_fused)
            gates = {
                'source unavailable': bool(source and source.healthy),
                'adapter unavailable': bool(adapter and adapter.healthy and source and adapter.session_id==source.session_id),
                'canonical odometry invalid/stale': finite(odom),
                'backend not publishing EV': bool(sent and sent.data),
                'MAVROS disconnected/stale': bool(fcu and fcu.connected),
                'PX4 critical/unknown status': bool(fcu and fcu.system_status in (3, 4)),
                'PX4 sensor health unavailable/failed': health,
                'PX4 local odometry invalid/stale': finite(local),
                'PX4 estimator validity unavailable/failed': not ekf_failures,
                'PX4 landed-state telemetry unavailable': bool(extended and extended.landed_state!=0),
                'PX4 EV received/fused evidence unavailable': ev_fused,
                'reset fault': not fault[0],
            }
            simulated = source.simulated if source else True
            calibrated = bool(source and source.calibrated and adapter and adapter.calibrated)
            ready, arm_ready, reasons = evaluate(gates, calibrated, simulated, p['simulation_transport'])
            reasons=[('reset fault: '+fault[0]) if reason=='reset fault' else
                     ('PX4 estimator validity unavailable/failed: '+ '; '.join(ekf_failures))
                     if reason=='PX4 estimator validity unavailable/failed' else
                     ('PX4 EV received/fused evidence unavailable: '+ev.detail)
                     if reason=='PX4 EV received/fused evidence unavailable' and ev else reason
                     for reason in reasons]
            checks = [(labels[name], passed) for name, passed in gates.items()]
            checks.extend([
                ('Source calibration/world alignment confirmed', bool(source and source.calibrated)),
                ('Adapter calibration/world alignment confirmed', bool(adapter and adapter.calibrated)),
                ('Real source or permitted simulation transport', not simulated or p['simulation_transport']),
                ('ON_GROUND (additional Mission Executor prerequisite)', bool(extended and extended.landed_state == 1)),
            ])
            report = reporter.update(checks, ready, arm_ready)
            if report is not None:
                detail = '\n  Details: source=%s; adapter=%s; reset=%s; PX4 system_status=%s; estimator=%s' % (
                    source.detail if source else 'unavailable/stale',
                    adapter.detail if adapter else 'unavailable/stale',
                    fault[0] or 'none', fcu.system_status if fcu else 'unavailable/stale',
                    '; '.join(ekf_failures) if ekf_failures else 'valid')
                log = rospy.loginfo if all(passed for _, passed in checks) else rospy.logwarn
                log('%s', report + detail)
            msg = SystemStatus(); msg.header.stamp=rospy.Time.now()
            msg.source_session_id=session[0] or ''; msg.ready=ready; msg.arm_ready=arm_ready
            msg.simulated=simulated; msg.calibrated=calibrated
            msg.backend_output_enabled=p['backend_output_enabled']; msg.simulation_transport=p['simulation_transport']
            msg.reasons=reasons
            text = box.data.get('text')
            msg.latest_px4_text=text[0].text if text else ''
            pub.publish(msg); ready_pub.publish(Bool(ready)); arm_pub.publish(Bool(arm_ready))
    wall_loop(20, tick)


if __name__ == '__main__':
    main()
