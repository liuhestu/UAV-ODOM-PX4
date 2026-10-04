#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus, SysStatus, StatusText
from uav_msgs.msg import SourceStatus, SystemStatus, EvStatus
from uav_core.geometry import rotation
from uav_core.readiness import evaluate
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
                if reset[0] is not None and ev.reset_counter != reset[0]:
                    fault[0]='PX4 estimator reset; restart system'
                reset[0]=ev.reset_counter
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
            gates = {
                'source unavailable': bool(source and source.healthy),
                'adapter unavailable': bool(adapter and adapter.healthy and source and adapter.session_id==source.session_id),
                'canonical odometry invalid/stale': finite(odom),
                'backend not publishing EV': bool(sent and sent.data),
                'MAVROS disconnected/stale': bool(fcu and fcu.connected),
                'PX4 critical/unknown status': bool(fcu and fcu.system_status in (3, 4)),
                'PX4 sensor health unavailable/failed': health,
                'PX4 local odometry invalid/stale': finite(local),
                'PX4 estimator validity unavailable/failed': bool(ekf and ekf.attitude_status_flag and
                    ekf.pos_horiz_rel_status_flag and ekf.pos_vert_abs_status_flag and
                    ekf.velocity_horiz_status_flag and ekf.velocity_vert_status_flag and
                    not ekf.const_pos_mode_status_flag and not ekf.accel_error_status_flag),
                'PX4 landed-state telemetry unavailable': bool(extended and extended.landed_state!=0),
                'PX4 EV received/fused evidence unavailable': bool(ev and ev.received and ev.fused and source and ev.source_session_id==source.session_id),
                'reset fault': not fault[0],
            }
            simulated = source.simulated if source else True
            calibrated = bool(source and source.calibrated and adapter and adapter.calibrated)
            ready, arm_ready, reasons = evaluate(gates, calibrated, simulated, p['simulation_transport'])
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
