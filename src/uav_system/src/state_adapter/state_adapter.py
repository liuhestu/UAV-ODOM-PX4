#!/usr/bin/env python3
import threading
import time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from uav_system.msg import SourceStatus
from uav_core.source_config import load_source
from uav_core.geometry import Adapter
from uav_core.runtime import assign, xyz, xyzw, wall_loop


def covariance_summary(values):
    try:
        c = np.asarray(values, dtype=float).reshape(6, 6)
        finite = bool(np.isfinite(c).all())
        if not finite:
            bad = np.argwhere(~np.isfinite(c))
            return 'finite=False bad_index=%s' % (tuple(int(v) for v in bad[0]) if bad.size else None,)
        asym = float(np.max(np.abs(c-c.T)))
        eig = np.linalg.eigvalsh((c+c.T)*0.5)
        index = np.unravel_index(int(np.argmax(np.abs(c-c.T))), c.shape)
        return 'finite=True asym=%.3g asym_index=%s min_eig=%.6g diag=%s' % (asym, index, eig.min(), np.diag(c).round(6).tolist())
    except Exception as exc:
        return 'summary_error=%s' % (exc,)


def main():
    rospy.init_node('state_adapter')
    cfg = load_source(rospy.get_param('~source_config'))
    adapter = Adapter(cfg)
    odom_pub = rospy.Publisher('/uav/state/odom', Odometry, queue_size=1)
    health_pub = rospy.Publisher('/uav/state/health', SourceStatus, queue_size=1)
    mutex = threading.RLock()
    state = dict(last_stamp=None, last_p=None, last_receive=None, source=None, source_at=None,
                 session=None, last_q=None, fault='', error='waiting for odometry', last_reject=None)
    def source(msg):
        with mutex:
            if state['session'] and msg.session_id != state['session']:
                state['fault'] = 'source session changed; restart system'
            state['session'] = msg.session_id
            state['source'], state['source_at'] = msg, time.monotonic()
    def raw(msg):
        with mutex:
            try:
                stamp = msg.header.stamp.to_sec()
                if stamp <= 0 or not -0.05 <= (rospy.Time.now()-msg.header.stamp).to_sec() <= cfg['adapter']['stale_timeout']:
                    raise ValueError('stale/future raw timestamp')
                if state['last_stamp'] is not None and stamp <= state['last_stamp']:
                    state['fault'] = 'timestamp regression/reset; restart system'
                if state['fault']:
                    raise ValueError(state['fault'])
                if msg.header.frame_id != cfg['input']['world_frame'] or msg.child_frame_id != cfg['input']['body_frame']:
                    raise ValueError('unexpected source frames')
                p, q, v, w, pc, tc = adapter.adapt(xyz(msg.pose.pose.position), xyzw(msg.pose.pose.orientation),
                    xyz(msg.twist.twist.linear), xyz(msg.twist.twist.angular), msg.pose.covariance, msg.twist.covariance)
                if state['last_p'] is not None and np.linalg.norm(p-state['last_p']) > cfg['adapter']['max_position_step']:
                    state['fault'] = 'position jump/reset; restart system'
                    raise ValueError(state['fault'])
                if state['last_q'] is not None:
                    angle=2*np.arccos(min(1.0,abs(float(np.dot(q,state['last_q'])))))
                    if angle>cfg['adapter']['max_rotation_step_rad']:
                        state['fault']='orientation jump/reset; restart system'
                        raise ValueError(state['fault'])
                out = Odometry()
                out.header.stamp = msg.header.stamp  # Preserve measurement time.
                out.header.frame_id = 'odom'
                out.child_frame_id = 'base_link'
                assign(out.pose.pose.position, p); assign(out.pose.pose.orientation, q)
                assign(out.twist.twist.linear, v); assign(out.twist.twist.angular, w)
                out.pose.covariance = pc.ravel().tolist(); out.twist.covariance = tc.ravel().tolist()
                state.update(last_stamp=stamp, last_p=p, last_q=q, last_receive=time.monotonic(), error='')
                odom_pub.publish(out)
            except (ValueError, KeyError, TypeError) as exc:
                detail = str(exc)
                first = detail != state['last_reject']
                state['last_reject'] = detail
                state['error'] = detail
                state['last_receive'] = None
                label = 'first' if first else 'repeated'
                rospy.logwarn_throttle(
                    2, 'Adapter %s rejection: reason=%s stamp=%.9f age=%.6f frame=%s child=%s p=%s q=%s q_norm=%.6f pose_cov={%s} twist_cov={%s}',
                    label, detail, stamp, (rospy.Time.now()-msg.header.stamp).to_sec(),
                    msg.header.frame_id, msg.child_frame_id,
                    np.asarray(xyz(msg.pose.pose.position)).round(6).tolist(),
                    np.asarray(xyzw(msg.pose.pose.orientation)).round(6).tolist(),
                    float(np.linalg.norm(np.asarray(xyzw(msg.pose.pose.orientation)))),
                    covariance_summary(msg.pose.covariance), covariance_summary(msg.twist.covariance))
    rospy.Subscriber('/uav/source/status', SourceStatus, source, queue_size=1)
    rospy.Subscriber(cfg['input']['topic'], Odometry, raw, queue_size=1)
    def tick():
        with mutex:
            msg = SourceStatus()
            msg.header.stamp = rospy.Time.now()
            s = state['source']
            msg.session_id = state['session'] or ''
            msg.simulated = cfg['source']['simulated']
            msg.calibrated = cfg['extrinsic']['calibrated'] and cfg['world_alignment']['verified']
            fresh = state['last_receive'] is not None and time.monotonic()-state['last_receive'] <= cfg['adapter']['stale_timeout']
            source_fresh = s and time.monotonic()-state['source_at'] <= 0.5 and -0.05 <= (msg.header.stamp-s.header.stamp).to_sec() <= 0.5
            msg.healthy = bool(fresh and source_fresh and s.healthy and not state['fault'] and not state['error'])
            msg.detail = state['fault'] or state['error'] or ('canonical odometry fresh' if msg.healthy else 'source or canonical odometry stale')
            health_pub.publish(msg)
    wall_loop(20, tick)


if __name__ == '__main__':
    main()
