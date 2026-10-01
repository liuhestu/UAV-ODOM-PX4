#!/usr/bin/env python3
"""Optional read-only yaw recorder. ROS bag/ULog retain full-rate evidence."""
import json
import math
import os
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus, TimesyncStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String
from uav_system.msg import SystemStatus, SourceStatus, EvStatus


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def vector(v):
    return [v.x, v.y, v.z]


def ros_value(value):
    if hasattr(value, '__slots__'):
        return {key: ros_value(getattr(value, key)) for key in value.__slots__}
    if isinstance(value, (tuple, list)):
        return [ros_value(item) for item in value]
    return value


def summarize(msg):
    if isinstance(msg, Odometry):
        q = msg.pose.pose.orientation
        return dict(position=vector(msg.pose.pose.position),
                    quaternion=[q.x, q.y, q.z, q.w], yaw_rad=yaw(q),
                    velocity=vector(msg.twist.twist.linear),
                    angular_velocity=vector(msg.twist.twist.angular),
                    pose_covariance=list(msg.pose.covariance),
                    twist_covariance=list(msg.twist.covariance),
                    frame_id=msg.header.frame_id, child_frame_id=msg.child_frame_id)
    if isinstance(msg, PoseStamped):
        q = msg.pose.orientation
        return dict(position=vector(msg.pose.position), yaw_rad=yaw(q),
                    quaternion=[q.x, q.y, q.z, q.w], frame_id=msg.header.frame_id)
    if isinstance(msg, String):
        try:
            return json.loads(msg.data)
        except ValueError:
            return msg.data
    return ros_value(msg)


def main():
    rospy.init_node('yaw_diagnostics')
    path = os.path.expanduser(rospy.get_param('~output_file'))
    rate = float(rospy.get_param('~rate_hz', 10.0))
    if not math.isfinite(rate) or not 0.0 < rate <= 50.0:
        raise ValueError('rate_hz must be finite and between 0 and 50')
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    lock = threading.Lock()
    latest = {}
    specs = [
        ('/ov_msckf/odomimu', Odometry),
        ('/uav/state/odom', Odometry),
        ('/mavros/odometry/out', Odometry),
        ('/mavros/local_position/odom', Odometry),
        ('/mavros/imu/data', Imu),
        ('/mavros/imu/mag', MagneticField),
        ('/mavros/setpoint_position/local', PoseStamped),
        ('/uav/command/trajectory', PoseStamped),
        ('/mavros/state', State),
        ('/mavros/extended_state', ExtendedState),
        ('/mavros/estimator_status', EstimatorStatus),
        ('/mavros/timesync_status', TimesyncStatus),
        ('/uav/source/status', SourceStatus),
        ('/uav/state/health', SourceStatus),
        ('/uav/system/status', SystemStatus),
        ('/uav/px4/ev_status', EvStatus),
        ('/uav/px4/observer_snapshot', String),
        ('/uav/mission_executor/state', String),
    ]

    def callback(msg, topic):
        received_ros = rospy.Time.now().to_sec()
        received_mono = time.monotonic()
        stamp = msg.header.stamp.to_sec() if hasattr(msg, 'header') else None
        entry = dict(received_ros=received_ros, received_monotonic=received_mono,
                     stamp=stamp, data=summarize(msg))
        with lock:
            latest[topic] = entry

    subscriptions = [rospy.Subscriber(topic, cls, callback,
                     callback_args=topic, queue_size=5) for topic, cls in specs]
    rospy.loginfo('Yaw diagnostics recording to %s at %.1f Hz', path, rate)
    # Monotonic pacing avoids a ROS clock step blocking this diagnostic recorder.
    with open(path, 'x', buffering=1) as output:
        output.write(json.dumps(dict(kind='metadata', topics=[t for t, _ in specs],
                                    rate_hz=rate, units='SI; yaw in radians')) + '\n')
        while not rospy.is_shutdown():
            ros_now = rospy.Time.now().to_sec()
            mono_now = time.monotonic()
            with lock:
                samples = dict(latest)
            row = dict(kind='sample', ros=ros_now, monotonic=mono_now, topics={})
            for topic, sample in samples.items():
                row['topics'][topic] = dict(sample,
                    receipt_age=mono_now - sample['received_monotonic'],
                    stamp_age=None if sample['stamp'] is None else ros_now - sample['stamp'])
            output.write(json.dumps(row, sort_keys=True) + '\n')
            time.sleep(max(0.001, 1.0 / rate - (time.monotonic() - mono_now)))
    del subscriptions


if __name__ == '__main__':
    main()
