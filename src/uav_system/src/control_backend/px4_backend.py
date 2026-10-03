#!/usr/bin/env python3
"""MAVROS boundary. No controller math, firmware writes, or automatic arm."""
import copy
import time
import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from mavros_msgs.msg import State, ExtendedState, AttitudeTarget
from mavros_msgs.srv import SetMode, SetModeResponse, CommandBool, CommandBoolResponse
from uav_system.msg import SourceStatus, SystemStatus
from uav_core.geometry import rotation, covariance
from uav_core.validation import positive, attitude_target
from uav_core.runtime import Inbox, xyz, xyzw, wall_loop


def main():
    rospy.init_node('control_backend')
    params = rospy.get_param('~')
    p = dict(params['backend']); p.update({k:v for k,v in params.items() if k not in ('backend','observer')})
    positive(p,('max_attitude_thrust',))
    if p['max_attitude_thrust']>1: raise ValueError('invalid max_attitude_thrust')
    box = Inbox()
    for topic, cls, key in [('/uav/state/odom', Odometry, 'odom'), ('/uav/state/health', SourceStatus, 'health'),
        ('/uav/command/trajectory', PoseStamped, 'command'),
        ('/uav/command/attitude', AttitudeTarget, 'attitude'), ('/uav/system/status', SystemStatus, 'system'),
        ('/mavros/state', State, 'fcu'), ('/mavros/extended_state', ExtendedState, 'extended')]:
        box.subscribe(topic, cls, key)
    ev_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=1)
    setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    sent_pub = rospy.Publisher('/uav/backend/ev_sent', Bool, queue_size=1)
    tf_buffer = tf2_ros.Buffer(); tf_listener = tf2_ros.TransformListener(tf_buffer)
    last_ev = [None]; last_tx = [None]; stream_start=[None]; last_stream=[None]; stream_kind=[None]
    def transport_ok():
        return not p['simulation_transport'] or rospy.get_param('/mavros/fcu_url', '')=='udp://:14540@127.0.0.1:14557'
    def eligible():
        status = box.get('system', p['status_timeout'])
        return bool(p['output_enabled'] and transport_ok() and status and status.arm_ready and
                    status.backend_output_enabled and status.simulation_transport==p['simulation_transport'] and
                    (not status.simulated or p['simulation_transport']))
    def mode(req):
        with box.lock:
            fcu=box.get('fcu', p['telemetry_timeout'])
            valid=bool(p['output_enabled'] and transport_ok() and fcu and fcu.connected)
            if req.custom_mode=='OFFBOARD':
                ext=box.get('extended', p['telemetry_timeout'])
                valid = valid and ext and ext.landed_state==1 and eligible() and stream_start[0] is not None and last_stream[0] is not None and time.monotonic()-last_stream[0]<0.2 and time.monotonic()-stream_start[0]>=p['prestream_seconds']
            elif req.custom_mode=='AUTO.LAND':
                # Never override a pilot takeover.
                valid = valid and fcu.armed and fcu.mode=='OFFBOARD'
            else:
                valid=False
        if not valid: return SetModeResponse(mode_sent=False)
        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=1)
            return rospy.ServiceProxy('/mavros/set_mode', SetMode)(base_mode=0, custom_mode=req.custom_mode)
        except (rospy.ROSException, rospy.ServiceException):
            return SetModeResponse(mode_sent=False)
    def arm(req):
        with box.lock:
            fcu=box.get('fcu', p['telemetry_timeout'])
            ext=box.get('extended', p['telemetry_timeout'])
            valid=bool(p['output_enabled'] and transport_ok() and fcu and fcu.connected)
            if req.value and valid and fcu.armed:
                return CommandBoolResponse(success=True, result=0)
            if req.value:
                valid = valid and eligible() and fcu.mode=='OFFBOARD' and ext and ext.landed_state==1
            else:
                valid = valid and ext and ext.landed_state==1
        if not valid: return CommandBoolResponse(success=False, result=1)
        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=1)
            return rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)(value=req.value)
        except (rospy.ROSException, rospy.ServiceException):
            return CommandBoolResponse(success=False, result=4)
    rospy.Service('/uav/backend/set_mode', SetMode, mode)
    rospy.Service('/uav/backend/arming', CommandBool, arm)
    def tick():
        with box.lock:
            odom=box.get('odom', p['state_timeout']); health=box.get('health', p['state_timeout'])
            fcu=box.get('fcu', p['telemetry_timeout'])
            can_tx=bool(p['output_enabled'] and transport_ok() and fcu and fcu.connected and health and health.healthy and
                        (not health.simulated or p['simulation_transport']))
            if can_tx and odom and odom.header.stamp!=last_ev[0]:
                try:
                    if odom.header.frame_id!='odom' or odom.child_frame_id!='base_link': raise ValueError('wrong canonical frames')
                    rotation(xyzw(odom.pose.pose.orientation)); covariance(odom.pose.covariance); covariance(odom.twist.covariance)
                    if not np.isfinite(xyz(odom.pose.pose.position)+xyz(odom.twist.twist.linear)+xyz(odom.twist.twist.angular)).all(): raise ValueError('nonfinite canonical odometry')
                    transforms_ok = tf_buffer.can_transform('odom_ned', 'odom', rospy.Time(0)) and tf_buffer.can_transform('base_link_frd', 'base_link', rospy.Time(0))
                    if transforms_ok and ev_pub.get_num_connections()>0:
                        ev_pub.publish(odom); last_ev[0]=odom.header.stamp; last_tx[0]=time.monotonic()
                except ValueError as exc:
                    rospy.logwarn_throttle(2, '%s', exc)
            sent_pub.publish(Bool(bool(can_tx and last_tx[0] is not None and time.monotonic()-last_tx[0]<p['state_timeout'])))
            command=box.get('command', p['command_timeout'])
            attitude=box.get('attitude', p['command_timeout'])
            stream=False; kind=None
            # Never choose between two simultaneous command sources.
            if command and attitude:
                rospy.logwarn_throttle(2,'conflicting position/attitude commands; output stopped')
            stopping=bool(p['output_enabled'] and transport_ok() and attitude and attitude.thrust==0 and
                          fcu and fcu.connected and fcu.armed and fcu.mode=='OFFBOARD')
            if (eligible() or stopping) and fcu and fcu.connected and not (command and attitude):
                try:
                    if attitude:
                        if attitude.header.frame_id!='px4_local' or attitude.type_mask!=7:
                            raise ValueError('invalid attitude command frame or mask')
                        attitude_target(xyzw(attitude.orientation),attitude.thrust,p['max_attitude_thrust'])
                        if not np.isfinite(xyz(attitude.body_rate)).all() or any(xyz(attitude.body_rate)):
                            raise ValueError('body rates must be zero and ignored')
                        scaling=rospy.get_param('/mavros/setpoint_raw/thrust_scaling',None)
                        if type(scaling) not in (int,float) or scaling!=1.0:
                            raise ValueError('attitude output requires MAVROS thrust_scaling=1.0')
                        if not fcu.armed and attitude.thrust!=0:
                            raise ValueError('positive attitude thrust before ARM')
                        if attitude.thrust>0 and fcu.mode!='OFFBOARD':
                            raise ValueError('attitude command after mode takeover')
                        if attitude_pub.get_num_connections()>0:
                            output=copy.deepcopy(attitude); output.header.stamp=rospy.Time.now()
                            output.header.frame_id='map'
                            attitude_pub.publish(output); stream=True; kind='attitude'
                    elif command and command.header.frame_id=='px4_local':
                        rotation(xyzw(command.pose.orientation))
                        if not np.isfinite(xyz(command.pose.position)).all(): raise ValueError('nonfinite command')
                        output=copy.deepcopy(command)
                        # MAVROS interprets position as local ENU.
                        output.header.frame_id='map'; output.header.stamp=rospy.Time.now()
                        setpoint_pub.publish(output); stream=True; kind='position'
                except ValueError as exc:
                    rospy.logwarn_throttle(2,'%s',exc)
            now=time.monotonic()
            if stream:
                if last_stream[0] is None or now-last_stream[0]>0.2 or kind!=stream_kind[0]: stream_start[0]=now
                last_stream[0]=now; stream_kind[0]=kind
            else:
                stream_start[0]=None; last_stream[0]=None; stream_kind[0]=None
    wall_loop(p['rate_hz'], tick)


if __name__ == '__main__':
    main()
