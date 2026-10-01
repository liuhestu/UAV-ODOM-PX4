#!/usr/bin/env python3
import math
import threading
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
from uav_system.msg import SystemStatus
from uav_core.geometry import rotation
from uav_core.mission import Mission
from uav_core.runtime import Inbox, assign, xyz, xyzw, wall_loop


def main():
    rospy.init_node('takeoff_hover_land')
    cfg=rospy.get_param('~'); mission=Mission(cfg)
    box=Inbox()
    for topic, cls, key in [('/uav/system/status',SystemStatus,'system'), ('/mavros/state',State,'fcu'),
        ('/mavros/extended_state',ExtendedState,'extended'), ('/mavros/local_position/odom',Odometry,'local')]:
        box.subscribe(topic,cls,key)
    pub=rospy.Publisher('/uav/command/trajectory',PoseStamped,queue_size=1)
    state_pub=rospy.Publisher('/uav/commander/state',String,queue_size=1)
    pending={'action':None,'result':None,'start':None}
    result_lock=threading.Lock()
    def request(action):
        with result_lock:
            if pending['action'] is not None:
                raise RuntimeError('overlapping flight service request')
            pending.update(action=action,result=None,start=time.monotonic())
        def worker():
            accepted=False
            try:
                service='/uav/backend/set_mode' if action in ('OFFBOARD','LAND') else '/uav/backend/arming'
                rospy.wait_for_service(service,timeout=cfg['transition_timeout'])
                if action in ('OFFBOARD','LAND'):
                    reply=rospy.ServiceProxy(service,SetMode)(base_mode=0,custom_mode='OFFBOARD' if action=='OFFBOARD' else 'AUTO.LAND')
                    accepted=reply.mode_sent
                else:
                    reply=rospy.ServiceProxy(service,CommandBool)(value=action=='ARM'); accepted=reply.success
            except (rospy.ROSException,rospy.ServiceException) as exc:
                rospy.logerr('%s service failed: %s',action,exc)
            with result_lock:
                pending['result']=(action,accepted)
        threading.Thread(target=worker,daemon=True).start()
    previous=[None]; last_local=[None]; jump_fault=[False]
    def tick():
        with box.lock:
            status=box.get('system',cfg['status_timeout']); fcu=box.get('fcu',cfg['telemetry_timeout'])
            ext=box.get('extended',cfg['telemetry_timeout']); odom=box.get('local',cfg['state_timeout'])
            ready=bool(status and status.arm_ready and status.ready)
            local=None
            if odom:
                try:
                    R=rotation(xyzw(odom.pose.pose.orientation)); pos=xyz(odom.pose.pose.position)
                    if not np.isfinite(pos).all(): raise ValueError('invalid local position')
                    if mission.state in ('PRESTREAM','WAIT_OFFBOARD','WAIT_ARM','TAKEOFF','HOVER') and not jump_fault[0] and last_local[0] and np.linalg.norm(np.asarray(pos)-last_local[0])>cfg['max_local_position_step']:
                        jump_fault[0]=True
                        with result_lock: busy=pending['action'] is not None
                        mission.enter('ABORT_LAND_MODE' if fcu and fcu.armed and fcu.mode=='OFFBOARD' and not busy else 'ABORTED',time.monotonic(),'PX4 local position jump')
                        if mission.state=='ABORT_LAND_MODE': request('LAND')
                    last_local[0]=pos
                    local=tuple(pos)+(math.atan2(R[1,0],R[0,0]),)
                except ValueError: ready=False
            with result_lock:
                result=pending['result']
                busy=pending['action'] is not None and result is None
                if result:
                    pending.update(action=None,result=None,start=None)
                elif pending['action'] and time.monotonic()-pending['start']>cfg['transition_timeout']:
                    mission.enter('ABORTED',time.monotonic(),'flight service timeout; inspect actual FCU state')
            cmd,action=mission.step(time.monotonic(),ready,status.source_session_id if status else '',
                (fcu.connected,fcu.armed,fcu.mode) if fcu else None,local,ext.landed_state if ext else 0,result,busy)
        if action: request(action)
        if cmd:
            msg=PoseStamped(); msg.header.stamp=rospy.Time.now(); msg.header.frame_id='px4_local'
            assign(msg.pose.position,cmd[:3]); assign(msg.pose.orientation,[0,0,math.sin(cmd[3]/2),math.cos(cmd[3]/2)])
            pub.publish(msg)
        text=mission.state+(': '+mission.reason if mission.reason else '')
        state_pub.publish(String(text))
        if text!=previous[0]:
            rospy.loginfo('Commander: %s',text); previous[0]=text
    wall_loop(cfg['rate_hz'],tick)


if __name__ == '__main__':
    main()
