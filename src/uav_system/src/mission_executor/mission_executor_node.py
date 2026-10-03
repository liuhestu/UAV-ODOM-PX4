#!/usr/bin/env python3
"""Mission Executor entry point, ROS communication and execution loop."""
import sys
import math
import struct
import threading
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mavros_msgs.msg import State, ExtendedState, Mavlink, AttitudeTarget
from mavros_msgs.srv import SetMode, CommandBool
from uav_system.msg import SystemStatus
from uav_core.geometry import rotation
from mission_executor.execution import ExecutionController, FcuGuard, AttitudeCommand
from uav_core.runtime import Inbox, assign, xyz, xyzw, wall_loop

from mission_executor.mission_loader import ExecutorLock, load_task


def run(cfg, task):
    mission=ExecutionController(cfg, task)
    box=Inbox()
    guard=FcuGuard(cfg.get('fcu_system_id',1),cfg.get('fcu_component_id',1))
    offboard_sent_at=[None]
    def connection(msg):
        with box.lock: guard.observe_connection(msg.connected)
    def boot_clock(msg):
        if msg.framing_status!=Mavlink.FRAMING_OK or msg.msgid not in (2,30,31,32): return
        if msg.len<=0 or len(msg.payload64)*8<msg.len: return
        payload=b''.join(struct.pack('<Q',value) for value in msg.payload64)[:msg.len].ljust(12,b'\0')
        offset=8 if msg.msgid==2 else 0
        boot_ms=struct.unpack_from('<I',payload,offset)[0]
        with box.lock: guard.observe_boot(msg.sysid,msg.compid,msg.msgid,boot_ms)
    rospy.Subscriber('/mavros/state',State,connection,queue_size=10)
    rospy.Subscriber('/mavlink/from',Mavlink,boot_clock,queue_size=100)
    for topic, cls, key in [('/uav/system/status',SystemStatus,'system'), ('/mavros/state',State,'fcu'),
        ('/mavros/extended_state',ExtendedState,'extended'), ('/mavros/local_position/odom',Odometry,'local')]:
        box.subscribe(topic,cls,key)
    pub=rospy.Publisher('/uav/command/trajectory',PoseStamped,queue_size=1)
    attitude_pub=rospy.Publisher('/uav/command/attitude',AttitudeTarget,queue_size=1)
    state_pub=rospy.Publisher('/uav/mission_executor/state',String,queue_size=1)
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
                # A blocked worker must not dispatch its old request when a
                # service reappears after disconnect/reboot or mission abort.
                with box.lock:
                    fcu=box.get('fcu',cfg['telemetry_timeout'])
                    expected={'OFFBOARD':('WAIT_OFFBOARD',),'ARM':('WAIT_ARM',),
                              'LAND':('WAIT_LAND_MODE','ABORT_LAND_MODE'),
                              'DISARM':('WAIT_DISARM','ABORT_DISARM')}[action]
                    if guard.fault or mission.state not in expected or not fcu or not fcu.connected:
                        rospy.logwarn('Executor %s cancelled before service call: state=%s connected=%s fault=%s',
                                      action, mission.state, bool(fcu and fcu.connected), guard.fault)
                        with result_lock: pending['result']=(action,False)
                        return
                    if action in ('OFFBOARD','ARM'):
                        status=box.get('system',cfg['status_timeout'])
                        ext=box.get('extended',cfg['telemetry_timeout'])
                        valid=bool(status and status.ready and status.arm_ready and
                                   status.source_session_id==mission.session and ext and ext.landed_state==1)
                        if action=='ARM':
                            if fcu.armed:
                                with result_lock: pending['result']=(action,True)
                                return
                            valid=valid and fcu.mode=='OFFBOARD'
                        if not valid:
                            rospy.logwarn('Executor %s rejected locally: ready=%s arm_ready=%s session_matches=%s landed=%s mode=%s',
                                          action, getattr(status,'ready',None), getattr(status,'arm_ready',None),
                                          bool(status and status.source_session_id==mission.session),
                                          getattr(ext,'landed_state',None), getattr(fcu,'mode',None))
                            with result_lock: pending['result']=(action,False)
                            return
                    if action=='OFFBOARD': offboard_sent_at[0]=time.monotonic()
                if action in ('OFFBOARD','LAND'):
                    reply=rospy.ServiceProxy(service,SetMode)(base_mode=0,custom_mode='OFFBOARD' if action=='OFFBOARD' else 'AUTO.LAND')
                    accepted=reply.mode_sent
                else:
                    reply=rospy.ServiceProxy(service,CommandBool)(value=action=='ARM'); accepted=reply.success
                    rospy.loginfo('Executor %s: Backend success=%s result=%s', action, accepted, getattr(reply,'result',None))
            except (rospy.ROSException,rospy.ServiceException) as exc:
                rospy.logerr('%s service failed: %s',action,exc)
            with result_lock:
                pending['result']=(action,accepted)
        threading.Thread(target=worker,daemon=True).start()
    previous=[None]; last_local=[None]; jump_fault=[False]
    def tick():
        with box.lock:
            if guard.fault and mission.state not in mission.TERMINAL:
                mission.enter('ABORTED',time.monotonic(),guard.fault)
            status=box.get('system',cfg['status_timeout']); fcu=box.get('fcu',cfg['telemetry_timeout'])
            ext=box.get('extended',cfg['telemetry_timeout']); odom=box.get('local',cfg['state_timeout'])
            ready=bool(status and status.arm_ready and status.ready)
            local=None
            if odom:
                try:
                    R=rotation(xyzw(odom.pose.pose.orientation)); pos=xyz(odom.pose.pose.position)
                    if not np.isfinite(pos).all(): raise ValueError('invalid local position')
                    if (mission.is_active or mission.state in ('PRESTREAM','WAIT_OFFBOARD','WAIT_ARM')) and not jump_fault[0] and last_local[0] and np.linalg.norm(np.asarray(pos)-last_local[0])>cfg['max_local_position_step']:
                        jump_fault[0]=True
                        with result_lock: busy=pending['action'] is not None
                        can_stop=mission.is_active and fcu and fcu.armed and fcu.mode=='OFFBOARD' and not busy
                        if can_stop and mission.command_kind=='attitude':
                            mission.stop_attitude(time.monotonic(),'PX4 local position jump',abort=True)
                        else:
                            mission.enter('ABORT_LAND_MODE' if can_stop else 'ABORTED',time.monotonic(),'PX4 local position jump')
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
                (fcu.connected,fcu.armed,fcu.mode) if fcu else None,local,ext.landed_state if ext else 0,result,busy,
                bool(fcu and offboard_sent_at[0] is not None and
                     box.data['fcu'][1]>offboard_sent_at[0]))
        if action: request(action)
        if cmd is not None:
            if isinstance(cmd,AttitudeCommand):
                msg=AttitudeTarget(); msg.header.stamp=rospy.Time.now(); msg.header.frame_id='px4_local'
                msg.type_mask=7  # Ignore all body rates; use orientation and thrust.
                assign(msg.orientation,cmd.orientation); assign(msg.body_rate,(0.0,0.0,0.0))
                msg.thrust=cmd.thrust
                attitude_pub.publish(msg)
            else:
                msg=PoseStamped(); msg.header.stamp=rospy.Time.now(); msg.header.frame_id='px4_local'
                assign(msg.pose.position,cmd[:3]); assign(msg.pose.orientation,[0,0,math.sin(cmd[3]/2),math.cos(cmd[3]/2)])
                pub.publish(msg)
        text=mission.state+(': '+mission.reason if mission.reason else '')
        state_pub.publish(String(text))
        if text!=previous[0]:
            rospy.loginfo('Mission Executor: %s',text); previous[0]=text
    wall_loop(cfg['rate_hz'],tick)


def main():
    import rospkg
    lock = ExecutorLock()  # Before init_node: duplicate startup cannot replace /mission_executor.
    try:
        rospy.init_node('mission_executor')
        loaded = load_task(
            rospkg.RosPack().get_path('uav_system'),
            source=rospy.get_param('~mission_source', 'propellerless_motor_check'),
            config=rospy.get_param('~mission_config', ''),
            executor_config=rospy.get_param('~executor_config', ''),
            auto_arm=rospy.get_param('~auto_arm_override', ''),
            arm_method=rospy.get_param('~arm_method_override', ''), warn=rospy.logwarn)
        for key, value in loaded.executor_config.items():
            rospy.set_param('~' + key, value)
        rospy.set_param('~mission', loaded.mission_config)
        rospy.loginfo('Mission Executor source=%s mission_config=%s executor_config=%s auto_arm=%s',
                      loaded.source_path, loaded.mission_config_path, loaded.executor_config_path,
                      loaded.executor_config['auto_arm'])
        run(loaded.executor_config, loaded.task)
    finally:
        lock.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('Mission Executor startup failed: ' + str(exc), file=sys.stderr)
        sys.exit(1)
