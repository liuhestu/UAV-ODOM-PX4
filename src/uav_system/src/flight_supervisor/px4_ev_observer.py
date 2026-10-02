#!/usr/bin/env python3
"""PX4 single-EKF listener observer through MAVROS SERIAL_CONTROL.

Only five fixed read-only listener commands are permitted. Unsupported firmware,
missing output, multi-EKF, shell interference, or stale fields -> no evidence.
No new serial connection, parameter writes, arming, or mode commands.
"""
import struct
import re
import threading
import time
import rospy
from mavros_msgs.msg import Mavlink, State
from mavros_msgs.srv import ParamGet
from uav_system.msg import SourceStatus, EvStatus
from flight_supervisor.checks import (TOPICS, SINGLE_EKF_PARAMETERS, parse_listener,
                               evaluate_listener, verify_single_ekf_parameters)
from uav_core.runtime import Inbox


def encode(data, seq, system, component, flags=6):
    raw=data.encode('ascii')
    if len(raw)>70: raise ValueError('SERIAL_CONTROL data exceeds 70 bytes')
    payload=struct.pack('<IHBBB70s',0,0,10,flags,len(raw),raw.ljust(70,b'\0'))
    header=bytes([79,seq,system,component,126])
    crc=0xffff
    for b in header+payload+bytes([220]):
        tmp=b^(crc&255); tmp^=(tmp<<4)&255
        crc=((crc>>8)^(tmp<<8)^(tmp<<3)^(tmp>>4))&65535
    return payload,crc


def main():
    rospy.init_node('px4_ev_observer')
    p=rospy.get_param('~observer'); box=Inbox()
    box.subscribe('/uav/source/status',SourceStatus,'source'); box.subscribe('/mavros/state',State,'fcu')
    to=rospy.Publisher('/mavlink/to',Mavlink,queue_size=10)
    pub=rospy.Publisher('/uav/px4/ev_status',EvStatus,queue_size=1)
    cv=threading.Condition(); received={'text':''}
    def serial(msg):
        if msg.framing_status!=Mavlink.FRAMING_OK or msg.msgid!=126 or msg.sysid!=p['target_system_id'] or msg.compid!=p['target_component_id']: return
        payload=b''.join(struct.pack('<Q',n) for n in msg.payload64)[:msg.len]
        payload=payload.ljust(79,b'\0')
        device,count=payload[6],payload[8]
        if device!=10 or count>70: return
        with cv:
            received['text']=(received['text']+payload[9:9+count].decode('utf-8',errors='replace'))[-16000:]
            cv.notify_all()
    rospy.Subscriber('/mavlink/from',Mavlink,serial,queue_size=100)
    seq=[0]
    def send(data,flags=6):
        # Construct MAVLink v1 SERIAL_CONTROL (CRC extra 220); MAVROS forwards it.
        # Wire order: baudrate uint32, timeout uint16, device, flags, count, data[70].
        payload,crc=encode(data,seq[0],p['sender_system_id'],p['sender_component_id'],flags)
        msg=Mavlink(); msg.header.stamp=rospy.Time.now(); msg.framing_status=Mavlink.FRAMING_OK
        msg.magic=Mavlink.MAVLINK_V10; msg.len=79; msg.seq=seq[0]
        msg.sysid=p['sender_system_id']; msg.compid=p['sender_component_id']; msg.msgid=126; msg.checksum=crc
        msg.payload64=list(struct.unpack('<10Q',payload.ljust(80,b'\0')))
        seq[0]=(seq[0]+1)%256; to.publish(msg)
    rospy.on_shutdown(lambda:send('',0))
    def verify_single_ekf():
        rospy.wait_for_service('/mavros/param/get',timeout=1)
        get=rospy.ServiceProxy('/mavros/param/get',ParamGet)
        parameters={}
        for name in SINGLE_EKF_PARAMETERS:
            reply=get(param_id=name)
            if not reply.success:
                raise ValueError('single EKF parameter unavailable: '+name)
            parameters[name]=reply.value.integer
        verify_single_ekf_parameters(parameters)
    def query(topic):
        with cv: received['text']=''
        send('listener '+topic+' -n 1 -i 0\n')
        deadline=time.monotonic()+p['query_timeout']
        with cv:
            while not rospy.is_shutdown() and time.monotonic()<deadline:
                # PX4 shell prompt closes each fixed command response.
                clean=re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', received['text'])
                if re.search(r'nsh>\s*$',clean) and ('TOPIC:' in clean or 'never published' in clean or 'not found' in clean):
                    return parse_listener(topic,received['text'])
                cv.wait(timeout=min(0.1,max(0,deadline-time.monotonic())))
        raise ValueError('PX4 listener timeout: '+topic)
    send('\n')
    while not rospy.is_shutdown():
        start=time.monotonic(); msg=EvStatus(); msg.header.stamp=rospy.Time.now()
        try:
            with box.lock:
                source=box.get('source',p['source_timeout']); fcu=box.get('fcu',p['source_timeout'])
            if not source or not source.healthy or not fcu or not fcu.connected: raise ValueError('source/FCU unavailable')
            if to.get_num_connections()==0: raise ValueError('MAVROS raw MAVLink bridge unavailable')
            verify_single_ekf()
            samples={topic:query(topic) for topic in TOPICS}
            # Timestamp represents completion of the observation, not poll start.
            msg.header.stamp=rospy.Time.now(); msg.source_session_id=source.session_id
            msg.received,msg.fused,msg.reset_counter=evaluate_listener(samples,p['max_px4_age'])
            msg.reset_valid=True
            msg.detail='single EKF instance 0: EV position/height fusion observed'
        except (ValueError,KeyError,TypeError,rospy.ROSException,rospy.ServiceException) as exc:
            msg.detail=str(exc)
        pub.publish(msg)
        time.sleep(max(0.01,p['poll_seconds']-(time.monotonic()-start)))


if __name__ == '__main__':
    main()
