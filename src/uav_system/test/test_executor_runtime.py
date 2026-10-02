"""Executor watchdog/jump checks with isolated in-memory ROS substitutes."""
import importlib.util
import threading
import types
import unittest
from unittest.mock import patch
import yaml
from support import ROOT, takeoff_task
from mission_executor.execution import ExecutionController as Mission


class ExecutorRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.cfg=yaml.safe_load((ROOT/'config/mission_executor.yaml').read_text())
        self.cfg['auto_arm']=True
        self.m=Mission(self.cfg, takeoff_task(self.cfg)); self.m.session='s'; self.m.preparation_mode='OFFBOARD'
        self.m.hold=(0,0,0,0);self.m.target=self.m.hold
        self.clock=0.;self.tick=None;self.commands=[];self.threads=[];self.callbacks={};self.service_calls=[]
        ns=types.SimpleNamespace
        self.data={'system':ns(ready=True,arm_ready=True,source_session_id='s'),
                   'fcu':ns(connected=True,armed=True,mode='OFFBOARD'),
                   'extended':ns(landed_state=1),
                   'local':ns(pose=ns(pose=ns(position=ns(x=0.,y=0.,z=0.),
                                             orientation=ns(x=0.,y=0.,z=0.,w=1.))))}
        fake=types.ModuleType('rospy'); fake.init_node=lambda *args:None
        fake.get_param=lambda *args:self.cfg;fake.set_param=lambda *args:None
        fake.Time=ns(now=lambda:0)
        fake.loginfo=lambda *args:None;fake.logwarn=lambda *args:None
        fake.logerr=lambda *args:None
        fake.Subscriber=lambda topic,cls,callback,**kwargs:self.callbacks.update({topic:callback})
        fake.wait_for_service=lambda *args,**kwargs:None
        fake.ServiceProxy=lambda name,cls:lambda **kwargs:self.service_calls.append((name,kwargs)) or ns(mode_sent=True,success=True)
        fake.ROSException=RuntimeError;fake.ServiceException=RuntimeError
        def publisher(topic,*args,**kwargs):
            return ns(publish=lambda msg:self.commands.append(msg) if topic=='/uav/command/trajectory' else None)
        fake.Publisher=publisher
        class Box:
            lock=threading.RLock()
            def __init__(box):
                box.data={};self.box=box
            def subscribe(box,*args):pass
            def get(box,key,timeout):return self.data.get(key)
        runtime=types.ModuleType('uav_core.runtime'); runtime.Inbox=Box
        runtime.xyz=lambda value:[value.x,value.y,value.z]
        runtime.xyzw=lambda value:[value.x,value.y,value.z,value.w]
        runtime.assign=lambda obj,values:None
        runtime.wall_loop=lambda rate,tick:setattr(self,'tick',tick)
        modules={'rospy':fake,'uav_core.runtime':runtime}
        def pose():return ns(header=ns(),pose=ns(position=ns(),orientation=ns()))
        for name, attrs in {'geometry_msgs.msg':{'PoseStamped':pose},'nav_msgs.msg':{'Odometry':ns},
                            'std_msgs.msg':{'String':lambda text:text},
                            'mavros_msgs.msg':{'State':ns,'ExtendedState':ns,'Mavlink':ns(FRAMING_OK=1)},
                            'mavros_msgs.srv':{'SetMode':ns,'CommandBool':ns},
                            'uav_system.msg':{'SystemStatus':ns}}.items():
            module=types.ModuleType(name)
            for key,value in attrs.items():setattr(module,key,value)
            modules[name]=module
        spec=importlib.util.spec_from_file_location('executor_under_test',ROOT/'src/mission_executor/mission_executor_node.py')
        self.module=importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules',modules):spec.loader.exec_module(self.module)
        self.module.ExecutionController=lambda cfg,task:self.m
        self.module.run(self.cfg, takeoff_task(self.cfg))
        clock=patch.object(self.module.time,'monotonic',lambda:self.clock);clock.start();self.addCleanup(clock.stop)
        worker=patch.object(self.module.threading,'Thread',lambda **kwargs:ns(start=lambda:self.threads.append(kwargs)))
        worker.start();self.addCleanup(worker.stop)

    def preparation_jump(self, phase):
        self.cfg['auto_arm']=False
        self.m.enter(phase,0)
        self.data['fcu'].armed=False
        if phase=='WAIT_OFFBOARD':
            self.data['fcu'].mode='POSCTL';self.m.preparation_mode='POSCTL'
        self.tick();self.data['local'].pose.pose.position.x=2
        self.clock=.1;self.tick()
        self.assertEqual(self.m.state,'ABORTED')
        self.assertEqual(self.threads,[])
        self.assertEqual(len(self.commands),1)

    def test_prestream_jump_stops_without_land_request(self):
        self.preparation_jump('PRESTREAM')

    def test_wait_offboard_jump_stops_without_land_request(self):
        self.preparation_jump('WAIT_OFFBOARD')

    def test_wait_arm_jump_stops_without_land_request(self):
        self.preparation_jump('WAIT_ARM')

    def test_flight_jump_requests_land_once(self):
        self.m.task.start(0,self.m.hold);self.m.enter('TAKEOFF',0);self.m.is_active=True;self.data['extended'].landed_state=2
        self.tick();self.data['local'].pose.pose.position.x=2
        self.clock=.1;self.tick()
        self.assertEqual(self.m.state,'ABORT_LAND_MODE');self.assertEqual(len(self.threads),1)
        self.clock=.2;self.tick();self.assertEqual(len(self.threads),1)

    def test_stuck_service_watchdog_latches(self):
        self.m.enter('PRESTREAM',0);self.data['fcu'].mode='POSCTL';self.m.preparation_mode='POSCTL'
        self.clock=3;self.tick();self.assertEqual(len(self.threads),1)
        self.clock=12;self.tick()
        self.assertEqual(self.m.state,'ABORTED');self.assertIn('service timeout',self.m.reason)
        count=len(self.commands)
        self.data['fcu'].mode='OFFBOARD';self.clock=13;self.tick()
        self.assertEqual(len(self.commands),count);self.assertEqual(len(self.threads),1)

    def test_nonfinite_local_stops_preparation(self):
        self.m.enter('PRESTREAM',0)
        self.data['local'].pose.pose.position.z=float('nan');self.tick()
        self.assertEqual(self.m.state,'ABORTED');self.assertEqual(self.commands,[])

    def test_disconnect_between_ticks_latches_and_cancels_waiting_service(self):
        self.m.enter('PRESTREAM',0);self.data['fcu'].armed=False
        self.clock=3;self.tick();self.assertEqual(len(self.threads),1)
        callback=self.callbacks['/mavros/state']
        callback(types.SimpleNamespace(connected=True))
        callback(types.SimpleNamespace(connected=False))
        callback(types.SimpleNamespace(connected=True))
        self.threads[0]['target']()
        self.assertEqual(self.service_calls,[])
        self.clock=4;self.tick()
        self.assertEqual(self.m.state,'ABORTED')
        count=len(self.commands);self.clock=20;self.tick()
        self.assertEqual(len(self.commands),count)

    def test_boot_rollback_without_connection_loss_stops_task(self):
        import struct
        callback=self.callbacks['/mavlink/from']
        for boot_ms in (100000,100):
            payload=struct.pack('<I',boot_ms).ljust(8,b'\0')
            callback(types.SimpleNamespace(framing_status=1,msgid=32,sysid=1,compid=1,
                                            len=4,payload64=[struct.unpack('<Q',payload)[0]]))
        self.m.task.start(0,self.m.hold);self.m.enter('TAKEOFF',0);self.m.is_active=True;self.tick()
        self.assertEqual(self.m.state,'ABORTED')
        self.assertEqual(self.commands,[]);self.assertEqual(self.threads,[])

    def test_existing_offboard_requires_a_request_and_post_request_heartbeat(self):
        self.data['fcu'].armed=False
        self.box.data['fcu']=(self.data['fcu'],0)
        self.m.enter('PRESTREAM',0)
        self.clock=3;self.tick();self.threads[0]['target']()
        self.assertEqual(self.service_calls[0][1]['custom_mode'],'OFFBOARD')
        self.clock=3.1;self.tick()
        self.assertEqual(self.m.state,'WAIT_OFFBOARD')
        self.assertEqual(len(self.threads),1)
        self.box.data['fcu']=(self.data['fcu'],3.2)
        self.clock=3.2;self.tick()
        self.assertEqual(self.m.state,'WAIT_ARM')
        self.assertEqual(len(self.threads),2)
        self.threads[1]['target']()
        self.assertEqual(self.service_calls[1][1],{'value':True})

    def test_waiting_arm_service_is_canceled_after_boot_reset(self):
        import struct
        self.data['fcu'].armed=False
        self.m.offboard_requested=True;self.m.enter('WAIT_ARM',0)
        self.tick();self.assertEqual(len(self.threads),1)
        callback=self.callbacks['/mavlink/from']
        for boot_ms in (100000,100):
            payload=struct.pack('<I',boot_ms).ljust(8,b'\0')
            callback(types.SimpleNamespace(framing_status=1,msgid=32,sysid=1,compid=1,
                                            len=4,payload64=[struct.unpack('<Q',payload)[0]]))
        self.threads[0]['target']()
        self.assertEqual(self.service_calls,[])
        self.clock=1;self.tick();self.assertEqual(self.m.state,'ABORTED')

    def test_generic_plugin_phase_jump_uses_active_fault_landing(self):
        self.m.task.start(0,self.m.hold)
        self.m.enter('CUSTOM',0);self.m.is_active=True
        self.tick();self.data['local'].pose.pose.position.x=2
        self.clock=.1;self.tick()
        self.assertEqual(self.m.state,'ABORT_LAND_MODE')
        self.assertEqual(len(self.threads),1)

    def test_arm_worker_does_not_forward_after_actual_early_arm(self):
        self.data['fcu'].armed=False
        self.m.offboard_requested=True;self.m.enter('WAIT_ARM',0)
        self.tick();self.assertEqual(len(self.threads),1)
        self.data['fcu'].armed=True
        self.threads[0]['target']()
        self.assertEqual(self.service_calls,[])
