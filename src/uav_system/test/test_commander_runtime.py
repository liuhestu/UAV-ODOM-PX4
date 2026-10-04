"""Commander watchdog/jump checks with isolated in-memory ROS substitutes."""
import importlib.util
import threading
import types
import unittest
from unittest.mock import patch
import yaml
from support import ROOT
from uav_core.mission import Mission


class CommanderRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.cfg=yaml.safe_load((ROOT/'config/commander.yaml').read_text())
        self.cfg['auto_arm']=True
        self.m=Mission(self.cfg); self.m.session='s'; self.m.preparation_mode='OFFBOARD'
        self.m.hold=(0,0,0,0);self.m.target=self.m.hold
        self.clock=0.;self.tick=None;self.commands=[];self.threads=[]
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
        def publisher(topic,*args,**kwargs):
            return ns(publish=lambda msg:self.commands.append(msg) if topic=='/uav/command/trajectory' else None)
        fake.Publisher=publisher
        class Box:
            lock=threading.RLock()
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
                            'mavros_msgs.msg':{'State':ns,'ExtendedState':ns},
                            'mavros_msgs.srv':{'SetMode':ns,'CommandBool':ns},
                            'uav_system.msg':{'SystemStatus':ns}}.items():
            module=types.ModuleType(name)
            for key,value in attrs.items():setattr(module,key,value)
            modules[name]=module
        spec=importlib.util.spec_from_file_location('commander_under_test',ROOT/'src/commander/takeoff_hover_land.py')
        self.module=importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules',modules):spec.loader.exec_module(self.module)
        self.module.Mission=lambda cfg:self.m
        self.module.main()
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
        self.m.enter('TAKEOFF',0);self.data['extended'].landed_state=2
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
