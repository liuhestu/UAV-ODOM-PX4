"""Service boundary tests with no ROS master, MAVROS or FCU connection."""
import importlib.util
import types
import unittest
from unittest.mock import patch
import yaml
from support import ROOT


class BackendTests(unittest.TestCase):
    def setUp(self):
        self.services={}; self.calls=[]; self.data={}
        self.clock=10.0
        self.params=yaml.safe_load((ROOT/'config/px4.yaml').read_text())
        self.params.update(output_enabled=True,simulation_transport=False)
        def response(**kwargs): return types.SimpleNamespace(**kwargs)
        fake=types.ModuleType('rospy')
        fake.init_node=lambda *args: None
        fake.get_param=lambda name, *args: self.params
        fake.Publisher=lambda *args, **kwargs: None
        fake.Service=lambda name, cls, handler: self.services.update({name:handler})
        fake.wait_for_service=lambda *args, **kwargs: None
        def proxy(name, cls):
            def call(**kwargs):
                self.calls.append((name,kwargs))
                return response(success=True,result=0,mode_sent=True)
            return call
        fake.ServiceProxy=proxy
        fake.ROSException=RuntimeError;fake.ServiceException=RuntimeError
        class Box:
            lock=__import__('threading').RLock()
            def subscribe(box,*args): pass
            def get(box,key,timeout): return self.data.get(key)
        runtime=types.ModuleType('uav_core.runtime')
        runtime.Inbox=Box;runtime.wall_loop=lambda *args: None
        runtime.xyz=lambda *args: None;runtime.xyzw=lambda *args: None
        modules={'rospy':fake,'uav_core.runtime':runtime}
        for name, attrs in {
            'nav_msgs.msg':['Odometry'], 'geometry_msgs.msg':['PoseStamped'],
            'std_msgs.msg':['Bool'], 'mavros_msgs.msg':['State','ExtendedState'],
            'mavros_msgs.srv':['SetMode','SetModeResponse','CommandBool','CommandBoolResponse'],
            'uav_system.msg':['SourceStatus','SystemStatus']}.items():
            module=types.ModuleType(name)
            for attr in attrs: setattr(module,attr,response)
            modules[name]=module
        tf=types.ModuleType('tf2_ros');tf.Buffer=lambda:None;tf.TransformListener=lambda *args:None
        modules['tf2_ros']=tf
        spec=importlib.util.spec_from_file_location('backend_under_test',ROOT/'src/control_backend/px4_backend.py')
        self.backend=importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules',modules):
            spec.loader.exec_module(self.backend);self.backend.main()
        self.data.update(fcu=response(connected=True,armed=False,mode='OFFBOARD'),
                         extended=response(landed_state=1),
                         system=response(arm_ready=True,backend_output_enabled=True,
                                         simulation_transport=False,simulated=False))
        self.arm=self.services['/uav/backend/arming']; self.mode=self.services['/uav/backend/set_mode']
        # Seed only the isolated closure's prestream timestamps.
        values=dict(zip(self.mode.__code__.co_freevars,(c.cell_contents for c in self.mode.__closure__)))
        values['stream_start'][0]=0;values['last_stream'][0]=10
        clock=patch.object(self.backend.time,'monotonic',lambda:self.clock)
        clock.start();self.addCleanup(clock.stop)

    def test_offboard_requires_fresh_on_ground(self):
        req=types.SimpleNamespace(custom_mode='OFFBOARD')
        for ext in (None,types.SimpleNamespace(landed_state=0),types.SimpleNamespace(landed_state=2)):
            self.data['extended']=ext
            self.assertFalse(self.mode(req).mode_sent)
        self.assertEqual(self.calls,[])
        self.data['extended']=types.SimpleNamespace(landed_state=1)
        self.assertTrue(self.mode(req).mode_sent)
        self.assertEqual(len(self.calls),1)

    def test_already_armed_never_forwards_redundant_arm(self):
        self.data['fcu'].armed=True
        self.assertTrue(self.arm(types.SimpleNamespace(value=True)).success)
        self.assertEqual(self.calls,[])

    def test_arm_gates_and_airborne_disarm(self):
        req=types.SimpleNamespace(value=True)
        self.data['extended']=None;self.assertFalse(self.arm(req).success)
        self.data['extended']=types.SimpleNamespace(landed_state=2)
        self.assertFalse(self.arm(req).success)
        self.assertFalse(self.arm(types.SimpleNamespace(value=False)).success)
        self.assertEqual(self.calls,[])
        self.data['extended'].landed_state=1
        self.assertTrue(self.arm(req).success)
        self.assertTrue(self.arm(types.SimpleNamespace(value=False)).success)
        self.assertEqual(len(self.calls),2)

    def test_output_disabled_denies_services(self):
        self.params['output_enabled']=False
        # main copies the configuration; set its isolated service closure copy.
        values=dict(zip(self.arm.__code__.co_freevars,(c.cell_contents for c in self.arm.__closure__)))
        values['p']['output_enabled']=False
        self.data['fcu'].armed=True
        self.assertFalse(self.arm(types.SimpleNamespace(value=True)).success)
        self.assertFalse(self.mode(types.SimpleNamespace(custom_mode='OFFBOARD')).mode_sent)
        self.assertEqual(self.calls,[])
