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
        self.tick=None; self.published={}
        self.clock=10.0
        self.params=yaml.safe_load((ROOT/'config/px4.yaml').read_text())
        self.params.update(output_enabled=True,simulation_transport=False)
        def response(**kwargs): return types.SimpleNamespace(**kwargs)
        fake=types.ModuleType('rospy')
        fake.init_node=lambda *args: None
        fake.get_param=lambda name, *args: 1.0 if name=='/mavros/setpoint_raw/thrust_scaling' else self.params
        fake.Publisher=lambda topic,*args,**kwargs: response(publish=lambda msg:self.published.setdefault(topic,[]).append(msg),get_num_connections=lambda:1)
        fake.Time=response(now=lambda:0)
        fake.logwarn_throttle=lambda *args:None
        fake.loginfo=lambda *args:None;fake.logwarn=lambda *args:None;fake.logerr=lambda *args:None
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
        runtime.Inbox=Box;runtime.wall_loop=lambda rate,tick:setattr(self,'tick',tick)
        runtime.xyz=lambda v:[v.x,v.y,v.z];runtime.xyzw=lambda v:[v.x,v.y,v.z,v.w]
        modules={'rospy':fake,'uav_core.runtime':runtime}
        for name, attrs in {
            'nav_msgs.msg':['Odometry'], 'geometry_msgs.msg':['PoseStamped'],
            'std_msgs.msg':['Bool'], 'mavros_msgs.msg':['State','ExtendedState','AttitudeTarget'],
            'mavros_msgs.srv':['SetMode','SetModeResponse','CommandBool','CommandBoolResponse'],
            'uav_system.msg':['SourceStatus','SystemStatus']}.items():
            module=types.ModuleType(name)
            for attr in attrs: setattr(module,attr,response)
            modules[name]=module
        modules['std_msgs.msg'].Bool=lambda value:value
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

    def attitude_command(self, thrust=0.1):
        ns=types.SimpleNamespace
        return ns(header=ns(frame_id='px4_local',stamp=0),type_mask=7,
                  orientation=ns(x=0.,y=0.,z=0.,w=1.),body_rate=ns(x=0.,y=0.,z=0.),thrust=thrust)

    def test_attitude_forwarding_cap_mask_and_exclusive_stream(self):
        self.data['fcu'].armed=True
        command=self.attitude_command()
        self.data['attitude']=command
        self.tick()
        output=self.published['/mavros/setpoint_raw/attitude'][-1]
        self.assertEqual(output.thrust,.1);self.assertEqual(output.type_mask,7)
        self.assertEqual(output.header.frame_id,'map')
        self.assertNotIn('/mavros/setpoint_position/local',self.published)
        count=len(self.published['/mavros/setpoint_raw/attitude'])
        for field,value in (('thrust',.31),('thrust',float('nan')),('type_mask',128)):
            cmd=self.attitude_command();setattr(cmd,field,value);self.data['attitude']=cmd
            self.tick()
            self.assertEqual(len(self.published['/mavros/setpoint_raw/attitude']),count)
        self.data['attitude']=self.attitude_command()
        self.data['command']=object()
        self.tick()
        self.assertEqual(len(self.published['/mavros/setpoint_raw/attitude']),count)
        self.assertFalse(self.mode(types.SimpleNamespace(custom_mode='OFFBOARD')).mode_sent)

    def test_attitude_prearm_takeover_scaling_and_health_gates(self):
        self.data['attitude']=self.attitude_command(0.)
        self.data['fcu'].mode='MANUAL'
        self.tick()
        self.assertEqual(len(self.published['/mavros/setpoint_raw/attitude']),1)
        self.data['attitude']=self.attitude_command(.1)
        self.tick();self.assertEqual(len(self.published['/mavros/setpoint_raw/attitude']),1)
        self.data['fcu'].armed=True;self.data['fcu'].mode='OFFBOARD'
        self.data['system'].arm_ready=False
        self.tick();self.assertEqual(len(self.published['/mavros/setpoint_raw/attitude']),1)
        self.data['system'].arm_ready=True
        for scaling in (None,0.,2.,float('nan')):
            self.backend.rospy.get_param=lambda *args:scaling
            self.tick();self.assertEqual(len(self.published['/mavros/setpoint_raw/attitude']),1)

    def test_bad_quaternion_and_body_rate_never_forward(self):
        self.data['fcu'].armed=True
        for kind in ('zero','nonunit','nan','rate'):
            command=self.attitude_command()
            if kind=='zero':command.orientation.w=0
            if kind=='nonunit':command.orientation.w=2
            if kind=='nan':command.orientation.x=float('nan')
            if kind=='rate':command.body_rate.x=.1
            self.data['attitude']=command;self.tick()
        self.assertNotIn('/mavros/setpoint_raw/attitude',self.published)

    def test_only_zero_thrust_cleanup_can_bypass_lost_readiness(self):
        self.data['fcu'].armed=True
        self.data['system'].arm_ready=False
        self.data['attitude']=self.attitude_command(0.)
        self.tick()
        self.assertEqual(len(self.published['/mavros/setpoint_raw/attitude']),1)
        self.data['attitude']=self.attitude_command(.01)
        self.tick()
        self.assertEqual(len(self.published['/mavros/setpoint_raw/attitude']),1)
        self.data['fcu'].mode='MANUAL';self.data['attitude']=self.attitude_command(0.)
        self.tick()
        self.assertEqual(len(self.published['/mavros/setpoint_raw/attitude']),1)
        values=dict(zip(self.arm.__code__.co_freevars,(c.cell_contents for c in self.arm.__closure__)))
        values['p']['output_enabled']=False;self.data['fcu'].mode='OFFBOARD'
        self.tick()
        self.assertEqual(len(self.published['/mavros/setpoint_raw/attitude']),1)
