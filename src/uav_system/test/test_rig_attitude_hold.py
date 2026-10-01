"""Attitude rig envelope, lifecycle and fault regression without hardware."""
import math
import unittest
from support import ROOT, load_fixture_task
from mission_executor.mission_loader import load_task
from mission_executor.execution import AttitudeCommand, ExecutionController, TaskUpdate
from mission_executor.shutdown import ThrustStop


class StandTests(unittest.TestCase):
    def setUp(self):
        self.loaded=load_fixture_task('rig_attitude_hold')
        self.task=self.loaded.task
        self.m=ExecutionController(self.loaded.executor_config,self.task)
        self.local=(1.,2.,3.,.6)
        self.fcu=(True,False,'MANUAL')

    def step(self,now,ready=True,session='s',landed=1):
        return self.m.step(now,ready,session,self.fcu,self.local,landed,offboard_confirmed=True)

    def activate(self):
        self.step(0);self.step(2)
        command,action=self.step(4.6)
        self.assertIsInstance(command,AttitudeCommand);self.assertEqual(command.thrust,0.)
        self.assertEqual(action,'OFFBOARD')
        self.fcu=(True,False,'OFFBOARD')
        self.assertEqual(self.step(4.7)[1],'ARM')
        self.fcu=(True,True,'OFFBOARD')
        command,action=self.step(4.9)
        self.assertEqual(command.thrust,0.);self.assertIsNone(action)

    def test_level_attitude_yaw_fixed_and_bounded_envelope(self):
        self.task.start(0,self.local)
        finish=self.task.up+self.task.hold
        for now,thrust in ((0,0),(1,.1/3),(3,.1),(finish-.01,.1),(finish,.1),(30,.1)):
            update=self.task.step(now,(100,200,300,-1))
            self.assertAlmostEqual(update.target.thrust,thrust)
            self.assertEqual(update.target.orientation,(0.,0.,math.sin(.3),math.cos(.3)))
            self.assertEqual(update.done,now>=finish)

    def test_normal_shutdown_monotonic_with_smooth_endpoints_and_zero_dwell(self):
        stop=ThrustStop(2.0,0.5)
        start=8.0;end=10.0
        stop.start(start,.1)
        values=[stop.step(start+2.0*i/100)[0] for i in range(101)]
        self.assertTrue(all(0<=value<=self.task.peak_thrust for value in values))
        self.assertTrue(all(a>=b for a,b in zip(values,values[1:])))
        delta=.001
        self.assertLess((.1-stop.step(start+delta)[0])/delta,.0001)
        self.assertLess(stop.step(end-delta)[0]/delta,.0001)
        for now in (end,end+.25,end+.499):
            self.assertEqual(stop.step(now),(0.0,'ZERO_THRUST',False))
        self.assertTrue(stop.step(end+.5)[2])

    def finish_normally(self):
        stop_start=4.9+self.task.up+self.task.hold+.001
        command,action=self.step(stop_start)
        self.assertEqual(self.m.state,'RAMP_DOWN')
        self.assertAlmostEqual(command.thrust,.1)
        ramp=self.m.thrust_stop.ramp_seconds
        zero=self.m.thrust_stop.zero_seconds
        self.assertAlmostEqual(self.step(stop_start+ramp/2)[0].thrust,.05)
        self.assertEqual(self.step(stop_start+ramp+.001)[0].thrust,0.)
        self.assertEqual(self.m.state,'ZERO_THRUST')
        finish=stop_start+ramp+zero+.001
        command,action=self.step(finish)
        self.assertEqual(command.thrust,0.);self.assertIsNone(action)
        self.assertEqual(self.m.state,'WAIT_GROUND')
        return finish

    def test_actual_arm_then_zero_thrust_finish_land_ground_disarm(self):
        self.activate()
        self.assertAlmostEqual(self.step(5.9)[0].thrust,.1/3)
        self.assertEqual(self.m.state,'RIG_RAMP_UP')
        self.assertAlmostEqual(self.step(7.9)[0].thrust,.1)
        finish=self.finish_normally()
        self.assertIsNone(self.step(finish+.1,landed=2)[1])
        self.assertIsNone(self.step(finish+.2,landed=0)[1])
        self.assertEqual(self.step(finish+.3)[1],'DISARM')
        self.fcu=(True,False,'OFFBOARD');self.step(finish+.4)
        self.assertEqual(self.m.state,'DONE')
        self.assertEqual(self.step(30),(None,None))

    def test_invalid_kind_quaternion_thrust_and_completion_abort_once(self):
        for target,done in ((self.local,False),(AttitudeCommand((0,0,0,2),.1),False),
                            (AttitudeCommand((0,0,0,1),.31),False),
                            (AttitudeCommand((0,0,0,1),float('nan')),False)):
            self.setUp();self.activate()
            self.task.step=lambda now,local:TaskUpdate(target,'RIG_HOLD',done)
            command,action=self.step(5)
            self.assertEqual(command.thrust,0.);self.assertIsNone(action)
            self.assertEqual(self.m.state,'ABORT_GROUND')
            self.assertEqual(self.step(6)[1],'DISARM')
            self.assertIsNone(self.step(7)[1])

    def test_takeover_disarm_disconnect_and_health_failures(self):
        for fcu,ready,session,expected in (
            ((True,True,'MANUAL'),True,'s','TAKEN_OVER'),
            ((True,False,'OFFBOARD'),True,'s','ABORTED'),
            ((False,True,'OFFBOARD'),True,'s','ABORTED'),
            ((True,True,'OFFBOARD'),False,'s','ABORT_GROUND'),
            ((True,True,'OFFBOARD'),True,'new','ABORT_GROUND')):
            self.setUp();self.activate();self.fcu=fcu
            self.step(6,ready,session)
            self.assertEqual(self.m.state,expected)
            if expected in self.m.TERMINAL:
                self.fcu=(True,True,'OFFBOARD')
                self.assertEqual(self.step(30),(None,None))

    def test_peak_thrust_validated_before_startup(self):
        cfg=dict(self.loaded.executor_config,max_attitude_thrust=.05)
        with self.assertRaises(ValueError):ExecutionController(cfg,self.task)
        with self.assertRaises(ValueError):load_task(ROOT,'hover')

    def test_fault_zero_thrust_cleanup_never_lands_or_resumes(self):
        self.activate()
        command,action=self.step(6,ready=False)
        self.assertEqual(command.thrust,0.);self.assertIsNone(action)
        command,action=self.step(7,ready=False,landed=2)
        self.assertEqual(command.thrust,0.);self.assertIsNone(action)
        self.assertEqual(self.step(8,ready=False)[1],'DISARM')
        self.fcu=(True,False,'OFFBOARD')
        self.step(9,ready=False)
        self.assertEqual(self.m.state,'ABORTED')
        self.fcu=(True,True,'OFFBOARD')
        self.assertEqual(self.step(20),(None,None))

    def test_ground_unknown_timeout_does_not_force_disarm(self):
        self.activate()
        self.step(7.9)
        finish=self.finish_normally()
        command,action=self.step(finish+.1,landed=0)
        self.assertEqual(command.thrust,0.);self.assertIsNone(action)
        self.assertEqual(self.step(finish+self.loaded.executor_config['landing_timeout']+1,landed=0),(None,None))
        self.assertEqual(self.m.state,'ABORTED')

    def test_task_completion_zero_cannot_bypass_shared_ramp(self):
        self.activate();self.step(7.9)
        self.task.step=lambda now,local:TaskUpdate(AttitudeCommand((0,0,0,1),0.),'CUSTOM_HOLD',True)
        command,action=self.step(8.)
        self.assertEqual(self.m.state,'RAMP_DOWN')
        self.assertAlmostEqual(command.thrust,.1)
        self.assertIsNone(action)
        self.assertAlmostEqual(self.step(9.)[0].thrust,.05)

    def test_fault_during_shared_shutdown_immediately_zeroes_thrust(self):
        self.activate();self.step(7.9);self.step(13.)
        self.assertEqual(self.m.state,'RAMP_DOWN')
        command,action=self.step(13.1,ready=False)
        self.assertEqual(command.thrust,0.)
        self.assertEqual(self.m.state,'ABORT_GROUND')
        self.assertIsNone(action)

    def test_shared_shutdown_never_continues_after_takeover_or_disconnect(self):
        for fcu,expected in (((True,True,'MANUAL'),'TAKEN_OVER'),((False,True,'OFFBOARD'),'ABORTED')):
            self.setUp();self.activate();self.step(7.9);self.step(13.)
            self.fcu=fcu
            self.assertEqual(self.step(13.1),(None,None))
            self.assertEqual(self.m.state,expected)
            self.fcu=(True,True,'OFFBOARD')
            self.assertEqual(self.step(20),(None,None))
