"""Attitude rig envelope, lifecycle and fault regression without hardware."""
import math
import unittest
from support import ROOT
from mission_executor.mission_loader import load_task
from mission_executor.execution import AttitudeCommand, ExecutionController, TaskUpdate


class StandTests(unittest.TestCase):
    def setUp(self):
        self.loaded=load_task(ROOT,'rig_attitude_hold')
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
        for now,thrust in ((0,0),(1,.1/3),(3,.1),(7.99,.1),(8,.1),(9.5,.05),(11,0),(30,0)):
            update=self.task.step(now,(100,200,300,-1))
            self.assertAlmostEqual(update.target.thrust,thrust)
            self.assertEqual(update.target.orientation,(0.,0.,math.sin(.3),math.cos(.3)))
            self.assertEqual(update.done,now>=11)

    def test_actual_arm_then_zero_thrust_finish_land_ground_disarm(self):
        self.activate()
        self.assertAlmostEqual(self.step(5.9)[0].thrust,.1/3)
        self.assertEqual(self.m.state,'RIG_RAMP_UP')
        self.assertAlmostEqual(self.step(7.9)[0].thrust,.1)
        command,action=self.step(15.9)
        self.assertEqual(command.thrust,0.);self.assertIsNone(action)
        self.assertEqual(self.m.state,'WAIT_GROUND')
        self.assertIsNone(self.step(16,landed=2)[1])
        self.assertIsNone(self.step(16.1,landed=0)[1])
        self.assertEqual(self.step(16.2)[1],'DISARM')
        self.fcu=(True,False,'OFFBOARD');self.step(16.3)
        self.assertEqual(self.m.state,'DONE')
        self.assertEqual(self.step(30),(None,None))

    def test_invalid_kind_quaternion_thrust_and_completion_abort_once(self):
        for target,done in ((self.local,False),(AttitudeCommand((0,0,0,2),.1),False),
                            (AttitudeCommand((0,0,0,1),.31),False),
                            (AttitudeCommand((0,0,0,1),float('nan')),False),
                            (AttitudeCommand((0,0,0,1),.1),True)):
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
        self.step(15.9)
        command,action=self.step(16,landed=0)
        self.assertEqual(command.thrust,0.);self.assertIsNone(action)
        self.assertEqual(self.step(80,landed=0),(None,None))
        self.assertEqual(self.m.state,'ABORTED')
