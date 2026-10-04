import unittest
import yaml
from test_support import ROOT
from uav_core.mission import Mission


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.cfg=yaml.safe_load((ROOT/'src/uav_system/config/commander.yaml').read_text())
        self.m=Mission(self.cfg);self.fcu=(True,False,'POSCTL');self.local=(0,0,0,0)
    def step(self,t,ready=True,session='s',landed=1,result=None,pending=False):
        return self.m.step(t,ready,session,self.fcu,self.local,landed,result,pending)
    def reach_arm(self):
        self.step(0);self.step(2);self.step(4.6)
        self.fcu=(True,False,'OFFBOARD');self.step(4.7,result=('OFFBOARD',True))
    def test_full_mission_services_once_and_confirmed(self):
        self.assertEqual(self.step(0),(None,None));self.step(2)
        self.assertEqual(self.m.state,'PRESTREAM')
        self.assertEqual(self.step(4.6)[1],'OFFBOARD')
        self.assertIsNone(self.step(4.65)[1])
        self.fcu=(True,False,'OFFBOARD');self.assertEqual(self.step(4.7,result=('OFFBOARD',True))[1],'ARM')
        self.assertIsNone(self.step(4.8)[1]);self.assertEqual(self.m.state,'WAIT_ARM')
        self.fcu=(True,True,'OFFBOARD');self.step(4.9,result=('ARM',True));self.assertEqual(self.m.state,'TAKEOFF')
        self.local=(0,0,.5,0);cmd,act=self.step(8.4,landed=2);self.assertAlmostEqual(cmd[2],.5);self.assertEqual(self.m.state,'HOVER')
        self.assertEqual(self.step(13.5,landed=2)[1],'LAND')
        self.fcu=(True,True,'AUTO.LAND');self.step(13.6,landed=2,result=('LAND',True))
        self.assertEqual(self.step(16,landed=1)[1],'DISARM');self.assertEqual(self.m.state,'WAIT_DISARM')
        self.step(16.1,result=('DISARM',True));self.assertEqual(self.m.state,'WAIT_DISARM')
        self.fcu=(True,False,'AUTO.LAND');self.step(16.2);self.assertEqual(self.m.state,'DONE')
        self.assertEqual(self.step(20),(None,None))
    def test_rejected_arm_latches(self):
        self.reach_arm();self.step(4.8,result=('ARM',False));self.assertEqual(self.m.state,'BLOCKED')
        self.assertEqual(self.step(9),(None,None))
    def test_readiness_loss_no_auto_resume(self):
        self.reach_arm();self.fcu=(True,True,'OFFBOARD');self.step(4.8,result=('ARM',True))
        self.assertEqual(self.step(5,False)[1],'LAND');self.assertEqual(self.m.state,'ABORT_LAND_MODE')
        self.fcu=(True,True,'AUTO.LAND');self.step(5.1,False,landed=2)
        self.step(8,landed=1);self.assertEqual(self.m.state,'ABORTED')
        self.assertEqual(self.step(10),(None,None))
    def test_manual_takeover_is_not_overridden(self):
        self.reach_arm();self.fcu=(True,True,'OFFBOARD');self.step(4.8,result=('ARM',True))
        self.fcu=(True,True,'POSCTL');self.assertEqual(self.step(5),(None,None));self.assertEqual(self.m.state,'TAKEN_OVER')
    def test_session_reset_aborts(self):
        self.reach_arm();self.fcu=(True,True,'OFFBOARD');self.step(4.8,result=('ARM',True))
        self.assertEqual(self.step(5,session='other')[1],'LAND')
    def test_services_do_not_progress_while_pending(self):
        self.step(0);self.step(2);self.step(4.6);self.fcu=(True,False,'OFFBOARD')
        self.assertIsNone(self.step(4.7,pending=True)[1]);self.assertEqual(self.m.state,'WAIT_OFFBOARD')
        self.assertEqual(self.step(4.8,result=('OFFBOARD',True))[1],'ARM')
    def test_start_already_armed_blocks(self):
        self.fcu=(True,True,'OFFBOARD');self.step(0);self.assertEqual(self.m.state,'BLOCKED')
    def test_prestream_uses_no_service_until_ready_stable(self):
        self.step(0,False);self.step(10);self.step(11,False);self.step(20)
        self.assertEqual(self.m.state,'WAIT_SYSTEM');self.assertIsNone(self.step(22)[1])
    def test_mode_timeout_no_repeat(self):
        self.step(0);self.step(2);self.step(4.6);self.step(13)
        self.assertEqual(self.m.state,'BLOCKED');self.assertEqual(self.step(14),(None,None))
    def test_manual_arm_option(self):
        self.cfg['arm_method']='manual';self.m=Mission(self.cfg)
        self.step(0);self.step(2);self.step(4.6);self.fcu=(True,False,'OFFBOARD')
        self.assertIsNone(self.step(4.7,result=('OFFBOARD',True))[1]);self.assertEqual(self.m.state,'WAIT_ARM')
    def test_takeoff_timeout_requests_land(self):
        self.reach_arm();self.fcu=(True,True,'OFFBOARD');self.step(4.8,result=('ARM',True))
        self.assertEqual(self.step(26,landed=2)[1],'LAND')
    def test_landing_never_disarms_while_airborne(self):
        self.m.enter('WAIT_LAND',0);self.fcu=(True,True,'AUTO.LAND')
        self.assertIsNone(self.step(2,landed=2)[1]);self.assertEqual(self.m.state,'WAIT_LAND')
    def test_pending_request_readiness_loss_latches(self):
        self.reach_arm();self.assertEqual(self.step(5,False,pending=True),(None,None));self.assertEqual(self.m.state,'ABORTED')
