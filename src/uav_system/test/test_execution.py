import unittest
from mission_executor.execution import ExecutionController as MissionController
from mission_executor.mission_loader import normalize_config

from support import executor_config, takeoff_task

def Mission(cfg):
    return MissionController(cfg, takeoff_task())


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.cfg=executor_config()
        self.cfg['auto_arm']=True
        self.m=Mission(self.cfg);self.fcu=(True,False,'POSCTL');self.local=(0,0,0,0)
    def step(self,t,ready=True,session='s',landed=1,result=None,pending=False,confirmed=True):
        return self.m.step(t,ready,session,self.fcu,self.local,landed,result,pending,confirmed)
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
    def test_start_already_armed_waits(self):
        self.fcu=(True,True,'OFFBOARD');self.step(0);self.assertEqual(self.m.state,'WAIT_SYSTEM')
    def test_prestream_uses_no_service_until_ready_stable(self):
        self.step(0,False);self.step(10);self.step(11,False);self.step(20)
        self.assertEqual(self.m.state,'WAIT_SYSTEM');self.assertIsNone(self.step(22)[1])
    def test_mode_timeout_no_repeat(self):
        self.step(0);self.step(2);self.step(4.6);self.step(13)
        self.assertEqual(self.m.state,'BLOCKED');self.assertEqual(self.step(14),(None,None))
    def test_manual_arm_option(self):
        self.cfg['auto_arm']=False;self.m=Mission(self.cfg)
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

    def test_early_arm_all_stages_and_options(self):
        for auto in (False, True):
            for arm_time in (0, 1, 3, 5):
                for early_offboard in (False, True):
                    with self.subTest(auto=auto, arm_time=arm_time, early_offboard=early_offboard):
                        self.cfg['auto_arm']=auto; self.m=Mission(self.cfg)
                        actions=[]; entered=None; offboard_requested=False
                        for t in (0, 1, 2, 3, 4.6, 4.7, 5, 5.1):
                            mode='OFFBOARD' if early_offboard or offboard_requested else 'POSCTL'
                            self.fcu=(True,t>=arm_time,mode)
                            self.local=(10+t*.01,-3,2+t*.01,.7)
                            previous=self.m.state
                            cmd, action=self.step(t)
                            if action: actions.append(action)
                            if action=='OFFBOARD': offboard_requested=True
                            if self.m.state=='TAKEOFF' and previous!='TAKEOFF':
                                entered=t
                                self.assertEqual(cmd,self.local)
                                self.assertEqual(self.m.hold,self.local)
                                self.assertGreaterEqual(t,4.6)
                            elif previous=='TAKEOFF':
                                self.assertAlmostEqual(cmd[2],self.m.hold[2]+.15*(t-entered))
                            elif previous!='WAIT_SYSTEM': self.assertEqual(cmd,self.local)
                        self.assertEqual(self.m.state,'TAKEOFF')
                        self.assertEqual(actions.count('OFFBOARD'),1)
                        self.assertLessEqual(actions.count('ARM'),1)
                        if not auto or arm_time<=4.7: self.assertNotIn('ARM',actions)

    def test_unknown_or_airborne_initial_ground_recovers(self):
        for landed in (0,2):
            self.m=Mission(self.cfg);self.fcu=(True,True,'OFFBOARD')
            self.assertEqual(self.step(0,landed=landed),(None,None))
            self.assertEqual(self.step(20,landed=landed),(None,None))
            self.assertIn('ON_GROUND',self.m.reason)
            self.step(21);self.step(23);self.assertEqual(self.m.state,'PRESTREAM')

    def test_ready_window_arm_change_does_not_reset_but_session_does(self):
        self.step(0);self.fcu=(True,True,'POSCTL');self.step(1);self.step(2)
        self.assertEqual(self.m.state,'PRESTREAM')
        self.m=Mission(self.cfg);self.step(0);self.step(1,session='new');self.step(2,session='new')
        self.assertEqual(self.m.state,'WAIT_SYSTEM');self.step(3,session='new')
        self.assertEqual(self.m.state,'PRESTREAM')

    def test_manual_wait_has_no_timeout_and_updates_hold(self):
        self.cfg['auto_arm']=False;self.m=Mission(self.cfg)
        self.step(0);self.step(2)
        self.assertEqual(self.step(4.6),(self.local,'OFFBOARD'))
        self.fcu=(True,False,'OFFBOARD');self.step(4.7)
        for t in (5,15,1000):
            self.local=(1+t*.0001,2,3,.5)
            self.assertEqual(self.step(t),(self.local,None))
            self.assertEqual(self.m.state,'WAIT_ARM')
        self.fcu=(True,True,'OFFBOARD');self.assertEqual(self.step(1001),(self.local,None))
        self.assertEqual(self.m.state,'TAKEOFF')

    def test_already_offboard_still_requests_and_requires_new_state(self):
        self.fcu=(True,False,'OFFBOARD')
        self.step(0);self.step(2)
        self.assertEqual(self.step(4.6),(self.local,'OFFBOARD'))
        self.assertEqual(self.m.state,'WAIT_OFFBOARD')
        self.assertEqual(self.step(4.7,result=('OFFBOARD',True),confirmed=False),(self.local,None))
        self.assertEqual(self.m.state,'WAIT_OFFBOARD')
        self.assertEqual(self.step(4.8,confirmed=True),(self.local,'ARM'))
        self.assertEqual(self.step(4.9),(self.local,None))

    def test_initial_connection_wait_but_later_loss_latches(self):
        self.fcu=None;self.step(0,ready=False)
        self.assertEqual(self.m.state,'WAIT_SYSTEM')
        self.fcu=(True,False,'POSCTL');self.step(1,ready=False)
        self.fcu=(False,False,'POSCTL');self.assertEqual(self.step(2),(None,None))
        self.assertEqual(self.m.state,'ABORTED')
        self.fcu=(True,False,'OFFBOARD');self.assertEqual(self.step(20),(None,None))

    def test_connection_loss_every_phase_never_rearms_after_reconnect(self):
        for phase in ('WAIT_SYSTEM','PRESTREAM','WAIT_OFFBOARD','WAIT_ARM','TAKEOFF','HOVER',
                      'WAIT_LAND_MODE','WAIT_LAND','WAIT_DISARM','ABORT_LAND_MODE','ABORT_LAND'):
            with self.subTest(phase=phase):
                self.m=Mission(self.cfg);self.fcu=(True,False,'OFFBOARD')
                self.step(0,ready=False);self.m.enter(phase,1)
                self.fcu=None;self.assertEqual(self.step(2),(None,None))
                self.assertEqual(self.m.state,'ABORTED')
                self.fcu=(True,False,'OFFBOARD')
                self.assertEqual(self.step(100),(None,None))

    def test_manual_arm_health_failure_never_requests_offboard(self):
        self.cfg['auto_arm']=False;self.m=Mission(self.cfg)
        self.step(0);self.step(2);self.step(4.6)
        self.fcu=(True,True,'POSCTL')
        self.assertEqual(self.step(5,ready=False),(None,None))
        self.assertEqual(self.m.state,'ABORTED')
        self.assertEqual(self.step(6),(None,None))

    def test_preparation_faults_stop_without_landing_or_resume(self):
        for phase in ('PRESTREAM','WAIT_OFFBOARD','WAIT_ARM'):
            for fault in ('ground','health','local','connection','session','mode','disarm'):
                with self.subTest(phase=phase,fault=fault):
                    self.m=Mission(self.cfg);self.fcu=(True,True,'POSCTL');self.local=(1,2,3,.4)
                    self.step(0);self.step(2)
                    if phase!='PRESTREAM': self.step(4.6)
                    if phase=='WAIT_ARM':
                        # Hold the diagnostic phase while injecting its next observation.
                        self.m.enter('WAIT_ARM',4.7);self.m.preparation_mode='OFFBOARD'
                        self.fcu=(True,True,'OFFBOARD')
                    kwargs={}
                    if fault=='ground': kwargs['landed']=0
                    if fault=='health': kwargs['ready']=False
                    if fault=='local': self.local=None
                    if fault=='connection': self.fcu=None
                    if fault=='session': kwargs['session']='new'
                    if fault=='mode': self.fcu=(True,True,'ALTCTL')
                    if fault=='disarm': self.fcu=(True,False,self.fcu[2])
                    self.assertEqual(self.step(5,**kwargs),(None,None))
                    self.assertIn(self.m.state,self.m.TERMINAL)
                    self.fcu=(True,True,'OFFBOARD');self.local=(1,2,3,.4)
                    self.assertEqual(self.step(10),(None,None))

    def test_actual_state_overrides_service_rejection(self):
        self.step(0);self.step(2);self.step(4.6)
        self.fcu=(True,True,'OFFBOARD')
        self.assertEqual(self.step(4.7,result=('OFFBOARD',False)),(self.local,None))
        self.assertEqual(self.m.state,'TAKEOFF')
        self.m=Mission(self.cfg);self.fcu=(True,False,'POSCTL');self.reach_arm()
        self.fcu=(True,True,'OFFBOARD');self.step(4.8,result=('ARM',False))
        self.assertEqual(self.m.state,'TAKEOFF')

    def test_service_acceptance_is_not_actual_confirmation(self):
        self.step(0);self.step(2);self.step(4.6);self.step(4.7,result=('OFFBOARD',True))
        self.assertEqual(self.m.state,'WAIT_OFFBOARD')
        self.fcu=(True,False,'OFFBOARD');self.step(4.8);self.step(4.9,result=('ARM',True))
        self.assertEqual(self.m.state,'WAIT_ARM');self.step(13)
        self.assertEqual(self.m.state,'BLOCKED');self.assertEqual(self.step(14),(None,None))

    def test_offboard_rejected_latches(self):
        self.step(0);self.step(2);self.step(4.6);self.step(4.7,result=('OFFBOARD',False))
        self.assertEqual(self.m.state,'BLOCKED')

    def test_disarm_during_initialization_latches(self):
        self.fcu=(True,True,'POSCTL');self.step(0,ready=False)
        self.fcu=(True,False,'POSCTL');self.assertEqual(self.step(1),(None,None))
        self.assertEqual(self.m.state,'ABORTED')


class ConfigTests(unittest.TestCase):
    def test_default_and_boolean_validation(self):
        self.assertFalse(normalize_config({})['auto_arm'])
        for value in (False,True): self.assertIs(normalize_config({'auto_arm':value})['auto_arm'],value)
        for value in ('false',0,1,None):
            with self.assertRaises(ValueError): normalize_config({'auto_arm':value})
        for value in ('true','false'):
            self.assertEqual(normalize_config({'auto_arm_override':value})['auto_arm'],value=='true')
        with self.assertRaises(ValueError): normalize_config({'auto_arm_override':'yes'})

    def test_legacy_mapping_warning_and_override(self):
        for old in ('auto','manual'):
            for cfg in ({'arm_method':old},{'auto_arm':False,'arm_method_override':old}):
                warnings=[];actual=normalize_config(cfg,warnings.append)
                self.assertEqual(actual['auto_arm'],old=='auto');self.assertNotIn('arm_method',actual)
                self.assertEqual(len(warnings),1)
        self.assertTrue(normalize_config({'auto_arm':False,'auto_arm_override':'true'})['auto_arm'])

    def test_conflicts_and_unknown_legacy_rejected(self):
        for cfg in ({'auto_arm':False,'arm_method':'manual'},
                    {'auto_arm_override':'false','arm_method_override':'auto'},
                    {'arm_method':'auto','auto_arm_override':'true'},
                    {'arm_method':'invalid'},{'arm_method_override':'invalid'}):
            with self.subTest(cfg=cfg),self.assertRaises(ValueError): normalize_config(cfg)
