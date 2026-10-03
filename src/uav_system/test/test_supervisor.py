import unittest
from types import SimpleNamespace
from support import ROOT
from flight_supervisor.checks import evaluate, estimator_valid, estimator_failures, HealthReporter


class SupervisorTests(unittest.TestCase):
    def test_health_report_initial_failure_recovery_and_no_tick_spam(self):
        reporter = HealthReporter()
        checks = [('FCU connected', True), ('EV fusion', False)]
        report = reporter.update(checks, False, False)
        self.assertIn('[PASS] FCU connected', report)
        self.assertIn('[FAIL] EV fusion', report)
        for _ in range(100):
            self.assertIsNone(reporter.update(checks, False, False))
        recovered = [('FCU connected', True), ('EV fusion', True)]
        self.assertIn('[PASS] EV fusion', reporter.update(recovered, True, True))
        self.assertIsNone(reporter.update(recovered, True, True))
        self.assertIn('[FAIL] EV fusion', reporter.update(checks, False, False))

    def test_health_report_ground_and_arm_readiness_changes_are_visible(self):
        reporter = HealthReporter()
        ground = [('ON_GROUND', True)]
        reporter.update(ground, True, False)
        self.assertIn('arm_ready=True', reporter.update(ground, True, True))
        self.assertIn('[FAIL] ON_GROUND', reporter.update([('ON_GROUND', False)], True, True))

    def test_real_external_fusion_disambiguates_at_rest_without_landing_dependency(self):
        flags=dict(attitude_status_flag=True,pos_horiz_rel_status_flag=True,
                   pos_vert_abs_status_flag=True,velocity_horiz_status_flag=True,
                   velocity_vert_status_flag=True,accel_error_status_flag=False,
                   const_pos_mode_status_flag=True)
        ekf=SimpleNamespace(**flags)
        self.assertFalse(estimator_valid(ekf,False))
        self.assertTrue(estimator_valid(ekf,True))
        self.assertFalse(estimator_valid(None,True))
        for name in flags:
            if name=='const_pos_mode_status_flag':continue
            bad=dict(flags);bad[name]=not bad[name]
            self.assertFalse(estimator_valid(SimpleNamespace(**bad),True),name)
            self.assertTrue(any(name in failure for failure in estimator_failures(SimpleNamespace(**bad),True)))
    def test_every_gate_required(self):
        gates={k:True for k in ('source','adapter','ev_sent','ev_received','ev_fused','ekf_valid','fcu_connected')}
        self.assertEqual(evaluate(gates,True,False,False),(True,True,[]))
        for name in gates:
            g=dict(gates);g[name]=False
            ready,arm,reasons=evaluate(g,True,False,False)
            self.assertFalse(ready);self.assertFalse(arm);self.assertIn(name,reasons)
    def test_summary_status_and_sensor_bitmap_are_diagnostic_only(self):
        gates = {'source': True, 'adapter': True, 'ev_fused': True,
                 'ekf_valid': True, 'fcu_connected': True,
                 'PX4 critical/unknown status': False,
                 'PX4 sensor health unavailable/failed': False}
        self.assertEqual(evaluate(gates, True, False, False), (True, True, []))
        for core in ('source', 'adapter', 'ev_fused', 'ekf_valid', 'fcu_connected'):
            failed = dict(gates, **{core: False})
            self.assertEqual(evaluate(failed, True, False, False), (False, False, [core]))
        self.assertEqual(evaluate(gates, False, False, False)[:2], (True, False))
    def test_mock_on_hardware_cannot_arm(self):
        self.assertEqual(evaluate({'data':True},True,True,False)[:2],(True,False))
        self.assertEqual(evaluate({'data':True},True,True,True)[:2],(True,True))
    def test_uncalibrated_blocks_arm(self):
        self.assertEqual(evaluate({'data':True},False,False,False)[:2],(True,False))
