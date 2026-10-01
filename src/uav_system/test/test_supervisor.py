import unittest
from support import ROOT
from uav_core.readiness import evaluate


class SupervisorTests(unittest.TestCase):
    def test_every_gate_required(self):
        gates={k:True for k in ('source','adapter','ev_sent','ev_received','ev_fused','ekf_valid','fcu_connected')}
        self.assertEqual(evaluate(gates,True,False,False),(True,True,[]))
        for name in gates:
            g=dict(gates);g[name]=False
            ready,arm,reasons=evaluate(g,True,False,False)
            self.assertFalse(ready);self.assertFalse(arm);self.assertIn(name,reasons)
    def test_mock_on_hardware_cannot_arm(self):
        self.assertEqual(evaluate({'data':True},True,True,False)[:2],(True,False))
        self.assertEqual(evaluate({'data':True},True,True,True)[:2],(True,True))
    def test_uncalibrated_blocks_arm(self):
        self.assertEqual(evaluate({'data':True},False,False,False)[:2],(True,False))
