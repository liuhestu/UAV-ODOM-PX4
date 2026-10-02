import unittest
from support import ROOT
from uav_core.fcu_guard import FcuGuard


class FcuGuardTests(unittest.TestCase):
    def test_initial_disconnect_allowed_but_connection_loss_is_latched(self):
        guard=FcuGuard();guard.observe_connection(False)
        self.assertEqual(guard.fault,'')
        guard.observe_connection(True);guard.observe_connection(False)
        self.assertIn('connection interrupted',guard.fault)
        guard.observe_connection(True)
        self.assertIn('connection interrupted',guard.fault)

    def test_reboot_without_disconnected_heartbeat_is_latched(self):
        for message_id in (2,30,31,32):
            guard=FcuGuard();guard.observe_connection(True)
            guard.observe_boot(1,1,message_id,100000)
            guard.observe_boot(1,1,message_id,100)
            self.assertIn('boot clock reset',guard.fault)
            guard.observe_boot(1,1,message_id,200)
            self.assertIn('boot clock reset',guard.fault)

    def test_other_sources_stream_order_and_wrap_do_not_trigger(self):
        guard=FcuGuard();guard.observe_boot(1,1,32,10000)
        guard.observe_boot(1,1,30,100)
        guard.observe_boot(1,1,32,9990)
        guard.observe_boot(2,1,32,1);guard.observe_boot(1,191,32,1)
        guard.observe_boot(1,1,99,1)
        self.assertEqual(guard.fault,'')
        guard=FcuGuard();guard.observe_boot(1,1,32,0xfffffff0)
        guard.observe_boot(1,1,32,20)
        self.assertEqual(guard.fault,'')

    def test_invalid_target_identity_is_rejected(self):
        for value in (0,256,True,'1'):
            with self.assertRaises(ValueError):FcuGuard(value,1)
