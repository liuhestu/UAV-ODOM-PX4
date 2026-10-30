"""Ground recovery must be bounded and cannot run after armed telemetry."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('px4_prepare', Path(__file__).resolve().parents[1] / 'scripts/prepare_px4.py')
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


class PrepareTests(unittest.TestCase):
    def test_healthy_link_is_not_rebooted(self):
        conn = Mock()
        with patch.object(prepare, 'observe', return_value=(True, 11)):
            prepare.recover(lambda: conn, Mock(), False, Mock(), wait=Mock())
        conn.mav.command_long_send.assert_not_called()
        conn.close.assert_called_once()

    def test_silent_link_requires_ground_confirmation(self):
        conn = Mock()
        with patch.object(prepare, 'observe', return_value=(False, 0)):
            with self.assertRaisesRegex(RuntimeError, 'ground confirmation'):
                prepare.recover(lambda: conn, Mock(), False, Mock(), wait=Mock())
        conn.mav.command_long_send.assert_not_called()
        conn.close.assert_called_once()

    def test_reboot_reconnect_then_stable(self):
        conn = Mock()
        with patch.object(prepare, 'observe', side_effect=[(False, 0), (True, 11)]):
            prepare.recover(lambda: conn, Mock(), True, Mock(), wait=Mock())
        conn.mav.command_long_send.assert_called_once()
        self.assertEqual(conn.close.call_count, 2)

    def test_three_reboots_maximum(self):
        conn = Mock()
        with patch.object(prepare, 'observe', return_value=(False, 0)):
            with self.assertRaisesRegex(RuntimeError, 'exhausted'):
                prepare.recover(lambda: conn, Mock(), True, Mock(), wait=Mock())
        self.assertEqual(conn.mav.command_long_send.call_count, 3)
        self.assertEqual(conn.close.call_count, 4)

    def test_armed_observation_never_reboots(self):
        conn = Mock()
        with patch.object(prepare, 'observe', side_effect=RuntimeError('FCU armed')):
            with self.assertRaisesRegex(RuntimeError, 'armed'):
                prepare.recover(lambda: conn, Mock(), True, Mock(), wait=Mock())
        conn.mav.command_long_send.assert_not_called()
        conn.close.assert_called_once()

    def test_occupied_port_prevents_open(self):
        open_connection = Mock()
        with self.assertRaisesRegex(RuntimeError, 'occupied'):
            prepare.recover(open_connection, Mock(side_effect=RuntimeError('occupied')), True, Mock())
        open_connection.assert_not_called()

    def test_observe_checks_target_armed_bit(self):
        from pymavlink import mavutil
        msg = Mock(autopilot=mavutil.mavlink.MAV_AUTOPILOT_PX4,
                   base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        msg.get_srcSystem.return_value = 1
        msg.get_srcComponent.return_value = 1
        conn = Mock()
        conn.recv_match.return_value = msg
        with self.assertRaisesRegex(RuntimeError, 'armed'):
            prepare.observe(conn, 10, 5)
