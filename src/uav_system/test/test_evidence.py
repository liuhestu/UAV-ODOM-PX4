import copy
import unittest
from support import ROOT
from uav_core.evidence import TOPICS, parse_listener, evaluate_listener
from uav_core.mavlink_serial import encode


def sample():
    aid=dict(timestamp=10000000,time_last_fuse=9900000,fused=True,innovation_rejected=False,estimator_instance=0)
    return dict(zip(TOPICS,[dict(timestamp=10000000,timestamp_sample=9900000),aid,dict(aid),
        dict(timestamp=10010000,xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True,
             xy_reset_counter=1,z_reset_counter=2,vxy_reset_counter=3,vz_reset_counter=4,heading_reset_counter=5)]))


class EvidenceTests(unittest.TestCase):
    def test_real_format_and_fail_closed(self):
        text='listener estimator_aid_src_ev_pos -n 1 -i 0\nTOPIC: estimator_aid_src_ev_pos instance 0 #1\n timestamp: 10000000 (0.01 seconds ago)\n time_last_fuse: 9999999\n fused: True\n innovation_rejected: False\n estimator_instance: 0\nnsh> '
        parsed=parse_listener(TOPICS[1],text)
        self.assertIs(parsed['fused'],True);self.assertEqual(parsed['time_last_fuse'],9999999)
        with self.assertRaises(ValueError):parse_listener(TOPICS[1],'listener estimator_aid_src_ev_pos\nnot found\nnsh>')
    def test_numeric_px4_booleans(self):
        text='TOPIC: estimator_aid_src_ev_pos instance 0 #1\n fused: 1\n innovation_rejected: 0\n'
        parsed=parse_listener(TOPICS[1],text)
        self.assertIs(parsed['fused'],True);self.assertIs(parsed['innovation_rejected'],False)
    def test_fresh_fused_and_reset_encoding(self):
        received,fused,reset=evaluate_listener(sample(),2)
        self.assertTrue(received);self.assertTrue(fused);self.assertEqual(reset,0x0504030201)
    def test_stale_rejected_other_instance_never_fused(self):
        for field,value in [('fused',False),('innovation_rejected',True),('estimator_instance',1),('time_last_fuse',1)]:
            s=sample();s[TOPICS[1]]=dict(s[TOPICS[1]]);s[TOPICS[1]][field]=value
            self.assertFalse(evaluate_listener(s,2)[1])
    def test_missing_field_is_not_inferred(self):
        s=sample();del s[TOPICS[3]]['xy_valid']
        with self.assertRaises(KeyError):evaluate_listener(s,2)
    def test_old_input_not_received(self):
        s=sample();s[TOPICS[0]]['timestamp_sample']=1;self.assertFalse(evaluate_listener(s,2)[0])
    def test_serial_control_against_official_pymavlink(self):
        from pymavlink.dialects.v10 import common
        data='listener vehicle_visual_odometry -n 1 -i 0\n'
        payload,crc=encode(data,17,245,191)
        mav=common.MAVLink(None,srcSystem=245,srcComponent=191);mav.seq=17
        msg=common.MAVLink_serial_control_message(10,6,0,0,len(data),list(data.encode())+[0]*(70-len(data)))
        packed=msg.pack(mav)
        self.assertEqual(payload,packed[6:-2]);self.assertEqual(crc,int.from_bytes(packed[-2:],'little'))
