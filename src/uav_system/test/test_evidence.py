import copy
import importlib.util
import types
import unittest
from unittest.mock import patch
from support import ROOT
from flight_supervisor.checks import (TOPICS, SINGLE_EKF_PARAMETERS, parse_listener,
                               evaluate_listener, inspect_listener, advance_reset_counter, verify_single_ekf_parameters)


# Import the actual observer with inert ROS substitutes; main() is never called.
modules={}
for name,fields in {'rospy':(), 'mavros_msgs.msg':('Mavlink','State'),
                    'mavros_msgs.srv':('ParamGet','ParamPull'),
                    'uav_system.msg':('SourceStatus','EvStatus')}.items():
    module=types.ModuleType(name)
    for field in fields:setattr(module,field,object)
    modules[name]=module
spec=importlib.util.spec_from_file_location('observer_protocol_under_test',
                                          ROOT/'src/flight_supervisor/px4_ev_observer.py')
observer=importlib.util.module_from_spec(spec)
with patch.dict('sys.modules',modules):spec.loader.exec_module(observer)
encode=observer.encode


def sample():
    aid=dict(timestamp=10000000,time_last_fuse=9900000,fused=True,innovation_rejected=False,estimator_instance=0)
    return dict(zip(TOPICS,[dict(timestamp=10000000,timestamp_sample=9900000),aid,dict(aid),
        dict(timestamp=10010000,xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True,
             xy_reset_counter=1,z_reset_counter=2,vxy_reset_counter=3,vz_reset_counter=4,heading_reset_counter=5),
        dict(timestamp=10000000,cs_ev_pos=True,cs_ev_hgt=True,cs_fake_pos=False,
             cs_valid_fake_pos=False,cs_inertial_dead_reckoning=False)]))


class EvidenceTests(unittest.TestCase):
    def test_startup_retained_counters_do_not_establish_reset_baseline(self):
        previous=None
        for received,fused in ((False,False),(True,False),(False,True)):
            previous,changed=advance_reset_counter(previous,10,received,fused)
            self.assertIsNone(previous)
            self.assertFalse(changed)
        previous,changed=advance_reset_counter(previous,11,True,True)
        self.assertEqual(previous,11)
        self.assertFalse(changed)

    def test_reset_after_fusion_is_fault_even_during_fusion_loss(self):
        for received,fused in ((True,True),(True,False),(False,False)):
            current,changed=advance_reset_counter(11,12,received,fused)
            self.assertEqual(current,12)
            self.assertTrue(changed)
        self.assertEqual(advance_reset_counter(11,11,False,False),(11,False))

    def test_single_ekf_sensor_voting_without_optional_mag_parameter(self):
        verify_single_ekf_parameters(dict(SINGLE_EKF_PARAMETERS))

    def test_single_ekf_missing_unknown_or_multi_parameters_rejected(self):
        for name in SINGLE_EKF_PARAMETERS:
            for value in (None, -1, 2, True, '1'):
                parameters=dict(SINGLE_EKF_PARAMETERS);parameters[name]=value
                with self.subTest(name=name,value=value),self.assertRaises(ValueError):
                    verify_single_ekf_parameters(parameters)
            parameters=dict(SINGLE_EKF_PARAMETERS);del parameters[name]
            with self.assertRaises(ValueError):verify_single_ekf_parameters(parameters)
        parameters=dict(SINGLE_EKF_PARAMETERS);parameters['SENS_IMU_MODE']=0
        with self.assertRaises(ValueError):verify_single_ekf_parameters(parameters)
        parameters=dict(SINGLE_EKF_PARAMETERS);parameters['SENS_MAG_MODE']=0
        with self.assertRaises(ValueError):verify_single_ekf_parameters(parameters)

    def test_real_format_and_fail_closed(self):
        text='listener estimator_aid_src_ev_pos -n 1 -i 0\nTOPIC: estimator_aid_src_ev_pos instance 0 #1\n timestamp: 10000000 (0.01 seconds ago)\n time_last_fuse: 9999999\n fused: True\n innovation_rejected: False\n estimator_instance: 0\nnsh> '
        parsed=parse_listener(TOPICS[1],text)
        self.assertIs(parsed['fused'],True);self.assertEqual(parsed['time_last_fuse'],9999999)
        with self.assertRaises(ValueError):parse_listener(TOPICS[1],'listener estimator_aid_src_ev_pos\nnot found\nnsh>')
    def test_numeric_px4_booleans(self):
        text='TOPIC: estimator_aid_src_ev_pos instance 0 #1\n fused: 1\n innovation_rejected: 0\n'
        parsed=parse_listener(TOPICS[1],text)
        self.assertIs(parsed['fused'],True);self.assertIs(parsed['innovation_rejected'],False)
    def test_additional_firmware_boolean_fields_do_not_break_parsing(self):
        text=('TOPIC: estimator_aid_src_ev_pos instance 0 #1\n'
              ' timestamp: 10000000\n fusion_enabled: False\n'
              ' fused: True\n innovation_rejected: False\n')
        parsed=parse_listener(TOPICS[1],text)
        self.assertIs(parsed['fusion_enabled'],False)
        self.assertIs(parsed['fused'],True)
        self.assertEqual(parsed['timestamp'],10000000)
    def test_fresh_fused_and_reset_encoding(self):
        received,fused,reset=evaluate_listener(sample(),2)
        self.assertTrue(received);self.assertTrue(fused);self.assertEqual(reset,0x0504030201)
    def test_later_async_status_publication_is_not_future_data(self):
        s=sample()
        s[TOPICS[4]]['timestamp']=s[TOPICS[3]]['timestamp']+238
        self.assertEqual(inspect_listener(s,2)[3],[])
        self.assertTrue(evaluate_listener(s,2)[1])
    def test_newer_status_does_not_hide_expired_local_or_aiding_samples(self):
        s=sample()
        s[TOPICS[4]]['timestamp']=s[TOPICS[3]]['timestamp']+2000001
        received,fused,_,failures=inspect_listener(s,2)
        self.assertFalse(received)
        self.assertFalse(fused)
        self.assertIn('vehicle_local_position.timestamp=10010000',failures)
        self.assertTrue(any('time_last_fuse' in failure for failure in failures))
    def test_failure_details_distinguish_position_height_and_local_validity(self):
        s=sample()
        s[TOPICS[1]]['innovation_rejected']=True
        s[TOPICS[3]]['v_z_valid']=False
        received,fused,reset,failures=inspect_listener(s,2)
        self.assertTrue(received)
        self.assertFalse(fused)
        self.assertEqual(failures,['estimator_aid_src_ev_pos.innovation_rejected=True',
                                   'vehicle_local_position.v_z_valid=False'])
        self.assertEqual((received,fused,reset),evaluate_listener(s,2))
    def test_failure_details_report_height_fusion_and_stale_input(self):
        s=sample()
        s[TOPICS[2]]['fused']=False
        s[TOPICS[0]]['timestamp_sample']=1
        received,fused,_,failures=inspect_listener(s,2)
        self.assertFalse(received)
        self.assertFalse(fused)
        self.assertIn('vehicle_visual_odometry.timestamp_sample=1',failures)
        self.assertIn('estimator_aid_src_ev_hgt.fused=False',failures)
    def test_stale_rejected_other_instance_never_fused(self):
        for field,value in [('fused',False),('innovation_rejected',True),('estimator_instance',1),('time_last_fuse',1)]:
            s=sample();s[TOPICS[1]]=dict(s[TOPICS[1]]);s[TOPICS[1]][field]=value
            self.assertFalse(evaluate_listener(s,2)[1])
    def test_missing_field_is_not_inferred(self):
        s=sample();del s[TOPICS[3]]['xy_valid']
        with self.assertRaises(KeyError):evaluate_listener(s,2)
    def test_old_input_not_received(self):
        s=sample();s[TOPICS[0]]['timestamp_sample']=1;self.assertFalse(evaluate_listener(s,2)[0])
    def test_fake_stale_or_missing_control_state_never_fused(self):
        for name,value in [('cs_ev_pos',False),('cs_ev_hgt',False),('cs_fake_pos',True),
                           ('cs_valid_fake_pos',True),('cs_inertial_dead_reckoning',True),('timestamp',1)]:
            s=sample();s[TOPICS[4]][name]=value
            self.assertFalse(evaluate_listener(s,2)[1])
        s=sample();del s[TOPICS[4]]['cs_fake_pos']
        with self.assertRaises(KeyError):evaluate_listener(s,2)
    def test_serial_control_against_official_pymavlink(self):
        from pymavlink.dialects.v10 import common
        data='listener vehicle_visual_odometry -n 1 -i 0\n'
        payload,crc=encode(data,17,245,191)
        mav=common.MAVLink(None,srcSystem=245,srcComponent=191);mav.seq=17
        msg=common.MAVLink_serial_control_message(10,6,0,0,len(data),list(data.encode())+[0]*(70-len(data)))
        packed=msg.pack(mav)
        self.assertEqual(payload,packed[6:-2]);self.assertEqual(crc,int.from_bytes(packed[-2:],'little'))
