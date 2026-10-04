import importlib.util
import types
import unittest
from unittest.mock import patch
from support import ROOT


class FakeStamp:
    def __init__(self,value):self.value=value
    def to_sec(self):return self.value
    def __sub__(self,other):return FakeStamp(self.value-other.value)


class WatchdogTests(unittest.TestCase):
    def setUp(self):
        self.clock=10.0;self.wall=100.0
        fake=types.ModuleType('rospy')
        fake.Time=types.SimpleNamespace(now=lambda:FakeStamp(self.clock))
        spec=importlib.util.spec_from_file_location('watchdog_runtime',ROOT/'src/support/uav_core/python/uav_core/runtime.py')
        self.module=importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules',{'rospy':fake}):spec.loader.exec_module(self.module)
        self.box=self.module.Inbox()
        self.msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=FakeStamp(10.0)))
        self.box.data['data']=(self.msg,100.0)
        self.mock=patch.object(self.module.time,'monotonic',side_effect=lambda:self.wall);self.mock.start()
        self.addCleanup(self.mock.stop)
    def test_fresh_data(self):self.assertIs(self.box.get('data',.5),self.msg)
    def test_wall_timeout_when_ros_clock_pauses(self):
        self.wall=101.0;self.assertIsNone(self.box.get('data',.5))
    def test_old_measurement_with_recent_receipt(self):
        self.clock=11.0;self.assertIsNone(self.box.get('data',.5))
    def test_clock_regression_and_zero_stamp(self):
        self.clock=9.0;self.assertIsNone(self.box.get('data',.5))
        self.clock=10.0;self.msg.header.stamp=FakeStamp(0);self.assertIsNone(self.box.get('data',.5))
    def test_unseen_data(self):self.assertIsNone(self.box.get('missing',.5))
