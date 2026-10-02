"""Task selection and common flight gates, without any ROS/FCU services."""
import tempfile
import unittest
from pathlib import Path
import yaml
from support import ROOT
from mission_executor.mission_loader import ExecutorLock, load_task
from mission_executor.execution import ExecutionController as Mission
from mission_executor.execution import TaskUpdate


class TaskTests(unittest.TestCase):
    def test_source_spellings_and_matching_yaml(self):
        for source in ('hover','hover.py','src/mission/hover.py',str(ROOT/'src/mission/hover.py')):
            loaded = load_task(ROOT,source)
            self.assertEqual(loaded.source_path, (ROOT/'src/mission/hover.py').resolve())
            self.assertEqual(loaded.mission_config_path.name,'hover.yaml')
            self.assertEqual(loaded.mission_config, {'hover_seconds': 5.0})
            self.assertNotIn('hover_seconds',loaded.executor_config)
            self.assertEqual(loaded.task.start(10,(1,2,3,.4)).target,(1,2,3,.4))

    def test_invalid_paths_and_configuration_fail_before_runtime(self):
        for source in ('missing','../../support/uav_core/geometry.py','contract.py'):
            with self.assertRaises((ValueError, ImportError, FileNotFoundError)):
                load_task(ROOT,source)
        with self.assertRaises(FileNotFoundError):load_task(ROOT,'hover','missing.yaml')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'custom.yaml'
            loaded=load_task(ROOT,'hover')
            for key,value in (('hover_seconds',0),('hover_seconds',float('nan')),('auto_arm',False)):
                bad=dict(loaded.mission_config);bad[key]=value;path.write_text(yaml.safe_dump(bad))
                with self.assertRaises(ValueError):load_task(ROOT,'hover',str(path))
            for key,value in (('auto_arm','true'),('rate_hz',True),('fcu_system_id',0)):
                bad=dict(loaded.executor_config);bad[key]=value;path.write_text(yaml.safe_dump(bad))
                with self.assertRaises(ValueError):load_task(ROOT,'hover',executor_config=str(path))
            path.write_text('[]')
            with self.assertRaises(ValueError):load_task(ROOT,'hover',str(path))
            with self.assertRaises(ValueError):load_task(ROOT,'hover',executor_config=str(path))
            path.write_text(yaml.safe_dump(loaded.mission_config))
            self.assertEqual(load_task(ROOT,'hover',str(path)).mission_config,loaded.mission_config)
        with self.assertRaises(ValueError):load_task(ROOT,'hover',auto_arm='true',arm_method='auto')

    def test_invalid_module_contract_and_symlink_escape(self):
        with tempfile.TemporaryDirectory() as directory:
            package=Path(directory)
            tasks=package/'src/mission';tasks.mkdir(parents=True)
            configs=package/'config/mission';configs.mkdir(parents=True)
            loaded=load_task(ROOT,'hover')
            (configs/'bad.yaml').write_text(yaml.safe_dump(loaded.mission_config))
            (package/'config/mission_executor.yaml').write_text(yaml.safe_dump(loaded.executor_config))
            script=tasks/'bad.py'
            for contents in ('value = 1', 'def create_task(config): return object()',
                             'def create_task(config): raise ValueError("bad config")'):
                script.write_text(contents)
                with self.assertRaises(ValueError):load_task(package,'bad')
            script.write_text('from __future__ import annotations\n'
                              'from dataclasses import dataclass\n'
                              '@dataclass\nclass Task:\n    value: float = 1.0\n'
                              '    def start(self, now, origin): pass\n'
                              '    def step(self, now, local): pass\n'
                              'def create_task(config): return Task()\n')
            self.assertEqual(load_task(package,'bad').task.value,1.0)
            outside=package/'outside.py';outside.write_text('value = 1')
            script.unlink();script.symlink_to(outside)
            with self.assertRaises(ValueError):load_task(package,'bad')

    def test_config_paths_and_task_policy_separation(self):
        for path in ('hover.yaml','config/mission/hover.yaml',str(ROOT/'config/mission/hover.yaml')):
            loaded=load_task(ROOT,'hover',path)
            self.assertEqual(loaded.mission_config,{'hover_seconds':5.0})
        for path in ('mission_executor.yaml','config/mission_executor.yaml',str(ROOT/'config/mission_executor.yaml')):
            self.assertTrue(load_task(ROOT,'hover',executor_config=path).executor_config['auto_arm'])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'executor.yaml'
            cfg=dict(load_task(ROOT,'hover').executor_config,auto_arm=False)
            path.write_text(yaml.safe_dump(cfg))
            loaded=load_task(ROOT,'hover',executor_config=str(path))
            self.assertFalse(loaded.executor_config['auto_arm'])
            self.assertEqual(loaded.task.start(0,(1,2,3,0)).target,(1,2,3,0))

    def test_singleton_before_ros_registration_and_release(self):
        lock=ExecutorLock('http://localhost:19129')
        try:
            with self.assertRaisesRegex(RuntimeError,'already running'):
                ExecutorLock('http://127.0.0.1:19129/')
            independent=ExecutorLock('http://localhost:19130');independent.close()
        finally:lock.close()
        replacement=ExecutorLock('http://localhost:19129');replacement.close()

    def active_hover(self, auto=True):
        cfg=load_task(ROOT,'hover',auto_arm='true' if auto else 'false').executor_config
        task=load_task(ROOT,'hover').task
        mission=Mission(cfg,task)
        local=(1.,2.,3.,.4)
        mission.step(0,True,'s',(True,False,'MANUAL'),local,1)
        mission.step(2,True,'s',(True,False,'MANUAL'),local,1)
        self.assertEqual(mission.step(4.6,True,'s',(True,False,'MANUAL'),local,1)[1],'OFFBOARD')
        self.assertEqual(mission.step(4.7,True,'s',(True,False,'OFFBOARD'),local,1,offboard_confirmed=True)[1], 'ARM' if auto else None)
        cmd,action=mission.step(10,True,'s',(True,True,'OFFBOARD'),local,1)
        self.assertEqual((cmd,action),(local,None));self.assertTrue(mission.is_active)
        return mission,local

    def test_hover_fixed_origin_timing_and_normal_landing(self):
        for auto in (False,True):
            mission,origin=self.active_hover(auto)
            self.assertEqual(mission.step(14.99,True,'s',(True,True,'OFFBOARD'),(1.1,2.1,3.1,.5),2),(origin,None))
            self.assertEqual(mission.step(15,True,'s',(True,True,'OFFBOARD'),origin,2),(origin,'LAND'))
            self.assertEqual(mission.state,'WAIT_LAND_MODE')
            mission.step(15.1,True,'s',(True,True,'AUTO.LAND'),origin,2)
            self.assertEqual(mission.step(16,True,'s',(True,True,'AUTO.LAND'),origin,1)[1],'DISARM')
            mission.step(16.1,True,'s',(True,False,'AUTO.LAND'),origin,1)
            self.assertEqual(mission.state,'DONE')

    def test_generic_phase_and_plugin_failures_land_once(self):
        for bad in (TaskUpdate((1,2,float('nan'),0),'CUSTOM'),TaskUpdate((1,2,3,0),'DONE'),RuntimeError('plugin error')):
            mission,origin=self.active_hover()
            def step(now,local):
                if isinstance(bad,Exception):raise bad
                return bad
            mission.task.step=step
            self.assertEqual(mission.step(11,True,'s',(True,True,'OFFBOARD'),origin,2),(None,'LAND'))
            self.assertEqual(mission.state,'ABORT_LAND_MODE')
            self.assertNotEqual(mission.step(12,True,'s',(True,True,'OFFBOARD'),origin,2)[1],'LAND')
        mission,origin=self.active_hover()
        mission.task.step=lambda now,local:TaskUpdate(origin,'CUSTOM')
        mission.step(11,True,'s',(True,True,'OFFBOARD'),origin,2)
        self.assertTrue(mission.is_active);self.assertEqual(mission.state,'CUSTOM')
        self.assertEqual(mission.step(12,False,'s',(True,True,'OFFBOARD'),origin,2)[1],'LAND')

    def test_hover_faults_and_takeover_do_not_resume(self):
        for ready,session,fcu,local,expected in (
            (False,'s',(True,True,'OFFBOARD'),(1,2,3,.4),'ABORT_LAND_MODE'),
            (True,'new',(True,True,'OFFBOARD'),(1,2,3,.4),'ABORT_LAND_MODE'),
            (True,'s',(False,True,'OFFBOARD'),None,'ABORTED'),
            (True,'s',(True,True,'MANUAL'),(1,2,3,.4),'TAKEN_OVER'),
            (True,'s',(True,False,'OFFBOARD'),(1,2,3,.4),'ABORTED')):
            mission,origin=self.active_hover()
            mission.step(11,ready,session,fcu,local,2)
            self.assertEqual(mission.state,expected)
            if expected in mission.TERMINAL:
                self.assertEqual(mission.step(20,True,'s',(True,True,'OFFBOARD'),origin,1),(None,None))
