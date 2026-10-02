#!/usr/bin/env python3
"""Opt-in ROS software check on an isolated master; never starts MAVROS."""
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time


def main():
    port = 11329
    with socket.socket() as probe:
        # Permit immediate reruns after the previous master's TIME_WAIT sockets.
        # An active listener still makes bind fail; never reuse another master.
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(('127.0.0.1', port))
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:{}'.format(port)
    os.environ['ROS_HOSTNAME'] = 'localhost'
    import rospy
    import rosgraph
    import yaml
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Log
    from uav_system.msg import SystemStatus, SourceStatus
    from mission_executor.mission_loader import load_task
    package = Path(__file__).resolve().parents[1]
    def resolved(params):
        cfg = {key.rsplit('/',1)[-1]: value for key,value in params.items()}
        return load_task(package,source=cfg.get('mission_source','takeoff_hover_land'),
                         config=cfg.get('mission_config',''),executor_config=cfg.get('executor_config',''),
                         auto_arm=cfg.get('auto_arm_override',''),
                         arm_method=cfg.get('arm_method_override','')).executor_config
    logs = Path(tempfile.mkdtemp(prefix='uav_executor_ros_'))
    children = []
    handles = []
    def start(name, args):
        handle = (logs / (name + '.log')).open('w')
        handles.append(handle)
        child = subprocess.Popen(args, stdout=handle, stderr=subprocess.STDOUT)
        children.append(child)
        return child
    def stop(child):
        child.terminate()
        try:
            child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            child.kill(); child.wait()
    def wait_for(predicate, timeout=15):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate(): return
            time.sleep(.1)
        raise AssertionError('condition timed out; logs: ' + str(logs))
    try:
        start('master', ['roscore', '-p', str(port)])
        master = rosgraph.Master('/executor_wait_validation')
        def online():
            try: master.getPid(); return True
            except Exception: return False
        wait_for(online)
        rospy.init_node('executor_wait_validation', anonymous=True)
        health_reports = []
        def received_log(msg):
            if msg.name == '/flight_supervisor' and msg.msg.startswith('Health checks:'):
                health_reports.append(msg.msg)
        rospy.Subscriber('/rosout', Log, received_log, queue_size=100)
        for args, expected in (([],True), (['auto_arm:=false'],False),
                               (['auto_arm:=true'],True), (['arm_method:=auto'],True),
                               (['arm_method:=manual'],False)):
            dumped = subprocess.check_output(['roslaunch','--dump-params','uav_system','mission_executor.launch'] + args, text=True)
            params = yaml.safe_load(dumped)
            assert resolved(params)['auto_arm'] is expected
        dumped = subprocess.check_output(['roslaunch','--dump-params','uav_system','mission_executor.launch',
                                           'auto_arm:=false','arm_method:=auto'],text=True)
        try:
            resolved(yaml.safe_load(dumped))
        except ValueError: pass
        else: raise AssertionError('conflicting launch arguments accepted')
        base = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/mission_executor.yaml').read_text())
        base.pop('auto_arm')
        legacy = logs / 'legacy.yaml'
        legacy.write_text(yaml.safe_dump(dict(base,arm_method='manual')))
        conflicts = logs / 'conflict.yaml'
        conflicts.write_text(yaml.safe_dump(dict(base,arm_method='manual',auto_arm=False)))
        for args in (['executor_config:=' + str(legacy), 'auto_arm:=true'],
                     ['executor_config:=' + str(conflicts)]):
            dumped = subprocess.check_output(['roslaunch','--dump-params','uav_system','mission_executor.launch'] + args,text=True)
            try:
                resolved(yaml.safe_load(dumped))
            except ValueError: pass
            else: raise AssertionError('conflicting YAML/launch arguments accepted')
        for args in (['mission_source:=missing'], ['executor_config:=' + str(conflicts)],
                     ['auto_arm:=false','arm_method:=auto']):
            name='invalid_' + str(len(children))
            invalid=start(name,['roslaunch','uav_system','mission_executor.launch'] + args)
            wait_for(lambda: invalid.poll() is not None)
            assert 'Mission Executor startup failed' in (logs/(name+'.log')).read_text()
            publishers=master.getSystemState()[0]
            assert not any(topic=='/uav/command/trajectory' and '/mission_executor' in nodes
                           for topic,nodes in publishers)
        mock = start('mock', ['roslaunch','uav_system','mock_system.launch'])
        def status():
            msg = rospy.wait_for_message('/uav/system/status', SystemStatus, timeout=10)
            assert not msg.ready and not msg.arm_ready and not msg.backend_output_enabled
        def health(expected):
            return rospy.wait_for_message('/uav/state/health',SourceStatus,timeout=5).healthy is expected
        wait_for(lambda: health(True))
        wait_for(lambda: any('[PASS] Source odometry fresh and healthy' in report and
                             '[FAIL] FCU connected with fresh telemetry' in report
                             for report in health_reports))
        status()
        for args, expected in (([],True),(['auto_arm:=false'],False),(['auto_arm:=true'],True),(['arm_method:=manual'],False),(['executor_config:=' + str(legacy)],False),(['mission_source:=hover'],True)):
            executor = start('executor_' + str(len(children)), ['roslaunch','uav_system','mission_executor.launch'] + args)
            for _ in range(4):
                state = rospy.wait_for_message('/uav/mission_executor/state',String,timeout=10).data
                assert state.startswith('WAIT_SYSTEM'), state
                status()
            assert rospy.get_param('/mission_executor/auto_arm') is expected
            task_params=rospy.get_param('/mission_executor/mission')
            assert task_params['hover_seconds']==5.0 and 'auto_arm' not in task_params
            if args==['mission_source:=hover']:
                assert task_params=={'hover_seconds':5.0},task_params
            assert executor.poll() is None
            if not args:
                duplicate = start('duplicate', ['roslaunch','uav_system','mission_executor.launch'])
                wait_for(lambda: duplicate.poll() is not None)
                assert executor.poll() is None
                assert 'already running' in (logs/'duplicate.log').read_text()
                assert rospy.wait_for_message('/uav/mission_executor/state',String,timeout=5).data.startswith('WAIT_SYSTEM')
            stop(executor)
        for mode, expected in (('timeout',False),('static',True),('nan',False),
                               ('static',True),('jump',False),('static',False)):
            rospy.set_param('/mock_state_source/mode',mode)
            time.sleep(1)
            wait_for(lambda: health(expected))
            status()
        nodes = {node for group in master.getSystemState() for _, names in group for node in names}
        assert not any('mavros' in node for node in nodes), nodes
        assert mock.poll() is None
        print('PASS: launch defaults/overrides/conflicts, mock health PASS/FAIL diagnostics, both tasks WAIT_SYSTEM and duplicate rejection; logs:',logs)
    finally:
        for child in reversed(children):
            if child.poll() is None: stop(child)
        for handle in handles: handle.close()


if __name__ == '__main__':
    main()
