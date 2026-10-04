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
        probe.bind(('127.0.0.1', port))
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:{}'.format(port)
    os.environ['ROS_HOSTNAME'] = 'localhost'
    import rospy
    import rosgraph
    import yaml
    from std_msgs.msg import String
    from uav_system.msg import SystemStatus, SourceStatus
    from uav_core.mission import normalize_config
    logs = Path(tempfile.mkdtemp(prefix='uav_early_arm_ros_'))
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
        master = rosgraph.Master('/commander_wait_validation')
        def online():
            try: master.getPid(); return True
            except Exception: return False
        wait_for(online)
        rospy.init_node('commander_wait_validation', anonymous=True)
        for args, expected in (([],False), (['auto_arm:=false'],False),
                               (['auto_arm:=true'],True), (['arm_method:=auto'],True),
                               (['arm_method:=manual'],False)):
            dumped = subprocess.check_output(['roslaunch','--dump-params','uav_system','commander.launch'] + args, text=True)
            params = yaml.safe_load(dumped)
            cfg = {key.rsplit('/',1)[-1]: value for key,value in params.items()}
            assert normalize_config(cfg)['auto_arm'] is expected
        dumped = subprocess.check_output(['roslaunch','--dump-params','uav_system','commander.launch',
                                           'auto_arm:=false','arm_method:=auto'],text=True)
        try:
            normalize_config({key.rsplit('/',1)[-1]:value for key,value in yaml.safe_load(dumped).items()})
        except ValueError: pass
        else: raise AssertionError('conflicting launch arguments accepted')
        base = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/commander.yaml').read_text())
        base.pop('auto_arm')
        legacy = logs / 'legacy.yaml'
        legacy.write_text(yaml.safe_dump(dict(base,arm_method='manual')))
        conflicts = logs / 'conflict.yaml'
        conflicts.write_text(yaml.safe_dump(dict(base,arm_method='manual',auto_arm=False)))
        for args in (['config:=' + str(legacy), 'auto_arm:=true'],
                     ['config:=' + str(conflicts)]):
            dumped = subprocess.check_output(['roslaunch','--dump-params','uav_system','commander.launch'] + args,text=True)
            try:
                normalize_config({key.rsplit('/',1)[-1]:value for key,value in yaml.safe_load(dumped).items()})
            except ValueError: pass
            else: raise AssertionError('conflicting YAML/launch arguments accepted')
        mock = start('mock', ['roslaunch','uav_system','mock_system.launch'])
        def status():
            msg = rospy.wait_for_message('/uav/system/status', SystemStatus, timeout=10)
            assert not msg.ready and not msg.arm_ready and not msg.backend_output_enabled
        def health(expected):
            return rospy.wait_for_message('/uav/state/health',SourceStatus,timeout=5).healthy is expected
        wait_for(lambda: health(True))
        status()
        for args, expected in (([],False),(['auto_arm:=true'],True),(['arm_method:=manual'],False),(['config:=' + str(legacy)],False)):
            commander = start('commander_' + str(len(children)), ['roslaunch','uav_system','commander.launch'] + args)
            for _ in range(4):
                state = rospy.wait_for_message('/uav/commander/state',String,timeout=10).data
                assert state.startswith('WAIT_SYSTEM'), state
                status()
            assert rospy.get_param('/takeoff_hover_land/auto_arm') is expected
            assert commander.poll() is None
            stop(commander)
        for mode, expected in (('timeout',False),('static',True),('nan',False),
                               ('static',True),('jump',False),('static',False)):
            rospy.set_param('/mock_state_source/mode',mode)
            time.sleep(1)
            wait_for(lambda: health(expected))
            status()
        nodes = {node for group in master.getSystemState() for _, names in group for node in names}
        assert not any('mavros' in node for node in nodes), nodes
        assert mock.poll() is None
        print('PASS: launch defaults/overrides/conflicts, mock health, Commander WAIT_SYSTEM; logs:',logs)
    finally:
        for child in reversed(children):
            if child.poll() is None: stop(child)
        for handle in handles: handle.close()


if __name__ == '__main__':
    main()
