#!/usr/bin/env python3
"""Own one configured source launch tree. No shell commands or flight services."""
import fcntl
import hashlib
import os
import threading
import time
import uuid
import roslaunch
import rosnode
import rospy
from nav_msgs.msg import Odometry
from uav_system.msg import SourceStatus
from uav_core.source_config import load_source
from uav_core.runtime import wall_loop


class DeathListener(roslaunch.pmon.ProcessListener):
    def __init__(self):
        self.failure = ''
    def process_died(self, name, exit_code):
        self.failure = 'source process exited: %s (%s)' % (name, exit_code)


def main():
    key = hashlib.sha256(os.environ.get('ROS_MASTER_URI', '').encode()).hexdigest()[:16]
    lock_file = open('/tmp/uav-source-'+key+'.lock', 'w')
    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    rospy.init_node('state_source_manager')
    cfg = load_source(rospy.get_param('~source_config'))
    start_source = rospy.get_param('~start_source', True)
    pub = rospy.Publisher('/uav/source/status', SourceStatus, queue_size=1)
    session = str(uuid.uuid4())
    listener = DeathListener()
    last_raw = [None]
    mutex = threading.RLock()
    def received(msg):
        with mutex:
            last_raw[0] = (time.monotonic(), msg.header.stamp)
    rospy.Subscriber(cfg['input']['topic'], Odometry, received, queue_size=1)
    if start_source:
        existing = set(rosnode.get_node_names())
        conflicts = existing.intersection(cfg['source'].get('owned_nodes', []))
        if conflicts:
            raise RuntimeError('source nodes already running: '+str(sorted(conflicts)))
    launch_files = []
    launches = cfg['source']['launches'] if start_source else []
    for item in launches:
        args = [str(k)+':='+str(v).lower() if isinstance(v, bool) else str(k)+':='+str(v)
                for k, v in item.get('args', {}).items()]
        filename = roslaunch.rlutil.resolve_launch_arguments([item['package'], item['file']])[0]
        launch_files.append((filename, args))
    parent = None
    if launch_files:
        launch_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(launch_uuid)
        parent = roslaunch.parent.ROSLaunchParent(launch_uuid, launch_files, process_listeners=[listener])
        rospy.on_shutdown(parent.shutdown)
        parent.start()
    def tick():
        now = rospy.Time.now()
        with mutex:
            raw = last_raw[0]
        timeout = cfg['adapter']['stale_timeout']
        fresh = raw and time.monotonic()-raw[0] <= timeout and raw[1].to_sec()>0 and -0.05 <= (now-raw[1]).to_sec() <= timeout
        msg = SourceStatus()
        msg.header.stamp = now
        msg.session_id = session
        msg.simulated = cfg['source']['simulated']
        msg.calibrated = cfg['extrinsic']['calibrated'] and cfg['world_alignment']['verified']
        msg.healthy = bool(fresh and not listener.failure)
        msg.detail = listener.failure or ('raw odometry fresh' if fresh else 'waiting for fresh raw odometry')
        pub.publish(msg)
    wall_loop(10, tick)


if __name__ == '__main__':
    main()
