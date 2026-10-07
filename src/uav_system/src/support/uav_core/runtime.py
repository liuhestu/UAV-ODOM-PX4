"""Shared ROS helpers. Watchdogs run on wall time even if /clock stops."""
import threading
import time
import rospy


def wall_loop(hz, tick):
    def run():
        while not rospy.is_shutdown():
            start = time.monotonic()
            try:
                tick()
            except Exception as exc:
                rospy.logfatal('watchdog failed: %s', exc)
                rospy.signal_shutdown(str(exc))
                return
            time.sleep(max(0.001, 1.0/hz-(time.monotonic()-start)))
    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    rospy.spin()


class Inbox:
    def __init__(self):
        self.data = {}
        self.lock = threading.RLock()

    def subscribe(self, topic, cls, key):
        def cb(msg):
            with self.lock:
                self.data[key] = (msg, time.monotonic())
        return rospy.Subscriber(topic, cls, cb, queue_size=1)

    def get(self, key, timeout):
        item = self.data.get(key)
        if item and 0 <= time.monotonic()-item[1] <= timeout:
            msg = item[0]
            if hasattr(msg, 'header'):
                age = (rospy.Time.now()-msg.header.stamp).to_sec()
                if msg.header.stamp.to_sec() <= 0 or not -0.05 <= age <= timeout:
                    return None
            return msg
        return None


def xyz(v):
    return [v.x, v.y, v.z]


def xyzw(q):
    return [q.x, q.y, q.z, q.w]


def assign(v, values):
    v.x, v.y, v.z = map(float, values[:3])
    if len(values)==4:
        v.w = float(values[3])
