#!/usr/bin/env python3
"""Software-only raw source. Does not connect to MAVROS or provide PX4 evidence."""
import time
import rospy
from nav_msgs.msg import Odometry
from uav_core.runtime import wall_loop


def main():
    rospy.init_node('mock_state_source')
    pub = rospy.Publisher('/mock/odom', Odometry, queue_size=1)
    start = time.monotonic()
    def tick():
        mode = rospy.get_param('~mode', 'static')
        t = time.monotonic()-start
        if mode == 'timeout':
            return
        msg = Odometry()
        msg.header.stamp = rospy.Time.now(); msg.header.frame_id='mock_world'; msg.child_frame_id='mock_body'
        msg.pose.pose.orientation.w = 1
        msg.pose.covariance = [0.01 if i%7==0 else 0 for i in range(36)]
        msg.twist.covariance = [0.01 if i%7==0 else 0 for i in range(36)]
        if mode == 'linear':
            msg.pose.pose.position.x=0.05*t; msg.twist.twist.linear.x=0.05
        elif mode == 'nan':
            msg.pose.pose.position.x=float('nan')
        elif mode == 'jump':
            msg.pose.pose.position.x=10 if int(t)%2 else 0
        elif mode != 'static':
            raise ValueError('unknown mock mode')
        pub.publish(msg)
    wall_loop(50, tick)


if __name__ == '__main__':
    main()
