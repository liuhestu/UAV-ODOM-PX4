#!/usr/bin/env python3
"""发布固定外部里程计，仅用于静止、无桨的台架测试。

功能说明
--------
该节点周期性发布 ``nav_msgs/Odometry``。默认输出话题是
``/mavros/odometry/out``，MAVROS会把消息转换成MAVLink ODOMETRY并发送给
PX4。它适合检查下面这条数据链路是否连通：

    ROS测试节点 -> MAVROS -> MAVLink -> PX4外部视觉/里程计输入

它不是定位算法：发布期间位置、姿态和速度始终保持为参数指定的固定值。
因此不能用于真实起飞、悬停或飞行，否则控制器无法观察到飞机的真实运动。

安全设计
--------
这些布尔参数不是自动检测结果，而是操作者给出的明确安全确认：

``confirm_stationary``：
    * false（默认）：没有确认机体静止，节点直接拒绝启动且不发布任何消息。
    * true：操作者确认机体已经固定并保持静止，允许进入后续检查。

``allow_armed``：
    * false（默认）：只允许DISARMED测试；如果启动时已经ARMED则拒绝启动，
      发布过程中一旦检测到ARMED就立即中止。
    * true：即使PX4变为ARMED也继续发布固定Odometry。该选项只允许用于无桨
      台架测试，并且必须同时将confirm_propellers_removed设为true。

``confirm_propellers_removed``：
    * false（默认）：没有确认拆桨；当allow_armed=true时节点拒绝启动。
    * true：操作者确认所有桨叶已经拆除。它本身不会允许ARMED，也不会检测
      桨叶是否真的拆除；只有与allow_armed=true组合时才改变启动许可。

参数组合结果：

    confirm_stationary=false, 任意其他值
        -> 拒绝启动，不发布Odometry。
    confirm_stationary=true, allow_armed=false
        -> 只在DISARMED时发布；ARMED后立即停止。
    confirm_stationary=true, allow_armed=true,
    confirm_propellers_removed=false
        -> 拒绝启动，因为允许ARMED但没有确认拆桨。
    confirm_stationary=true, allow_armed=true,
    confirm_propellers_removed=true
        -> 允许在ARMED状态继续发布，仅限无桨台架测试。

无论这些参数如何设置，本节点都只发送Odometry，不切换模式、不请求ARM、
不发送推力，也不会绕过PX4的任何预检。

常用命令
--------
仅检查ROS到PX4的Odometry链路：

    roslaunch px4_takeoff_test fixed_odometry_test.launch \\
      confirm_stationary:=true duration:=30.0

将固定Odometry直接提供给当前订阅 ``/test/odometry`` 的px4ctrl，仅可用于
无桨数据链路检查：

    roslaunch px4_takeoff_test fixed_odometry_test.launch \\
      confirm_stationary:=true output_topic:=/test/odometry duration:=30.0

需要在无桨、已解锁状态下继续发布时，必须给出双重确认：

    roslaunch px4_takeoff_test fixed_odometry_test.launch \\
      confirm_stationary:=true allow_armed:=true \\
      confirm_propellers_removed:=true duration:=30.0
"""

import math
import sys
import time

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry


class FixedOdometryTest:
    """发送固定ENU/FLU里程计，但不向飞行器发送任何控制命令。"""

    def __init__(self):
        # 话题与坐标系参数。
        # /mavros/odometry/out用于把外部Odometry交给MAVROS/PX4；也可以通过
        # output_topic参数改为测试控制器实际订阅的话题，例如/test/odometry。
        self.output_topic = rospy.get_param("~output_topic", "/mavros/odometry/out")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")

        # 固定状态参数。x/y/z采用ROS常见的ENU世界坐标表达；yaw单位为弧度。
        # 这里只构造绕z轴的航向四元数，横滚角和俯仰角恒为0。
        self.x = float(rospy.get_param("~x", 0.0))
        self.y = float(rospy.get_param("~y", 0.0))
        self.z = float(rospy.get_param("~z", 0.0))
        self.yaw = float(rospy.get_param("~yaw", 0.0))
        # rate_hz：发布频率，允许10~100 Hz；duration：本次测试持续秒数。
        self.rate_hz = float(rospy.get_param("~rate", 30.0))
        self.duration = float(rospy.get_param("~duration", 30.0))

        # 三个stddev参数是测量标准差，稍后会平方后写入6x6协方差矩阵。
        # 它们只是测试数据的置信度描述，不会产生随机噪声。
        self.position_stddev = float(rospy.get_param("~position_stddev", 0.05))
        self.orientation_stddev = float(rospy.get_param("~orientation_stddev", 0.05))
        self.velocity_stddev = float(rospy.get_param("~velocity_stddev", 0.05))

        # 三个显式安全确认参数。它们是“操作者声明”，不是程序检测结果。
        #
        # confirm_stationary：
        #   false -> 立即拒绝启动，不发布任何Odometry；
        #   true  -> 操作者确认机体已经固定且静止，允许继续执行。
        self.confirm_stationary = bool(rospy.get_param("~confirm_stationary", False))

        # allow_armed：
        #   false -> 仅允许DISARMED测试；检测到ARMED便中止发布；
        #   true  -> ARMED后仍继续发布，但必须同时确认已经拆除桨叶。
        self.allow_armed = bool(rospy.get_param("~allow_armed", False))

        # confirm_propellers_removed：
        #   false -> 没有确认拆桨，禁止与allow_armed=true组合；
        #   true  -> 操作者明确确认全部桨叶已拆除。
        # 它不会自动识别桨叶，也不会单独触发ARM或改变发布行为。
        self.confirm_propellers_removed = bool(
            rospy.get_param("~confirm_propellers_removed", False)
        )

        # state保存最新的/mavros/state；state_received_at用于判断是否真正收到过
        # MAVROS状态。stop_requested由回调设置，让发布循环尽快、安全地退出。
        self.state = State()
        self.state_received_at = None
        self.stop_requested = False

        # 防止PX4保持ARMED期间以发布频率重复打印同一条警告。
        self.armed_warning_logged = False

        # 先校验参数，再创建发布器，保证非法配置不会向PX4发送任何消息。
        self._validate_parameters()

        # publisher发送固定Odometry；subscriber只监视PX4状态，不发送控制指令。
        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        rospy.Subscriber(self.state_topic, State, self._state_callback, queue_size=10)

    def _validate_parameters(self):
        """校验数值范围和安全确认；任何一项不满足都拒绝启动。"""
        # NaN和Inf会污染估计器，所以位置和航向必须是有限数。
        values = (self.x, self.y, self.z, self.yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("x, y, z, and yaw must be finite")
        if not 10.0 <= self.rate_hz <= 100.0:
            raise ValueError("rate must be between 10 and 100 Hz")
        if not 0.0 < self.duration <= 300.0:
            raise ValueError("duration must be greater than 0 and no more than 300 seconds")
        if min(
            self.position_stddev,
            self.orientation_stddev,
            self.velocity_stddev,
        ) <= 0.0:
            raise ValueError("covariance standard deviations must be positive")
        if not self.confirm_stationary:
            raise ValueError(
                "confirm_stationary is false; this bench-test node will not publish"
            )
        # 只有明确允许ARMED且再次确认无桨，才允许在解锁后继续发送固定数据。
        if self.allow_armed and not self.confirm_propellers_removed:
            raise ValueError(
                "allow_armed=true requires confirm_propellers_removed=true"
            )

    def _state_callback(self, message):
        """记录PX4连接/解锁状态，并在未授权解锁时触发紧急停止。"""
        self.state = message
        self.state_received_at = rospy.Time.now()
        if message.armed and not self.allow_armed:
            self.stop_requested = True
            rospy.logfatal(
                "PX4 reports ARMED. Stopping fixed odometry immediately; "
                "set allow_armed and confirm_propellers_removed only for a "
                "propellers-off bench test."
            )
        # allow_armed=true时不停止，但只打印一次醒目的无桨警告。
        elif message.armed and not self.armed_warning_logged:
            rospy.logwarn(
                "PX4 reports ARMED. Fixed odometry will continue because "
                "allow_armed=true and propeller removal was confirmed."
            )
            self.armed_warning_logged = True
        elif not message.armed:
            self.armed_warning_logged = False

    def _wait_for_fcu(self):
        """最多等待15秒，直到MAVROS确认已连接PX4。"""
        # 使用单调时钟计算超时，避免电脑系统时间被校准时影响等待时长。
        deadline = time.monotonic() + 15.0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.state_received_at is not None and self.state.connected:
                if self.state.armed and not self.allow_armed:
                    raise RuntimeError("PX4 is already armed")
                return
            rate.sleep()
        raise RuntimeError("timed out waiting for MAVROS FCU connection")

    def _make_message(self):
        """创建一份固定Odometry模板，时间戳会在每次发布前更新。"""
        message = Odometry()

        # frame_id描述世界/里程计参考系，child_frame_id描述机体参考系。
        message.header.frame_id = self.frame_id
        message.child_frame_id = self.child_frame_id

        # 固定位置。默认(0,0,0)只是测试原点，并不代表真实飞机位置。
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.position.z = self.z
        # 将单一yaw角转换为单位四元数：(x,y,z,w)=(0,0,sin(yaw/2),cos(yaw/2))。
        message.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(self.yaw * 0.5)

        # ROS协方差填写的是方差，因此需要把标准差平方。
        position_variance = self.position_stddev ** 2
        orientation_variance = self.orientation_stddev ** 2
        velocity_variance = self.velocity_stddev ** 2

        # pose协方差顺序为[x,y,z,roll,pitch,yaw]，这里只填写对角线。
        message.pose.covariance[0] = position_variance
        message.pose.covariance[7] = position_variance
        message.pose.covariance[14] = position_variance
        message.pose.covariance[21] = orientation_variance
        message.pose.covariance[28] = orientation_variance
        message.pose.covariance[35] = orientation_variance

        # 未显式赋值的twist默认全部为0，表示机体静止；这里补充速度协方差。
        # twist协方差顺序为[vx,vy,vz,wx,wy,wz]，同样只填写对角线。
        message.twist.covariance[0] = velocity_variance
        message.twist.covariance[7] = velocity_variance
        message.twist.covariance[14] = velocity_variance
        message.twist.covariance[21] = orientation_variance
        message.twist.covariance[28] = orientation_variance
        message.twist.covariance[35] = orientation_variance
        return message

    def run(self):
        """等待飞控后按设定频率发布，并持续执行连接与解锁安全检查。"""
        self._wait_for_fcu()
        if self.allow_armed:
            rospy.logwarn(
                "Publishing FIXED odometry for %.1f seconds, including while "
                "PX4 is ARMED. Propellers must remain removed and the vehicle "
                "must remain stationary.",
                self.duration,
            )
        else:
            rospy.logwarn(
                "Publishing FIXED odometry for %.1f seconds. Keep the vehicle "
                "stationary, disarmed, and without propellers.",
                self.duration,
            )
        rospy.logwarn(
            "The odometry and takeoff nodes use different ROS topics, but fixed "
            "odometry is not valid for real flight."
        )

        # 消息内容保持固定，但header.stamp必须随每一帧更新，否则PX4会把数据
        # 判断为陈旧测量。使用time.monotonic()计算时长可避免系统时间跳变。
        message = self._make_message()
        deadline = time.monotonic() + self.duration  # 测试结束的单调时钟时间点。
        rate = rospy.Rate(self.rate_hz)               # 控制ROS循环频率。
        published = 0                                 # 统计实际发布帧数。

        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.stop_requested or (self.state.armed and not self.allow_armed):
                raise RuntimeError("stopped because PX4 became armed")
            if not self.state.connected:
                raise RuntimeError("MAVROS FCU connection was lost")
            # 每帧使用当前ROS时间戳；其余字段保持固定。
            message.header.stamp = rospy.Time.now()
            self.publisher.publish(message)
            published += 1
            rate.sleep()

        rospy.loginfo(
            "Fixed odometry test complete: published %d messages to %s",
            published,
            self.output_topic,
        )


def main():
    """ROS节点入口：将可预期的拒绝原因转换为清晰日志和非零退出码。"""
    rospy.init_node("fixed_odometry_test")
    try:
        FixedOdometryTest().run()
    except (RuntimeError, ValueError) as exc:
        rospy.logerr("Fixed odometry test refused/aborted: %s", exc)
        return 1
    except rospy.ROSInterruptException:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
