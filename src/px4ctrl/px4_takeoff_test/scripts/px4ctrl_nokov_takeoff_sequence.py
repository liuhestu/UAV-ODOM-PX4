#!/usr/bin/env python3
"""等待MAVROS、NOKOV、RC和px4ctrl全部就绪后，发送一次自动起飞触发。"""

import collections
import math
import sys
import time

import rospy
from mavros_msgs.msg import AttitudeTarget, ExtendedState, RCIn, State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand
from sensor_msgs.msg import Imu


class ReadinessError(RuntimeError):
    """启动条件在限定时间内没有满足。"""


class NokovTakeoffSequence:
    """负责就绪检查、可靠TAKEOFF触发、定时悬停和LAND。"""

    # MAVLink MAV_STATE_STANDBY：PX4已经完成初始化，处于可解锁待机状态。
    MAV_STATE_STANDBY = 3

    DATA_TIMEOUT = 0.5
    SETPOINT_TIMEOUT = 0.25

    def __init__(self):
        self.confirm_flight = bool(rospy.get_param("~confirm_flight", False))
        self.mavros_ns = str(rospy.get_param("~mavros_ns", "/mavros")).rstrip("/")
        self.odom_topic = str(rospy.get_param("~odom_topic", "/nokov/odometry"))
        self.trigger_topic = str(
            rospy.get_param("~trigger_topic", "/px4ctrl/takeoff_land")
        )
        self.start_timeout = float(rospy.get_param("~start_timeout", 60.0))
        self.stable_time = float(rospy.get_param("~stable_time", 3.0))
        self.arm_timeout = float(rospy.get_param("~arm_timeout", 10.0))
        self.takeoff_retry_interval = float(
            rospy.get_param("~takeoff_retry_interval", 3.0)
        )
        self.takeoff_max_attempts = int(
            rospy.get_param("~takeoff_max_attempts", 3)
        )
        # MAVROS state/extended_state通常只有约1Hz，不能使用0.5秒传感器超时。
        self.state_timeout = float(rospy.get_param("~state_timeout", 2.5))
        self.extended_state_timeout = float(
            rospy.get_param("~extended_state_timeout", 2.5)
        )
        # hover_duration只计算“已经达到目标高度并稳定”之后的悬停时间，
        # 不包含电机预转和爬升阶段，是当前自动起飞流程实际使用的计时参数。
        self.hover_duration = float(rospy.get_param("~hover_duration", 15.0))

        # flight_duration是可选飞行计时，从确认达到起飞目标高度后开始。
        # 默认-1表示不额外设置飞行时间；设置为非负数时优先于hover_duration，
        # 到时会在当前位置请求LAND。
        self.flight_duration = float(rospy.get_param("~flight_duration", -1.0))

        # takeoff_timeout只限制ARM之后的起飞阶段。若在此时间内没有达到
        # 目标高度并稳定，节点会请求LAND，防止持续爬升或长时间卡在AUTO_TAKEOFF。
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 30.0))

        # 判定“起飞完成”不能只看某一帧高度。飞机必须达到目标高度附近，
        # 垂直速度足够小，并连续保持takeoff_settle_time，才开始计算悬停时间。
        self.takeoff_height_tolerance = float(
            rospy.get_param("~takeoff_height_tolerance", 0.05)
        )
        self.takeoff_settle_time = float(
            rospy.get_param("~takeoff_settle_time", 0.5)
        )
        self.takeoff_settle_vertical_speed = float(
            rospy.get_param("~takeoff_settle_vertical_speed", 0.20)
        )

        # 直接读取px4ctrl实际加载的起飞高度，避免时序节点与控制器各维护一份
        # takeoff_height而产生不一致。该参数来自ctrl_param_fpv.yaml。
        self.takeoff_height = float(
            rospy.get_param("/px4ctrl/auto_takeoff_land/takeoff_height", 0.5)
        )
        self.landing_timeout = float(rospy.get_param("~landing_timeout", 60.0))
        self.minimum_odom_rate = float(rospy.get_param("~minimum_odom_rate", 30.0))
        self.maximum_takeoff_speed = float(
            rospy.get_param("~maximum_takeoff_speed", 0.1)
        )
        self.require_rc = bool(rospy.get_param("~require_rc", True))
        self.rc_switch_threshold = int(
            rospy.get_param("~rc_switch_threshold", 1750)
        )
        self.rc_center_tolerance = int(
            rospy.get_param("~rc_center_tolerance", 125)
        )
        # 自动起飞时允许油门保持最低位；其他三个姿态摇杆仍必须居中。
        self.allow_low_throttle = bool(
            rospy.get_param("~allow_low_throttle", True)
        )
        self.rc_low_throttle_max = int(
            rospy.get_param("~rc_low_throttle_max", 1125)
        )

        if not self.confirm_flight:
            raise ReadinessError(
                "confirm_flight=false；未确认飞行区域、定位方向和急停状态，不发送起飞"
            )
        if not 10.0 <= self.start_timeout <= 180.0:
            raise ValueError("start_timeout必须在10到180秒之间")
        if not 1.0 <= self.stable_time <= 15.0:
            raise ValueError("stable_time必须在1到15秒之间")
        if not 3.0 <= self.arm_timeout <= 30.0:
            raise ValueError("arm_timeout必须在3到30秒之间")
        if not 1.0 <= self.takeoff_retry_interval <= 10.0:
            raise ValueError("takeoff_retry_interval必须在1到10秒之间")
        if not 1 <= self.takeoff_max_attempts <= 5:
            raise ValueError("takeoff_max_attempts必须在1到5之间")
        retry_span = (self.takeoff_max_attempts - 1) * self.takeoff_retry_interval
        if retry_span >= self.arm_timeout:
            raise ValueError("TAKEOFF最后一次重试必须早于arm_timeout")
        if not 1.1 <= self.state_timeout <= 10.0:
            raise ValueError("state_timeout必须在1.1到10秒之间")
        if not 1.1 <= self.extended_state_timeout <= 10.0:
            raise ValueError("extended_state_timeout必须在1.1到10秒之间")
        if not 900 <= self.rc_low_throttle_max <= 1300:
            raise ValueError("rc_low_throttle_max必须在900到1300之间")
        if not 1.0 <= self.hover_duration <= 3600.0:
            raise ValueError("hover_duration必须在1到3600秒之间")
        if self.flight_duration != -1.0 and not 0.0 <= self.flight_duration <= 86400.0:
            raise ValueError("flight_duration必须为-1，或在0到86400秒之间")
        if not 5.0 <= self.takeoff_timeout <= 180.0:
            raise ValueError("takeoff_timeout必须在5到180秒之间")
        if not 0.0 <= self.takeoff_height_tolerance <= 0.20:
            raise ValueError("takeoff_height_tolerance必须在0到0.20米之间")
        if not 0.1 <= self.takeoff_settle_time <= 5.0:
            raise ValueError("takeoff_settle_time必须在0.1到5秒之间")
        if not 0.05 <= self.takeoff_settle_vertical_speed <= 1.0:
            raise ValueError("takeoff_settle_vertical_speed必须在0.05到1.0m/s之间")
        if not 0.1 <= self.takeoff_height <= 10.0:
            raise ValueError("px4ctrl起飞高度必须在0.1到10米之间")
        if not 20.0 <= self.landing_timeout <= 180.0:
            raise ValueError("landing_timeout必须在20到180秒之间")
        if not 1.0 <= self.minimum_odom_rate <= 240.0:
            raise ValueError("minimum_odom_rate必须在1到240Hz之间")
        if not 0.01 <= self.maximum_takeoff_speed <= 0.5:
            raise ValueError("maximum_takeoff_speed必须在0.01到0.5m/s之间")

        self.state = None
        self.extended_state = None
        self.odom = None
        self.imu = None
        self.rc = None
        self.state_time = None
        self.extended_state_time = None
        self.odom_time = None
        self.imu_time = None
        self.rc_time = None
        self.setpoint_time = None
        self.odom_receive_times = collections.deque()

        rospy.Subscriber(self._topic("state"), State, self._state_cb, queue_size=10)
        rospy.Subscriber(
            self._topic("extended_state"),
            ExtendedState,
            self._extended_state_cb,
            queue_size=10,
        )
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber(self._topic("imu/data"), Imu, self._imu_cb, queue_size=50)
        rospy.Subscriber(self._topic("rc/in"), RCIn, self._rc_cb, queue_size=20)
        rospy.Subscriber(
            self._topic("setpoint_raw/attitude"),
            AttitudeTarget,
            self._setpoint_cb,
            queue_size=50,
        )
        self.trigger_pub = rospy.Publisher(
            self.trigger_topic, TakeoffLand, queue_size=1
        )

    def _topic(self, suffix):
        """在配置的MAVROS命名空间下构造话题名。"""
        return self.mavros_ns + "/" + suffix.lstrip("/")

    def _state_cb(self, message):
        self.state = message
        self.state_time = time.monotonic()

    def _extended_state_cb(self, message):
        self.extended_state = message
        self.extended_state_time = time.monotonic()

    def _odom_cb(self, message):
        now = time.monotonic()
        self.odom = message
        self.odom_time = now
        self.odom_receive_times.append(now)
        while self.odom_receive_times and now - self.odom_receive_times[0] > 2.0:
            self.odom_receive_times.popleft()

    def _imu_cb(self, message):
        self.imu = message
        self.imu_time = time.monotonic()

    def _rc_cb(self, message):
        self.rc = message
        self.rc_time = time.monotonic()

    def _setpoint_cb(self, _message):
        self.setpoint_time = time.monotonic()

    @staticmethod
    def _fresh(received_time, timeout=DATA_TIMEOUT):
        return received_time is not None and time.monotonic() - received_time < timeout

    def _odom_rate(self):
        """按最近两秒的真实接收时刻估算Odometry频率，不重复旧消息。"""
        if len(self.odom_receive_times) < 2:
            return 0.0
        elapsed = self.odom_receive_times[-1] - self.odom_receive_times[0]
        return (len(self.odom_receive_times) - 1) / elapsed if elapsed > 0.0 else 0.0

    def _odom_problem(self):
        if not self._fresh(self.odom_time) or self.odom is None:
            return "等待新鲜NOKOV Odometry"
        pose = self.odom.pose.pose
        velocity = self.odom.twist.twist.linear
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
            velocity.x, velocity.y, velocity.z,
        )
        if not all(math.isfinite(value) for value in values):
            return "NOKOV Odometry包含NaN或Inf"
        quaternion_norm = math.sqrt(
            pose.orientation.x ** 2 + pose.orientation.y ** 2
            + pose.orientation.z ** 2 + pose.orientation.w ** 2
        )
        if not 0.9 <= quaternion_norm <= 1.1:
            return "NOKOV姿态四元数未归一化"
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        if speed >= self.maximum_takeoff_speed:
            return "飞机未静止：Odometry速度为%.3fm/s" % speed
        rate = self._odom_rate()
        if rate < self.minimum_odom_rate:
            return "NOKOV Odometry频率%.1fHz低于%.1fHz" % (
                rate, self.minimum_odom_rate
            )
        return None

    def _rc_problem(self):
        if not self.require_rc:
            return None
        if not self._fresh(self.rc_time) or self.rc is None:
            return "等待新鲜RC数据"
        if len(self.rc.channels) < 6:
            return "RC通道数量少于6"
        # CH1/CH2/CH4分别为横滚、俯仰、偏航，必须居中。
        for index in (0, 1, 3):
            if abs(int(self.rc.channels[index]) - 1500) > self.rc_center_tolerance:
                return "RC通道%d未居中，当前值%d" % (
                    index + 1, self.rc.channels[index]
                )

        throttle = int(self.rc.channels[2])
        throttle_centered = abs(throttle - 1500) <= self.rc_center_tolerance
        throttle_low = (
            self.allow_low_throttle and throttle <= self.rc_low_throttle_max
        )
        if not throttle_centered and not throttle_low:
            return (
                "RC通道3油门既未居中也未处于最低安全区，当前值%d" % throttle
            )
        if self.rc.channels[4] <= self.rc_switch_threshold:
            return "RC通道5未处于auto hover高位"
        if self.rc.channels[5] <= self.rc_switch_threshold:
            return "RC通道6未处于command control高位"
        return None

    def _not_ready_reason(self):
        if self.state is None:
            return "等待MAVROS状态"
        if not self.state.connected:
            return "MAVROS已启动，但未收到PX4 heartbeat（FCU未连接）"
        if not self._fresh(self.state_time, self.state_timeout):
            return "MAVROS状态已超时，检查FCU链路"
        if self.state.system_status != self.MAV_STATE_STANDBY:
            return "PX4系统状态%d尚未进入可解锁STANDBY(3)" % (
                self.state.system_status
            )
        if self.state.armed:
            return "PX4启动前已经ARMED"
        if (
            not self._fresh(
                self.extended_state_time, self.extended_state_timeout
            )
            or self.extended_state is None
        ):
            return "等待PX4落地状态"
        if self.extended_state.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
            return "PX4未报告ON_GROUND"
        problem = self._odom_problem()
        if problem:
            return problem
        if not self._fresh(self.imu_time) or self.imu is None:
            return "等待新鲜IMU"
        problem = self._rc_problem()
        if problem:
            return problem
        if not self._fresh(self.setpoint_time, self.SETPOINT_TIMEOUT):
            return "等待px4ctrl持续发布姿态setpoint"
        if self.trigger_pub.get_num_connections() < 1:
            return "等待px4ctrl订阅takeoff_land"
        return None

    def _in_flight_problem(self):
        """检查飞行阶段仍然必须持续有效的数据链路。

        起飞前的_not_ready_reason()还会检查“必须静止、必须STANDBY”等条件，
        这些条件在飞行中必然不再成立，所以不能直接复用。这里仅检查飞行控制
        真正依赖的FCU连接、Odometry、IMU和px4ctrl姿态setpoint是否仍然新鲜。
        """
        if self.state is None or not self._fresh(self.state_time, self.state_timeout):
            return "MAVROS状态超时"
        if not self.state.connected:
            return "MAVROS与PX4连接中断"
        if self.odom is None or not self._fresh(self.odom_time):
            return "NOKOV Odometry超时"
        if self.imu is None or not self._fresh(self.imu_time):
            return "PX4 IMU超时"
        if not self._fresh(self.setpoint_time, self.SETPOINT_TIMEOUT):
            return "px4ctrl姿态setpoint中断"

        pose = self.odom.pose.pose
        velocity = self.odom.twist.twist.linear
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
            velocity.x, velocity.y, velocity.z,
        )
        if not all(math.isfinite(value) for value in values):
            return "NOKOV Odometry在飞行中出现NaN或Inf"
        return None

    def _wait_for_takeoff_completion(self, rate, start_z):
        """等待飞机达到目标高度并稳定，成功后才允许开始悬停计时。

        start_z是发送TAKEOFF前的NOKOV高度；目标高度等于start_z加上px4ctrl
        配置的相对起飞高度。单帧越过阈值不算完成，还要求垂直速度较小并连续
        稳定一段时间，以免飞机正在高速穿过目标高度时提前开始悬停计时。
        """
        target_z = start_z + self.takeoff_height
        deadline = time.monotonic() + self.takeoff_timeout
        stable_since = None

        rospy.logwarn(
            "PX4已ARM：开始起飞阶段，NOKOV起始z=%.3fm，目标z=%.3fm，超时=%.1fs",
            start_z,
            target_z,
            self.takeoff_timeout,
        )

        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.state is not None and not self.state.armed:
                rospy.logwarn("起飞阶段PX4已提前DISARM，不再发送LAND")
                return False

            problem = self._in_flight_problem()
            if problem:
                raise ReadinessError("起飞阶段数据异常: %s" % problem)

            current_z = self.odom.pose.pose.position.z
            vertical_speed = self.odom.twist.twist.linear.z
            height_reached = current_z >= target_z - self.takeoff_height_tolerance
            vertical_speed_stable = (
                abs(vertical_speed) <= self.takeoff_settle_vertical_speed
            )

            if height_reached and vertical_speed_stable:
                if stable_since is None:
                    stable_since = time.monotonic()
                    rospy.loginfo(
                        "已进入目标高度范围，开始%.1f秒到高稳定确认",
                        self.takeoff_settle_time,
                    )
                elif time.monotonic() - stable_since >= self.takeoff_settle_time:
                    rospy.logwarn(
                        "起飞完成：当前z=%.3fm、垂直速度=%.3fm/s；开始悬停计时",
                        current_z,
                        vertical_speed,
                    )
                    return True
            else:
                # 任一条件失效就重新计时，防止使用不连续的若干帧拼出“稳定”。
                stable_since = None

            rospy.loginfo_throttle(
                1.0,
                "自动起飞中: z=%.3fm / 目标%.3fm, vz=%.3fm/s, 剩余超时%.1fs",
                current_z,
                target_z,
                vertical_speed,
                max(0.0, deadline - time.monotonic()),
            )
            rate.sleep()

        if rospy.is_shutdown():
            return False
        raise ReadinessError(
            "起飞超过%.1f秒仍未在目标高度%.3fm稳定；将请求LAND"
            % (self.takeoff_timeout, target_z)
        )

    def _hover_for_duration(self, rate):
        """到达目标高度后计时，并在计时结束时准备原地降落。

        flight_duration显式设置为非负数时，它就是本次飞行计时，优先于
        hover_duration；到时在当前位置请求LAND。保持默认-1时，不设置额外的
        总飞行时间，当前起飞测试仍按hover_duration完成定点悬停后降落。
        将来加入轨迹或航点任务时，可以继续用flight_duration作为任务飞行时间。
        """
        timer_start = time.monotonic()
        if self.flight_duration >= 0.0:
            active_duration = self.flight_duration
            timer_is_flight_duration = True
            rospy.loginfo(
                "目标高度已确认，开始计算flight_duration=%.1f秒；到时原地请求LAND",
                active_duration,
            )
        else:
            active_duration = self.hover_duration
            timer_is_flight_duration = False
            rospy.loginfo(
                "flight_duration=-1，不设置额外飞行时限；按当前任务悬停%.1f秒",
                active_duration,
            )
        deadline = timer_start + active_duration

        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.state is not None and not self.state.armed:
                rospy.logwarn("飞行计时阶段PX4已提前DISARM，不再发送LAND")
                return False
            problem = self._in_flight_problem()
            if problem:
                raise ReadinessError("飞行计时阶段数据异常: %s" % problem)
            rate.sleep()

        if rospy.is_shutdown():
            return False
        if timer_is_flight_duration:
            rospy.loginfo("flight_duration计时完成，准备在当前位置请求缓慢自动降落")
        else:
            rospy.loginfo("悬停任务计时完成，准备请求缓慢自动降落")
        return True

    def run(self):
        """依次执行就绪检查、ARM、限时起飞、定时悬停和安全降落。"""
        deadline = time.monotonic() + self.start_timeout
        stable_since = None
        last_reason = "尚未开始检查"
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            reason = self._not_ready_reason()
            if reason is None:
                if stable_since is None:
                    stable_since = time.monotonic()
                    rospy.loginfo("所有起飞条件已满足，开始稳定性计时")
                elif time.monotonic() - stable_since >= self.stable_time:
                    break
            else:
                last_reason = reason
                stable_since = None
                rospy.loginfo_throttle(2.0, "等待自动起飞条件: %s", reason)
            rate.sleep()
        else:
            raise ReadinessError("等待自动起飞条件超时；最后状态: %s" % last_reason)

        rospy.logwarn(
            "真实NOKOV定位、PX4 STANDBY和px4ctrl已连续稳定%.1f秒，准备发送TAKEOFF",
            self.stable_time,
        )
        message = TakeoffLand()
        message.takeoff_land_cmd = TakeoffLand.TAKEOFF

        # 在首次发送TAKEOFF前记录基准高度。px4ctrl也会在收到该触发时记录起点，
        # 两者使用同一个NOKOV话题，因此目标高度定义保持一致。
        takeoff_start_z = self.odom.pose.pose.position.z

        arm_deadline = time.monotonic() + self.arm_timeout
        next_attempt = 0.0
        attempts = 0
        last_retry_problem = None
        while not rospy.is_shutdown() and time.monotonic() < arm_deadline:
            if self.state is not None and self.state.armed:
                break
            now = time.monotonic()
            if now >= next_attempt and attempts < self.takeoff_max_attempts:
                # PX4可能在第一次触发时仍处于预检收尾阶段。若px4ctrl因ARM
                # 被拒而回到MANUAL_CTRL，则重新检查全部条件后有限次数重试。
                problem = self._not_ready_reason()
                if problem is None:
                    attempts += 1
                    self.trigger_pub.publish(message)
                    rospy.logwarn(
                        "已向px4ctrl发送第%d/%d次TAKEOFF触发",
                        attempts,
                        self.takeoff_max_attempts,
                    )
                    next_attempt = now + self.takeoff_retry_interval
                    last_retry_problem = None
                else:
                    last_retry_problem = problem
                    rospy.loginfo_throttle(
                        2.0, "TAKEOFF重试前等待安全条件: %s", problem
                    )
            rate.sleep()
        else:
            suffix = (
                "；最后状态: " + last_retry_problem
                if last_retry_problem else ""
            )
            raise ReadinessError(
                "发送%d次TAKEOFF后未在%.1f秒内确认PX4 ARMED%s"
                % (attempts, self.arm_timeout, suffix)
            )

        try:
            # takeoff_timeout只负责起飞阶段。确认起飞完成后才开始计算
            # flight_duration（已设置时）或hover_duration，因此爬升耗时不会
            # 占用后续飞行/悬停时间。
            if not self._wait_for_takeoff_completion(rate, takeoff_start_z):
                return
            if not self._hover_for_duration(rate):
                return
        finally:
            # 正常悬停结束、起飞超时或飞行数据异常时，只要PX4仍处于ARMED，
            # 都先通过px4ctrl请求LAND并等待DISARM，而不是直接结束控制节点。
            if self.state is not None and self.state.armed and not rospy.is_shutdown():
                self._land_and_wait(rate)

    def _land_and_wait(self, rate):
        """重复发送px4ctrl LAND，直到PX4落地并由px4ctrl确认DISARM。"""
        message = TakeoffLand()
        message.takeoff_land_cmd = TakeoffLand.LAND
        deadline = time.monotonic() + self.landing_timeout
        next_publish = 0.0
        first_publish = True

        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.state is not None and not self.state.armed:
                rospy.loginfo("PX4已安全落地并锁定")
                return
            now = time.monotonic()
            if now >= next_publish:
                self.trigger_pub.publish(message)
                if first_publish:
                    rospy.logwarn(
                        "已向px4ctrl发送LAND；下降速度由takeoff_land_speed参数控制"
                    )
                    first_publish = False
                next_publish = now + 1.0
            rate.sleep()

        if rospy.is_shutdown():
            raise ReadinessError(
                "ROS关闭信号中断了LAND等待；控制节点可能已同时退出，请使用遥控器接管"
            )
        raise ReadinessError(
            "LAND后%.1f秒内未确认PX4 DISARM；请使用遥控器接管"
            % self.landing_timeout
        )


def main():
    rospy.init_node("px4ctrl_nokov_takeoff_sequence")
    try:
        NokovTakeoffSequence().run()
    except (ReadinessError, ValueError) as exc:
        rospy.logerr("NOKOV自动起飞未执行/已中止: %s", exc)
        return 1
    except rospy.ROSInterruptException:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
