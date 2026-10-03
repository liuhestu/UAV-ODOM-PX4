#!/usr/bin/env python3
"""通过MAVROS执行受限的无桨电机台架测试。

本节点与px4ctrl和Odometry完全解耦：它直接向
``/mavros/setpoint_raw/attitude`` 发布短时间、低推力的姿态目标，并通过
MAVROS服务请求PX4进入OFFBOARD和解锁。测试结束或发生异常时，它会立即发送
零推力、请求锁定，并尝试恢复测试前的飞行模式。

这并不意味着与PX4解耦。电机连接在PX4上时，任何测试代码都必须通过PX4或
电调协议下发指令。本节点只是不依赖px4ctrl，也不需要伪造固定Odometry。

安全限制：

* 必须同时确认已经拆除全部桨叶，并明确确认执行低推力测试；
* 只允许从“未解锁、在地面、MAVROS已连接”的状态开始；
* 推力最大限制为0.10，持续时间最大限制为3秒；
* 检测到其他MAVROS setpoint发布者时拒绝运行；
* 不会绕过PX4的电源、传感器、RC或其他解锁检查。

使用示例（必须先启动MAVROS，并拆除全部桨叶）：

    roslaunch px4_takeoff_test bench_motor_test.launch \\
      confirm_propellers_removed:=true \\
      confirm_low_thrust_test:=true \\
      thrust:=0.05 duration:=2.0
"""

import math
import sys
import time

import rosgraph
import rospy
from mavros_msgs.msg import AttitudeTarget, ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Imu


class BenchTestError(RuntimeError):
    """台架测试的参数、前置检查或状态转换失败。"""


class BenchMotorTest:
    """执行一次短时间、低推力、自动锁定的无桨台架测试。"""

    # 这些是写死在代码中的安全边界，launch参数不能突破。
    MAX_THRUST = 0.10      # MAVROS AttitudeTarget归一化推力上限，范围0~1。
    MAX_DURATION = 3.0     # 电机实际接收非零推力的最长时间，单位秒。
    MIN_RATE = 20.0        # setpoint最低频率；PX4需要持续的OFFBOARD心跳。
    MAX_RATE = 100.0       # 避免测试脚本无意义地占用过多CPU和串口带宽。
    DATA_TIMEOUT = 1.0     # 状态或IMU超过1秒未更新即视为失效。
    MODE_RESTORE_TIMEOUT = 3.0  # DISARM确认后等待恢复原模式的最长时间。

    def __init__(self):
        # mavros_ns允许MAVROS运行在非默认命名空间；其余所有话题和服务都由
        # _topic()统一拼接，避免在代码中散落硬编码路径。
        self.mavros_ns = str(rospy.get_param("~mavros_ns", "/mavros")).rstrip("/")

        # thrust是0~1归一化总推力而不是PWM值；duration只计算非零推力阶段；
        # rate_hz是AttitudeTarget发布频率；prestream_seconds是在切OFFBOARD前
        # 发送零推力setpoint的时间。
        self.thrust = float(rospy.get_param("~thrust", 0.05))
        self.duration = float(rospy.get_param("~duration", 2.0))
        self.rate_hz = float(rospy.get_param("~setpoint_rate", 50.0))
        self.prestream_seconds = float(rospy.get_param("~prestream_seconds", 2.0))
        # PX4接受DISARM服务后，/mavros/state可能稍晚才更新，因此单独提供
        # 状态确认超时。该时间只用于等待反馈，不会延长非零推力阶段。
        self.disarm_timeout = float(rospy.get_param("~disarm_timeout", 5.0))
        # 两个确认量必须由操作者显式设为true，默认启动不会转动电机。
        self.confirm_propellers_removed = bool(
            rospy.get_param("~confirm_propellers_removed", False)
        )
        self.confirm_low_thrust_test = bool(
            rospy.get_param("~confirm_low_thrust_test", False)
        )

        # 三份消息分别保存PX4连接/模式、落地状态和当前姿态。
        self.state = State()
        self.extended_state = ExtendedState()
        self.imu = None

        # 使用系统单调时钟保存最后接收时间，专门用于数据新鲜度判断。
        self.state_received_at = None
        self.extended_state_received_at = None
        self.imu_received_at = None
        # previous_mode用于测试完成后恢复模式；cleanup_started保证清理函数
        # 即使被finally和rospy.on_shutdown重复调用也只执行一次。
        self.previous_mode = None
        self.cleanup_started = False

        # 只有本节点确实发出过ARM请求，清理阶段才有权请求DISARM。
        # 这样如果节点误在一架原本已解锁的飞机上启动，它只会拒绝运行，绝不会
        # 把不属于本次测试的飞机直接锁定。
        self.arm_request_sent = False

        self._validate_parameters()

        # 缓存实际使用的话题和服务名称，便于日志、冲突检查和ServiceProxy复用。
        self.attitude_topic = self._topic("setpoint_raw/attitude")
        self.arm_service_name = self._topic("cmd/arming")
        self.mode_service_name = self._topic("set_mode")

        # 输入只用于状态监视；唯一控制输出是AttitudeTarget发布器和两个服务。
        rospy.Subscriber(
            self._topic("state"), State, self._state_callback, queue_size=10
        )
        rospy.Subscriber(
            self._topic("extended_state"),
            ExtendedState,
            self._extended_state_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            self._topic("imu/data"), Imu, self._imu_callback, queue_size=20
        )
        self.attitude_pub = rospy.Publisher(
            self.attitude_topic, AttitudeTarget, queue_size=20
        )
        self.arm_service = rospy.ServiceProxy(self.arm_service_name, CommandBool)
        self.mode_service = rospy.ServiceProxy(self.mode_service_name, SetMode)

        # 即使用户按Ctrl+C，也尽最大努力先发送零推力并锁定。
        rospy.on_shutdown(self._cleanup)

    def _topic(self, suffix):
        """把MAVROS命名空间和相对话题/服务名拼接为绝对名称。"""
        return self.mavros_ns + "/" + suffix.lstrip("/")

    def _validate_parameters(self):
        """对危险参数设置硬上限，并要求两个独立的人工确认。"""
        if not self.confirm_propellers_removed:
            raise BenchTestError("未确认已拆除全部桨叶，拒绝启动")
        if not self.confirm_low_thrust_test:
            raise BenchTestError("未确认执行低推力台架测试，拒绝启动")
        if not math.isfinite(self.thrust) or not 0.0 < self.thrust <= self.MAX_THRUST:
            raise BenchTestError(
                "thrust必须大于0且不超过%.2f" % self.MAX_THRUST
            )
        if not math.isfinite(self.duration) or not 0.1 <= self.duration <= self.MAX_DURATION:
            raise BenchTestError(
                "duration必须在0.1到%.1f秒之间" % self.MAX_DURATION
            )
        if not self.MIN_RATE <= self.rate_hz <= self.MAX_RATE:
            raise BenchTestError(
                "setpoint_rate必须在%.0f到%.0f Hz之间"
                % (self.MIN_RATE, self.MAX_RATE)
            )
        if not 1.0 <= self.prestream_seconds <= 5.0:
            raise BenchTestError("prestream_seconds必须在1到5秒之间")
        if not math.isfinite(self.disarm_timeout) or not 2.0 <= self.disarm_timeout <= 10.0:
            raise BenchTestError("disarm_timeout必须在2到10秒之间")

    def _state_callback(self, message):
        """保存PX4连接、模式和解锁状态，并记录本地接收时刻。"""
        self.state = message
        self.state_received_at = time.monotonic()

    def _extended_state_callback(self, message):
        """保存PX4的落地状态；测试只接受LANDED_STATE_ON_GROUND。"""
        self.extended_state = message
        self.extended_state_received_at = time.monotonic()

    def _imu_callback(self, message):
        """保存当前IMU姿态，用作姿态目标，避免切OFFBOARD时姿态跳变。"""
        self.imu = message
        self.imu_received_at = time.monotonic()

    @staticmethod
    def _is_fresh(received_at):
        """判断一类输入是否在DATA_TIMEOUT内更新过。"""
        return (
            received_at is not None
            and time.monotonic() - received_at < BenchMotorTest.DATA_TIMEOUT
        )

    def _imu_is_valid(self):
        """姿态目标使用当前IMU四元数，因此必须保证其新鲜且有效。"""
        if self.imu is None or not self._is_fresh(self.imu_received_at):
            return False
        q = self.imu.orientation
        values = (q.x, q.y, q.z, q.w)
        # 除检查NaN/Inf外，还排除长度近似为0的非法四元数。
        return all(math.isfinite(value) for value in values) and sum(
            value * value for value in values
        ) > 1e-6

    def _wait_for_inputs(self):
        """等待MAVROS状态、落地状态和IMU，并执行起始状态检查。"""
        deadline = time.monotonic() + 15.0  # 全部输入共同使用的等待截止时间。
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            ready = (
                self._is_fresh(self.state_received_at)
                and self.state.connected
                and self._is_fresh(self.extended_state_received_at)
                and self._imu_is_valid()
            )
            if ready:
                break
            rate.sleep()
        else:
            raise BenchTestError("等待MAVROS连接、落地状态或IMU超时")

        if self.state.armed:
            raise BenchTestError("PX4已经解锁；测试必须从锁定状态开始")
        if self.extended_state.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
            raise BenchTestError("PX4未明确报告ON_GROUND，拒绝执行")
        self.previous_mode = self.state.mode

    def _wait_for_services(self):
        """确认模式切换和解锁服务可用。"""
        try:
            rospy.wait_for_service(self.mode_service_name, timeout=10.0)
            rospy.wait_for_service(self.arm_service_name, timeout=10.0)
        except rospy.ROSException as exc:
            raise BenchTestError("MAVROS服务不可用: %s" % exc)

    def _reject_conflicting_publishers(self):
        """拒绝与px4ctrl、safe_takeoff等外部控制节点同时运行。

        MAVROS本身会发布 ``target_local``、``target_attitude``、``desired``
        等目标回显/反馈话题。它们虽然也位于 ``/mavros/setpoint_*`` 下，但不
        是另一个控制器向PX4发送命令，因此必须忽略MAVROS节点自身。
        """
        try:
            publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        except Exception as exc:  # ROS master/XMLRPC异常统一转为安全拒绝。
            raise BenchTestError("无法检查ROS发布者: %s" % exc)

        prefix = self.mavros_ns + "/setpoint_"
        own_node = rospy.get_name()
        mavros_node = self.mavros_ns
        conflicts = []  # 保存“节点名 (话题名)”，最终一次性打印给操作者。
        for topic, nodes in publishers:
            if topic.startswith(prefix):
                conflicts.extend(
                    "%s (%s)" % (node, topic)
                    for node in nodes
                    # 忽略本测试节点和MAVROS自身。其他节点只要向任意
                    # setpoint_*话题发布，就可能与本测试争夺控制权。
                    if node not in (own_node, mavros_node)
                )
        if conflicts:
            raise BenchTestError(
                "检测到其他MAVROS控制指令发布者，请先停止: " + ", ".join(conflicts)
            )

    def _make_target(self, thrust):
        """保持当前姿态，仅设置受限的归一化总推力。"""
        if not self._imu_is_valid():
            raise BenchTestError("IMU姿态超时或无效")
        message = AttitudeTarget()
        message.header.stamp = rospy.Time.now()
        # 忽略三个角速度字段，告诉PX4使用orientation四元数和thrust字段。
        message.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE
            | AttitudeTarget.IGNORE_PITCH_RATE
            | AttitudeTarget.IGNORE_YAW_RATE
        )
        message.orientation = self.imu.orientation
        message.thrust = max(0.0, min(float(thrust), self.MAX_THRUST))
        return message

    def _publish(self, thrust):
        """构造并发布一帧姿态/推力目标。"""
        self.attitude_pub.publish(self._make_target(thrust))

    def _stream(self, seconds, thrust):
        """在指定时间内连续发送setpoint，维持PX4的OFFBOARD心跳。"""
        deadline = time.monotonic() + seconds
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if not self.state.connected:
                raise BenchTestError("MAVROS与PX4连接断开")
            self._publish(thrust)
            rate.sleep()
        if rospy.is_shutdown():
            raise BenchTestError("收到停止请求")

    def _enter_offboard(self):
        """保持零推力setpoint，并重复请求OFFBOARD直到成功或超时。"""
        deadline = time.monotonic() + 8.0
        next_attempt = 0.0
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._publish(0.0)
            if self.state.mode == "OFFBOARD":
                return
            now = time.monotonic()
            if now >= next_attempt:
                try:
                    response = self.mode_service(base_mode=0, custom_mode="OFFBOARD")
                    if not response.mode_sent:
                        rospy.logwarn("PX4拒绝进入OFFBOARD")
                except rospy.ServiceException as exc:
                    rospy.logwarn("模式服务调用失败: %s", exc)
                next_attempt = now + 1.0
            rate.sleep()
        raise BenchTestError("PX4未进入OFFBOARD")

    def _arm(self):
        """保持零推力setpoint，并请求解锁；PX4的全部预检仍然有效。"""
        deadline = time.monotonic() + 8.0
        next_attempt = 0.0
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._publish(0.0)
            if self.state.armed:
                return
            if self.state.mode != "OFFBOARD":
                raise BenchTestError("解锁前PX4退出了OFFBOARD")
            now = time.monotonic()
            if now >= next_attempt:
                try:
                    # 从这一刻开始，本节点对可能发生的ARM负责；即使服务响应
                    # 丢失，finally中的_cleanup也会尝试DISARM。
                    self.arm_request_sent = True
                    response = self.arm_service(value=True)
                    if not response.success:
                        rospy.logwarn("PX4拒绝ARM，请查看QGC预检消息")
                except rospy.ServiceException as exc:
                    rospy.logwarn("解锁服务调用失败: %s", exc)
                next_attempt = now + 1.0
            rate.sleep()
        raise BenchTestError("PX4未能解锁")

    def _run_motor_stage(self):
        """在硬限制时间内发送低推力，并持续检查连接、模式和解锁状态。"""
        rospy.logwarn(
            "电机台架测试开始：thrust=%.3f，duration=%.1f秒；全部桨叶必须已拆除",
            self.thrust,
            self.duration,
        )
        deadline = time.monotonic() + self.duration
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if not self.state.connected:
                raise BenchTestError("测试中PX4连接断开")
            if self.state.mode != "OFFBOARD":
                raise BenchTestError("测试中PX4退出OFFBOARD")
            if not self.state.armed:
                raise BenchTestError("测试中PX4意外锁定")
            self._publish(self.thrust)
            rate.sleep()

    def _publish_zero_best_effort(self):
        """尽最大努力发送一帧零推力；清理阶段不让发布异常中断DISARM。"""
        try:
            if self.imu is not None:
                self._publish(0.0)
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "清理阶段发送零推力失败: %s", exc)

    def _disarm_and_wait(self):
        """每秒重试DISARM，并等待/mavros/state确认armed=false。

        服务返回success只表示PX4接受了请求，最终状态仍以/mavros/state为准。
        等待期间持续发送零推力，避免仍在OFFBOARD时留下非零目标。
        """
        if not self.arm_request_sent or not self.state.armed:
            return True

        deadline = time.monotonic() + self.disarm_timeout
        next_attempt = 0.0
        sleep_seconds = 1.0 / self.rate_hz

        while time.monotonic() < deadline:
            self._publish_zero_best_effort()
            if not self.state.armed:
                rospy.loginfo("已从/mavros/state确认PX4锁定")
                return True

            now = time.monotonic()
            if now >= next_attempt:
                try:
                    response = self.arm_service(value=False)
                    if response.success:
                        rospy.loginfo("PX4已接受DISARM请求，等待ARM状态更新")
                    else:
                        rospy.logwarn("PX4拒绝DISARM请求，将继续重试")
                except (rospy.ServiceException, rospy.ROSException) as exc:
                    rospy.logwarn("DISARM服务调用失败，将继续重试: %s", exc)
                next_attempt = now + 1.0

            # 使用系统sleep，使ROS进入shutdown流程时仍可完成有限时间的清理。
            time.sleep(sleep_seconds)

        # 截止点之后再读取一次最新回调状态，避免边界时刻误报。
        return not self.state.armed

    def _restore_previous_mode_and_wait(self):
        """确认锁定后恢复测试前模式，并等待状态反馈；失败只警告不再ARM。"""
        if (
            not self.state.connected
            or self.state.armed
            or not self.previous_mode
            or self.previous_mode == "OFFBOARD"
        ):
            return

        deadline = time.monotonic() + self.MODE_RESTORE_TIMEOUT
        next_attempt = 0.0
        sleep_seconds = 1.0 / self.rate_hz
        while time.monotonic() < deadline:
            if self.state.mode == self.previous_mode:
                rospy.loginfo("PX4已恢复测试前模式: %s", self.previous_mode)
                return

            self._publish_zero_best_effort()
            now = time.monotonic()
            if now >= next_attempt:
                try:
                    response = self.mode_service(
                        base_mode=0, custom_mode=self.previous_mode
                    )
                    if not response.mode_sent:
                        rospy.logwarn("PX4拒绝恢复模式%s", self.previous_mode)
                except (rospy.ServiceException, rospy.ROSException) as exc:
                    rospy.logwarn("恢复模式服务调用失败: %s", exc)
                next_attempt = now + 1.0
            time.sleep(sleep_seconds)

        rospy.logwarn(
            "PX4已锁定，但未在%.1f秒内确认恢复模式%s；当前模式=%s",
            self.MODE_RESTORE_TIMEOUT,
            self.previous_mode,
            self.state.mode,
        )

    def _cleanup(self):
        """幂等清理：零推力、等待确认锁定，再等待恢复原模式。"""
        if self.cleanup_started:
            return
        self.cleanup_started = True

        # 先发若干帧零推力，再请求锁定。即使ROS正在退出，也使用短暂的系统睡眠
        # 尽可能完成请求；PX4仍是最终决定是否接受锁定的一方。
        for _ in range(5):
            self._publish_zero_best_effort()
            time.sleep(0.02)

        disarm_confirmed = self._disarm_and_wait()
        if disarm_confirmed:
            self._restore_previous_mode_and_wait()

        if not disarm_confirmed and self.arm_request_sent:
            rospy.logfatal(
                "%.1f秒内未确认自动锁定成功，请立即使用实体急停/安全开关",
                self.disarm_timeout,
            )
        elif self.arm_request_sent:
            rospy.loginfo("台架测试结束，PX4已锁定")

    def run(self):
        """执行完整流程：前检、预发送、OFFBOARD、ARM、低推力、自动锁定。"""
        self._wait_for_inputs()
        self._wait_for_services()
        self._reject_conflicting_publishers()
        rospy.loginfo("前置检查通过，先发送%.1f秒零推力setpoint", self.prestream_seconds)

        try:
            self._stream(self.prestream_seconds, 0.0)
            self._enter_offboard()
            self._arm()
            self._run_motor_stage()
        finally:
            self._cleanup()


def main():
    """初始化ROS节点，执行一次测试，并用退出码表示成功或拒绝。"""
    rospy.init_node("bench_motor_test")
    try:
        BenchMotorTest().run()
    except (BenchTestError, ValueError) as exc:
        rospy.logerr("无桨台架测试拒绝/中止: %s", exc)
        return 1
    except rospy.ROSInterruptException:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
