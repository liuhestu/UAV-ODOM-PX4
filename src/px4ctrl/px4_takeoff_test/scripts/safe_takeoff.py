#!/usr/bin/env python3
"""使用MAVROS位置OFFBOARD模式执行带前置检查的起飞、悬停和降落测试。

该脚本读取PX4本地位置，以起飞瞬间的位置为原点，缓慢增加目标z坐标；达到
目标高度后保持指定时间，最后请求PX4进入AUTO.LAND。默认
``confirm_flight=false``，只做检查而不会切换模式或解锁。

真实飞行示例（必须使用真实定位，禁止使用fixed_odometry_test.py）：

    roslaunch px4_takeoff_test safe_takeoff.launch \\
      confirm_flight:=true altitude:=2.0 hover_seconds:=10.0

Ctrl+C不会直接在空中停转电机；脚本会请求AUTO.LAND。PX4的全部预检和失效
保护始终有效，脚本不会绕过ARM拒绝。
"""

import copy
import math
import signal
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode


class FlightError(RuntimeError):
    """前置检查、模式切换或飞行阶段失败时抛出的可预期异常。"""


class SafeTakeoff:
    # 写死在代码中的基本输入边界，防止launch参数出现明显危险值。
    MIN_ALTITUDE = 0.5       # 相对起飞点的最小爬升高度，单位m。
    MAX_ALTITUDE = 5.0       # 示例脚本允许的最大爬升高度，单位m。
    MIN_SETPOINT_RATE = 10.0 # OFFBOARD位置目标的最低发布频率，单位Hz。

    def __init__(self):
        # 飞行轨迹参数：相对爬升高度、爬升速度和悬停时长。
        self.altitude = float(rospy.get_param("~altitude", 2.0))
        self.climb_rate = float(rospy.get_param("~climb_rate", 0.5))
        self.hover_seconds = float(rospy.get_param("~hover_seconds", 10.0))
        # 只有confirm_flight=true才会进入OFFBOARD并请求ARM；默认仅检查。
        self.confirm_flight = bool(rospy.get_param("~confirm_flight", False))

        # MAVROS命名空间及setpoint频率；rstrip避免拼接出双斜杠。
        self.mavros_ns = str(rospy.get_param("~mavros_ns", "/mavros")).rstrip("/")
        self.rate_hz = float(rospy.get_param("~setpoint_rate", 20.0))

        # 各阶段超时。prestream_seconds是进入OFFBOARD前预发送setpoint的时间。
        self.connect_timeout = float(rospy.get_param("~connect_timeout", 30.0))
        self.position_timeout = float(rospy.get_param("~position_timeout", 30.0))
        self.prestream_seconds = float(rospy.get_param("~prestream_seconds", 2.0))
        self.land_timeout = float(rospy.get_param("~land_timeout", 60.0))

        # 最新PX4状态、落地状态和本地位姿反馈。
        self.state = State()
        self.extended_state = ExtendedState()
        self.pose = None
        self.pose_received_at = None

        # stop_requested由信号处理函数设置；takeoff_z保存起飞点高度；target是
        # 反复更新时间戳和z坐标后发布的PoseStamped目标。
        self.stop_requested = False
        self.flight_started = False
        self.takeoff_z = None
        self.target = None

        self._validate_parameters()

        # 三个订阅器只读取反馈；setpoint_pub是唯一的连续控制输出。
        rospy.Subscriber(self._topic("state"), State, self._state_callback, queue_size=10)
        rospy.Subscriber(
            self._topic("extended_state"),
            ExtendedState,
            self._extended_state_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            self._topic("local_position/pose"),
            PoseStamped,
            self._pose_callback,
            queue_size=10,
        )
        self.setpoint_pub = rospy.Publisher(
            self._topic("setpoint_position/local"), PoseStamped, queue_size=20
        )

        self.arm_service_name = self._topic("cmd/arming")
        self.mode_service_name = self._topic("set_mode")
        self.arm_service = rospy.ServiceProxy(self.arm_service_name, CommandBool)
        self.mode_service = rospy.ServiceProxy(self.mode_service_name, SetMode)

    def _topic(self, suffix):
        """将MAVROS命名空间与相对话题/服务名拼接为绝对名称。"""
        return self.mavros_ns + "/" + suffix.lstrip("/")

    def _validate_parameters(self):
        """检查高度、速度、频率等参数是否处于脚本允许范围。"""
        if not self.MIN_ALTITUDE <= self.altitude <= self.MAX_ALTITUDE:
            raise FlightError(
                "altitude must be between %.1f and %.1f metres"
                % (self.MIN_ALTITUDE, self.MAX_ALTITUDE)
            )
        if not 0.1 <= self.climb_rate <= 2.0:
            raise FlightError("climb_rate must be between 0.1 and 2.0 m/s")
        if self.hover_seconds < 0.0:
            raise FlightError("hover_seconds must not be negative")
        if self.rate_hz < self.MIN_SETPOINT_RATE:
            raise FlightError("setpoint_rate must be at least 10 Hz")
        if self.prestream_seconds < 1.0:
            raise FlightError("prestream_seconds must be at least 1 second")

    def _state_callback(self, message):
        """保存PX4连接状态、当前模式和ARM状态。"""
        self.state = message

    def _extended_state_callback(self, message):
        """保存PX4报告的空中/地面状态。"""
        self.extended_state = message

    def _pose_callback(self, message):
        """深拷贝最新本地位姿，并记录本机接收时刻用于超时检查。"""
        self.pose = copy.deepcopy(message)
        self.pose_received_at = rospy.Time.now()

    def _pose_is_fresh(self):
        """确认已收到1秒内的、位置数值有限的本地位姿。"""
        if self.pose is None or self.pose_received_at is None:
            return False
        age = (rospy.Time.now() - self.pose_received_at).to_sec()
        position = self.pose.pose.position
        return age < 1.0 and all(
            math.isfinite(value) for value in (position.x, position.y, position.z)
        )

    def _wait_until(self, predicate, timeout, description):
        """通用条件等待器；超时或收到停止请求时抛出FlightError。"""
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.stop_requested:
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise FlightError("timed out waiting for " + description)
            rate.sleep()
        raise FlightError("stopped while waiting for " + description)

    def wait_for_preflight_data(self):
        """等待连接、位置和服务，并确认飞机未解锁且位于地面。"""
        rospy.loginfo("Waiting for MAVROS FCU connection on %s", self.mavros_ns)
        self._wait_until(lambda: self.state.connected, self.connect_timeout, "FCU connection")
        rospy.loginfo("FCU connected; waiting for a valid local position")
        self._wait_until(self._pose_is_fresh, self.position_timeout, "local position")

        rospy.loginfo("Waiting for MAVROS arming and mode services")
        try:
            rospy.wait_for_service(self.arm_service_name, timeout=10.0)
            rospy.wait_for_service(self.mode_service_name, timeout=10.0)
        except rospy.ROSException as exc:
            raise FlightError("MAVROS services unavailable: %s" % exc)

        if self.state.armed:
            raise FlightError("vehicle is already armed; refusing to start")
        if self.extended_state.landed_state not in (
            ExtendedState.LANDED_STATE_UNDEFINED,
            ExtendedState.LANDED_STATE_ON_GROUND,
        ):
            raise FlightError("vehicle does not report that it is on the ground")

        # 目标高度采用相对值：最终z = 起飞瞬间z + altitude。
        self.takeoff_z = self.pose.pose.position.z
        self.target = copy.deepcopy(self.pose)
        self.target.header.frame_id = self.target.header.frame_id or "map"
        orientation = self.target.pose.orientation
        norm = math.sqrt(
            orientation.x ** 2
            + orientation.y ** 2
            + orientation.z ** 2
            + orientation.w ** 2
        )
        if norm < 1e-6:
            orientation.w = 1.0

        rospy.loginfo(
            "Preflight OK: local position=(%.2f, %.2f, %.2f), requested climb=%.2f m",
            self.target.pose.position.x,
            self.target.pose.position.y,
            self.target.pose.position.z,
            self.altitude,
        )

    def _publish(self, z=None):
        """更新时间戳和可选z坐标，然后发布一帧位置setpoint。"""
        if self.target is None:
            return
        if z is not None:
            self.target.pose.position.z = z
        self.target.header.stamp = rospy.Time.now()
        self.setpoint_pub.publish(self.target)

    def _stream_for(self, seconds, z):
        """在指定时长内持续发布同一高度，并检查FCU连接。"""
        deadline = time.monotonic() + seconds
        rate = rospy.Rate(self.rate_hz)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.stop_requested:
                raise FlightError("stop requested")
            if not self.state.connected:
                raise FlightError("FCU connection lost")
            self._publish(z)
            rate.sleep()

    def _set_mode(self, mode, timeout=8.0, publish_z=None):
        """每秒请求一次目标模式，等待期间持续发送位置setpoint。"""
        deadline = time.monotonic() + timeout
        next_attempt = 0.0
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and not self.stop_requested:
            if self.state.mode == mode:
                rospy.loginfo("PX4 mode is %s", mode)
                return
            now = time.monotonic()
            if now >= deadline:
                raise FlightError("PX4 did not enter %s mode" % mode)
            if now >= next_attempt:
                try:
                    response = self.mode_service(base_mode=0, custom_mode=mode)
                    if not response.mode_sent:
                        rospy.logwarn("PX4 rejected mode request: %s", mode)
                except rospy.ServiceException as exc:
                    rospy.logwarn("Mode service call failed: %s", exc)
                next_attempt = now + 1.0
            if publish_z is not None:
                self._publish(publish_z)
            rate.sleep()
        raise FlightError("stopped while setting mode " + mode)

    def _set_armed(self, armed, timeout=8.0, publish_z=None):
        """请求ARM/DISARM并等待状态确认，不把服务调用成功等同于状态成功。"""
        deadline = time.monotonic() + timeout
        next_attempt = 0.0
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and not self.stop_requested:
            if self.state.armed == armed:
                rospy.loginfo("Vehicle %s", "armed" if armed else "disarmed")
                return
            now = time.monotonic()
            if now >= deadline:
                raise FlightError("PX4 did not %s" % ("arm" if armed else "disarm"))
            if now >= next_attempt:
                try:
                    response = self.arm_service(value=armed)
                    if not response.success:
                        rospy.logwarn("PX4 rejected %s request", "arm" if armed else "disarm")
                except rospy.ServiceException as exc:
                    rospy.logwarn("Arming service call failed: %s", exc)
                next_attempt = now + 1.0
            if publish_z is not None:
                self._publish(publish_z)
            rate.sleep()
        raise FlightError("stopped while changing arming state")

    def _check_flight_state(self):
        """飞行中持续验证连接、ARM、OFFBOARD和位置新鲜度。"""
        if not self.state.connected:
            raise FlightError("FCU connection lost during flight")
        if not self.state.armed:
            raise FlightError("vehicle unexpectedly disarmed during flight")
        if self.state.mode != "OFFBOARD":
            raise FlightError("vehicle left OFFBOARD mode: " + self.state.mode)
        if not self._pose_is_fresh():
            raise FlightError("local position became unavailable during flight")

    def climb(self):
        """按climb_rate线性增加目标z，直到反馈到达目标高度附近。"""
        target_z = self.takeoff_z + self.altitude
        start_time = time.monotonic()
        timeout = self.altitude / self.climb_rate + 15.0
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("Ascending to local z=%.2f m", target_z)

        while not rospy.is_shutdown() and not self.stop_requested:
            self._check_flight_state()
            elapsed = time.monotonic() - start_time
            commanded_z = min(self.takeoff_z + self.climb_rate * elapsed, target_z)
            self._publish(commanded_z)

            actual_z = self.pose.pose.position.z
            if commanded_z >= target_z and abs(actual_z - target_z) <= 0.25:
                rospy.loginfo("Takeoff altitude reached: local z=%.2f m", actual_z)
                return target_z
            if elapsed >= timeout:
                raise FlightError("vehicle did not reach the requested altitude")
            rate.sleep()
        raise FlightError("takeoff interrupted")

    def hover(self, target_z):
        """在目标高度持续发布位置setpoint，直到悬停时间结束。"""
        rospy.loginfo("Hovering for %.1f seconds", self.hover_seconds)
        deadline = time.monotonic() + self.hover_seconds
        rate = rospy.Rate(self.rate_hz)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.stop_requested:
                raise FlightError("stop requested during hover")
            self._check_flight_state()
            self._publish(target_z)
            rate.sleep()

    def land(self):
        """请求AUTO.LAND，并等待PX4落地后自动锁定。"""
        if not self.state.armed:
            return
        rospy.logwarn("Requesting AUTO.LAND")
        self.stop_requested = False
        try:
            self._set_mode("AUTO.LAND", timeout=8.0)
        except FlightError as exc:
            rospy.logerr("Could not enter AUTO.LAND: %s", exc)
            return

        deadline = time.monotonic() + self.land_timeout
        rate = rospy.Rate(10)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if not self.state.armed:
                rospy.loginfo("Landing complete; vehicle disarmed")
                return
            if self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                rospy.loginfo("Vehicle reports landed; waiting for automatic disarm")
            rate.sleep()
        if self.state.armed:
            rospy.logerr("Landing wait timed out; vehicle still reports armed")

    def run(self):
        """执行检查；确认飞行后依次预发送、OFFBOARD、ARM、爬升、悬停和降落。"""
        self.wait_for_preflight_data()
        if not self.confirm_flight:
            rospy.logwarn(
                "DRY RUN ONLY: checks passed, but confirm_flight is false. "
                "No OFFBOARD, arm, or takeoff command was sent."
            )
            return

        rospy.logwarn(
            "LIVE FLIGHT CONFIRMED: keep the area clear and maintain a manual abort option"
        )
        hold_z = self.takeoff_z
        rospy.loginfo("Pre-streaming position setpoints for %.1f seconds", self.prestream_seconds)
        self._stream_for(self.prestream_seconds, hold_z)
        self._set_mode("OFFBOARD", publish_z=hold_z)
        self._set_armed(True, publish_z=hold_z)
        self.flight_started = True

        target_z = self.climb()
        self.hover(target_z)
        self.land()


def main():
    """安装安全信号处理，运行控制器，并保证异常时优先请求AUTO.LAND。"""
    rospy.init_node("safe_takeoff", disable_signals=True)
    controller = None

    def request_stop(_signum, _frame):
        """SIGINT/SIGTERM回调：只请求停止，由主流程负责安全降落。"""
        if controller is not None:
            controller.stop_requested = True
        rospy.logwarn("Stop requested; the node will request AUTO.LAND if armed")

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    exit_code = 0
    try:
        controller = SafeTakeoff()
        controller.run()
    except FlightError as exc:
        rospy.logerr("Flight aborted: %s", exc)
        exit_code = 1
    except rospy.ROSInterruptException:
        exit_code = 1
    except Exception as exc:  # Keep an unexpected exception from abandoning an armed vehicle.
        rospy.logfatal("Unexpected error: %s", exc)
        exit_code = 1
    finally:
        if controller is not None and controller.state.armed:
            controller.land()
        rospy.signal_shutdown("safe_takeoff finished")

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
