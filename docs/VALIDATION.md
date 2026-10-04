# 第一版云端验证记录

日期：2026-10-04。基线：`301af823ee526855bcf5b96dc725d4c5dc8d1177`。分支：`refactor/native-px4-ros1`。

## 已执行

- `python3 -m unittest discover -s test -v`：37 项通过。
- Python AST/compile 检查；本地包 manifest 经 catkin_pkg 解析；launch XML、节点可执行文件和源 YAML schema 检查通过。
- SE(3)姿态/世界对齐、杠杆臂速度、协方差PSD及有限差分杠杆臂 Jacobian 检查通过。
- wall-time heartbeat过期、ROS时间暂停/回退、最近接收但测量过期的 watchdog 测试通过。
- readiness未知/失败 gate、未标定解锁限制、mock硬件限制；Commander正常流程、拒绝、超时、session变化、人工接管、空中禁上锁与终止不续飞测试通过。
- PX4 listener 文本缺字段/旧时间/融合拒绝/错误实例 fail-closed，以及 SERIAL_CONTROL 字节/CRC 与官方 pymavlink 对照通过。
- `git diff --check`：通过。
- 对照仓库中 OpenVINS Propagator/ROS1Visualizer、保留 PX4 uORB/Tools/mavlink_shell，以及 MAVROS ROS1与RealSense ros1-legacy源代码核对接口。

## 未执行

当前容器无 `/opt/ros`、rospy、catkin或ROS master。因此未进行 catkin构建、ROS节点通信、roslaunch实际运行、PX4 SITL或实机试验。
纯软件测试不证明云端ROS集成通过，也不证明飞控接受/融合实机OpenVINS数据。

没有执行任何 ARM、DISARM、Offboard或飞行指令，没有连接实机串口；没有写入PX4参数、修改固件控制代码或刷写固件。
本地接续必须记录新的版本、构建结果和各 gate 证据，参见 LOCAL_VALIDATION.md。
