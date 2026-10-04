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

## 目录整理

将 launch/config 移到根目录，任务入口统一为 commander.launch；五个业务包与讨论中的名称一致。
mock源文件移到 test/；三个基础支持包集中到 src/support/；PX4源码移到 third_party/，删除活动目录中的旧历史文档。
新增布局测试检查唯一配置/启动入口、source YAML 内部 launch路径、mock导出、devel符号链接及catkin安装路径。
目录整理只改变文件组织、包名与引用，没有调整飞行控制策略。
目录整理后的完整软件测试：39项通过；其中两个新增测试验证根目录布局和catkin对嵌套support包的发现。

## 第三方与节点路径整理

OpenVINS源码原样移到third_party/open_vins，src/support/open_vins仅为catkin发现链接。五个业务包去除scripts子目录，CMake导出与launch可执行文件名同步验证。
六个测试模块保留；test/support.py为公共路径辅助，mock源与依赖清单保留。CI改用scripts/run_checks.sh作为与本地相同的唯一运行入口。
完整软件测试39项通过；测试验证catkin发现OpenVINS包、链接目的地、直接节点路径和launch可执行文件引用。

## 自有ROS包合并与定位配置重命名

合并为单一uav_system包，五个业务目录继续独立，原重复CMake/package.xml移除；消息由uav_system生成，共享Python从support/uav_core安装。
config/state_sources替代config/sources。结构测试核对唯一自有包、全部节点导出、消息文件、launch与YAML路径以及第三方包发现，39项测试通过。
使用官方catkin的interrogate_setup_dot_py.py验证Python安装描述，结果为uav_core模块路径../support/uav_core。仍未执行ROS/catkin全构建或消息生成。
