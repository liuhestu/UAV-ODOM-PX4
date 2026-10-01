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

## Jetson 接续：2026-10-04/05

本节记录云端目录重构前（`1cbec26`）的验证。通过 `ssh jetson-uav-odom` 在 `/home/jetson/uav_odom_px4` 构建和运行，文件在 SSHFS 工作区编辑。分支仍为 `refactor/native-px4-ros1`。

### 环境与构建

- Jetson 提供 `/opt/ros/noetic` 和 Humble；本轮仅 source Noetic。默认 shell 还带有 yahboom interfaces underlay。
- Jetson 系统时间读为 2026-10-02，与客户端日期不一致；上述 ROS 样本使用 Jetson 同机时间，不把其时间戳解释为客户端采集日期。
- 37 项 unittest 在 Jetson 通过。系统 Python 缺 pymavlink；在线 pip 安装遇到 TLS/连接错误后停止，测试借用现有 `/home/jetson/px4_exp/.venv/lib/python3.10/site-packages`，未宣称系统依赖已安装齐全。
- `rosdep check` 在 Jammy 无法解析 Noetic RealSense key，并报告 apt OpenCV contrib 缺失；本机实际使用 `/usr/local` OpenCV，未直接安装 Focal 包覆盖已有环境。
- 运行 ROS C++ 程序需在 `LD_LIBRARY_PATH` 加 `/usr/local/lib`，否则 rosout 找不到本机已有的 `liblog4cxx.so.10`。
- RealSense 2.3.2 位于 `/home/jetson/jin_ws/uav_ws`；仅 source 系统 Noetic 不可发现该包。MAVROS 1.20.1。
- 旧布局下选定 8 个自有包及 OpenVINS core/init/msckf 共 11 个包全部 catkin 构建成功；2 个未选包跳过，2 个包有警告，无失败，耗时 16 分 53 秒。下述运行检查也基于旧布局。合并云端 `673a062` 后未重新构建，旧产物不代表新版消息/单包布局已通过。

### 独立软件门

独立 master `http://localhost:11321`，不连接 MAVROS。启动 mock_system 和 Commander；backend_output_enabled=false。

| 注入 | 实际结果 |
|---|---|
| static | adapter healthy=true；ready=false、arm_ready=false，缺少 PX4/EV 证据 |
| timeout | healthy=false，source or canonical odometry stale |
| 恢复 static | healthy=true |
| nan | healthy=false，invalid vector |
| jump | healthy=false，position jump/reset; restart system |
| jump 后恢复 static | 故障仍锁定，不自动恢复 |
| Commander | WAIT_SYSTEM |

日志：Jetson `/tmp/uav_native_mock.log`、`/tmp/uav_native_mock_commander.log`。最初 mock master 的 rosout 因上述库路径问题失败，Python 节点通信检查仍完成；后续应使用完整运行环境。

### 已有硬件链路的只读检查

- 已有 MAVROS 和外部工作区的 RealSense/OpenVINS 在运行，未重复启动。新增 `start_source:=false` 参数，统一使用原 `openvins.yaml`，manager 仅检测外部输入，不接管进程；不另建重复源 YAML。
- 相机序列号 `335222075432`。848×480 双目约 30 Hz；合并 IMU 约 200 Hz；OpenVINS odomimu 约 200 Hz。gyro/accel 配置均为 200，合并方式 linear_interpolation。
- `/camera/stereo_module/parameter_updates` 实际动态配置 emitter_enabled=0，确认关闭。
- 图像输入为 image_rect_raw。当前 CameraInfo D 全零、cam0 fx/fy=425.343353、cx=429.261169、cy=244.626053；继承 Kalibr 文件含非零 radtan 畸变且内参不同，匹配验证未通过，不能直接认定当前相机已标定。
- 外部 OpenVINS 样本位置约 `[2024, 13637, -1628]` m，线速度约 `[-12.09, 3.63, 5.77]` m/s，位置协方差巨大；该输出不能作为有效定位证据。输入心跳 healthy=true，但 adapter 检出位置跳变后锁定 unhealthy。设备静止/运动状态尚待用户说明。
- 实机只读基础链 `start_source:=false start_mavros:=false output_enabled:=false` 验证成功；ev_sent=false，ready=false、arm_ready=false。外参 calibrated 和世界对齐 verified 保持 false。
- observer 实际报告 `observer requires single EKF: EKF2_MULTI_IMU == 0`。参数读取为 EKF2_MULTI_IMU=2，EKF2_MULTI_MAG 读取失败；received/fused/reset_valid 均 false。当前 observer 不适配此实机多 EKF 配置，未写参数绕过门控。
- MAVROS 初始 connected=true、armed=false、MANUAL、landed_state=1。后续模式变为 OFFBOARD，仍 armed=false，system_status=0。12 秒原始 MAVLink HEARTBEAT 样本也显示 sysid/compid=1/1、custom_mode=393216、base_mode=17、system_status=0，非仅 MAVROS 显示变化。
- 用户表示未主动操作模式，并怀疑 RC 通道设置。RC 输入 `[1500,1500,1000,1500,1880,2000,1000,1500]`；RC_MAP_FLTMODE=5，RC_MAP_MODE_SW/OFFB_SW/ARM_SW=0，RC_MAP_THROTTLE=3，COM_RC_IN_MODE=0；COM_FLTMODE1..6=`[0,1,2,-1,-1,7]`。仓库 PX4 源码将 7 映射为 Offboard，但实机精确参数语义和 CH5 档位仍需核对固件与 RC 标定。
- RC5_MIN/MAX/TRIM/REV 实读为 1000/2000/1500/1；CH5 当前接近高端，模式档位是重点排查线索，但没有执行开关移动实验以证实因果关系。
- 检查时无其他 ROS Commander/px4ctrl 控制节点，setpoint_raw/local、rc/override、mavlink/to 无发布者（只读 observer 已退出）。这不足以排除 RC、QGC 或其他 MAVLink 链路来源，未确证模式变化原因。
- AUTOPILOT_VERSION 报告 flight_sw_version=17891583、flight_custom_version=d6f12ad1c4000000，后续应据此核对实机固件，而非以仓库版本代替。

实机诊断日志：Jetson `/tmp/uav_native_readonly.log`、`/tmp/uav_native_external_mode.log`。本轮未启动实机 Commander，没有 ARM/DISARM、模式请求、EV/setpoint 数据发送、PX4 参数写入或固件刷写。只读 observer 使用了固定 SERIAL_CONTROL 查询接口。硬件轴向/外参标定、有效 VIO、PX4 融合和 SITL/飞行门均未通过；应先解决上述阻塞。
