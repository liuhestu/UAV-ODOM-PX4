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


## 完整 catkin 工作区迁移：2026-10-05

基线 `0aae208`，分支 `refactor/native-px4-ros1`。工作区根目录保留 Git/CI、构建脚本和入口 README；业务包、文档、测试、配置与 PX4 源码归入真实目录 `src/uav_system/`，OpenVINS 源码位于真实目录 `src/open_vins/`。删除原三个源码/资源发现链接。

- 第三方源码保持内容不变：446 个 OpenVINS 文件和 13,194 个 PX4 文件按原跟踪清单迁移，包含新路径被 ignore 规则匹配的原跟踪文件；未修改固件或估计器算法。
- 原 build/devel/logs/.catkin_tools 保留在 `.legacy_catkin/20261005-011623/`。新工作区从空构建空间编译，未复用旧包/消息产物。
- Jetson 上全部 6 个源码包及 catkin prebuild 成功，首次编译 21 分 8 秒，3 个第三方包有编译警告，没有失败。首次脚本收尾因运行期间脚本更新而报错；最终版本 `./scripts/build.sh --no-status` 重跑并正常退出，全部 6 个源码包成功，最后一次增量耗时 3.6 秒。
- 构建入口显式使用 Noetic、2 个编译任务和 1 个并行包；检测 ROS2 环境时拒绝执行。调用参数转交 catkin build。
- 39 项软件测试通过；结构检查核对真实目录、独立包发现、源码导出和显式 launch 资源引用。
- rospack 实际解析 uav_system 到 `src/uav_system`、ov_msckf 到 `src/open_vins/ov_msckf`；新 devel 中 uav_core 和 SourceStatus/SystemStatus/EvStatus 可导入，消息类型均为 uav_system。
- 当前 mock 配置使用 `config/test/mock.yaml`，mock launch 显式加载；配置内容未变，测试与启动引用同步。实机源配置继续位于 `config/state_sources/`。
- 独立 ROS master `localhost:11323` 上启动新版 mock 与 Commander：static healthy，timeout/NaN unhealthy，恢复 static 可恢复健康；jump 锁定，切回 static 仍 unhealthy。ready/arm_ready=false、backend_output_enabled=false、ev_sent=false，Commander 保持 WAIT_SYSTEM。图中没有 MAVROS；验证结束已清理该轮子进程。
- 加载外部 RealSense 工作区后使用新 devel/setup.bash --extend，确认 RealSense 可被发现，同时新的 OpenVINS 包优先于旧工作区。

Jetson 日志：`/tmp/uav_workspace_layout_checks_final.log`、`/tmp/uav_workspace_layout_build.log`、`/tmp/uav_workspace_layout_build_final.log`、`/tmp/uav_layout_mock.log`、`/tmp/uav_layout_commander.log`。本次迁移未启动实机 Commander、未修改 PX4 参数或标定；以前记录的真实 VIO/融合/RC 阻塞仍未解决。本次验证不等于实机飞行或 PX4 固件构建通过。

## Commander 提前遥控器 ARM：2026-10-05

分支 `refactor/native-px4-ros1`，在 SSHFS 工作区编辑并通过 `ssh jetson-uav-odom` 在 Jetson 做软件验证。保留未跟踪的根目录 `AGENTS.md`，未提交或推送。

- 默认配置迁移为严格布尔值 `auto_arm: false`，只控制 ARM 请求；落地 DISARM 保持原流程。launch 使用独立字符串覆盖参数区分 YAML/显式启动来源，归一化后将有效值写回节点私有 `auto_arm`；launch 清理该节点旧私有参数，避免前一次运行遗留新旧键造成来源冲突。旧 `arm_method=auto/manual` 映射并警告；新旧冲突及非法类型拒绝。
- WAIT_SYSTEM 允许提前 ARM；健康、标定、通信、鲜活 local 和 ON_GROUND 连续通过稳定窗口，ARM 变化不重置窗口，session/条件中断重新计时。初始化期间地面未知/空中持续等待。
- 准备阶段每轮更新鲜活 local 地面目标；预发送、实际 OFFBOARD/ARM、健康与地面确认通过后固定起飞基准，当轮保持地面目标。提前 OFFBOARD/ARM 不重复请求；`auto_arm=false` 长期等待遥控器 ARM，true 最多请求一次并保留确认超时。
- 准备阶段故障或地面确认丢失停止命令并锁定，定位跳变不触发起飞前 LAND。首次鲜活 ARM 后主动上锁终止任务。保留飞行故障降落、人工接管、服务 watchdog、正常落地确认及禁止空中 DISARM。
- Supervisor 审核：`flight_supervisor.py` 的 gates 与 `readiness.evaluate` 不要求 armed=false；健康/标定/融合门未修改，无消息变更。Backend OFFBOARD 服务增加鲜活 ON_GROUND 门，已 ARM 的 ARM 请求返回成功但不转发；飞行设定点转发未增加地面门。

验证结果：

| 检查 | 结果 |
|---|---|
| `PYTHONPATH=/home/jetson/px4_exp/.venv/lib/python3.10/site-packages bash src/uav_system/scripts/run_checks.sh` | 61 项 unittest 通过，Python/XML/YAML/结构与 Git whitespace 检查通过 |
| `./scripts/build.sh --no-status` | 全部 6 个 catkin 包增量构建成功，6.3 秒，1 个包警告、无失败 |
| Noetic + 新 devel 环境下 `python3 src/uav_system/test/verify_commander_wait.py` | 独立 master `localhost:11329`；launch 默认值、显式 true/false、旧 auto/manual、参数及 YAML 冲突检查通过；默认/true/旧 manual 参数与旧 YAML 的 Commander 均保持 WAIT_SYSTEM，有效私有 auto_arm 符合配置 |
| mock static/timeout/nan/jump/恢复 | static healthy；timeout/nan unhealthy，恢复 static 后 healthy；jump 故障锁定，恢复 static 仍 unhealthy；ready/arm_ready/backend_output_enabled 始终 false |
| Backend/Commander runtime 服务回归 | 使用内存 ROS 替身；ON_GROUND 门、冗余 ARM、禁止空中 DISARM、准备阶段跳变停止、飞行跳变 LAND 和服务卡死 watchdog 通过；未连接 MAVROS/FCU |

系统 Python 首次完整检查因缺 pymavlink 失败；使用此前已有虚拟环境依赖后通过，未安装/更改系统依赖。ROS 软件图中没有 MAVROS 节点，本轮新建进程已清理。日志位于 Jetson `/tmp/uav_early_arm_checks.log`、`/tmp/uav_early_arm_build.log`、`/tmp/uav_early_arm_ros.log` 和 `/tmp/uav_early_arm_ros_98cr4yua/`。

未执行 PX4 SITL、实机 ARM/DISARM、模式切换、飞行、PX4 参数写入或固件刷写。提前 ARM 的任务运动行为由确定性/替身测试覆盖，mock 检查只验证未就绪等待，不代表真实 OFFBOARD 接管或飞行通过。SITL 与实机流程仍按 `LOCAL_VALIDATION.md` 分阶段授权验证。

## 无桨台架链路及 RC 解锁顺序修正：2026-10-05

本段为后续实机操作记录，区别于上面的软件验证。用户确认拆除桨叶，并请求真实 OpenVINS → PX4 接收/融合可观察 → 短时高度目标 → 电机响应。实机 PX4 为 1.17.0，git d6f12ad1c4。用户明确要求将 EKF2_EV_CTRL 永久设为 3；写入后及飞控重启后读回均为 3。未改解锁阈值、未强制 ARM、未刷固件。

- 原 OpenVINS 配置输出发散，adapter 检出跳变后停止 EV。临时配置位于 Jetson `/tmp/uav_bench_factory/`，使用当前相机 CameraInfo、厂家 camera/IMU TF 和约 49.923 mm 双目基线，开启 ZUPT 静态初始化。设备静止约 25 秒的位置跨度约 1.3 mm、最大速度约 0.0043 m/s，仅证明该次静止输出；不是人工精确标定或运动精度验收。生产外参 calibrated/world verified 保持 false。
- 固定只读 uORB 查询按 estimator_instance 字段匹配 selector primary，观察到当前主 EKF 的 EV 位置/高度 fused=true。原生 observer 仍因 EKF2_MULTI_IMU=2 拒绝报告融合，未伪造 ready。飞控重启后此前 High Accelerometer Bias 消失，一度 pre_flight_checks_pass=true；这不代表之后持续通过。
- 独立临时脚本 `/tmp/uav_bench_height_response.py` 直接预发送 MAVROS 目标、请求 OFFBOARD 和正常 ARM，确认实际状态后开始相对当轮 local 高度 +0.1 m 的目标。该脚本未通过 Commander 完整标定/融合门，不应计为三重门控验收；用户随后明确要求由遥控器 ARM。
- 高度目标期间 ON_GROUND 丢失，脚本停止并记录 FAILED，未完成计划中的完整保持/恢复周期，未重复测试。用户观察到电机转动。结束检查确认 OFFBOARD、armed=false、landed=1、四路输出 1000。PX4 日志记录外部命令解锁、failsafe、落地检测和落地上锁；结束检查脚本没有再发送 LAND/DISARM。
- 后续只读排查：RC_MAP_ARM_SW=6、RC_MAP_KILL_SW=7，RC 输入仍鲜活。日志反复报告 `Arming denied: switch to manual mode first`；模式残留 OFFBOARD、offboard_control_signal_lost=true，当前 pre_flight_checks_pass=false。ARM/KILL 均处于 OFF，未发送新的 ARM 或模式请求。
- 当前 ROS 图没有项目 Commander。source/adapter healthy=true、ev_sent=true，但 system ready/arm_ready=false，原因含 PX4 状态/估计器证据缺失、原生 EV observer 不支持当前配置、外参/世界对齐未验证。故不能声称 Commander 已成功执行，亦不能通过直接服务脚本证明健康门正确。

根据实机拒绝原因，修正 `auto_arm=false` 顺序：预发送后保持初始模式无限等待遥控器实际 ARM，再请求一次 OFFBOARD；实际 OFFBOARD 确认前保持地面目标。自动 ARM 顺序保留，人工接管/故障锁定保持。新增回归验证长期等待不请求模式/ARM、服务成功不替代实际模式、ARM 当轮健康失败不请求 OFFBOARD。仅做软件验证，不启动实机 Commander。遥控器解锁怠速独立于 Commander 任务门控。

修正后 Jetson `run_checks.sh` 的 62 项测试通过，全部 6 个 catkin 包增量构建成功（3.7 秒，无警告/失败），独立 master 的 `verify_commander_wait.py` 通过（日志 `/tmp/uav_early_arm_ros_4bmlsrvk`）。用户随后选择不 ARM；后续保持只读验证，不继续电机测试，也未执行所询问的 POSCTL 恢复或永久模式配置。

原始日志/证据保存在 Jetson `/tmp/uav_bench_height_response.log`、`/tmp/uav_bench_height_history.json`、`/tmp/uav_bench_finish.log`、`/tmp/uav_bench_rc_review.log` 和 `/tmp/uav_bench_px4_*snapshot.json`。未执行带桨测试、真实飞行、精确标定或 SITL；完整任务和失效后真实运动尚未验收。

## Commander 默认自动 ARM：2026-10-05

用户随后明确选择“改用 Commander 自动 ARM”。将 `config/commander.yaml` 改为严格布尔值 `auto_arm: true`，同步使用说明和隔离 launch 验证预期。全部健康/标定/融合、地面确认、预发送和实际 OFFBOARD 门保留，实际未解锁时最多请求一次 ARM。显式 `auto_arm:=false` 仍支持遥控器流程，无配置键时的安全缺省仍为 false。未修改 PX4 遥控器 ARM/KILL 映射。

Jetson `run_checks.sh` 的 62 项测试通过；独立 ROS master 上默认 true、显式 true/false、旧参数及冲突验证通过，未就绪的 Commander 均停留 WAIT_SYSTEM，mock backend 输出禁用且 ROS 图没有 MAVROS。日志 `/tmp/uav_early_arm_ros_qmqk7hq7`。本次仅修改配置/文档/验证预期，不启动实机 Commander，不发送 ARM/模式/高度请求，不写 PX4 参数，未提交或推送。

## 实机 Commander 完整流程尝试：2026-10-05

用户随后授权启动 OpenVINS、实机 Commander 并完整测试自动解锁。复用已运行的独立相机、OpenVINS、MAVROS 和 native system，未重复启动；定位源仍使用前述临时厂家参数配置，健康且数据鲜活，ev_sent=true。

在实际 master 11311 启动 `commander.launch auto_arm:=true`，私有参数确认 true。观察 15 秒，Commander 始终 WAIT_SYSTEM，`/uav/command/trajectory` 消息数为 0；结束遥测 connected=true、armed=false、OFFBOARD、landed=1，四路输出均为 1000。OFFBOARD 是此前残留状态，本轮未请求模式或 ARM。观察结束停止本轮 Commander，并确认节点退出；基础数据链保持运行。

完整电机测试未完成：ready/arm_ready=false，具体阻塞为 PX4 critical/unknown status（MAVROS system_status=0）、估计器有效性证据 unavailable/failed、原生 EV observer 要求单 EKF而实机使用多 EKF，以及 calibrated=false/外参与世界对齐未验证。未绕过这些门、未直接调用 MAVROS 解锁、未修改参数或标定标志。此次仅确认实机未就绪等待行为，不是电机/飞行成功验收。

Jetson 日志 `/tmp/uav_full_commander.log`、`/tmp/uav_full_commander_observation.log`、`/tmp/uav_full_commander_observation.json`。首次 shell 启动因 ROS setup 与 nounset 不兼容而在启动节点前退出；调整 source 顺序后成功启动。

## 单 EKF 实机配置与观察器兼容修正：2026-10-05

用户明确授权直接修改单 EKF 参数，后续再次确认机体水平静止在地面、未装桨叶。Commander 已停止，鲜活 FCU/landed 确认未解锁且 ON_GROUND 后，写入 SENS_IMU_MODE=1、EKF2_MULTI_IMU=0；SENS_MAG_MODE 原为 1，保持并读回确认。当前固件参数表未提供 EKF2_MULTI_MAG，MAVROS ParamGet 失败且只读 `param show` 无匹配条目，没有尝试写入不存在的参数。正常 reboot 命令获飞控接受，重启后再次读回前三项分别 1/0/1，EKF2_EV_CTRL=3 仍持久化。`ekf2 status` 仅显示 ekf2:0，实际主实例 EV pos/hgt 均 fused=true。未刷固件、未强制 ARM、未改阈值或 RC 映射。

- 观察器严格读取 EKF2_MULTI_IMU=0、SENS_IMU_MODE=1、SENS_MAG_MODE=1，依据实际固件 EKF2.cpp 的单实例启动路径兼容缺少 EKF2_MULTI_MAG 的构建；必需参数读取失败/值不符仍拒绝。增加缺失/未知/多实例配置回归。
- 修复 shell prompt 尾部 ANSI 控制码使完整回答被误判超时的问题，以及当前固件附加布尔字段被错误转为 float 的问题；保留格式/缺失字段/鲜活时间与 instance 检查。
- 发现原 Supervisor 把静止时的 legacy CONST_POS_MODE 标志当作恒定假位置。对应固件 ekf_helper.cpp 明确包含 vehicle_at_rest。观察器增加第五条固定只读 estimator_status_flags 查询，必须鲜活确认真实 EV 位置/高度启用、假位置/有效假位置/惯性推算关闭，再结合原 fused、创新拒绝、local valid 门。Supervisor 仅在鲜活 ON_GROUND 且同 session 真实 EV 证据通过时允许这个静止标志；空中/假位置/缺失证据/其他 EKF 故障仍拒绝，标定门不变。
- 重启恢复链路时，旧 MAVROS 因 USB 脱离退出，确认节点不可达后只清理其残留 master 注册并重启本轮 MAVROS；相机保持独立运行，未重复启动。native system 使用 start_ev_observer=false，定位初始化稳定后单独启动修正后的原生 observer，避免共享 shell 查询。新 session 恢复后 reset 稳定。

最终实机观察：连续原生 EvStatus received/fused/reset_valid=true；FCU POSCTL、system_status=3、armed=false。Supervisor ready=true、arm_ready=false，唯一原因 extrinsic/world alignment unverified。实际 Commander auto_arm=true 复跑 15 秒始终 WAIT_SYSTEM，轨迹发布数 0、landed=1、四路输出均 1000；观察结束停止本轮 Commander。OpenVINS/相机/MAVROS/native system/observer 保持运行。生产及临时源标定标志保持 false，未以无桨/静止条件假装完成外参验证，完整电机任务仍未执行。

用户确认机体水平后追加鲜活姿态核对：OpenVINS imu/global 和当前 canonical base_link 均 roll≈-85.25°、pitch≈3.02°，PX4 local base_link roll≈2.48°、pitch≈-0.93°。这表明当前把 IMU sensor 姿态直接当机体姿态的单位旋转假设未通过水平一致性检查；不同世界 yaw 原点不作为该 roll/pitch 差异的解释。日志 `/tmp/uav_single_ekf_attitude_review.log`。当前未设置 verified/calibrated=true。

Jetson 软件验证：67 项 unittest 及结构/语法/whitespace 检查通过，6 个 catkin 包增量构建成功（6.6 秒，1 个 distutils 弃用警告，无失败）。实机日志 `/tmp/uav_single_ekf_parameter_write.log`、`/tmp/uav_single_ekf_parameters_before.json`、`/tmp/uav_single_ekf_px4_topics.log`、`/tmp/uav_single_ekf_final_health.log`、`/tmp/uav_single_ekf_commander_verification.log` 及同名 JSON；进程 pidfiles 为 `/tmp/uav_single_ekf_mavros.pid`、`/tmp/uav_single_ekf_system.pid`、`/tmp/uav_single_ekf_observer.pid`。未执行带桨/飞行/SITL测试、精确标定、提交或推送。

用户随后明确约定仅由程序控制飞行，遥控器保留 KILL，遥控器 ARM 将由用户在 QGC 自行关闭。已写入 LOCAL_VALIDATION.md 和 README，源码核对 Commander/Backend 仅请求 OFFBOARD/AUTO.LAND，没有请求 POSCTL/MANUAL 的路径。此轮仅记录运行约定，未写 RC/模式/failsafe 参数，未切模式或解锁；未宣称飞控开机/失效时仍能无条件保持 OFFBOARD。

## Commander 每次请求 OFFBOARD 与飞控重启锁定：2026-10-05

按用户最新要求覆盖前述提前 OFFBOARD 不重复请求及遥控器先 ARM 的历史行为：每次启动在连续就绪、地面确认与预发送完成后主动请求一次 OFFBOARD，即使缓存 mode 已是 OFFBOARD；必须收到请求发出之后的新 MAVROS State 确认实际模式，再进入自动 ARM。auto_arm=false 仍不请求 ARM，可无限等待实际解锁。有效 ARM 不重复请求；服务返回成功不能代替实际状态。

Commander 从首次连接后锁定任何断连/鲜活遥测丢失，并只读监测目标 FCU 的 SYSTEM_TIME/ATTITUDE/ATTITUDE_QUATERNION/LOCAL_POSITION_NED 启动毫秒计数。按消息流分别检测超过 1 秒的回退，排除小幅乱序及 uint32 正常回卷；快速重启未出现 disconnected 边缘时也终止任务。等待服务的旧工作线程在 dispatch 前复核故障、任务阶段、通信及准备门，重连后不发送旧请求。已经发出的服务命令不能撤回；此检查不构成分布式原子保证。

Jetson run_checks.sh：78 项 unittest 及语法/XML/YAML/结构/whitespace 检查通过。覆盖缓存 OFFBOARD、请求后新状态、断连再连、启动时间回退、旧 ARM 服务取消及原有任务/落地回归。6 个 catkin 包增量构建成功（6.5 秒，1 个 distutils 警告）；独立 master 11329 的 verify_commander_wait.py 通过默认 true、显式 true/false、旧配置映射和冲突拒绝，日志 `/tmp/uav_early_arm_ros_7j8k8dc0`。本次控制代码修改后未启动实机 Commander，未对真实飞控注入重启/断连，也未执行 SITL 或电机任务；这些行为由软件替身验证。

## OpenVINS optical IMU 到机体旋转试验：2026-10-05

用户要求修改外参并确认镜头朝前、顶部朝上。当前厂家 camera/IMU 配置对应 optical IMU 轴（右、下、前），采用机体 FLU（前、左、上）在 sensor 中的 T_SB 旋转矩阵 `[[0,-1,0],[0,0,-1],[1,0,0]]`，Hamilton xyzw `[0.5,-0.5,0.5,0.5]`。修改包内 config/state_sources/openvins.yaml 及当前临时 source.yaml；零平移和单位世界对齐保持，calibrated/verified 均保持 false。此前临时配置备份为 `/tmp/uav_extrinsic_source_before.yaml`。

试验前旧 OpenVINS 已输出 NaN quaternion，adapter 拒绝输出；不是本次旋转产生的现象。停止本轮 observer/system，完整重启 system 及其拥有的 OpenVINS，保留独立相机和 MAVROS、防止重复进程；初始化恢复后再启动原生 observer。新 session 为 eeb531b4-af9b-479b-a2af-3aa1c9d1427a。

静止水平采样 30 次、间隔约 0.2 秒，末次 roll/pitch（度）：

| 姿态来源 | roll | pitch |
|---|---:|---:|
| OpenVINS 原始 IMU | -86.923 | 1.337 |
| 适配后机体 | 1.339 | -3.077 |
| PX4 local 机体 | 2.250 | -0.952 |

三路数据末次 age 分别约 7/7/14 ms。约 90° 的横滚轴向偏差消失，剩余 roll/pitch 差异与不同世界 yaw 仍需实际标定及运动验证。随后鲜活原生 EvStatus received/fused/reset_valid=true；SystemStatus ready=true、arm_ready=false，唯一原因 extrinsic/world alignment unverified。FCU connected=true、armed=false、POSCTL、system_status=3，ROS 图没有 Commander。本轮未发送模式/ARM/高度请求，未写 PX4 参数。

日志 `/tmp/uav_extrinsic_trial.log`、`/tmp/uav_extrinsic_trial.json`、`/tmp/uav_extrinsic_system.log`、`/tmp/uav_extrinsic_observer.log`。修改后软件检查再次 78 项通过。此结果只验证当前安装的静止姿态轴映射与当时融合状态，不证明长期 OpenVINS 稳定、精确标定、完整电机响应、SITL 或飞行验收。未提交或推送。

## 用户确认标定标志与逐项健康日志：2026-10-05

用户明确确认当前 Calibrated/verified 为 true，并授权直接修改 YAML。已将包内 config/state_sources/openvins.yaml 的 extrinsic.calibrated/world_alignment.verified 设为严格布尔 true，并同步实机当前加载路径 `/tmp/uav_bench_factory/source.yaml`；临时源修改前备份为 `/tmp/uav_bench_factory/source_before_user_confirmation.yaml`。旋转、平移及其他参数保持。本记录表明用户确认，未新增软件独立精确标定证据；其余健康、融合、地面、模式与解锁门不变。

Supervisor 首次运行及任何检查通过/失败、ready/arm_ready 变化时输出完整检查快照，各项以 PASS/FAIL 标记，包括定位源、适配器、两路里程计、EV 发布/融合、FCU 连接/状态、传感器、估计器、地面遥测、reset、标定与仿真传输限制。附加 ON_GROUND 为 Commander 准备条件，不改变 ready 定义。日志附定位源/适配器 detail、reset 锁定原因和实际 PX4 system_status；未通过项继续写原有 SystemStatus.reasons。相同结果不在 20 Hz 检查中重复打印。

Jetson run_checks.sh：80 项测试及结构/语法/whitespace 检查通过；新增回归覆盖首次失败、恢复、再次失败、相同快照抑制及地面/ARM 就绪变化。配置结构测试更新为接受用户已确认的 OpenVINS true，同时保持其他未确认实机模板 false。全部 6 个 catkin 包增量构建成功（6.5 秒，1 个包警告）。独立 ROS master 11329 的 verify_commander_wait.py 通过，并从真实 ROS `/rosout` 验证同一健康日志同时包含 Source PASS 和缺失 FCU FAIL；默认/覆盖/旧参数与冲突、mock 健康故障和 Commander WAIT_SYSTEM 回归保持通过。

日志 `/tmp/uav_health_reporting_checks.log`、`/tmp/uav_health_reporting_build.log`、`/tmp/uav_health_reporting_ros.log`、`/tmp/uav_early_arm_ros_q04yu3k8/mock.log`。隔离进程已清理，未连接隔离 master 到真实 MAVROS/FCU。实机 system/OpenVINS/observer 没有重启，因此新配置与日志尚未在实机进程中加载；需下次完整 system 重启生效。本轮未启动实机 Commander、未发送 ARM/模式/高度命令或写 PX4 参数；未执行 SITL/飞行，未提交或推送。

## 仅 USB 相机/Pixhawk 的实机复测与静止发布修正：2026-10-05

用户确认电机电池未连接、无桨、遥控器未开启，仅相机和 Pixhawk 接入 Jetson，并授权直接测试。保持独立 camera/MAVROS 单实例，先确认 connected=true、armed=false、landed=1 且无 Commander，再停止本轮 system/observer，完整重启 system 及其拥有的 OpenVINS。两份源 YAML 已加载用户确认的 true，日志标定项 PASS；未写 PX4 参数。

第一轮 session d98b405e-ba33-477e-b048-8fe72374d7b3：OpenVINS 反复接受 ZUPT，camera/imu 约 200 Hz，但 odomimu 无消息。定位、EV 与估计器门 FAIL；Commander auto_arm=true 验证 15 秒始终 WAIT_SYSTEM、轨迹数 0，随后停止。旧运行日志还显示此前 VIO 严重位置发散并以 boost mutex 异常退出；本次发布修正没有证明那个长期漂移/崩溃已解决。

定位到 OpenVINS VioManager::initialized() 要求 is_initialized_vio 且 timelastupdate != -1，而成功的 ZUPT 分支提前返回，未更新 timelastupdate；静止初始化后一直 ZUPT 时没有首次 feature update，发布器永远认为未就绪。对 src/open_vins/ov_msckf/src/core/VioManager.cpp 做独立、局部修正：真实 camera 路径和 simulation 路径成功 ZUPT 后记录本次完成更新时间。不修改滤波计算、参数、传感器标定或项目健康门。实际冷启动静止输出是此修正的运行回归；simulation 分支只完成编译，未做仿真运动验收。

修正后 80 项软件检查通过；6 个 catkin 包全部构建成功（77.1 秒，无警告/失败）。完整重新启动 system/OpenVINS/observer，新 session d8918b56-8e69-4d0f-a7ff-efa3b20d5b7e：静止初始化后直接恢复鲜活 odomimu；约 35 秒采样 140 次，canonical 位置 X/Y/Z 跨度约 1.13/1.80/0.54 mm，最大速度 0.00448 m/s。随后真实 EV received/fused/reset_valid=true，ready=true、arm_ready=true，健康日志各项 PASS。仅证明这段静止输出，不代表长期或运动精度。

在新会话启动 Commander auto_arm=true，实际状态顺序 WAIT_SYSTEM → PRESTREAM → WAIT_OFFBOARD → WAIT_ARM → TAKEOFF。通过 Backend 正常请求 OFFBOARD 并收到请求后的实际模式确认，然后正常 ARM 成功；没有强制解锁或绕过门。138 条轨迹包含地面预发送及短暂爬升目标。进入 TAKEOFF 后约 0.94 秒，Supervisor 的 PX4 estimator validity 门短暂 FAIL，同时 landed 不再为 ON_GROUND；source/adapter、EV 发布与融合仍 PASS。Commander 进入 ABORT_LAND_MODE，正常请求 AUTO.LAND，确认后 ABORT_LAND，地面确认恢复后锁定 ABORTED，不恢复任务。历史采样未保存该瞬间的所有 EKF 子标志，不能进一步断言是哪一位导致有效性失败；不能把无电机电池的此状态变化当作实际起飞。

随后停止本轮 Commander。收尾检查准备在鲜活 ON_GROUND 下通过 Backend 正常 DISARM，但读到 armed=false，故未发送额外 DISARM 请求；最终 connected=true、armed=false、OFFBOARD、landed=1，ROS 图无 Commander。未请求恢复 OFFBOARD/POSCTL/MANUAL；结束模式按实际遥测记录。当前 ready/arm_ready=false，唯一原因 PX4 critical/unknown status（system_status=0），其余数据/融合门恢复通过。不能声称完整任务或电机响应成功；没有电机电池、未执行带桨/飞行/SITL，未改 PX4 参数、未提交/推送，也未自动重试终止任务。

第一轮证据 `/tmp/uav_health_bench_before_zupt_result.json`、`/tmp/uav_health_bench_before_zupt_test.log`、`/tmp/uav_health_bench_before_zupt_system.log`。最终证据 `/tmp/uav_health_bench_result.json`、`/tmp/uav_health_bench_test.log`、`/tmp/uav_health_bench_system.log`、`/tmp/uav_health_bench_observer.log`、`/tmp/uav_health_bench_commander.log`；检查/构建日志 `/tmp/uav_static_zupt_checks.log`、`/tmp/uav_static_zupt_build.log`。验证脚本 `/tmp/uav_health_bench_test.py`。基础链路保持运行，system/observer pidfiles 延用 `/tmp/uav_single_ekf_system.pid`、`/tmp/uav_single_ekf_observer.pid`。

## 遥控依赖只读核对与测试日志归档：2026-10-05

用户询问无遥控启动与可关闭的检查项，本次未写 PX4 参数或关闭检查。实际只读 ParamGet：COM_RC_IN_MODE=0、COM_RCL_EXCEPT=0、NAV_RCL_ACT=2、RC_MAP_ARM_SW=6、RC_MAP_KILL_SW=7。此前 Offboard/ARM 无遥控测试成功，不等于这些参数已配置成全流程不依赖 RC，也不能声称 RC ARM 已由用户关闭。实际 FCU connected=true、armed=false、OFFBOARD。

核对时 Jetson 根盘已满，ROS setup/logging 报 No space left on device；本轮 OpenVINS INFO 重定向日志 `/tmp/uav_health_bench_system.log` 已增长到 2,109,546,496 字节。停止本轮拥有的 system/observer（相机和 MAVROS 保留），将完整日志压缩并校验后归档为 `/tmp/uav_health_bench_system.log.gz`（33,514,606 字节），原路径保留末尾 64 KiB。未压缩内容 SHA256 ea36214bd0812acc09baf7fc730dcc5b97085a92950089fb23f5ecd663cb12c4；释放后根盘约 2.08 GB 可用。未清理用户数据、其他日志或已有工作，未重启 Commander、未发模式/ARM 命令。基础定位/observer 当前已停止，下次测试需显式新建 system 会话；持续运行时应限制 OpenVINS verbosity 或配置日志轮转。

## Jetson 缓存清理与 MAVROS 日志轮转：2026-10-05

用户明确授权删除可清理日志/代码以释放空间。实际清理 pip 下载缓存、npm _cacache、VS Code CachedExtensionVSIXs/C++ 索引缓存、未被进程使用的旧 VS Code server 版本；通过 uv cache clean 清理 uv 缓存（工具报告 88,744 文件、3.6 GiB）。当前 VS Code server 07f806f999227108933c2e30515b26eecc1fda74 与正在使用的 Codex 扩展保留。清理路径和各路径原占用记录 `/tmp/uav_disk_cleanup_manifest.json`；缓存可能硬链接到安装环境，实际释放量以 df 为准。

本轮 MAVROS 重定向日志已有 2,262,003,233 字节。在确认 FCU connected=true、armed=false 且无 Commander 后，停止本轮 MAVROS，完整 gzip 归档并对未压缩内容做 SHA256 一致性校验，随后原日志保留末尾 64 KiB。归档 `/tmp/uav_single_ekf_mavros.log.gz` 为 94,319,165 字节，元数据 `/tmp/uav_mavros_log_archive.json`。恢复同一 MAVROS launch，使用 `/tmp/uav_mavros_rotating_runner.py` 将 stdout/stderr 按 2 MiB ×（当前文件 + 3 个历史文件）滚动，避免本轮日志再次无限增长；runner pidfile `/tmp/uav_mavros_rotating_runner.pid`，实际 roslaunch pidfile 延用 `/tmp/uav_single_ekf_mavros.pid`。恢复时首次子进程未保持运行，补齐本机 LD_LIBRARY_PATH 并使用 detached nohup runner 后正常恢复。最终收到鲜活 MAVROS State connected=true、armed=false、OFFBOARD；无 Commander，未发任何 ARM/模式请求。

根盘空闲从约 2.0 GiB 增至 9.4 GiB，使用率从 99% 降至 93%；最终精确 free=10,019,745,792 字节。项目源码、Git 数据、未提交改动、标定、当前 build/devel、完整测试证据、HuggingFace 模型和 Isaac ROS 素材保留。APT 下载缓存约 2.3 GiB 需要管理员权限，sudo -n 不可用，未清理。相机保持单实例，system/OpenVINS/observer 仍停止；只进行磁盘维护和 MAVROS 恢复，未执行飞行、参数写入、提交或推送。

## 保留核心门后的新会话启动复测：2026-10-05

用户指定保留位姿鲜活/有效、PX4 融合与估计器有效、通信、起飞前地面、实际模式/ARM、重启禁止续飞等核心门，并授权再次启动。沿用用户已确认的无电机电池/无桨台架条件，复用单实例独立相机和 MAVROS；每次建立新 system/OpenVINS session 后才启动新 Commander，没有续用终止任务。启动前读取鲜活 connected=true、armed=false、ON_GROUND，并确认无现有 Commander。

首次按原规则冷启动 session 5891e7be-bd23-48b0-a9f1-5f59e9c089bf：定位源、位姿、EV 真实融合及估计器恢复，但唯独 PX4 system_status=0 汇总状态门仍阻止 ready；Commander WAIT_SYSTEM、轨迹数 0，未发送 OFFBOARD/ARM，观察后停止。证据 `/tmp/uav_restart_core_before_diagnostic_result.json` 及同前缀 test/system/observer/commander 日志。

按用户指定核心范围，将 readiness 的 PX4 汇总 system_status 和 SYS_STATUS sensor mask 两项改为 diagnostic only：继续采集并打印 PASS/FAIL 与实际状态，但不单独阻止 ready/arm_ready，也不混入阻塞 reasons。保留全部源/适配器、canonical/PX4 local 位姿、EV 发布/实际融合、估计器姿态/位置/速度/加速度、通信、地面遥测、session/reset 锁定门，以及标定确认与禁止模拟反馈到实机门。Commander 起飞前 ON_GROUND、预发送、请求后新实际 OFFBOARD、实际 ARM 和断连/重启终止规则不变；未关闭 PX4 自身 ARM 检查、未改 PX4 参数或强制 ARM。

Jetson run_checks.sh：81 项测试通过；新增回归覆盖诊断项失败不阻止核心条件通过、任一核心条件失败仍阻止 ready/ARM、未确认标定仍阻止 ARM。全部 6 个包构建成功（6.9 秒，1 个包警告）。独立 master 11329 的 verify_commander_wait.py 通过默认/覆盖/旧参数/冲突、mock PASS/FAIL 日志与缺失核心证据时 WAIT_SYSTEM，日志目录 `/tmp/uav_early_arm_ros_ld3uqivu`；软件检查/构建/ROS 日志 `/tmp/uav_core_gates_checks.log`、`/tmp/uav_core_gates_build.log`、`/tmp/uav_core_gates_ros.log`。

重新完整启动 system/OpenVINS/observer，session bc8aea49-1871-41c3-be22-3cb32bea6944，静止初始化后连续鲜活输出、真实 EV 融合通过，ready/arm_ready=true。35 秒 140 次静止采样 X/Y/Z 跨度约 5.44/2.81/2.93 mm，最大速度 0.01496 m/s；这不是运动/长期精度验收。启动 auto_arm=true Commander 后实际顺序 WAIT_SYSTEM → PRESTREAM → WAIT_OFFBOARD → WAIT_ARM → TAKEOFF：主动请求一次 OFFBOARD 并用本次请求后的新 State 确认，正常 ARM 成功。20.016 秒 TAKEOFF 期间采集 399 条 SystemStatus，not-ready 样本数 0；没有重现前一次台架的短暂估计器失效终止。

反馈实际高度未达到相对起飞基准 +0.5 m（台架没有真实升高），触发既有 20 秒 takeoff timeout；Commander 请求/确认 AUTO.LAND，ABORT_LAND，地面确认恢复后锁定 ABORTED，随后停止本轮 Commander，未自动重试。共 707 条轨迹。不能把 TAKEOFF 状态/目标发送当作实际起飞、悬停或电机响应成功。

收尾确认时飞控已自行上锁，故未再发送 DISARM；最终 connected=true、armed=false、OFFBOARD、ON_GROUND、system_status=0，无 Commander。EV received/fused/reset_valid 仍 true；收尾时另一次 PX4 estimator reset 已使 Supervisor 锁定 reset fault，ready/arm_ready=false。保留此核心锁定，不自动清除或重启续飞，下一轮需显式新建 system 会话。未请求恢复 OFFBOARD/POSCTL/MANUAL。基础 system/observer 保留运行，日志使用 `/tmp/uav_rotating_launch_runner.py` 以 2 MiB × 4 文件限制增长，根盘仍约 9.4 GiB 可用。

完整带 EKF/地面/系统/FCU 历史的结果 `/tmp/uav_restart_core_result.json`，摘要 `/tmp/uav_restart_core_summary.json`；其他证据 `/tmp/uav_restart_core_test.log`、`/tmp/uav_restart_core_system.log`、`/tmp/uav_restart_core_observer.log`、`/tmp/uav_restart_core_commander.log`、`/tmp/uav_restart_core_finish.log`。验证脚本 `/tmp/uav_restart_core_test.py`。未执行带桨、实飞或 SITL，未写飞控参数、提交或推送。
