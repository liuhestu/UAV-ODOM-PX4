# 本地环境与当前验证

架构、启动方式、任务/状态源扩展和终端状态见 [ARCHITECTURE.md](ARCHITECTURE.md)。本文件只保留本机环境、验证方法与当前结果，不累积旧版本流水。

## Jetson 环境

SSHFS 工作区直接编辑源码；构建和 ROS 检查在 Jetson 的 `/home/jetson/uav_odom_px4` 执行。使用 ROS1 Noetic，不混入 Humble，也不加载 `.legacy_catkin` 的旧构建环境。

```bash
ssh jetson-uav-odom 'cd /home/jetson/uav_odom_px4 && ./scripts/build.sh'
```

本机运行环境需要外部 RealSense 工作区和 `/usr/local/lib` 中的兼容库；项目 overlay 最后加载：

```bash
source /opt/ros/noetic/setup.bash
source /home/jetson/jin_ws/uav_ws/devel/setup.bash
source /home/jetson/uav_odom_px4/devel/setup.bash --extend
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
```

Jetson 的 `~/.bashrc` 已默认加载 Noetic、上述 RealSense 工作区和当前项目 overlay，并清理 Humble/yahboom 残留路径。新终端自动生效；现有终端执行 `source ~/.bashrc`。显式 `ros 2` 切换 Humble，`ros 1` 切回 Noetic/UAV。通过新交互 shell 的包查找及 `roslaunch --files` 检查，不启动节点。修改前备份为 `/home/jetson/.bashrc.before_uav_noetic_20261003_234810`。

`--extend` 用于保留外部 RealSense 包路径。这是本机环境记录，不是其他机器的通用路径。依赖见 `requirements.txt`；本机曾使用 `px4_exp/.venv` 中的 pymavlink，若系统 Python 缺少该依赖，需明确所用解释器或 PYTHONPATH。

## 软件验证

在 Jetson 工作区根目录执行：

```bash
bash src/uav_system/scripts/run_checks.sh
./scripts/build.sh
source devel/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
python3 src/uav_system/test/verify_executor_wait.py
```

检查脚本包含数学、健康策略、协议、任务与执行状态机、服务替身、watchdog 和布局检查。隔离 ROS 脚本使用独立 master 11329，端口必须没有活动 listener；仅启动 mock 数据源与 Executor，不启动 MAVROS，也不连接实际 FCU。预期 ready/arm_ready=false、三任务 WAIT_SYSTEM、重复启动被拒绝；结束后清理本轮子进程。

安装布局验证使用临时 DESTDIR 和隔离 Python 路径，确认模块来自安装目录、任务能加载、旧模块不被复制。源码只有 uav_core 保留 __init__.py；catkin 生成的开发空间入口不属于源码，不能据此判定源码目录整理失败。当前 setuptools 对命名空间包可能打印入口文件不存在的提示，需以实际导入结果判断。

## 硬件验证边界

启动 Executor 会在条件满足后自动请求 OFFBOARD 和 ARM。硬件/飞行动作、PX4 参数写入和固件刷写需要本次任务明确授权；只做软件检查时不要启动实机 Executor。

- 防止重复启动相机、OpenVINS 或 MAVROS。已有外部进程时按架构文档使用 start_source/start_mavros，或调整源 YAML 的相机启动及 owned_nodes。
- 当前 OpenVINS 的 calibrated/verified=true 来自用户对本机外参与世界对齐的确认，软件没有独立完成精确标定。新状态源或新安装必须重新验证；不能直接复制这些确认标志。相机图像尺寸、raw/rectified 与内外参匹配在 OpenVINS 源侧核对。
- 本地 OpenVINS 曾修正静态 ZUPT 完成后的里程计发布时序；静止时有输出不证明运动轴向、漂移或标定准确。
- PX4 Observer 只支持单 EKF，读取验证 EKF2_MULTI_IMU=0、SENS_IMU_MODE=1、SENS_MAG_MODE=1；不把缺失参数当零，也不自动写参数。运行 Observer 时避免同时使用 QGC MAVLink Console。
- 用户约定程序控制任务，遥控器只保留 KILL。是否已经在 QGC 关闭遥控器 ARM，需要实际读取确认；程序不为恢复遥控器 ARM 而请求 POSCTL/MANUAL。
- 任务运行中断连、飞控重启、源 session/估计器 reset 或人工接管后不续飞。故障定位并结束旧任务后，重新建立基础系统与新任务；start_source=true 管理的 OpenVINS 会随基础系统停止/启动，外部源不会。
- 后续 SITL 必须提供随模拟机体运动的实际反馈，并分别验证任务控制、降落判定和失效保护；固定 mock 数据源不作为飞行反馈。

## 当前验证结论（2026-10-06）

| 范围 | 结果与边界 |
|---|---|
| 单元与结构检查 | 110 项通过，含调试架姿态/推力、目标互斥、实际模式/ARM 顺序、准备与飞行故障、断连/重启锁定、接管、正常降落和空中禁上锁 |
| catkin 构建 | uav_system 及依赖共 4 个包增量构建通过 |
| 隔离 ROS | 三任务 WAIT_SYSTEM、配置覆盖/冲突、健康诊断和重复启动拒绝通过；没有 MAVROS/FCU |
| 安装空间 | 领域模块、三个任务与配置在临时安装空间可加载；无旧 hover/stand_attitude 任务 |
| 无桨实机 hover 启动 | 2026-10-06 永久启用静态初始化后通过；真实 EV 接收/融合、OFFBOARD/ARM、保持约 5 秒、AUTO.LAND、DISARM、DONE；用户听到电机启动，结束 armed=false，输出回到 1000 |
| SITL / 实际悬停或轨迹飞行 | 本轮未执行，软件验证不证明实际运动或落地判定成功 |

本轮证据位于 Jetson：`/tmp/uav_rig_mission_checks.log`、`/tmp/uav_rig_mission_build.log`、`/tmp/uav_rig_mission_ros.log`；隔离 ROS 与安装目录见对应日志。这些是临时证据路径，不能保证长期存在。

后续验证更新本节的日期、当前代码验证结果及未覆盖范围，不恢复按版本累积的历史流水。旧 `VALIDATION.md` 已删除。


## 当前无桨电机测试状态（2026-10-06）

用户确认电池已接、桨叶已卸，明确授权启动 hover，并要求正式 OpenVINS 参数永久生效，不再使用临时配置。本次直接修改 `src/open_vins/config/realsense/estimator_config.yaml`：try_zupt=true、init_dyn_use=false，其他估计器、外参和健康阈值不变。配置解析与布尔值检查及该文件 git diff --check 通过；YAML 修改无需重编译。此前 try_zupt=false 的试运行卡在初始化，已停止；其失败不代表当前结果。

停止前一轮基础 launch 和独立相机 launch，确认退出后用 `uav_system.launch start_mavros:=false` 重建基础链，相机由正式源配置统一管理，已有 MAVROS 复用。没有传入 source_config/config_path/executor_config/mission_config 覆盖，不再使用 `/tmp/uav_bench_factory` 或 `/tmp/uav_hover_bench_source.yaml` 参数。日志和采集脚本仍在 /tmp，属于运行证据，不是配置文件。未写 PX4 参数或直接调用 ARM。

静止约 20 秒后 OpenVINS 初始化成功并持续输出真实里程计，Adapter healthy=true；约 21 秒时 Observer received/fused=true，确认单 EKF 实例 0 的位置/高度融合，Supervisor ready/arm_ready=true、reasons=[]，地面 ON_GROUND。随后实际启动 `mission_executor.launch mission_source:=hover`，有效 hover_seconds=5.0、auto_arm=true。该任务固定确认时 PX4 local 位置，无爬升阶段。

采集到的流程（相对任务采集开始）：PRESTREAM 约 3.70 秒；实际 armed=true、HOVER 约 8.16 秒；WAIT_LAND_MODE 约 13.18 秒；实际 AUTO.LAND 约 14.16 秒；WAIT_DISARM 约 14.22 秒；实际 armed=false 约 15.16 秒；DONE 约 15.19 秒。实际 HOVER 持续约 5.02 秒。前四路 RCOut 由 1000 升至约 1100–1102，约 14.29 秒回到 1000；用户随后确认听到电机启动声音，实际电机转动得到用户观察支持。

25 秒采集结束时任务 DONE，FCU connected=true、armed=false、OFFBOARD、ON_GROUND，前四路输出 1000；ready/arm_ready=true、EV received/fused=true。结束后停止本轮 Executor，基础系统/相机/MAVROS保持运行。结束模式为 PX4 遥测确认的 OFFBOARD，不宣称 Executor 主动恢复了模式；本轮确实观察到正常降落 AUTO.LAND。新任务必须重新启动，不能恢复或自动重复本轮任务。

正式配置从仓库加载、静止初始化、真实 EV 反馈/融合、目标预发送与实际 mode/ARM、短时保持、正常落地确认/上锁和电机输出响应已在此次无桨台架场景观察。未执行带桨、实际悬停/轨迹飞行或 SITL，不能据此宣称位置控制精度、运动中定位或飞行安全已验证。Jetson 系统日期显示 2026-10-03，与用户环境日期不同；未调整时钟。

证据：Jetson `/tmp/uav_hover_static_system.log`、`/tmp/uav_hover_static_executor.log`、`/tmp/uav_hover_static_precheck.log`/`.json`、`/tmp/uav_hover_static_test.log`、`/tmp/uav_hover_static_result.json`。采集包含 raw/canonical/EV/local、实际状态、任务阶段、目标、MAVROS设定点和电机输出。外部 launch 日志使用轮转；运行 PID 后续需重新核对，不凭历史记录假设仍存活。未提交或推送。


## 调试架任务与入口调整

本轮仅实施代码、配置、文档及隔离软件验证，没有启动实机 Executor、写 PX4 参数或执行带桨动作。新增 `rig_attitude_hold`：固定平移调试架上的水平姿态/初始偏航保持，归一化推力默认 0.10，升/保持/降各 3/5/3 秒；默认值没有经过带桨回正能力验证。预发送为零推力姿态目标，实际 OFFBOARD/ARM 确认后才开始包络；结束/健康故障先零推力，鲜活地面确认后上锁，不进入 AUTO.LAND。人工接管、断连/重启仍锁定终止，禁止空中强制上锁。

`hover` 重命名为 `propellerless_motor_check`，任务配置使用 duration_seconds；上节实机证据仍保留当时的 hover 名称，不能视为新姿态任务的硬件验证。模拟数据源与无桨任务通过 state_source/mission_source 区分，源配置统一移至 `config/state_sources/mock.yaml`。删除 mock_system.launch，模拟节点通过 source.nodes 直接交给 State Source Manager 管理。统一入口显式组合 mock 数据源和任务时，强制关闭 MAVROS、Observer、Backend 输出和自动 ARM。

新增测试使用内存 ROS/服务替身，验证姿态类型、数值和推力限制、MAVROS scaling、零推力准备/故障处理、目标互斥和地面确认。隔离 master 11329 上验证三个任务等待、配置归一化和故意传 true 后 mock 数据源仍禁止硬件路径。实机 MAVROS raw attitude 转换、电机推力响应、带桨自稳及 SITL 尚未验证。

当前隔离 ROS 证据目录：Jetson `/tmp/uav_executor_ros_9pchdy34`；安装空间验证：`/tmp/uav_rig_install_yjkYnp`（安装日志 `/tmp/uav_rig_mission_install.log`）。106 项检查、4 包增量构建、移动 mock 数据源配置后的隔离 ROS 检查及安装加载均通过。

命名职责修正：propellerless_motor_check 仅为 Mission，由 mission_executor.launch 加载 src/mission/propellerless_motor_check.py 与 config/mission/propellerless_motor_check.yaml，使用已有真实 OpenVINS/NOKOV/PX4 状态。模拟里程计源独立恢复为 mock（state_sources/mock.yaml 的 source.nodes、mock_state_source.py），仅用于软件测试。任务不启动或模拟数据源。

职责分离后的验证日志：`/tmp/uav_mock_source_separation_checks.log`、`/tmp/uav_mock_source_separation_build.log`、`/tmp/uav_mock_source_separation_ros.log`。

删除 mock_source.launch：模拟节点启动信息直接写入 state_sources/mock.yaml 的 source.nodes，由 State Source Manager 通过 roslaunch API 管理；复杂源仍引用原有 launch，进程退出监测、重复节点检查及 start_source=false 不变。新增直接节点配置的正反例检查，未启动实机节点。

直接源节点验证日志：`/tmp/uav_source_nodes_checks.log`（109 项通过）、`/tmp/uav_source_nodes_build.log`（4 包增量构建通过）、`/tmp/uav_source_nodes_ros.log`。

无桨任务最终命名为 propellerless_motor_check，任务类 PropellerlessMotorCheck；同名 Python/YAML、加载测试、隔离 ROS 检查脚本及使用示例已同步。109 项检查、4 包增量构建、只读 roslaunch --dump-params 和新任务加载/5 秒计时检查通过。证据为 `/tmp/uav_motor_check_rename_checks.log`、`/tmp/uav_motor_check_rename_build.log`、`/tmp/uav_motor_check_rename_params.yaml`、`/tmp/uav_motor_check_rename_install.log`。本轮仅重命名与软件验证，未启动实际 Executor、ARM 或写 PX4 参数；未提交/推送。

默认任务统一改为 propellerless_motor_check：mission_executor.launch、uav_system.launch、节点缺省参数和 Python 加载器均一致，起飞任务必须显式指定。保留当前任务 YAML 的 duration_seconds=10.0；测试读取配置并验证完成边界，避免把任务持续时间写死为 5 秒。110 项检查及 4 包增量构建通过。软件验证日志为 `/tmp/uav_default_motor_check_checks.log`、`/tmp/uav_default_motor_check_build.log`、`/tmp/uav_default_motor_check_ros.log`。本轮未重启实际基础系统、启动实机 Mission 或写 PX4 参数。默认仍可请求 ARM，仅限卸桨。


## 本次 ARM 拒绝诊断

用户明确授权按既有卸桨场景运行一次并检查原因。新增日志区分 Executor 本地门控、Backend 本地门控、MAVROS 服务异常和 FCU ACK 返回值；原有门控及单次请求策略保持。110 项软件检查及 4 包增量构建通过，日志 `/tmp/uav_arm_diagnostic_checks.log`、`/tmp/uav_arm_diagnostic_build.log`。

启动前确认没有相机/MAVROS/任务进程，发现旧崩溃 OpenVINS 留下不可达的 /ov_msckf 注册，仅清理该失效注册；这一步没有启动 Mission 或 ARM。随后从正式配置启动 OpenVINS 基础链，约 52 秒时真实 EV 融合、估计器、通信和地面门通过。启动一次 propellerless_motor_check，预发送后主动请求 OFFBOARD，收到 command=176/result=0 并确认实际 OFFBOARD，再发送一次 ARM。

Backend 明确转发了 ARM，FCU 1/1 返回 COMMAND_ACK(command=400,result=1)，即 MAV_RESULT_TEMPORARILY_REJECTED；MAVROS/Backend 均记录 success=false,result=1。请求已接收，没有 ACK 超时或重试；任务 BLOCKED，未进入电机运行阶段。

捕获 MAVLink EVENT，并用包内 PX4 官方事件提取脚本生成定义、匹配事件 ID，确认 check_estimator_high_accel_bias 和 commander_arm_denied_resolve_failures。轴索引 0 的偏置约 -0.379516 m/s²，大于当时测试阈值 0.300000 + 不确定度余量 0.067116 ≈ 0.367116 m/s²。相关参数为 EKF2_ABL_LIM，事件里的 0.300000 是 PX4 的测试基准，不直接等同于参数值。其他事件包括无全局位置、无任务、无地面站连接等，不把其他模式的检查或提示全部当成 OFFBOARD 解锁的阻塞。项目 ready/arm_ready=true 不代表 PX4 原生解锁检查通过。

结束确认 connected=true、armed=false、OFFBOARD、ON_GROUND，Executor 已停止；基础系统保留运行，PID 221886 为本次记录，后续应重新核对。没有写 PX4 参数、绕过检查、重试 ARM、带桨或飞行操作。本次不证明电机响应成功。

证据在 Jetson：`/tmp/uav_arm_diagnostic_system.log`、`/tmp/uav_arm_diagnostic_executor.log`、`/tmp/uav_arm_diagnostic_trace.jsonl`、`/tmp/uav_arm_diagnostic_result.json`、`/tmp/uav_arm_diagnostic_events.jsonl`、`/tmp/uav_arm_diagnostic_decoded_events.json`。事件定义 `/tmp/uav_arm_diagnostic_px4_events.json` 由仓库源码只读提取，未写入飞控。采集程序在 /tmp，不覆盖正式配置。

## 校准后基础系统未就绪排查

用户报告重新校准 PX4 IMU 后 uav_system.launch 启动失败。读取最新日志发现基础节点已启动，旧任务始终 WAIT_SYSTEM，随后用户 SIGINT 退出。仅重启基础系统（start_mission_executor:=false）复现；未启动 Mission、发送模式/ARM 请求或写 PX4 参数。

RealSense D435i 当前枚举为 USB 2.1，驱动拒绝两路 Infrared 848×480@30 Y8 配置。8 秒订阅观察收到 1565 条 camera/imu，双目图像和 OpenVINS odom 均未收到；source healthy=false，ready/arm_ready=false，PX4 connected=true、armed=false，实际模式 AUTO.LOITER（观测值，程序未请求该模式）。因此当前未就绪的直接原因是双目图像流未启动，不能归因于 PX4 加速度计校准。保持图像配置和标定参数，需恢复相机 USB 3 连接后再验证。

本轮自建诊断基础系统已发送 SIGINT 停止，避免用户重新插接后重复启动。证据：Jetson /tmp/uav_base_restart_diagnostic.log。未执行电机任务或飞行。

## 初始化重置基准与融合诊断修正

用户询问如何解决再次启动 Mission 持续 WAIT_SYSTEM。读取实际状态确认此前 EV received/fused 已恢复，唯一阻塞为 Supervisor 锁定的 PX4 estimator reset；用户随后已退出基础系统和任务。没有发送 ARM、模式请求或写 PX4 参数。

只启动基础链（start_mission_executor:=false）复现另一项启动误判：OpenVINS 尚未初始化时 PX4 保留旧 EV/aid 时间戳，Observer 的 received/fused=false 但 reset_valid=true；Supervisor 把该观察的重置计数当基准。约 20 秒恢复融合后计数变化，被锁定为 reset fault，导致显式重启基础链仍无法就绪。改为新 Supervisor 首次 received=true 且 fused=true 时才建立计数基准；建立后即使融合丢失，任何后续计数变化仍锁定，保留会话/重启终止策略，不自动恢复任务。

增加各融合检查字段的失败原因、失败快照与计数变化日志，修正 Observer 失败时仍显示“fusion observed”的描述。SystemStatus reasons 提供明确 reset 故障和 EV detail，Mission WAIT_SYSTEM 日志显示健康阻塞项。未改变健康/融合阈值、ZUPT、相机外参或 PX4 配置。

115 项检查通过，4 包增量构建通过。相机已枚举为 USB 3 5000 Mbps。修正后重启基础链，45 秒静态观察中约 25 秒起 ready=true，之后约 20 秒连续保持；EV received/fused=true、重置计数稳定，实际 armed=false。未启动实机 Mission 或重复电机试验；上次 ARM 后短暂融合失效的物理或估计原因尚未确定，不能由本次启动修正宣称解决。

保留本轮基础系统运行（本轮 PID 449824，仅用于记录，操作前重新核对），没有 Mission。证据：Jetson /tmp/uav_ev_diagnostics_checks.log、/tmp/uav_ev_diagnostics_build.log、/tmp/uav_ev_diagnostics_system.log、/tmp/uav_ev_diagnostics_static.json、/tmp/uav_ev_baseline_fixed_system.log、/tmp/uav_ev_baseline_fixed_static.json。未提交或推送。

## 解锁后健康失效：异步观察时间比较

用户再次运行任务后要求定位失效项，本轮仅读取日志、修改软件并离线验证，没有重启实机节点、ARM、切模式或写参数。最近一轮 LINK_TEST 于 01:16:51.689 开始，01:16:59.453 因 readiness lost 故障终止，保持约 7.764 秒。

01:16:59.412 的观察快照中，EV position/height fused=true、innovation_rejected=false，local 四项有效性均 true，cs_ev_pos/cs_ev_hgt=true，无惯性航位推算。唯一 EV 失败字段为 estimator_status_flags.timestamp：1517581895，比先前读取的 local.timestamp=1517581657 晚 238 微秒。观察器顺序读取五个异步发布的 topic，却使用 local 时间作为所有字段上限，因此误判正常较晚的状态发布为未来数据。该误判还使依赖地面 EV 证据的旧 estimator constant-position 位处理失效，造成第二个健康 FAIL。

修正为使用同一观察中最新 publication timestamp 作为比较基准，同时显式检查 local timestamp 的年龄，保留 2 秒过期限制、融合/创新/模式和所有重置门控。新增异步 238 微秒及过期 local/aiding 回归；117 项检查与 4 包增量构建通过。将实际退出快照离线重放，结果 received=true、fused=true、failures=[]。证据：Jetson /tmp/uav_ev_async_clock_checks.log、/tmp/uav_ev_async_clock_build.log、/tmp/uav_ev_async_abort_snapshot.json。

更早 01:14:05 的另一轮退出日志仅 estimator validity FAIL，EV 证据为 PASS、地面判定为非 ON_GROUND，不能把所有历史终止都归因于本次时间误判；当前回放也不证明带电机运行能完成 10 秒。现有后台观察器仍加载旧代码，需要用户结束任务并重启基础系统后，新代码才生效。本轮没有自动重复实机测试。

## Mission 结果 INFO

按用户要求，Executor 首次进入 DONE 时记录 INFO `MISSION SUCCESSFUL`，首次进入 BLOCKED/ABORTED/TAKEN_OVER 时记录 INFO `MISSION FAILED`。独立标记保证终止后的循环不会重复输出，保留现有状态/原因日志与进程生命周期。121 项检查及 4 包增量构建通过，含四种终止结果和重复循环回归；未启动实机 Mission、ARM 或切换模式。证据：Jetson /tmp/uav_mission_result_checks.log、/tmp/uav_mission_result_build.log。下一次启动 mission_executor.launch 生效，无需重启基础系统。

## 调试架静止估计与地面判定耦合修正

用户报告 rig_attitude_hold 在 RIG_RAMP_UP 阶段失败。本次读取 01:28:05 退出窗口日志：EV 融合 PASS、local 数据 PASS、估计器有效性 FAIL，同时 ON_GROUND 为 false；约 0.2 秒估计恢复，约 0.8 秒地面确认恢复。退出前曾出现 invalid covariance 警告，但退出当轮 Adapter 已健康，不能直接认定该警告触发任务故障。旧日志未保存各 MAVROS estimator flag，故不能确认该轮唯一具体 flag。

源码核对发现另一项门控耦合：PX4 ekf_helper.cpp 设置 ESTIMATOR_CONST_POS_MODE 为 fake_pos 或 valid_fake_pos 或 vehicle_at_rest；本项目先前只在 grounded_ev=true 时允许该位。静止机体的实际 EV 融合已由 Observer 确认且 fake/dead-reckoning 全部排除时，不应再用落地状态解释这个旧位。改用有效当前会话 EV 证据消除歧义，保留全部姿态/位置/速度有效性和 accel_error 门；真实融合失效仍阻塞。起飞前 ON_GROUND、调试架完成/故障后的地面确认与禁止空中 DISARM 都保留。

补充 estimator_failures 字段诊断到 SystemStatus reasons 和健康日志，后续可准确区分姿态、位置、速度、加速度或 constant-position 位失败。121 项检查和 4 包增量构建通过，含有效 EV/缺失 EV、全部关键 flags 翻转以及既有地面/故障处理回归。本轮没有启动实机节点、ARM、模式切换或参数写入；需要重启基础系统加载新健康逻辑，尚未重复无桨/带桨调试架任务。不宣称此次修正已解决所有估计器异常。日志：Jetson /tmp/uav_stationary_estimator_checks.log、/tmp/uav_stationary_estimator_build.log。
