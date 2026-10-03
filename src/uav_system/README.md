# UAV-ODOM-PX4 — ROS1 Native PX4

第一版已切换为 ROS1/catkin，使用 PX4 EKF2 和 PX4 原生位置/姿态控制器。

定位链：定位源 → 通用 State Adapter → `/uav/state/odom` → Control Backend → MAVROS ODOMETRY → PX4 EKF2。
任务链：Mission → Mission Executor → `/uav/command/trajectory` → Control Backend → MAVROS position setpoint → PX4 原生控制器。
Supervisor 汇总定位、遥测和 PX4 侧融合证据；Mission Executor 自动等待就绪，无第二次人工确认。

## 目录与职责

本包位于工作区的 `src/uav_system/`。CMakeLists.txt、package.xml 和 setup.py 直接定义唯一自有 ROS 包，配置与 launch 都是真实目录。

```text
uav_system/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── src/
│   ├── state_source_manager/
│   ├── state_adapter/
│   ├── flight_supervisor/
│   ├── control_backend/
│   ├── mission/
│   ├── mission_executor/
│   └── support/
│       ├── uav_core/
│       └── msg/
├── config/               # state_sources/ 为真实和模拟状态源
├── launch/
├── test/
├── scripts/run_checks.sh
├── docs/
└── third_party/px4_autopilot/
```

各业务目录是代码模块，不再各自包含包清单或 scripts 子目录。节点统一通过 `rosrun uav_system <node.py>` 或本包 launch 启动；消息类型为 `uav_system/SourceStatus` 等。
OpenVINS 实际源码在同级 `../open_vins/`，保留自己的 ROS 包清单；PX4 在包内 third_party，不参与 catkin 构建。包内没有 config、launch 或 OpenVINS 发现链接。

| 模块 | 职责 |
|---|---|
| `state_source_manager` | 源YAML与roslaunch API、单实例与进程/数据健康 |
| `state_adapter` | 外参、参考点、速度/协方差转换与数据校验 |
| `flight_supervisor` | 状态就绪、PX4诊断与只读融合观察 |
| `control_backend` | MAVROS里程计/目标点与门控的模式、解锁接口 |
| `mission` | 定义目标、轨迹、任务阶段和完成条件 |
| `mission_executor` | 共用运行器、任务生命周期与飞行状态转换 |
| `support/uav_core` | 跨模块共享的数学、状态源配置、数据 watchdog 和数值校验 |
| `support/msg` | 明确的状态消息，生成到 `uav_system.msg` |

旧 ROS2 `estimator_adapter`、`px4ctrl`（包括旧台架和一次性触发脚本）已删除。
旧评测文档和旧代码不再留在当前目录，需要时通过 Git 历史恢复。

## 构建

目标是已有 ROS1 Noetic 环境。Ubuntu 22.04/Jetson 的 ROS1 安装由本地环境提供，本仓库不尝试自动安装 Noetic。

```bash
cd uav_odom_px4
source /opt/ros/noetic/setup.bash
# 安装/补齐 rosdep、catkin_tools、MAVROS + GeographicLib、ROS1 RealSense 2.x、OpenVINS 依赖。
rosdep install --from-paths src --ignore-src -r -y
./scripts/build.sh
source devel/setup.bash
```

不要在同一终端 source ROS2 Humble。保留的 OpenVINS 本身支持两个 ROS 版本，当前工作区选择 ROS1。

## 两终端入口

完成 [本地验证](docs/LOCAL_VALIDATION.md) 后的运行入口如下。启动 Mission Executor 即代表授权该次任务；全部门控通过、实际进入 OFFBOARD 且已 ARM 后自动执行一次任务。默认由 Mission Executor 请求 OFFBOARD 和 ARM，任务结束请求降落和落地后 DISARM。

```bash
# 终端 1：只启动基础链，不启动 Mission Executor。
roslaunch uav_system uav_system.launch state_source:=openvins fcu_url:=/dev/ttyACM0:57600

# 终端 2：默认无桨电机检查；先确认卸桨，内部自动等待。
roslaunch uav_system mission_executor.launch
```

默认任务为 `propellerless_motor_check`：按任务 YAML 保持当前位置（当前 10 秒），不爬升，再请求 AUTO.LAND 并确认地面后上锁。仍可能自动 ARM，仅限卸桨测试。起飞必须显式指定 `mission_source:=takeoff_hover_land`，其高度 0.5 m、爬升目标速度 0.15 m/s、悬停 5 s。
默认任务参数在 `config/mission/propellerless_motor_check.yaml`，与任务程序同名；公共执行参数在 `config/mission_executor.yaml`。文件注释说明参数单位、相对起飞高度和悬停计时条件。默认严格布尔值 `auto_arm: true`。不需要遥控器 ARM 或额外开始开关；全部健康门和地面确认通过、预发送完成、实际 OFFBOARD 确认后，Mission Executor 最多请求一次 ARM。`auto_arm:=false` 仅禁止程序请求 ARM，仍等待实际 armed=true；本机 PX4 拒绝 OFFBOARD 中的遥控器 ARM，因此当前运行约定使用 true。此选项不改变降落及落地后的 DISARM，也不会修改 PX4 的遥控器 ARM/KILL 通道映射。

当前实机运行约定为程序控制任务、遥控器仅保留 KILL；用户自行通过 QGC 关闭遥控器 ARM。程序在任务控制阶段使用 OFFBOARD，不请求 POSCTL/MANUAL、不在结束时恢复手动模式。既有 AUTO.LAND 降落及 PX4 失效保护保持。OFFBOARD 依赖持续目标流，启动程序或修改 auto_arm 不保证飞控从开机到关机始终处于 OFFBOARD；实际退出/终止后不得自动抢回模式。

初始化期间若已 ARM 但缺少鲜活 ON_GROUND 确认，Mission Executor 持续等待，不发布设定点或请求模式。连续就绪窗口通过后开始预发送，每轮使用鲜活 PX4 local 位姿保持地面位置。每次 Mission Executor 启动，预发送完成后均主动请求一次 OFFBOARD，即使此前状态已显示 OFFBOARD。必须收到本次请求发出之后的新 `/mavros/state` 且 mode=OFFBOARD，才允许自动 ARM；服务成功或请求前缓存的模式不能替代确认。预发送完成、实际 OFFBOARD/ARM、地面确认和所有检查通过时固定起飞基准，当轮仍保持地面目标；仅显式选择的 takeoff_hover_land 随后限速爬升，默认无桨任务保持当前位置。自动 ARM 请求与实际状态确认仍有超时；禁用自动 ARM 后等待实际解锁没有 8 秒超时。

首次连接建立后，断连或遥测超时即终止本次任务。Mission Executor 还观察目标 FCU 的 MAVLink 启动时钟回退以识别重启，并锁定终止；即使重连、模式恢复或健康恢复，也不自动解锁/续飞。等待服务的线程在发送前重新检查终止、连接和门控状态，取消尚未发送的旧请求。`fcu_system_id/fcu_component_id` 必须匹配实际 FCU，默认 1/1。

遥控器 ARM 可使电机进入 PX4 怠速，即使 Mission Executor 未启动或 system_ready=false。上述门控控制 Mission Executor 的任务动作，不能阻止 PX4 独立接受遥控器解锁。KILL 保留飞控原有行为。未解锁且已处于 OFFBOARD 时，Mission Executor 不自动抢回手动模式；需先处理模式条件。

准备阶段发生健康/通信/local/session 故障、地面确认丢失、非预期模式变化或请求超时会锁定终止；起飞前不自动接管降落。从首次观察到鲜活 ARM 后，主动上锁也会终止本次任务。终止后不恢复、不重复任务、不抢回模式。

迁移：旧 YAML `arm_method: auto/manual` 或旧启动参数 `arm_method:=auto/manual` 仍映射为 `auto_arm=true/false`，启动时输出弃用警告。旧启动参数可覆盖新版 YAML 默认值。禁止同一 YAML 同时定义新旧键、同时传新旧启动参数，以及旧 YAML 配合显式新启动参数。`auto_arm` YAML 仅接受布尔类型，启动参数仅接受 `true/false`；未传参数时使用 YAML 值，无键时默认 false。可用 `mission_config:=/absolute/path/to/takeoff_hover_land.yaml` 选择配置。旧 `config/mission_executor.yaml` 已移至 `config/mission/takeoff_hover_land.yaml`，显式指定旧路径的命令需要更新。
Mission Executor 等待带时间戳的 `/uav/system/status`，不会依赖可能滞后的单个 Bool。

当前 OpenVINS 配置按用户确认设置 `extrinsic.calibrated: true`、`world_alignment.verified: true`；这两个标志是人工确认，不是程序自动标定结论，也不替代其余实时健康门或 PX4 自身解锁检查。NOKOV 模板仍未验证。源配置在启动时读取，修改后需重启 system 才生效。
Supervisor 首次检查及任何检查结果变化时，逐项打印 `[PASS]`/`[FAIL]`、ready/arm_ready 汇总，以及定位源、适配器和 reset 故障详情；结果不变时不反复刷屏。日志包括地面确认（Mission Executor 的额外条件），但不会把这项混入原有 ready 定义。可在 launch 终端或 `/rosout` 中查看；`/uav/system/status.reasons` 保持列出未通过项。
按当前用户指定的核心范围，PX4 汇总 system_status 和 SYS_STATUS 传感器位图仅作诊断，日志注明 diagnostic only，其异常不单独阻止 ready/arm_ready。定位源/适配器、鲜活有效的 canonical/PX4 local 位姿、EV 发布/实际融合、估计器有效性、通信、地面遥测及 session/reset 锁定仍强制检查；Mission Executor 保持起飞前 ON_GROUND、实际 OFFBOARD/ARM 确认和飞控重启/断连不续飞。PX4 自身飞行前与 ARM 检查未修改。

## 通用定位源配置

`state_source:=openvins` 选择 `config/state_sources/openvins.yaml`。
YAML 中定义 launch package/file/args、输入话题/语义、`T_SB` 外参和 `T_AW` 世界对齐。
它不执行 YAML shell 字符串。新增 Odometry 源通常只需要新增一份 YAML。
已有外部 RealSense/OpenVINS 时可传 `start_source:=false` 复用同一配置，仅监测输入，不管理外部源进程。已有 MAVROS 时同时传 `start_mavros:=false`。

`config/state_sources/mock.yaml` 仅由 state_source:=mock 软件入口显式加载，不作为实机源配置。

`nokov.yaml` 是外部 ROS Odometry 发布者的接入模板，不包含 NOKOV 私有 SDK。
输入必须是米、秒、rad；填入真实话题、坐标系及速度语义。特殊消息先通过源侧 converter 转成 Odometry。

## 软件测试

```bash
python3 -m pip install -r src/uav_system/requirements.txt
bash src/uav_system/scripts/run_checks.sh
# ROS1 环境，纯软件 mock 数据源；不启动 MAVROS、不发送飞控数据或命令。
roslaunch uav_system uav_system.launch state_source:=mock
rosparam set /mock_state_source/mode nan      # static / linear / timeout / nan / jump
```

mock 数据源不能在硬件传输上发送 EV 或进入 ARM。显式 SITL 模式还检查 MAVROS 的实际 FCU URL 为固定 loopback UDP 地址。
`launch/test/sitl_integration.launch` 是外部 PX4 SITL 集成入口，需要另行提供随仿真机体运动的模拟 Odometry 配置；不能用静止 mock 冒充飞行反馈。

`test/` 包含数学、执行状态机、任务插件、健康检查、协议、服务替身与布局测试，以及公共辅助模块、mock 数据源和依赖清单。
测试定义在本包 `test/test_*.py` 中；本包 `scripts/run_checks.sh` 仅执行这些测试和 `git diff --check`。GitHub CI调用同一入口，不另写一套测试逻辑。
完整用途见 [test/README.md](test/README.md)。

## 当前验证范围

软件验证结果以日期记录为准。当前真实目录布局已在 Jetson 完成全部 ROS 包构建、消息导入和独立 mock 数据源启动验证。
实机 VIO、PX4 融合和 RC 模式仍有此前记录的阻塞；未完成 SITL 或飞行测试。
固件、PX4 参数和硬件均未修改。详见 [系统架构与使用说明](docs/ARCHITECTURE.md) 和 [本地环境与当前验证](docs/LOCAL_VALIDATION.md)。


Mission Executor 任务选择使用 `mission_source`，省略时执行 `propellerless_motor_check`。可传任务名、`.py` 文件名、包内相对路径 `src/mission/propellerless_motor_check.py` 或该目录内的绝对路径；解析后的文件必须位于包内 `src/mission/`，包括符号链接目标。默认配置为 `config/mission/<任务文件名>.yaml`；显式 `mission_config` 可传绝对路径、配置目录内文件名或包内 `config/mission/...` 路径。原启动参数 `config` 已移除。

```bash
# 显式选择起飞、悬停、降落任务
roslaunch uav_system mission_executor.launch mission_source:=takeoff_hover_land
# 仅限已卸桨：确认实际 OFFBOARD/ARM 后，按配置保持当前位置，再降落/上锁
roslaunch uav_system mission_executor.launch mission_source:=propellerless_motor_check
# 自定义该任务的持续时间等配置
roslaunch uav_system mission_executor.launch mission_source:=propellerless_motor_check.py mission_config:=/absolute/path/propellerless_motor_check.yaml
```

所有任务均使用固定节点 `/mission_executor`。公共有效配置写入节点私有参数，任务有效配置写入 `/mission_executor/mission`。启动日志显示脚本、两份配置路径和 `auto_arm`。每个 ROS master 只允许一个 Mission Executor；第二个进程在注册 ROS 节点前拒绝启动。该锁约束同一主机的 Mission Executor，不能协调不同主机上分别启动的进程。

`propellerless_motor_check` 仅供无桨链路测试，不设置相对起飞高度、不爬升；`duration_seconds` 必须是有限正数，计时从实际模式/ARM 确认并冻结当前位置开始。到期后通过共用运行器请求 AUTO.LAND，确认地面后 DISARM。调试架约束下的实际运动和 PX4 落地判断仍需单独验证。Ctrl+C 退出节点，不作为任务的正常降落入口。

新增任务模块提供 `create_task(config)`，返回有 `start(now, origin)` 和 `step(now, local)` 的对象；两者返回 `mission_executor.execution.TaskUpdate(target=(x,y,z,yaw), phase='任务阶段', done=False)`。时间为单调秒，位置为 PX4 local 坐标，偏航为弧度。任务工厂仅接收任务 YAML；插件校验自己的配置，只计算目标和完成时机，不调用 ROS 或飞控服务。位置任务的 `done=True` 触发共用降落流程；插件异常、非法阶段或非有限目标触发故障处理。起始基准在所有门控与实际 OFFBOARD/ARM 确认后冻结，进入任务的当轮仍输出地面保持目标，下一轮才执行任务目标。

## 架构迁移

旧 `commander.launch`、`/commander` 和 `/uav/commander/state` 分别迁移为 `mission_executor.launch`、`/mission_executor` 和 `/uav/mission_executor/state`，不保留旧入口。旧 `commander_source`/`commander_config` 改为 `mission_source`/`mission_config`；launch 使用直接参数传递，不再额外检测旧参数名称。旧任务目录 `src/commander` 改为 `src/mission`，任务配置目录改为 `config/mission`。

旧每任务 YAML 的公共字段移入 `config/mission_executor.yaml`；任务 YAML 含公共执行字段时明确拒绝，不能用任务参数覆盖 ARM 或超时策略。公共配置通过 `executor_config:=/absolute/path/executor.yaml` 指定。Backend 节点由 `/px4_backend` 改为 `/control_backend`，已有 `/uav/backend/*` 服务与话题不变。Supervisor 仍是独立节点。

领域逻辑从 `uav_core` 移入各自 Python 包：任务接口属于 `mission_executor/execution.py`，执行状态机、运行器、加载与 FCU guard 属于 `mission_executor`，健康门和融合协议属于 `flight_supervisor`。`uav_core` 保留跨模块共享工具，不依赖这些领域包。

任务目录 `src/mission` 仅保留任务脚本；任务接口由 `mission_executor.execution` 提供。源码仅保留原有 `uav_core/__init__.py`，领域模块使用 Python 3 命名空间包；catkin 在开发空间生成包入口，任务脚本另行复制至安装空间。

`control_backend/px4_backend.py` 是 PX4/MAVROS 专用实现，ROS 节点仍叫 `/control_backend`。`config/px4.yaml` 只配置 Backend；`config/flight_supervisor.yaml` 配置 Supervisor，并包含 PX4 EV Observer 的 `observer` 节。旧 `safety.yaml` 已移除。任务和公共执行配置的加载与归一化统一在 `mission_executor/mission_loader.py`，包括现有单实例锁。

基础入口由 `system.launch` 更名为 `uav_system.launch`，旧文件不保留；mock 软件入口已同步引用。

Executor 的 ROS 循环已合入节点入口，FCU guard 已合入 execution.py；Supervisor 的纯检查逻辑集中在 checks.py，观察器协议封包合入 px4_ev_observer.py。TaskUpdate 也合入 execution.py；两目录共保留六个文件，任务接口不依赖 ROS。


## 调试架姿态保持与统一 mock 入口

`rig_attitude_hold` 用于允许横滚/俯仰转动、固定平移的调试架。任务保持水平姿态和启动时的偏航，由 PX4 原生姿态控制器闭环；不发送位置/高度目标。实机反馈选择 `state_source:=openvins`（默认）或 `state_source:=nokov`；融合、通信和实际模式/ARM 等公共健康检查仍有效。状态源由 `uav_system.launch` 选择，任务 YAML 只定义推力包络。NOKOV 模板的外参与世界对齐需验证后才能使用。

配置为 `config/mission/rig_attitude_hold.yaml`：归一化推力 `thrust: 0.10`，3 秒升推力、5 秒保持、3 秒降推力。0.10 是初始设置，不是已测定的悬停推力，不能保证足以验证回正能力。Executor 与 Backend 的 `max_attitude_thrust` 默认均为 0.30，超限目标被拒绝。

预发送及实际 OFFBOARD/ARM 确认期间只发送零推力姿态目标；实际 ARM 确认后的下一轮才开始升推力。结束后持续零推力目标，等待鲜活 ON_GROUND 后请求 DISARM；不切换 AUTO.LAND，避免位置控制器对抗机架约束。运行中健康故障则进入零推力终止流程，确认地面后上锁；地面确认超时不会强制空中上锁。人工切模式、断连、重启或主动上锁后任务锁定终止。

姿态任务返回 `TaskUpdate(target=AttitudeCommand(orientation=(x,y,z,w), thrust=value), phase=...)`，声明 `command_kind='attitude'` 和 `peak_thrust`。四元数表示 ROS base_link FLU 到 PX4 local ENU，必须有限且单位化；推力为 0–1。运行器发布 `/uav/command/attitude`（`mavros_msgs/AttitudeTarget`，type_mask=7），Backend 转发至 `/mavros/setpoint_raw/attitude`。MAVROS 负责 ENU/FLU 到 NED/FRD 转换。位置和姿态目标同时鲜活时两者均拒绝。

Backend 要求 MAVROS raw attitude 订阅者存在，且 `/mavros/setpoint_raw/thrust_scaling` 为数值 1.0；缺失或不匹配会阻止转发，程序不修改该参数。零推力目标不等于实际电机已停，实际停转还取决于 PX4 解锁/怠速状态。节点或通信退出仍由 PX4 既有失效保护处理。

```bash
# 纯软件：mock 数据源 + 调试架任务；强制关闭 FCU 输出及自动 ARM。
roslaunch uav_system uav_system.launch state_source:=mock start_mission_executor:=true mission_source:=rig_attitude_hold
```

`state_source:=mock` 选择模拟数据源，`mission_source:=propellerless_motor_check` 选择无桨链路任务。上述入口缺少实际 PX4 遥测/融合，会保持 WAIT_SYSTEM；完整任务流程由隔离测试替身验证。`mock_system.launch` 已删除，模拟节点直接由 `config/state_sources/mock.yaml` 的 `source.nodes` 声明，State Source Manager 负责启动、退出监测和停止，不再需要 source launch 文件。实机基础链已有时，明确授权后的任务入口为 `mission_executor.launch mission_source:=rig_attitude_hold`；本次未执行带桨测试。

原 `hover.py` 已改为 `propellerless_motor_check.py`，同名 YAML 使用 `duration_seconds`。该任务仅供卸桨后的真实链路/电机响应测试，不能代替调试架姿态任务。旧任务名和旧任务配置需要更新。

任务与模拟源配置分别位于 `config/mission/propellerless_motor_check.yaml`（无桨测试时长）和 `config/state_sources/mock.yaml`（模拟里程计）。仅使用 `mission_source:=propellerless_motor_check` 不会切换数据源或禁用 ARM，真实反馈下仍可自动解锁；纯软件组合需显式设置 `state_source:=mock`。

状态源配置可通过 `source.nodes` 直接声明简单节点（package、executable、name、标量 params），复杂源仍通过 `source.launches` 引用外部 launch。两种方式共用进程管理、重复节点检查及 start_source=false 外部源复用行为。

正常任务结束后可保留基础系统，先退出已结束的 Executor，再启动新任务。修改源配置、重新标定、出现源 session/reset 或通信/飞控重启故障时应处理故障并按需要重启基础系统，旧任务不自动恢复。结束实验时在基础系统终端 Ctrl+C 并等待退出完成。
