# UAV-ODOM-PX4 — ROS1 Native PX4

第一版已切换为 ROS1/catkin，只使用 PX4 EKF2 和 PX4 原生位置控制器。

定位链：定位源 → 通用 State Adapter → `/uav/state/odom` → PX4 Backend → MAVROS ODOMETRY → PX4 EKF2。
任务链：Commander → `/uav/command/trajectory` → PX4 Backend → MAVROS position setpoint → PX4 原生控制器。
Supervisor 汇总定位、遥测和 PX4 侧融合证据；Commander 自动等待就绪，无第二次人工确认。

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
│   ├── px4_backend/
│   ├── commander/
│   └── support/
│       ├── uav_core/
│       └── msg/
├── config/               # state_sources/ 为实机源；test/mock.yaml 为模拟源
├── launch/
├── test/
├── scripts/run_checks.sh
├── docs/
└── third_party/px4_autopilot/
```

五个业务目录是代码模块，不再各自包含包清单或 scripts 子目录。节点统一通过 `rosrun uav_system <node.py>` 或本包 launch 启动；消息类型为 `uav_system/SourceStatus` 等。
OpenVINS 实际源码在同级 `../open_vins/`，保留自己的 ROS 包清单；PX4 在包内 third_party，不参与 catkin 构建。包内没有 config、launch 或 OpenVINS 发现链接。

| 模块 | 职责 |
|---|---|
| `state_source_manager` | 源YAML与roslaunch API、单实例与进程/数据健康 |
| `state_adapter` | 外参、参考点、速度/协方差转换与数据校验 |
| `flight_supervisor` | 状态就绪、PX4诊断与只读融合观察 |
| `px4_backend` | MAVROS里程计/目标点与门控的模式、解锁接口 |
| `commander` | 一次性takeoff/hover/land状态机 |
| `support/uav_core` | 上述模块使用的数学、配置、watchdog、策略与协议代码 |
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

完成 [本地验证](docs/LOCAL_VALIDATION.md) 后的运行入口如下。启动 Commander 即代表授权该次任务；全部门控通过、实际进入 OFFBOARD 且已 ARM 后自动执行一次任务。默认由 Commander 请求 OFFBOARD 和 ARM，任务结束请求降落和落地后 DISARM。

```bash
# 终端 1：只启动基础链，不启动 Commander。
roslaunch uav_system system.launch state_source:=openvins fcu_url:=/dev/ttyACM0:57600

# 终端 2：可紧接着启动，内部自动等待。
roslaunch uav_system commander.launch
```

默认任务：爬升 0.5 m，爬升目标速度 0.15 m/s，悬停 5 s，再请求 AUTO.LAND。
参数在 `config/commander.yaml`，默认严格布尔值 `auto_arm: true`。不需要遥控器 ARM 或额外开始开关；全部健康门和地面确认通过、预发送完成、实际 OFFBOARD 确认后，Commander 最多请求一次 ARM。`auto_arm:=false` 仅禁止程序请求 ARM，仍等待实际 armed=true；本机 PX4 拒绝 OFFBOARD 中的遥控器 ARM，因此当前运行约定使用 true。此选项不改变降落及落地后的 DISARM，也不会修改 PX4 的遥控器 ARM/KILL 通道映射。

当前实机运行约定为程序控制任务、遥控器仅保留 KILL；用户自行通过 QGC 关闭遥控器 ARM。程序在任务控制阶段使用 OFFBOARD，不请求 POSCTL/MANUAL、不在结束时恢复手动模式。既有 AUTO.LAND 降落及 PX4 失效保护保持。OFFBOARD 依赖持续目标流，启动程序或修改 auto_arm 不保证飞控从开机到关机始终处于 OFFBOARD；实际退出/终止后不得自动抢回模式。

初始化期间若已 ARM 但缺少鲜活 ON_GROUND 确认，Commander 持续等待，不发布设定点或请求模式。连续就绪窗口通过后开始预发送，每轮使用鲜活 PX4 local 位姿保持地面位置。每次 Commander 启动，预发送完成后均主动请求一次 OFFBOARD，即使此前状态已显示 OFFBOARD。必须收到本次请求发出之后的新 `/mavros/state` 且 mode=OFFBOARD，才允许自动 ARM；服务成功或请求前缓存的模式不能替代确认。预发送完成、实际 OFFBOARD/ARM、地面确认和所有检查通过时固定起飞基准，当轮仍保持地面目标，随后限速爬升。自动 ARM 请求与实际状态确认仍有超时；禁用自动 ARM 后等待实际解锁没有 8 秒超时。

首次连接建立后，断连或遥测超时即终止本次任务。Commander 还观察目标 FCU 的 MAVLink 启动时钟回退以识别重启，并锁定终止；即使重连、模式恢复或健康恢复，也不自动解锁/续飞。等待服务的线程在发送前重新检查终止、连接和门控状态，取消尚未发送的旧请求。`fcu_system_id/fcu_component_id` 必须匹配实际 FCU，默认 1/1。

遥控器 ARM 可使电机进入 PX4 怠速，即使 Commander 未启动或 system_ready=false。上述门控控制 Commander 的任务动作，不能阻止 PX4 独立接受遥控器解锁。KILL 保留飞控原有行为。未解锁且已处于 OFFBOARD 时，Commander 不自动抢回手动模式；需先处理模式条件。

准备阶段发生健康/通信/local/session 故障、地面确认丢失、非预期模式变化或请求超时会锁定终止；起飞前不自动接管降落。从首次观察到鲜活 ARM 后，主动上锁也会终止本次任务。终止后不恢复、不重复任务、不抢回模式。

迁移：旧 YAML `arm_method: auto/manual` 或旧启动参数 `arm_method:=auto/manual` 仍映射为 `auto_arm=true/false`，启动时输出弃用警告。旧启动参数可覆盖新版 YAML 默认值。禁止同一 YAML 同时定义新旧键、同时传新旧启动参数，以及旧 YAML 配合显式新启动参数。`auto_arm` YAML 仅接受布尔类型，启动参数仅接受 `true/false`；未传参数时使用 YAML 值，无键时默认 false。可用 `config:=/absolute/path/to/commander.yaml` 选择配置。
Commander 等待带时间戳的 `/uav/system/status`，不会依赖可能滞后的单个 Bool。

当前 OpenVINS 配置按用户确认设置 `extrinsic.calibrated: true`、`world_alignment.verified: true`；这两个标志是人工确认，不是程序自动标定结论，也不替代其余实时健康门或 PX4 自身解锁检查。NOKOV 模板仍未验证。源配置在启动时读取，修改后需重启 system 才生效。
Supervisor 首次检查及任何检查结果变化时，逐项打印 `[PASS]`/`[FAIL]`、ready/arm_ready 汇总，以及定位源、适配器和 reset 故障详情；结果不变时不反复刷屏。日志包括地面确认（Commander 的额外条件），但不会把这项混入原有 ready 定义。可在 launch 终端或 `/rosout` 中查看；`/uav/system/status.reasons` 保持列出未通过项。
按当前用户指定的核心范围，PX4 汇总 system_status 和 SYS_STATUS 传感器位图仅作诊断，日志注明 diagnostic only，其异常不单独阻止 ready/arm_ready。定位源/适配器、鲜活有效的 canonical/PX4 local 位姿、EV 发布/实际融合、估计器有效性、通信、地面遥测及 session/reset 锁定仍强制检查；Commander 保持起飞前 ON_GROUND、实际 OFFBOARD/ARM 确认和飞控重启/断连不续飞。PX4 自身飞行前与 ARM 检查未修改。

## 通用定位源配置

`state_source:=openvins` 选择 `config/state_sources/openvins.yaml`。
YAML 中定义 launch package/file/args、输入话题/语义、`T_SB` 外参和 `T_AW` 世界对齐。
它不执行 YAML shell 字符串。新增 Odometry 源通常只需要新增一份 YAML。
已有外部 RealSense/OpenVINS 时可传 `start_source:=false` 复用同一配置，仅监测输入，不管理外部源进程。已有 MAVROS 时同时传 `start_mavros:=false`。

`config/test/mock.yaml` 仅由 mock 测试入口显式加载，不作为实机源配置。

`nokov.yaml` 是外部 ROS Odometry 发布者的接入模板，不包含 NOKOV 私有 SDK。
输入必须是米、秒、rad；填入真实话题、坐标系及速度语义。特殊消息先通过源侧 converter 转成 Odometry。

## 软件测试

```bash
python3 -m pip install -r src/uav_system/test/requirements.txt
bash src/uav_system/scripts/run_checks.sh
# ROS1 环境，纯软件 mock；不启动 MAVROS、不发送飞控数据或命令。
roslaunch uav_system mock_system.launch
rosparam set /mock_state_source/mode nan      # static / linear / timeout / nan / jump
```

mock 不能在硬件传输上发送 EV 或进入 ARM。显式 SITL 模式还检查 MAVROS 的实际 FCU URL 为固定 loopback UDP 地址。
`launch/test/sitl_integration.launch` 是外部 PX4 SITL 集成入口，需要另行提供随仿真机体运动的模拟 Odometry 配置；不能用静止 mock 冒充飞行反馈。

`test/` 中有6个测试模块、1个公共辅助模块、1个mock数据源和依赖清单。
测试定义在本包 `test/test_*.py` 中；本包 `scripts/run_checks.sh` 仅执行这些测试和 `git diff --check`。GitHub CI调用同一入口，不另写一套测试逻辑。
完整用途见 [test/README.md](test/README.md)。

## 当前验证范围

39 项独立软件测试通过。当前真实目录布局已在 Jetson 完成全部 ROS 包构建、消息导入和独立 mock 启动验证。
实机 VIO、PX4 融合和 RC 模式仍有此前记录的阻塞；未完成 SITL 或飞行测试。
固件、PX4 参数和硬件均未修改。详见 [架构](docs/ARCHITECTURE.md)、[本地接续](docs/LOCAL_VALIDATION.md) 和 [验证记录](docs/VALIDATION.md)。
