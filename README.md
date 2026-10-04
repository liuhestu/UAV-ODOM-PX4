# UAV-ODOM-PX4 — ROS1 Native PX4

第一版已切换为 ROS1/catkin，只使用 PX4 EKF2 和 PX4 原生位置控制器。

定位链：定位源 → 通用 State Adapter → `/uav/state/odom` → PX4 Backend → MAVROS ODOMETRY → PX4 EKF2。
任务链：Commander → `/uav/command/trajectory` → PX4 Backend → MAVROS position setpoint → PX4 原生控制器。
Supervisor 汇总定位、遥测和 PX4 侧融合证据；Commander 自动等待就绪，无第二次人工确认。

## 目录与职责

当前目录遵循已讨论的主结构：

```text
UAV-ODOM-PX4/
├── src/
│   ├── state_source_manager/ # 代码模块
│   ├── state_adapter/
│   ├── flight_supervisor/
│   ├── px4_backend/
│   ├── commander/
│   ├── uav_system/          # 唯一自有ROS包的构建定义
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── setup.py
│   └── support/
│       ├── uav_core/        # 共享Python代码
│       ├── msg/             # SourceStatus/SystemStatus/EvStatus
│       └── open_vins -> ../../third_party/open_vins
├── test/
├── launch/
│   ├── system.launch
│   ├── commander.launch
│   └── test/
├── config/
│   ├── state_sources/
│   │   ├── openvins.yaml
│   │   ├── nokov.yaml
│   │   └── mock.yaml
│   ├── px4.yaml
│   ├── safety.yaml
│   └── commander.yaml
├── third_party/
│   ├── open_vins/
│   └── px4_autopilot/
├── scripts/run_checks.sh
└── docs/
```

自有代码只构建一个ROS包 `uav_system`，共用 `src/uav_system/CMakeLists.txt` 与 `package.xml`。五个业务目录是独立代码模块，不再各自包含构建清单或 scripts 子目录。
节点统一通过 `rosrun uav_system <node.py>` 或根 launch 启动；ROS消息类型统一为 `uav_system/SourceStatus` 等。

配置和launch只在根目录保存一份。`src/uav_system/config` 与 `launch` 是引用根目录的符号链接，让ROS1 devel环境仍能使用 `$(find uav_system)`；安装时复制真实内容。
OpenVINS源码只在 `third_party/open_vins` 保存；`src/support/open_vins` 是catkin发现链接，保留第三方自身的ROS包定义。

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
cd ~/UAV-ODOM-PX4
source /opt/ros/noetic/setup.bash
# 安装/补齐 rosdep、catkin_tools、MAVROS + GeographicLib、ROS1 RealSense 2.x、OpenVINS 依赖。
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

不要在同一终端 source ROS2 Humble。保留的 OpenVINS 本身支持两个 ROS 版本，当前工作区选择 ROS1。

## 两终端入口

完成 [本地验证](docs/LOCAL_VALIDATION.md) 后的运行入口如下。启动 Commander 即代表授权该次任务；就绪后它会自动请求 Offboard/解锁并执行任务。

```bash
# 终端 1：只启动基础链，不启动 Commander。
roslaunch uav_system system.launch state_source:=openvins fcu_url:=/dev/ttyACM0:57600

# 终端 2：可紧接着启动，内部自动等待。
roslaunch uav_system commander.launch
```

默认任务：爬升 0.5 m，爬升目标速度 0.15 m/s，悬停 5 s，再请求 AUTO.LAND。
参数在 `config/commander.yaml`。需要遥控器解锁时传 `arm_method:=manual`。
Commander 等待带时间戳的 `/uav/system/status`，不会依赖可能滞后的单个 Bool。

OpenVINS/NOKOV 配置里的单位外参和世界轴对齐默认**未验证**，因此 `arm_ready=false`。
完成实际标定与坐标验证后填写 YAML；未标定状态仍能做里程计地面链路验证。

## 通用定位源配置

`state_source:=openvins` 选择 `config/state_sources/openvins.yaml`。
YAML 中定义 launch package/file/args、输入话题/语义、`T_SB` 外参和 `T_AW` 世界对齐。
它不执行 YAML shell 字符串。新增 Odometry 源通常只需要新增一份 YAML。

`nokov.yaml` 是外部 ROS Odometry 发布者的接入模板，不包含 NOKOV 私有 SDK。
输入必须是米、秒、rad；填入真实话题、坐标系及速度语义。特殊消息先通过源侧 converter 转成 Odometry。

## 软件测试

```bash
python3 -m pip install -r test/requirements.txt
bash scripts/run_checks.sh
# ROS1 环境，纯软件 mock；不启动 MAVROS、不发送飞控数据或命令。
roslaunch uav_system mock_system.launch
rosparam set /mock_state_source/mode nan      # static / linear / timeout / nan / jump
```

mock 不能在硬件传输上发送 EV 或进入 ARM。显式 SITL 模式还检查 MAVROS 的实际 FCU URL 为固定 loopback UDP 地址。
`test/sitl_integration.launch` 是外部 PX4 SITL 集成入口，需要另行提供随仿真机体运动的模拟 Odometry 配置；不能用静止 mock 冒充飞行反馈。

`test/` 中有6个测试模块、1个公共辅助模块、1个mock数据源和依赖清单。
测试定义在 `test_*.py` 中；`scripts/run_checks.sh` 仅执行这些测试和 `git diff --check`。GitHub CI调用同一入口，不另写一套测试逻辑。
完整用途见 [test/README.md](test/README.md)。

## 当前验证范围

云端已通过 39 项独立软件测试、Python/XML/YAML/manifest 检查及 MAVLink 编码比对。
云端没有 ROS/catkin，未完成 ROS 构建、SITL、RealSense/Pixhawk 或飞行测试。
固件、PX4 参数和硬件均未修改。详见 [架构](docs/ARCHITECTURE.md)、[本地接续](docs/LOCAL_VALIDATION.md) 和 [验证记录](docs/VALIDATION.md)。
