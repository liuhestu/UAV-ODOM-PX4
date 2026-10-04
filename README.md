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
│   ├── state_source_manager/
│   ├── state_adapter/
│   ├── flight_supervisor/
│   ├── px4_backend/
│   ├── commander/
│   └── support/             # 共享代码、ROS消息、启动包元数据
│   │   ├── uav_core/
│   │   ├── uav_msgs/
│   │   ├── uav_system/
│   │   └── open_vins -> ../../third_party/open_vins
├── test/                   # 单元测试与唯一的 mock 数据源
├── launch/
│   ├── system.launch
│   ├── commander.launch
│   └── test/
│       ├── mock_system.launch
│       ├── mock_source.launch
│       └── sitl_integration.launch
├── config/
│   ├── sources/
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

配置和 launch 只在根目录保存一份。`src/support/uav_system/config` 与 `launch` 是指向它们的符号链接，让 ROS1 devel 环境仍能使用 `$(find uav_system)`；安装时 CMake 复制根目录实际内容，不安装这些相对链接。
业务包的 Python 节点直接放在包目录，例如 `src/px4_backend/px4_backend.py`，不再嵌套 `scripts/`。
`src/support/open_vins` 仅是源码发现链接；OpenVINS 实际文件只在 `third_party/open_vins/` 中保存一份，catkin 仍能发现其ROS包。
`commander` 的包名与目录名一致。数学核心使用 Python，因此对应测试为 `test/test_state_adapter.py`，不是先前示意中的 C++ 文件。

| 包 | 职责 |
|---|---|
| `state_source_manager` | 根据源 YAML 用 roslaunch API 启动源；单实例锁、节点冲突和进程退出检查 |
| `state_adapter` | 源世界/机体系与参考点转换、速度/协方差变换、时间戳与跳变检查 |
| `flight_supervisor` | 心跳、状态就绪、PX4 诊断与只读 EV 融合观察 |
| `px4_backend` | MAVROS ODOMETRY/setpoint 边界与有门控的模式、解锁服务 |
| `commander` | 一次性 takeoff/hover/land 状态机 |
| `src/support/uav_core` / `uav_msgs` | 独立数学/策略核心与明确的状态消息 |
| `src/support/uav_system` | ROS 启动包元数据，引用根目录 `config/` 与 `launch/` |
| `third_party/open_vins` | 保留原估计器；包含 ROS1 RealSense 启动文件 |
| `third_party/px4_autopilot` | 原有固件源码，位于 catkin 工作区 src 之外 |

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

`state_source:=openvins` 选择 `config/sources/openvins.yaml`。
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
