# UAV-ODOM-PX4 — ROS1 Native PX4

第一版已切换为 ROS1/catkin，只使用 PX4 EKF2 和 PX4 原生位置控制器。

定位链：定位源 → 通用 State Adapter → `/uav/state/odom` → PX4 Backend → MAVROS ODOMETRY → PX4 EKF2。
任务链：Commander → `/uav/command/trajectory` → PX4 Backend → MAVROS position setpoint → PX4 原生控制器。
Supervisor 汇总定位、遥测和 PX4 侧融合证据；Commander 自动等待就绪，无第二次人工确认。

## 目录与职责

| 包 | 职责 |
|---|---|
| `state_source_manager` | 根据源 YAML 用 roslaunch API 启动源；单实例锁、节点冲突和进程退出检查 |
| `state_adapter` | 源世界/机体系与参考点转换、速度/协方差变换、时间戳与跳变检查 |
| `flight_supervisor` | 心跳、状态就绪、PX4 诊断与只读 EV 融合观察 |
| `px4_backend` | MAVROS ODOMETRY/setpoint 边界与有门控的模式、解锁服务 |
| `uav_commander` | 一次性 takeoff/hover/land 状态机 |
| `uav_core` / `uav_msgs` | 独立数学/策略核心与明确的状态消息 |
| `uav_system` | `config/` 和统一的 `launch/` 入口 |
| `open_vins` | 保留原估计器；新增 ROS1 RealSense 启动文件 |
| `px4_autopilot` | 保留原有固件源码，`CATKIN_IGNORE` 排除 catkin 扫描 |

旧 ROS2 `estimator_adapter`、`px4ctrl`（包括旧台架和一次性触发脚本）已删除。
旧评测文档移至 `docs/legacy/`，不作为当前操作入口；需要旧代码时使用 Git 历史。

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
roslaunch uav_system takeoff_hover_land.launch
```

默认任务：爬升 0.5 m，爬升目标速度 0.15 m/s，悬停 5 s，再请求 AUTO.LAND。
参数在 `src/uav_system/config/commander.yaml`。需要遥控器解锁时传 `arm_method:=manual`。
Commander 等待带时间戳的 `/uav/system/status`，不会依赖可能滞后的单个 Bool。

OpenVINS/NOKOV 配置里的单位外参和世界轴对齐默认**未验证**，因此 `arm_ready=false`。
完成实际标定与坐标验证后填写 YAML；未标定状态仍能做里程计地面链路验证。

## 通用定位源配置

`state_source:=openvins` 选择 `src/uav_system/config/sources/openvins.yaml`。
YAML 中定义 launch package/file/args、输入话题/语义、`T_SB` 外参和 `T_AW` 世界对齐。
它不执行 YAML shell 字符串。新增 Odometry 源通常只需要新增一份 YAML。

`nokov.yaml` 是外部 ROS Odometry 发布者的接入模板，不包含 NOKOV 私有 SDK。
输入必须是米、秒、rad；填入真实话题、坐标系及速度语义。特殊消息先通过源侧 converter 转成 Odometry。

## 软件测试

```bash
python3 -m pip install -r test/requirements.txt
python3 -m unittest discover -s test -v
# ROS1 环境，纯软件 mock；不启动 MAVROS、不发送飞控数据或命令。
roslaunch uav_system mock_system.launch
rosparam set /mock_state_source/mode nan      # static / linear / timeout / nan / jump
```

mock 不能在硬件传输上发送 EV 或进入 ARM。显式 SITL 模式还检查 MAVROS 的实际 FCU URL 为固定 loopback UDP 地址。
`test/sitl_integration.launch` 是外部 PX4 SITL 集成入口，需要另行提供随仿真机体运动的模拟 Odometry 配置；不能用静止 mock 冒充飞行反馈。

## 当前验证范围

云端已通过 37 项独立软件测试、Python/XML/YAML/manifest 检查及 MAVLink 编码比对。
云端没有 ROS/catkin，未完成 ROS 构建、SITL、RealSense/Pixhawk 或飞行测试。
固件、PX4 参数和硬件均未修改。详见 [架构](docs/ARCHITECTURE.md)、[本地接续](docs/LOCAL_VALIDATION.md) 和 [验证记录](docs/VALIDATION.md)。
