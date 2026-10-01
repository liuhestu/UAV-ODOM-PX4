# 系统架构与使用说明

本项目是 ROS1 Noetic catkin 工作区。自有代码统一属于 `uav_system` ROS 包；OpenVINS 是同级独立源码，PX4 固件位于包内 third_party，不参与 catkin 构建。

本文描述当前实现、启动方式、任务和状态源扩展方式及终端状态。Jetson 环境、验证命令和当前验证范围见 [LOCAL_VALIDATION.md](LOCAL_VALIDATION.md)。修改 YAML 后需重新启动对应节点；文件修改不会自动更新运行中的配置。

## 1. 模块与数据流

```mermaid
flowchart LR
    SM[State Source Manager] -->|启动或监测| S[OpenVINS / NOKOV / 其他状态源]
    S -->|Odometry| A[State Adapter]
    A -->|标准化里程计| B[Control Backend]
    B -->|MAVROS| P[PX4]
    M[Mission] -->|目标 / 阶段 / 完成| E[Mission Executor]
    P -->|实际 local 位姿 / 模式 / ARM| E
    E -->|目标 / 模式请求 / ARM 请求| B
    P -->|融合证据| O[PX4 EV Observer]
    O --> F[Flight Supervisor]
    SM --> F
    A --> F
    B -->|EV 发送心跳| F
    P -->|遥测 / 估计器状态| F
    F -->|SystemStatus| E
    F -->|SystemStatus| B
```

| 模块 | 负责什么 | 不负责什么 |
|---|---|---|
| `state_source_manager` | 启动或监测指定状态源，发布数据健康和 session | 坐标转换、任务执行 |
| `state_adapter` | 外参、世界对齐、速度和协方差转换，检查数据连续性 | 目标生成、ARM |
| `mission` | 计算目标、任务阶段和完成条件 | ROS 发布、模式切换、ARM、公共故障处理 |
| `mission_executor` | 共用运行循环、预发送、实际状态确认、任务执行和降落 | PX4 控制器计算、源坐标适配 |
| `flight_supervisor` | 汇总健康条件、标定状态和真实 PX4 融合证据 | 起飞命令、轨迹生成 |
| `control_backend` | 转发里程计与目标，在实际转发前检查命令条件 | 任务规划、自行发起 ARM |
| `support/uav_core` | 共享数学、状态源配置校验、消息鲜活性和数值校验 | 任务或 Supervisor 领域逻辑 |

当前 Backend 实现是 `control_backend/px4_backend.py`，采用 PX4 原生位置控制器；没有接入 px4ctrl。Supervisor 和 PX4 EV Observer 是两个独立 ROS 节点。


### 源码与配置

以下路径相对 `src/uav_system/`：

```text
src/mission/                       # 仅任务脚本
  takeoff_hover_land.py
  propellerless_motor_check.py
  rig_attitude_hold.py
src/mission_executor/
  mission_executor_node.py          # ROS 入口、通信和执行循环
  mission_loader.py                 # 配置/任务加载、单实例锁
  execution.py                      # TaskUpdate、执行状态机、FCU 断连/重启锁定
src/flight_supervisor/
  flight_supervisor_node.py         # 状态收集与健康发布
  checks.py                        # 纯健康判断与融合证据解析
  px4_ev_observer.py                # PX4 只读查询与协议封包
src/control_backend/px4_backend.py
src/support/uav_core/
config/state_sources/              # 实机状态源
config/state_sources/mock.yaml              # 软件 mock 状态源
config/mission/<任务名>.yaml        # 任务专属参数
config/mission_executor.yaml       # 公共执行参数
config/flight_supervisor.yaml      # 健康阈值及 observer 设置
config/px4.yaml                    # Backend 转发设置
```

源码只保留原有 `uav_core/__init__.py`；其他领域模块使用 Python 3 命名空间包，由 setup.py 导出。catkin 自动生成的包入口位于构建产物中。任务脚本单独安装并按路径加载。

## 2. 执行方式

以下命令在工作区根目录、已加载 Noetic 和本工作区环境的终端执行。Jetson 需要的额外环境见本地验证文档。

### 构建与软件等待验证

```bash
./scripts/build.sh
source devel/setup.bash
python3 -m pip install -r src/uav_system/requirements.txt
bash src/uav_system/scripts/run_checks.sh
python3 src/uav_system/test/verify_executor_wait.py
```

最后一个脚本自建独立 master 11329，启动 mock 数据源和 Executor，结束后清理本轮进程；不会启动 MAVROS。也可手动运行 `roslaunch uav_system uav_system.launch state_source:=mock` 查看定位链。mock 缺少真实 PX4 遥测和融合证据时，ready/arm_ready 为 false，Executor 保持 WAIT_SYSTEM。

### 基础系统与任务分别启动

先完成本地验证并确认本次任务可以执行，再使用两终端入口：

```bash
# 终端 1：基础数据链；默认启动 MAVROS、OpenVINS（包含相机）及融合观察器。
roslaunch uav_system uav_system.launch state_source:=openvins

# 终端 2：默认无桨电机检查；卸桨后启动，自动等待条件满足。
roslaunch uav_system mission_executor.launch mission_source:=propellerless_motor_check
```

基础 launch 不启动 Executor。Executor 默认 `auto_arm: true`，检查通过后会请求 OFFBOARD 和 ARM，无额外开始开关。任务执行阶段不请求 POSCTL/MANUAL；正常任务完成后请求 AUTO.LAND，并在地面确认后请求 DISARM。PX4 的失效保护仍有效，程序不会在退出 OFFBOARD 后反复抢回模式。

已有外部 MAVROS 或 OpenVINS 时，可分别设置 `start_mavros:=false`、`start_source:=false`。后者仅监测外部状态源，不管理其进程。若只复用外部相机、仍由系统启动 OpenVINS，需要在源 YAML 将 `start_camera` 设为 false，并从 `owned_nodes` 删除外部相机节点；不能只关闭 MAVROS 来避免重复相机。

### 任务选择与参数覆盖

```bash
# 实际模式/ARM 确认后保持当时 xyz/yaw，按任务配置时长后降落。
roslaunch uav_system mission_executor.launch mission_source:=takeoff_hover_land

# 指定任务配置与公共执行配置。
roslaunch uav_system mission_executor.launch mission_source:=propellerless_motor_check \
  mission_config:=/absolute/path/propellerless_motor_check.yaml \
  executor_config:=/absolute/path/mission_executor.yaml
```

| 参数 | 默认值与含义 |
|---|---|
| `state_source` | `openvins`；默认选择同名 `config/state_sources/<名称>.yaml` |
| `source_config` | 显式覆盖状态源 YAML 路径 |
| `start_source` / `start_mavros` | 默认 true；决定是否启动对应进程 |
| `start_ev_observer` | 默认 true；启动 PX4 侧融合观察器 |
| `output_enabled` | 默认 true；false 时 Backend 不输出 EV/目标或转发飞行服务 |
| `simulation_transport` | 默认 false；true 时检查固定 loopback SITL 端点；SITL launch 显式传入对应 `fcu_url` |
| `mission_source` | 默认 `propellerless_motor_check`；支持任务名、文件名、包内路径及 Mission 目录内的绝对路径 |
| `mission_config` | 默认同名 `config/mission/<任务名>.yaml`；支持显式配置路径 |
| `executor_config` | 默认 `config/mission_executor.yaml`；选择公共执行设置 |
| `auto_arm` | 未传时使用公共 YAML，当前默认 true；显式值只接受 true/false |

旧 `arm_method:=auto/manual` 仍有弃用兼容，但新命令应使用 `auto_arm`。`auto_arm=false` 只禁止程序请求 ARM，不禁止正常降落后的 DISARM；等待实际 ARM 不套用自动解锁超时。本机历史检查中 PX4 拒绝 OFFBOARD 内遥控器 ARM，不能把此选项当作遥控器解锁一定可用的保证。

显式选择的 takeoff_hover_land 起飞任务：相对起始 local 高度增加 0.5 m，目标爬升速率 0.15 m/s，实际到达目标后悬停 5 秒，再降落。`propellerless_motor_check` 只保持当前位置，不能把它当作自动起飞到指定高度的任务。

SITL 使用 `sitl_integration.launch source_config:=/absolute/path/sitl.yaml`，需要另外运行 PX4 SITL 和随机体运动的真实仿真反馈。该入口默认不启动任务；静止 mock 不能替代飞行反馈。

## 3. 配置的职责边界

| 文件 | 修改时机 | 主要内容 |
|---|---|---|
| `config/mission/<任务名>.yaml` | 改任务目标、轨迹或完成条件 | 高度、速度、航点、悬停时间、任务阶段超时等 |
| `config/mission_executor.yaml` | 改公共执行时序与反馈约束 | auto_arm、稳定窗口、预发送、模式确认/降落超时、local 鲜活与跳变阈值 |
| `config/state_sources/<源名>.yaml` | 换定位源、安装位置或输入语义 | 源 launch/topic/frame、外参、世界对齐、协方差和连续性阈值 |
| `config/flight_supervisor.yaml` | 改健康观察时序或观察器通信设置 | 状态/遥测/证据鲜活阈值、传感器诊断位图、MAVLink 身份与查询设置 |
| `config/px4.yaml` | 改 PX4/MAVROS 转发约束 | 输出开关、转发频率、canonical/命令鲜活阈值、实际目标流预发送门 |

Executor 与 Backend 的同名字段作用于各自节点，不互相覆盖。例如 Executor 的 `state_timeout` 检查 PX4 local 位姿，Backend 检查 canonical 里程计和 Adapter 健康；Executor 计时目标发布，Backend 计时实际转发。具体说明已写入两份 YAML 注释。

任务工厂只接收任务配置；任务 YAML 不能夹带公共执行字段。公共有效参数写入 `/mission_executor/*`，任务参数写入 `/mission_executor/mission`。这些伴随计算机配置不会写入 PX4 参数。

## 4. 后续添加或修改任务

| 需求 | 需要修改 | 通常不需要修改 |
|---|---|---|
| 调整现有任务的高度、时间或速率 | 任务 YAML | 任务代码、Executor、Supervisor、Backend、launch |
| 新增位置轨迹、航点或新的完成条件 | 新任务 Python 与同名 YAML，补充任务测试 | 公共执行状态机、健康门、状态源、基础 launch |
| 改公共模式确认、降落或 watchdog 行为 | Executor 与公共执行 YAML，补充执行回归 | 每个任务脚本 |
| 调试架姿态/推力保持 | `rig_attitude_hold` 与同名 YAML | 已有姿态指令通道，保持公共健康门 |
| 接入其他控制器或新指令种类 | Backend、指令契约及相关执行/健康判断 | 不能仅靠任务 YAML 完成 |

新任务放在 `src/mission/<任务名>.py`，配置放在 `config/mission/<任务名>.yaml`；用 `mission_source:=<任务名>` 选择，不增加新 ROS 节点或专属 launch。

任务接口：

```python
from mission_executor.execution import TaskUpdate

# create_task(config) 返回实现以下方法的任务对象：
# start(now, origin) -> TaskUpdate
# step(now, local) -> TaskUpdate
# TaskUpdate(target=(x, y, z, yaw), phase='TRACK', done=False)
```

`now` 为单调时间（秒）；`origin` 和 `local` 为 `(x, y, z, yaw)`。`start` 在健康、地面、实际 OFFBOARD/ARM 均确认后调用，获得当轮实际 local 位姿；起始当轮仍输出地面保持目标，下一轮才按任务目标执行。

任务负责校验自己的参数、限制速度/范围、定义超时和完成条件。目标必须是四个有限数值，`done` 为布尔值；阶段名不能占用 Executor 的公共状态名。`done=True` 交给 Executor 走统一降落流程。任务不得直接发布飞控目标、调用模式/ARM 服务或绕过公共检查；异常和非法输出交给公共故障流程。

当前支持位置/偏航目标，以及独立的姿态四元数/归一化推力目标；不支持速度、加速度或直接电机命令。任务闭环反馈来自 PX4 local 位姿，不直接使用原始 OpenVINS/NOKOV 位姿。

## 5. 后续更换状态源

| 需求 | 需要修改 | 通常不需要修改 |
|---|---|---|
| 换成兼容 Odometry 的定位源 | 新源 YAML，必要时提供源侧 launch | Mission、Executor、Supervisor、Backend、坐标适配代码 |
| 改安装外参或世界轴对齐 | 源 YAML 的 extrinsic/world_alignment | 任务目标与公共执行逻辑 |
| 源输出私有 SDK、Pose 或非标准单位 | 源侧 converter/驱动，再接入 Odometry | 不在 Mission 里兼容源格式 |
| 修改健康定义或 PX4 融合证据协议 | Supervisor/Observer 与测试 | 不属于仅切换状态源 |

新增 `config/state_sources/<名称>.yaml` 后使用 `state_source:=<名称>`；临时文件也可通过 `source_config:=/absolute/path/source.yaml` 选择。NOKOV 当前是外部 Odometry 接入模板，不包含私有 SDK。

核对源配置：

- `source.launches` 和 `owned_nodes`：需要系统启动的进程及重复占用检查；外部源用 `start_source=false`。
- `input`：topic、world/body frame，以及线速度、角速度、姿态协方差表达系。输入必须是米、秒、弧度；当前角速度要求源 body frame。
- `extrinsic`：`T_SB`，即机体 base_link 原点和轴在源 sensor body 中的表达。
- `world_alignment`：`T_AW`，源 world 到 canonical odom 的固定变换。
- `adapter`：数据鲜活性、位置/姿态连续性、协方差 floor。

四元数使用 ROS Hamilton xyzw。标准输出 pose 在 Z-up `odom`，body 为 FLU `base_link`，twist 在 body 中。MAVROS 完成 EV 的 ENU/NED、FLU/FRD 适配，Backend 不重复变换。任务目标数值必须在 PX4 local ENU 中；源 world 和 canonical odom 可以有不同原点/航向，不能直接当作任务 local 坐标。

新源的 `extrinsic.calibrated`、`world_alignment.verified` 在验证前保持 false。当前 OpenVINS 的 true 来自用户对本机配置的确认，不是所有相机安装都可复用的标定结果。更换源或发生 source session/估计器 reset 后，需要重新建立基础系统和新任务，不能恢复旧任务。

## 6. 执行流程与终止行为

显式选择的 takeoff_hover_land 正常流程：

```text
WAIT_SYSTEM → PRESTREAM → WAIT_OFFBOARD → WAIT_ARM
→ TAKEOFF → HOVER → WAIT_LAND_MODE → WAIT_LAND → WAIT_DISARM → DONE
```

`propellerless_motor_check` 跳过 TAKEOFF，新任务的活动阶段使用自己的 phase。公共部分保持：

- 连续健康、通信、local 鲜活和地面确认通过后才开始预发送；预发送期间用当轮 local 更新地面保持目标。
- 每次新任务预发送后主动请求一次 OFFBOARD，即使缓存状态已是 OFFBOARD；请求后的新 `/mavros/state` 才能确认成功。
- 自动 ARM 只在实际未 ARM、实际 OFFBOARD 且检查通过时请求一次；服务成功不能代替实际 mode/armed 确认。
- 初始地面确认未知或空中时等待；准备阶段丢失地面确认则停止命令并终止。
- 飞行中健康故障在条件允许时尝试一次 AUTO.LAND；人工接管、飞控重启、断连或服务 watchdog 终止后不恢复、不重复任务、不抢回模式。
- 正常降落需实际 AUTO.LAND、实际地面和实际 disarmed 确认；禁止空中 DISARM。故障降落完成后锁定 ABORTED，不自动重新执行任务。

同一主机、同一 ROS master 只允许一个 Executor，重复启动不会替换正在运行的节点。Ctrl+C 是退出节点，不是正常完成任务的降落入口；退出后的飞控响应由实际 PX4 失效保护决定。

## 7. 终端输出与排查

### 健康检查

Supervisor 首次检查及检查结果变化时打印快照，结果不变时不刷屏。下面是输出形式示例，不表示当前飞控状态：

```text
Health checks: ready=False arm_ready=False
  [PASS] Source odometry fresh and healthy
  [PASS] State adapter healthy and source session matches
  [FAIL] FCU connected with fresh telemetry
  [FAIL] PX4 EV received and position/height fused in current session
  Details: source=...; adapter=...; reset=...; PX4 system_status=...
```

| 信息 | 含义 |
|---|---|
| `ready` | 核心数据、通信、估计器和融合观察条件是否通过 |
| `arm_ready` | ready 通过，并满足标定确认与仿真传输限制 |
| `[PASS]` / `[FAIL]` | 每项条件的结果；ON_GROUND 是 Executor 的额外起飞前条件 |
| `(diagnostic only)` | PX4 汇总状态或传感器位图仅作诊断，FAIL 不单独阻止 ready |
| `Details` | 原始源/适配器错误、锁定 reset 原因与 PX4 汇总状态 |

日志出现 WARN 不一定代表 ready=false；以完整状态的 ready、arm_ready 和 reasons 为准。项目检查通过也不等于 PX4 完整飞行前检查通过，实际 ARM 仍可能被飞控拒绝。

### 任务状态

Executor 启动打印任务脚本、任务配置、公共配置的实际路径和有效 auto_arm；状态或原因变化时打印 `Mission Executor: <状态>: <原因>`。

| 状态 | 含义 |
|---|---|
| `WAIT_SYSTEM` | 等待健康、通信、鲜活 local 与地面确认及稳定窗口 |
| `PRESTREAM` | 发布地面保持目标 |
| `WAIT_OFFBOARD` | 已请求模式，等待请求后的实际 OFFBOARD 确认 |
| `WAIT_ARM` | 等待实际 ARM；自动请求是否允许取决于 auto_arm |
| `TAKEOFF` / `HOVER` / 自定义 phase | 任务自己的活动阶段 |
| `WAIT_LAND_MODE` / `WAIT_LAND` | 等待实际降落模式 / 地面确认 |
| `WAIT_DISARM` / `DONE` | 等待实际上锁 / 正常完成 |
| `BLOCKED` | 准备阶段模式或 ARM 被拒绝/超时，任务锁定 |
| `ABORT_LAND_MODE` / `ABORT_LAND` | 故障降落流程 |
| `ABORTED` | 故障终止，不恢复 |
| `TAKEN_OVER` | 模式被人工/飞控切走，停止任务命令 |

常见原因包括 `waiting for fresh ON_GROUND confirmation`、`waiting for system readiness and fresh local pose`、`ARM rejected; inspect PX4 report`、`FCU boot clock reset; restart Mission Executor for a new mission`。Adapter 拒绝输入时还会打印 `adapter rejected odometry: ...`，包括 frame、时间戳、数值或跳变原因。

Source Manager 和 Observer 的详细状态主要发布在消息中，不是每项都会直接打印到终端。查看：

```bash
rostopic echo /uav/system/status
rostopic echo /uav/mission_executor/state
rostopic echo /uav/source/status
rostopic echo /uav/state/health
rostopic echo /uav/px4/ev_status
rostopic echo /mavros/state
rostopic echo /mavros/statustext/recv
```

`/uav/backend/ev_sent=true` 只说明本机近期发布过 EV；是否收到并实际融合，以 `/uav/px4/ev_status` 和 Supervisor 结果为准。Observer 当前只支持单 EKF，并检查真实 EV 位置/高度融合；不适配当前固件时应修正观察协议，不通过假健康绕过检查。


## 外部里程计数据链路

外部里程计提供状态估计，任务目标另走 `/uav/command/trajectory`。当前 OpenVINS 链路如下：

```text
OpenVINS /ov_msckf/odomimu       nav_msgs/Odometry，global / imu
  → State Adapter
/uav/state/odom                 nav_msgs/Odometry，odom / base_link
  → Control Backend
/mavros/odometry/out             nav_msgs/Odometry，odom / base_link
  → MAVROS 坐标转换与封包
MAVLink ODOMETRY → PX4 EKF2
  → /mavros/local_position/odom → Mission Executor 的任务反馈
```

`config/state_sources/openvins.yaml` 定义源启动方式、输入语义、安装外参、世界对齐和校验阈值，不是发送给 PX4 的消息格式。Adapter 统一使用 ROS 标准类型 `nav_msgs/Odometry`，不存在名为 `navs_odometry` 的项目消息。PX4 接收的是 MAVROS 封装的 MAVLink 消息。

### **OpenVINS 输出的来源与参考点**

`odomimu` 是 OpenVINS 原生输出，表示 IMU 本体在其局部世界系 `global` 中的状态。滤波器维护 IMU 的位置、姿态、速度和偏置，视觉观测修正这些状态；发布时通过 `fast_state_propagate()` 使用后续 IMU 数据传播到请求时间。因此它是视觉惯性估计，不是原始加速度/陀螺仪，也不是纯 IMU 积分；每条高频输出不一定对应一次新的视觉更新。角速度字段来自修正后的陀螺仪测量，不是单独估计的角速度状态。实现见 [OpenVINS 发布代码](../../open_vins/ov_msckf/src/ros/ROS1Visualizer.cpp)。

`global` 在初始化时建立，不是把前几帧 odom 平均成 map。以当前静态初始化实现为例，静止 IMU 数据用于确定重力方向，世界 Z 向上，初始化时 IMU 位置设为零。水平 X/Y 由初始化选择，不能仅靠视觉与 IMU 获得真实北向；源重启后不保证与前次世界系相同。世界系固定在环境中，IMU 机体系随设备运动。参见 [本地初始化实现](../../open_vins/ov_init/src/static/StaticInitializer.cpp) 和 [OpenVINS 静态初始化说明](https://docs.openvins.com/classov__init_1_1StaticInitializer.html)。不同 VIO 的输出参考点可能是 IMU、相机光心或机体，接入时必须确认，不能仅凭话题中的 odom 名称判断。

### **标准消息与模块职责**

| 字段 | Adapter 标准输出与 Backend 转发约定 |
|---|---|
| `header.stamp` | 保留输入的测量时间戳 |
| `header.frame_id` | `odom`，Z-up 世界系；不保证与 PX4 local 原点/航向相同 |
| `child_frame_id` | `base_link`，机体 FLU |
| `pose.pose.position` | base_link 原点在 odom 中的位置，m |
| `pose.pose.orientation` | base_link 到 odom 的旋转，ROS Hamilton 四元数 xyzw |
| `twist.twist.linear` | base_link 参考点的线速度，在 base_link 中表达，m/s |
| `twist.twist.angular` | 在 base_link 中表达的角速度，rad/s |
| `pose.covariance` / `twist.covariance` | 各为 6×6 行优先矩阵，共 36 个数；顺序分别为位置/姿态误差和线速度/角速度 |

Adapter 应用世界对齐与机体安装外参，转换参考点、速度和协方差，并校验输入；安装平移还会影响旋转运动时的杆臂速度。Backend 检查健康、鲜活性、数值、frame、连接和输出条件后直接转发，发布前要求 `odom_ned ← odom`、`base_link_frd ← base_link` 的 TF 可用及 MAVROS 订阅者存在。MAVROS 按 TF 将世界/机体系表达转换为飞控所需表达，并封装位置、姿态、速度和协方差；Backend 不重复转换。

核对的 [ROS1 MAVROS 1.20.1 实现](https://github.com/mavlink/mavros/blob/1.20.1/mavros_extras/src/plugins/odom.cpp#L182) 发送 `ODOMETRY`，标记父系 `LOCAL_FRD`、子系 `BODY_FRD`、估计器类型 `VISION`；实际部署版本需与该转换约定核对。不能把任意 VIO 的局部水平轴直接当作地理北/东。Backend 的 EV 发送心跳只证明本机近期发布，实际接收与融合由 Observer/Supervisor 判断；任务反馈使用 PX4 融合后的 local 位姿。

### **坐标轴与当前旋转配置**

相机镜头朝前、顶部朝上，飞控安装方向正确时，各机体/传感器轴的约定如下。该表不描述世界系的水平轴：

| 坐标系 | +X | +Y | +Z |
|---|---|---|---|
| 相机光学系 | 右 | 下 | 镜头朝前 |
| 当前 OpenVINS 源 YAML 假设的 IMU 系 | 右 | 下 | 前 |
| Adapter 的 base_link，FLU | 机头前 | 左 | 上 |
| PX4 机体系，FRD | 机头前 | 右 | 下 |

相机光学系与内部 IMU 系不必完全一致；设备外观朝前不能证明输出轴向，应结合驱动 TF、camera–IMU 标定和运动数据确认。OpenVINS 世界 `global`、项目 `odom` 与 PX4 local 的水平轴不会随着机头转动，原点和航向对齐是独立问题。

当前安装四元数 `extrinsic.rotation_xyzw=[0.5,-0.5,0.5,0.5]` 将 base_link 轴表达在源 IMU 系中；源 IMU 向量转为 base_link 向量时，对应 `(x_B,y_B,z_B)=(z_S,-x_S,-y_S)`。采用 ZYX 欧拉角约定 `R=Rz(yaw)·Ry(pitch)·Rx(roll)`，一组表示为 roll=0°、pitch=−90°、yaw=90°；roll=90°、pitch=−90°、yaw=0° 也等价。pitch=−90° 时存在万向节锁，不能把某一组欧拉角当成唯一安装角，更不能当成飞机当前姿态。

当前 `world_alignment.rotation_xyzw=[0,0,0,1]`，两处平移均为零。因此 Adapter 位置数值不变，姿态和机体系速度仍按安装外参转换；世界对齐为单位变换不证明 OpenVINS 与 PX4 local 已实际对齐。

### **安装平移、世界平移与 PX4 参考点**

令 W 为源世界系、S 为源 IMU 系、A 为标准 odom、B 为目标 base_link。项目约定 `T_AB` 把 B 中坐标映射到 A；当前 Adapter 计算：

```text
R_AB = R_AW · R_WS · R_SB
p_AB = R_AW · (p_WS + R_WS · t_SB) + t_AW
```

`extrinsic` 是 `T_SB`：`translation_xyz` 为从源 IMU 原点指向目标 base_link 原点、在源 IMU 系表达的向量。`world_alignment` 是 `T_AW`：`translation_xyz` 为源世界原点在标准 odom 中的位置。机体上的安装偏移随姿态转动，应填前者；固定的世界原点偏移填后者。

下面仅为假设示例，不是当前参数或已完成的标定：源 IMU 在选定机体参考点正前方 10 cm、无左右/高度差，源轴为右/下/前，则从源 IMU 指向目标点是向后 10 cm：

```yaml
extrinsic:
  translation_xyz: [0.0, 0.0, -0.10]
  rotation_xyzw: [0.5, -0.5, 0.5, 0.5]
world_alignment:
  translation_xyz: [0.0, 0.0, 0.0]
  rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

测量起点是 `odomimu` 对应的 IMU 原点，不能直接用镜头或相机外壳中心代替；若测的是镜头位置，需结合 camera–IMU 标定换算。目标点也必须明确，不能混用机体重心、几何中心、飞控外壳中心和飞控 IMU 芯片位置。

PX4 的机体参考原点按参数约定为车辆重心；EKF 内部在飞控 IMU 点估计，再根据 `EKF2_IMU_POS_X/Y/Z` 补偿输出位置和速度到机体参考点。参数采用机体 FRD，单位 m，零值表示将 IMU 点与机体参考点视为重合。几何中心不一定是重心，也不自动等于 IMU 芯片位置。参见 [PX4 EKF 参考点补偿说明](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf) 和 [IMU 偏移参数定义](https://docs.px4.io/v1.15/en/advanced_config/parameter_reference#EKF2_IMU_POS_X)。

飞控接近重心时，小偏移近似为零不必然导致无法悬停；根据杆臂关系推断，影响在旋转运动时更明显。该近似不证明实际飞行正常，也不要求在未知时猜测每颗芯片的精确位置。应明确并记录参考点与近似，让 Adapter 输出和 PX4 参数保持一致。Adapter 已补偿到 PX4 机体参考点时，`EKF2_EV_POS_*` 应与该输出点一致，不应再重复填写同一相机安装偏移；本小节不读取或修改实际飞控参数。


### 调试架与 mock 入口

`rig_attitude_hold` 实机使用 `state_source:=openvins` 或 `state_source:=nokov`，由基础 launch 选择；任务 YAML 不切换状态源。它只计算水平姿态、固定初始偏航及归一化推力升/保持/降包络，不计算高度或位置误差。指令通过 `/uav/command/attitude` → Backend → `/mavros/setpoint_raw/attitude` → PX4 姿态控制器。位置任务继续使用原 trajectory 通道；两种目标不能同时转发。四元数和推力校验、上限、MAVROS 订阅及 thrust_scaling 检查见 [包说明](../README.md#调试架姿态保持与统一-mock-入口)。

姿态任务准备阶段保持零推力。任务只报告工作阶段完成，正常停止由共享 `mission_executor/shutdown.py` 和执行器负责：从最后发出的推力开始 RAMP_DOWN，使用起止斜率为零的平滑曲线，随后 ZERO_THRUST，再进入 WAIT_GROUND → WAIT_DISARM → DONE。公共 `config/mission_executor.yaml` 的 attitude_ramp_down_seconds 默认 2 秒、attitude_zero_thrust_seconds 默认 0.5 秒，具体 Mission 不再配置或实现正常停桨包络。故障时立即零推力 ABORT_GROUND → ABORT_DISARM → ABORTED，均要求鲜活地面确认才请求上锁。该任务不进入 AUTO.LAND；位置任务仍由执行器请求 PX4 AUTO.LAND，不把手动推力曲线叠加到位置控制上。终止后不恢复任务。平滑的是推力指令，不是电机转速或 ESC 制动；不能据此证明自紧桨在急停或上锁时不会松脱。

统一入口通过 `state_source:=mock start_mission_executor:=true mission_source:=rig_attitude_hold` 组合软件数据源和任务。mock 数据源强制禁用 MAVROS、EV Observer、Backend 输出及自动 ARM，即使显式传 true 也不会打开这些选项；因此只能观察 WAIT_SYSTEM，不能宣称电机或姿态控制验证成功。State Source Manager 根据 `config/state_sources/mock.yaml` 的 `source.nodes` 直接管理模拟节点生命周期，mock_source.launch 已删除。

默认 Mission 在两个 launch 入口、直接节点启动和 Python 加载器中均为 propellerless_motor_check。默认任务无爬升，但仍按公共 auto_arm 配置解锁，仅限卸桨测试。takeoff_hover_land 必须显式选择。正常任务之间可保留基础链；结束旧 Executor 后再启动新任务，终止任务不恢复。
