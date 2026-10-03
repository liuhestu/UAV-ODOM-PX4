# PX4 + NOKOV 动捕起降测试：项目说明

本文介绍工作空间中的 `px4_takeoff_test`、`px4ctrl`、`quadrotor_msgs` 与 MAVROS、PX4 的配合方式。当前 NOKOV 主流程实现自动起飞、定点悬停和定时降落；尚未实现航点或轨迹飞行。下文命令适用于 ROS Noetic 工作空间 `/home/jin/my_project/uav_ws`，串口、动捕主机和刚体需按现场修改。

## 项目组成

| 组件 | 职责 |
| --- | --- |
| `nokov_odometry_node.py` | 用 NOKOV 官方 Python SDK 获取指定刚体，转换坐标与单位，发布真实 `nav_msgs/Odometry`。 |
| `px4ctrl` | 读取动捕里程计和飞控 IMU，执行位置反馈控制，向 PX4 持续发送姿态与推力目标。 |
| `px4ctrl_nokov_takeoff_sequence.py` | 检查起飞条件，触发起飞，判断到高与稳定，计时后触发降落；不计算电机输出。 |
| `safe_nokov_takeoff_launcher.py` | 启动联合 launch；收到 Ctrl+C 后维持控制链路，发送 LAND 并等待 DISARM，再关闭子进程。 |
| `quadrotor_msgs` | 定义 `TakeoffLand`、`PositionCommand` 等消息。 |
| MAVROS / PX4 | 经串口和 MAVLink 通信；PX4负责预检、解锁判定和底层姿态控制。 |

本包还有固定里程计、无桨电机台架、MAVROS 位置控制等独立测试；它们不是 NOKOV 主流程的一部分。

## 数据流与接口

~~~text
NOKOV Seeker → 官方 SDK → nokov_odometry_node.py
  └→ /nokov/odometry [nav_msgs/Odometry：位置、姿态、速度]
       ├→ px4ctrl：位置控制的反馈
       └→ 起降脚本：起飞检查、到高判定、计时监测

PX4 ⇄ 串口/MAVLink ⇄ MAVROS
  ├→ /mavros/state、/mavros/extended_state → 起降脚本与 px4ctrl
  ├→ /mavros/imu/data、/mavros/rc/in → px4ctrl
  └← /mavros/setpoint_raw/attitude ← px4ctrl 的目标姿态与推力

起降脚本 / 安全启动器 → /px4ctrl/takeoff_land → px4ctrl
  └→ /mavros/set_mode、/mavros/cmd/arming → PX4
~~~

| ROS 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/nokov/odometry` | `nav_msgs/Odometry` | 真实动捕定位，默认由 px4ctrl 与起降脚本直接订阅。 |
| `/mavros/state` | `mavros_msgs/State` | 连接、模式、ARM 和系统状态。 |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | 落地状态。 |
| `/mavros/imu/data` | `sensor_msgs/Imu` | PX4 IMU 反馈。 |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | 遥控器开关和摇杆。 |
| `/px4ctrl/takeoff_land` | `quadrotor_msgs/TakeoffLand` | 起飞值 1，降落值 2。 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | px4ctrl 持续发送目标姿态与推力。 |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | 可选轨迹命令；当前自动起降脚本不发布。 |

默认的 `/nokov/odometry` **直接供 px4ctrl 使用，不会自动注入 PX4 EKF**。`/mavros/odometry/out` 是向 PX4 发送外部里程计的另一条路径，固定里程计台架节点默认使用它。若另行配置 EKF 融合，需要单独核对飞控参数、坐标系和时间戳。

## 起飞、悬停、降落流程

1. 联合 launch 启动 MAVROS、NOKOV，默认延迟 5 秒启动 px4ctrl；起降脚本仍会等待实际数据就绪。
2. 脚本确认 MAVROS 已连接、PX4 处于 STANDBY 且在地面，动捕、IMU、遥控器和姿态 setpoint 持续更新；条件稳定后发布 TAKEOFF。
3. px4ctrl 请求 OFFBOARD 和 ARM。PX4 可以拒绝模式切换或解锁；脚本不会绕过飞控预检。
4. px4ctrl 按 `takeoff_land_speed` 爬升。动捕高度达到“起飞点 + `takeoff_height`”时切换到 `AUTO_HOVER`，保存此刻位置与偏航角作为悬停目标。
5. 悬停目标速度为零。px4ctrl 每个控制周期用位置误差和速度误差计算所需加速度、姿态和推力，并持续发送给 PX4。摇杆离开中位会移动悬停目标；若持续收到 `/position_cmd` 且已允许指令控制，状态机可能切到 `CMD_CTRL`。
6. 起降脚本另按动捕高度和垂直速度判断是否在目标高度稳定；达到条件后才开始悬停/飞行计时。它不直接查询 px4ctrl 的 `AUTO_HOVER` 状态。
7. 计时结束发布 LAND；px4ctrl 请求下降，落地后请求 DISARM。安全启动器确认 DISARM 后关闭子 launch。

当前 [控制器配置](../px4ctrl/config/ctrl_param_fpv.yaml) 中 `takeoff_height=0.8 m`、`takeoff_land_speed=0.3 m/s`、`ctrl_freq_max=100 Hz`，各轴 `Kp=1.5`、`Kv=1.5`，`hover_percentage=0.25` 是近似悬停推力映射初值。实际控制效果还取决于坐标映射、动捕质量、速度估计与机体参数。

## 安装与现场配置

准备 ROS Noetic、MAVROS、`catkin_tools` 和 NOKOV 官方 Python SDK；SDK 必须安装在运行 ROS 节点所用的 Python 3 环境。首次安装 MAVROS 还需准备 GeographicLib 数据。构建：

~~~bash
cd /home/jin/my_project/uav_ws
source /opt/ros/noetic/setup.bash
catkin build px4ctrl px4_takeoff_test
source devel/setup.bash
~~~

编辑 [动捕配置](config/nokov_odometry.yaml)：

- `server_ip`：运行 Seeker 并发送数据的主机地址。
- `rigid_body_selector: "name"` 配合 `rigid_body_name`，或 `rigid_body_selector: "id"` 配合 `rigid_body_id`；两种方式二选一。
- `world_axis_map`、`body_axis_map`：映射 NOKOV 世界系和机体系。手持机体沿各轴移动、转动，确认 ROS 位置、速度和机头方向与实际一致。机体系通常为前 x、左 y、上 z。
- `position_scale: 0.001` 把毫米转换为米；当前 `twist_linear_frame: "world"` 对应 px4ctrl 使用世界系线速度。
- `expected_rate` 只用于频率提示，不会补帧或强制生成 100 Hz 数据。

编辑 [控制器配置](../px4ctrl/config/ctrl_param_fpv.yaml)，核对实际重量、起飞高度、起降速度、悬停推力、`Kp`、`Kv` 和遥控器模式。当前 `no_RC: false`，通道 5 应在 auto hover、通道 6 在 command control；横滚、俯仰、偏航摇杆应居中，油门保持最低位或居中。现场需有可用的遥控接管/急停手段。

飞控串口默认值在 [联合 launch](launch/px4ctrl_nokov_takeoff.launch) 的 `fcu_url` 中。不同设备先执行 `ls -l /dev/serial/by-id/` 查找稳定设备名，再用启动参数覆盖。QGroundControl 与 MAVROS 不应同时独占同一串口。

## 使用方法

### 1. 先验证连接与定位（不自动起飞）

~~~bash
source /home/jin/my_project/uav_ws/devel/setup.bash
roslaunch px4_takeoff_test px4ctrl_nokov_odometry.launch
~~~

在另一个已 source 的终端查看：

~~~bash
rostopic echo -n1 /mavros/state
rostopic echo -n1 /nokov/odometry
rostopic hz /nokov/odometry
rostopic hz /mavros/imu/data
rostopic hz /mavros/setpoint_raw/attitude
~~~

`/mavros/state` 应显示 `connected: True`。移动并转动机体核对轴向、速度和姿态。结束后关闭此 launch，再启动下一步；同一 ROS master 不要运行两套 MAVROS 或 px4ctrl。

### 2. 自动起飞、悬停、降落

用安全启动器启动联合 launch：

~~~bash
rosrun px4_takeoff_test safe_nokov_takeoff_launcher.py \
  confirm_flight:=true \
  takeoff_timeout:=30.0 \
  hover_duration:=15.0
~~~

串口不同可追加 `fcu_url:=/dev/serial/by-id/你的设备:57600`。`confirm_flight:=true` 表示操作者已确认现场条件，不是关闭飞控预检。

默认 `flight_duration=-1`：确认达到目标高度并稳定后，按 `hover_duration` 悬停，到时降落。显式设置 `flight_duration` 后，它优先于 `hover_duration`；例如：

~~~bash
rosrun px4_takeoff_test safe_nokov_takeoff_launcher.py \
  confirm_flight:=true \
  flight_duration:=60.0
~~~

此时到高后计时 60 秒，到时在当前位置请求 LAND；爬升时间不计入。当前没有独立的轨迹飞行阶段。

| 联合 launch 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `px4ctrl_start_delay` | 5.0 s | px4ctrl 的启动延迟；真正起飞仍受就绪检查约束。 |
| `stable_time` | 3.0 s | 起飞前条件连续满足的时间。 |
| `takeoff_timeout` | 30.0 s | ARM 后等待到高且稳定的上限。 |
| `takeoff_height_tolerance` | 0.05 m | 到高判定可低于目标的距离。 |
| `takeoff_settle_vertical_speed` | 0.20 m/s | 到高判定允许的垂直速度绝对值。 |
| `takeoff_settle_time` | 0.5 s | 到高条件连续满足的时间。 |
| `hover_duration` | 40.0 s | `flight_duration=-1` 时的悬停时间。 |
| `flight_duration` | -1.0 s | -1 不额外设置飞行时间；非负数表示到高后按此时间降落。 |
| `landing_timeout` | 60.0 s | 顺序节点请求 LAND 后等待 DISARM 的时间。 |
| `minimum_odom_rate` | 30.0 Hz | 起飞前动捕真实接收频率的最低检查值。 |

飞行中按 Ctrl+C 时，安全启动器保持 MAVROS、NOKOV 和 px4ctrl 运行，重复请求 LAND，确认 DISARM 后才关闭。联合 `px4ctrl_nokov_takeoff.launch` 默认开启自动触发；直接运行它会启动起飞流程，而对它按 Ctrl+C 会同时停止控制节点，不能有序降落。软件降落仍依赖飞控连接、动捕与控制链路；现场保留遥控接管能力。

### 3. 仅启动动捕桥接

~~~bash
roslaunch px4_takeoff_test nokov_odometry.launch
rostopic hz /nokov/odometry
~~~

该节点只发布定位，不会请求 OFFBOARD、ARM 或起飞。

## 常见问题与其他测试

| 现象 | 首先检查 |
| --- | --- |
| “等待 MAVROS 状态”或 `connected: False` | 飞控上电、串口路径、串口占用和 MAVROS 日志。 |
| PX4 系统状态 0，未进入 STANDBY(3) | 飞控初始化和 QGroundControl 预检提示；这条日志本身不能说明是机体倾斜。 |
| px4ctrl 拒绝 `AUTO_TAKEOFF` | 遥控器开关与摇杆、真实里程计速度、落地状态、是否已有 `/position_cmd`。 |
| PX4 拒绝 ARM | 查看 PX4/QGroundControl 报告的具体预检原因。 |
| 动捕频率告警或数据超时 | `rostopic hz /nokov/odometry`、SDK 网络、刚体遮挡；修改 `expected_rate` 不会增加真实帧数。 |
| 悬停明显漂移 | 坐标轴和偏航方向、动捕速度噪声/延迟、摇杆、推力映射与增益；仅修改 Odometry 协方差不会改变 px4ctrl 当前控制计算。 |
| 飞起几厘米就降落 | 检查到高条件、`flight_duration`/`hover_duration`、定位或 IMU 超时以及 LAND 日志。 |

- [固定里程计台架](launch/fixed_odometry_test.launch)：默认向 `/mavros/odometry/out` 发布固定 `nav_msgs/Odometry`，仅用于静止、无桨的数据链路检查，不能代替真实定位飞行。示例：`roslaunch px4_takeoff_test fixed_odometry_test.launch confirm_stationary:=true duration:=30.0`。
- [无桨低推力台架](launch/bench_motor_test.launch)：直接经 MAVROS 测试 OFFBOARD/ARM 与低推力输出，须拆桨并固定机架，不能与 px4ctrl 姿态 setpoint 同时运行。示例：`roslaunch px4_takeoff_test bench_motor_test.launch confirm_propellers_removed:=true confirm_low_thrust_test:=true`。
- [MAVROS 位置控制示例](launch/safe_takeoff.launch)：使用 `/mavros/local_position/pose` 反馈、向 `/mavros/setpoint_position/local` 发送位置目标；独立于 NOKOV + px4ctrl。默认 `confirm_flight:=false` 只检查。

更多独立测试参数见 [本包 README](README.md)；px4ctrl 的控制模式与遥控器配置见 [px4ctrl README](../px4ctrl/README.md)。
