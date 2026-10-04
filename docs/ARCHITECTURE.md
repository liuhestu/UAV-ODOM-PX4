# ROS1 Native PX4 第一版

## 目录约定

五个业务包放在 `src/`：state_source_manager、state_adapter、flight_supervisor、px4_backend、commander。
根目录的 `launch/`、`config/`、`test/` 是唯一编辑入口。mock 节点源文件位于 `test/mock_state_source.py`，由 state_source_manager 的 catkin_install_python 导出为 ROS可执行文件。
`launch/test/mock_source.launch` 是 Source Manager 选 mock 的内部启动入口；`mock_system.launch` 是完整的软件测试入口。
共享数学与策略模块、ROS消息和启动包元数据集中到 `src/support/`；该目录下的 open_vins 链接让catkin发现第三方源码中的ROS包。业务包的节点文件直接放在包目录，不嵌套scripts。OpenVINS与PX4固件均放在 `third_party/`，历史文档通过Git恢复。
启动包中的 config/launch 仅为指向根目录的符号链接；catkin install从根目录复制实际文件，以同时保持devel与install中的 `$(find uav_system)` 路径正确。

## 接口契约

| 接口 | 类型 | 语义 |
|---|---|---|
| 源 YAML `input.topic` | `nav_msgs/Odometry` | 测量时间；显式输入 world/body frame、线速度表达系、姿态误差协方差表达系 |
| `/uav/source/status` | `uav_msgs/SourceStatus` | 源进程/原始数据健康，启动 session UUID |
| `/uav/state/odom` | `nav_msgs/Odometry` | `odom` Z-up world，`base_link` FLU；pose 在 world，twist 在 body；米/rad/秒 |
| `/uav/state/health` | `uav_msgs/SourceStatus` | Adapter 健康，沿用源 session |
| `/uav/backend/ev_sent` | `std_msgs/Bool` | 本机近期发布过 EV，有 MAVROS subscriber 且所需 TF 存在；不代表 PX4 收到 |
| `/uav/px4/ev_status` | `uav_msgs/EvStatus` | PX4 侧 EV 接收、位置/高度融合与 local-position reset 证据 |
| `/uav/system/status` | `uav_msgs/SystemStatus` | 就绪/解锁前置条件、具体阻塞项、最近 PX4 文本 |
| `/uav/system/ready`, `/uav/system/arm_ready` | `std_msgs/Bool` | 方便人查看；控制使用带时间戳的完整 status |
| `/uav/command/trajectory` | `geometry_msgs/PoseStamped` | 第一版仅 position + yaw，`px4_local` 表示 PX4 local ENU 数值坐标；不使用 VIO world 作目标 |
| `/uav/backend/set_mode` | `mavros_msgs/SetMode` | 只接受 OFFBOARD/AUTO.LAND；状态/健康/预发送门控 |
| `/uav/backend/arming` | `mavros_msgs/CommandBool` | 解锁必须 arm_ready + Offboard + on-ground；上锁必须 on-ground |
| `/uav/commander/state` | `std_msgs/String` | 状态及失败原因 |

没有自定义姿态/推力控制器，也不解读 px4ctrl 的 CH5/CH6 状态机。

## Source Manager

配置由 `system.launch` 传给 manager 与 adapter。manager 在注册 ROS 节点前获取当前 ROS master 的本机进程锁，拒绝重复 manager；检查配置中源节点是否已运行。
启动使用 ROS1 roslaunch API，整组子节点生命周期由 parent 管理。进程退出锁定故障，不自动重启定位源。
原始 Odometry 心跳与时间戳决定源数据健康；初始化尚未输出里程计时为 unhealthy。
该锁只保证同一主机上的 manager；无法锁住其他主机或任意自行运行的相机程序，因此外部设备占用仍需本地检查。

## State Adapter 数学

约定 `T_AB` 把 B 中的坐标映到 A。S=源 sensor body，B=目标 base_link，W=源 world，A=canonical odom。

- `extrinsic` 是 **T_SB**：B 原点在 S 中的位置 `t_SB`，以及 B→S 的旋转 `R_SB`。
- `world_alignment` 是 **T_AW**：W→A 的固定刚体变换；确保 A 的 Z 向上。
- `R_WB = R_WS R_SB`，`p_WB = p_WS + R_WS t_SB`，然后左乘 T_AW。
- 源线速度先变到 S；`v_B = R_SBᵀ (v_S + ω_S × t_SB)`，`ω_B = R_SBᵀ ω_S`。

这个方向与旧 Adapter 的 T_PV 约定不同，不能原样复制旧配置。
Pose/Twist covariance 使用对应 6×6 Jacobian 变换，包含杠杆臂交叉项，再添加配置中的最小方差。
对于当前 OpenVINS，输入线/角速度都在 IMU/body 中，姿态误差协方差在 IMU 中；位置误差在 global 中。
OpenVINS 的 JPL world→IMU 四元数数组与 ROS Hamilton IMU→world 数组一致；本版不再额外共轭。

ROS Odometry 不携带 pose/twist 的完整交叉协方差。world-velocity 源的姿态与速度交叉项无法完整恢复；当前实现按已给定姿态旋转该速度协方差。
外参/轴对齐的不确定性也没有被自动估计。方差 floor 是最低值，不替代实际噪声建模。

保留测量时间，不用当前时间掩盖旧数据。拒绝非有限值、零四元数、非对称/非半正定协方差、错误 frame、过期或未来数据。
时间回退/重复、位置或姿态跳变、session 变化会锁定故障，重启整个 system 才清除。异常 NaN/超时可以恢复健康，但 Commander 的已退出任务不恢复。

## PX4 边界与融合证据

Backend 每条新 Odometry 最多发送一次。它要求 MAVROS odometry subscriber 和 `odom_ned ← odom`、`base_link_frd ← base_link` 静态 TF 都存在。
MAVROS odometry 插件完成 ROS→LOCAL_FRD/BODY_FRD 转换；Backend 不再重复做 ENU/NED/FLU/FRD 变换。

Supervisor 同时检查 source/adapter/canonical freshness、backend发送心跳、FCU heartbeat/system status、SYS_STATUS 选定 sensor mask、local odometry、ESTIMATOR_STATUS、landed telemetry 和 EV evidence。
`ready` 是可观测链路就绪；`arm_ready` 再加外参/世界轴验证与模拟源的传输限制。两者都不是 PX4 完整 preflight verdict。
完整解锁检查由 PX4 执行。服务拒绝后状态机锁定 BLOCKED，保留 PX4 STATUSTEXT；不自动修改参数、不反复请求 ARM。

`px4_ev_observer` 通过 MAVROS `/mavlink/to`、`/mavlink/from` 使用 PX4 SERIAL_CONTROL shell，仅发送四条固定只读命令：

```text
listener vehicle_visual_odometry -n 1 -i 0
listener estimator_aid_src_ev_pos -n 1 -i 0
listener estimator_aid_src_ev_hgt -n 1 -i 0
listener vehicle_local_position -n 1 -i 0
```

通过只读 `/mavros/param/get` 检查 `EKF2_MULTI_IMU == 0` 与 `EKF2_MULTI_MAG == 0`。
只支持单 EKF 实例 0，并要求 EV 水平位置和高度均在融合；其他融合组合、多 EKF 或缺失 topic 会保持 not-ready。
比较 FCU 内部 timestamp/time_last_fuse，检查 fused、innovation_rejected 和 local valid 位，聚合五个 reset counter。
暂时的查询失败不把默认 0 当成真实 reset。替换观察器时必须保留相同的时间与 session 契约，不能人工持续发布 true。

这个观察器是首版诊断桥，需要本地验证固件输出格式、串口带宽和持续心跳。它占用 PX4 MAVLink shell；运行时不能同时打开 QGC MAVLink Console。
完整生产版可换成适配实机固件的结构化 uORB/遥测桥，Supervisor 接口不变。PX4 源码未修改。

## Commander 与故障策略

正常路径：WAIT_SYSTEM → PRESTREAM → WAIT_OFFBOARD → WAIT_ARM → TAKEOFF → HOVER → WAIT_LAND_MODE → WAIT_LAND → WAIT_DISARM → DONE。
先等待 `arm_ready` 持续稳定；捕获 **PX4 local** pose 作为 hold 起点。持续发布 hold，达到预发送时长后只请求一次 Offboard，并通过实际 mode 确认。
随后软件请求一次 ARM 或等待人工 ARM，实际 armed=true 才开始爬升。服务 success 本身不等于状态切换完成。
降落请求 AUTO.LAND，看到实际 mode 后停止 Offboard setpoint；实际 on-ground 才可能请求 DISARM，实际 armed=false 才 DONE。

模式被 PX4/遥控器切走后进入 TAKEN_OVER，停止发布且不反抢控制。飞行时健康丢失或 session/reset 变化，若仍在 Offboard 且连接有效，尝试一次 AUTO.LAND。
连接失效、LAND拒绝/超时或飞行服务调用超时，则停止命令，由实机已配置的 PX4 failsafe/人工接管处理。
没有强制解锁、空中 DISARM、自动清除失败或恢复剩余轨迹。ROS service 的底层调用可能阻塞，工作线程与 wall-time watchdog 分离；无法撤回已经发出的 MAVLink 命令，超时后需查看真实 FCU 状态。

## 测试边界

独立数学、策略、协议与状态机测试是回归入口；一个通用 mock 用于测试 static/linear/timeout/NaN/jump。
Mock 软件启动禁止 MAVROS 输出。模拟源只有在固定 loopback SITL URL 下才可能输出/解锁；该限制不能识别恶意将真实飞控通过同一地址转发，测试端点应只连接本地 SITL。
SITL 集成入口不启动固件，不内置随飞机运动的定位发布者，需要接入实际仿真反馈。没有宣称已完成 SITL 飞行验证。
