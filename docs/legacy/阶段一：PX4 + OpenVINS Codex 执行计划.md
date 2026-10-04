# PX4 + OpenVINS 第一阶段 Codex 执行与验收记录

> 状态：**第一阶段已执行并通过（2026-09-12）**  
> 本机核对时间：2026-09-12（Asia/Shanghai）  
> 第一阶段定位：台架上的“离线数据 → PX4 uORB 收包”验证，不是位置环闭环验证。

## 执行摘要

- 用户确认飞控是未连接电机的裸板；全过程保持 `armed=false`，未发送 ARM、模式切换或控制设定值。
- OpenVINS commit：`69488123ed9362dd44b6f28e7f4680abbff1442b`，ROS 2 Humble 编译通过。
- PX4 飞控通过 AUTOPILOT_VERSION 实测为 v1.17.0，commit：`d6f12ad1c4000000`，与本地只读源码一致。
- MAVROS 2.14.0 经 USB by-id 路径连接成功，odometry plugin 已加载且订阅数为 1。
- `EKF2_EV_CTRL` 注入前、注入中、停止前均为 `0`；未修改任何 PX4 参数。
- OpenVINS 原始 `/ov_msckf/odomimu` 实测约 175–185 Hz；Adapter 输出稳定为 30.000 Hz。
- PX4 `listener vehicle_visual_odometry -n 5` 连续收到 5 条新样本；样本间隔约 33 ms，`timestamp_sample` 到接收时间约 0.5–5.6 ms。
- PX4 端 `pose_frame=2`，位置/四元数/位姿方差有限；未验证的线速度、角速度和速度方差均为 NaN，未伪造速度。
- MAVROS 自带的标准 `odom ↔ odom_ned`、`base_link ↔ base_link_frd` 旋转已用 `tf2_echo` 验证，未发布伪 identity TF。
- 停止顺序为 bag → Adapter → OpenVINS → MAVROS；停止前再次确认 `armed=false`、`EKF2_EV_CTRL=0`。

详细证据与复现命令见 `logs/phase1_execution_report.md` 和 `scripts/`。本结果只证明数据安全进入 `vehicle_visual_odometry`，不表示 EKF2 已融合或位置环可用。

## 0. 规划审查结论

总体路线合理，继续采用：

```text
EuRoC ROS 2 bag
  → OpenVINS
  → nav_msgs/msg/Odometry
  → estimator_adapter
  → MAVROS2 odometry plugin
  → MAVLink ODOMETRY
  → Pixhawk USB
  → PX4 vehicle_visual_odometry
```

原计划需要修正的关键点如下：

1. 规划核对时本机只有 ROS 2 Humble 和 rosbag2，尚未安装 MAVROS2；执行阶段已补装并复核版本，不能把事前假设当成实测基线。
2. Pixhawk 当前实际设备是 `/dev/ttyACM1`，稳定路径是 `/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00`，应优先使用 by-id 路径。
3. 本机 EuRoC 首选数据是 `~/datasets/euroc/V1_01_easy_db/`，已经是 sqlite3 格式的 ROS 2 bag，无需转换或重新下载。
4. OpenVINS 当前 `odomimu` 输出的 `frame_id` 是 `global`、`child_frame_id` 是 `imu`；其线速度注释明确为 local/global frame。它不满足 MAVROS odometry plugin 对 ROS Odometry 的直接输入语义，不能只改 topic 后原样转发。
5. MAVROS odometry plugin 会根据消息中的 frame id 查找 `<parent>_ned ← <parent>` 和 `<child>_frd ← <child>` 的 TF；不能把坐标问题完全推迟，也不能用“全 identity TF”冒充正确转换。
6. EuRoC 是 2014 年历史时间戳；发往实时 PX4 前需要在 Adapter 内做单调、实时的重打时间戳。MAVROS 和 Adapter 不使用 bag 的仿真时钟。
7. PX4 官方建议含协方差的外部视觉消息频率至少约 30 Hz、最高约 50 Hz。OpenVINS 可能按 IMU 更新频率发布，Adapter 应限频为 30 Hz，避免无意义占用 USB/MAVLink 带宽。
8. 第一阶段只证明 `vehicle_visual_odometry` 收到正确类型且时间连续的数据。**这不能证明 EKF2 已融合，更不能证明位置环可用。**

因此，第一阶段 Adapter 定位从“纯中继”调整为“受控桥接”：重打时间戳、限频、规范 frame id、校验数据，并默认禁止发送尚未验证的速度。

---

## 1. 本机实际环境基线

| 项目 | 2026-09-12 实测结果 | 第一阶段处理 |
| --- | --- | --- |
| 系统 | Ubuntu 22.04.5 LTS，x86_64 | 可用 |
| 内核 | 6.8.0-138-generic | 记录即可 |
| ROS | ROS 2 Humble，`ROS_DISTRO=humble` | 可用 |
| 构建工具 | `/usr/bin/colcon`、`/usr/bin/rosdep` | 可用 |
| 已 source 的额外工作区 | `/home/he/hno_vio_ws/install/hno_vio` | 构建本项目时避免继承，使用干净 shell |
| MAVROS2 | 已安装 `mavros`、`mavros_extras`、`mavros_msgs` | 2.14.0，已实机连通 |
| GeographicLib | 工作区本地 `egm96-5` 数据集 | 由 `GEOGRAPHICLIB_DATA` 指向，不需要 root |
| OpenVINS | 已克隆至 `src/open_vins` | commit `69488123ed9362dd44b6f28e7f4680abbff1442b` |
| PX4 源码 | 已克隆至 `src/px4_autopilot` | v1.17.0 / `d6f12ad1c4`，仅核对，未编译/刷写 |
| 飞控 USB | USB ID `26ac:0032 3D Robotics PX4 FMU v5.x` | 已识别 |
| 串口 | `/dev/ttyACM1` | 不硬编码编号 |
| 稳定串口路径 | `/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00` | MAVROS 首选 |
| 串口权限 | 用户 `he` 已属于 `dialout`，设备属组为 `dialout` | 可用，无需 `chmod 777` |
| 飞控固件 | AUTOPILOT_VERSION 实测 PX4 v1.17.0 / `d6f12ad1c4` | 与本地源码一致 |
| 项目根目录 | `/home/he/uav_vio_px4`，Git 仓库 | ROS 2 工作空间根目录 |
| 可用磁盘 | 约 741 GiB | 足够 |

### 1.1 EuRoC V1_01_easy 实测

```text
路径：/home/he/datasets/euroc/V1_01_easy_db
存储：sqlite3（V1_01_easy_db.db3 + metadata.yaml）
大小：约 2.0 GiB
时长：147.166583149 s
总消息数：107819
```

与 OpenVINS 直接相关的 topic：

| Topic | 类型 | 数量 | 估算频率 |
| --- | --- | ---: | ---: |
| `/imu0` | `sensor_msgs/msg/Imu` | 29120 | 约 197.9 Hz |
| `/cam0/image_raw` | `sensor_msgs/msg/Image` | 2912 | 约 19.8 Hz |
| `/cam1/image_raw` | `sensor_msgs/msg/Image` | 2912 | 约 19.8 Hz |
| `/vicon/firefly_sbx/firefly_sbx` | `geometry_msgs/msg/TransformStamped` | 14629 | 约 99.4 Hz |

bag 还包含 `/fcu/motor_speed`，其类型是当前系统可能未安装的 `asctec_hl_comm/msg/MotorSpeed`。第一阶段播放时只选择 OpenVINS 需要的三个传感器 topic，避免无关类型支持影响播放。

---

## 2. 第一阶段目标、边界与成功定义

### 2.1 唯一目标

在**拆桨、DISARM、禁止外部视觉融合**的台架条件下，让 PX4 的：

```text
listener vehicle_visual_odometry
```

持续看到来自 EuRoC → OpenVINS → ROS 2 → MAVROS → MAVLink 的新数据，并保存各级证据。

### 2.2 本阶段明确不做

- 不 ARM，不启动电机，不飞行。
- 不修改或重新刷写 PX4 固件。
- 不修改 PX4 EKF2、位置控制器或 OpenVINS 算法。
- 不宣称 EKF2 已融合外部视觉。
- 不进入 Position mode，不给位置环闭环 setpoint。
- 不用离线 EuRoC 轨迹评价真实机载 D435i 的效果。
- 不完成真实相机—IMU—机体外参、真实 yaw 对齐或延迟标定。

### 2.3 本阶段与最终目标的关系

```text
第一阶段：证明数据能安全、可观测地进入 PX4 vehicle_visual_odometry
    ↓
第二阶段：真实传感器标定 + 坐标/速度/时间/协方差正确性
    ↓
第三阶段：EKF2 融合与地面手持方向检查
    ↓
第四阶段：拆桨闭环/仿真验证，再考虑受控首飞
```

第一阶段完成后，距离“OpenVINS 定位接入 PX4 位置环”仍至少隔着第二、三阶段，不能跳级。

---

## 3. 安全门禁（Gate 0）

任何数据注入 PX4 前必须逐项确认：

```text
[x] 机体已拆桨，或飞控完全不连接动力系统
[x] PX4 为 DISARM
[x] 没有 ARM/模式切换/控制指令脚本在运行
[x] QGroundControl 或其他程序没有占用同一 USB 串口
[x] 记录当前 PX4 参数，不批量修改
[x] 确认 EKF2_EV_CTRL 当前值（实测为 0）
[ ] 若 EKF2_EV_CTRL 非 0，停止注入；经用户再次确认后才可备份原值并临时置 0
[x] 确认本次只验收 uORB 收包，不验收 EKF2 融合
```

说明：向正在运行的 EKF2 注入坐标和时间语义尚未验证的 EuRoC 数据，即使不 ARM，也会污染估计器状态。因此第一阶段的推荐做法是先确保 `EKF2_EV_CTRL=0`。任何参数写入均要记录原值和回滚值。

---

## 4. 计划目录结构

确认执行后创建：

```text
/home/he/uav_vio_px4/
├── src/
│   ├── open_vins/              # ROS 2 packages，参与 colcon build
│   ├── estimator_adapter/      # 自有 ROS 2 package，参与 colcon build
│   └── px4_autopilot/          # PX4 v1.17.0 参考源码，不参与 colcon
├── build/                      # colcon 生成
├── install/                    # colcon 生成
├── log/                        # colcon 生成
├── scripts/
└── docs/
```

数据集保持在：

```text
/home/he/datasets/euroc/V1_01_easy_db/
```

不复制 2 GiB bag 到项目内。

---

## 5. 版本与源码核对策略

### 5.1 OpenVINS

确认执行后克隆官方仓库，立即记录：

```bash
git -C /home/he/uav_vio_px4/src/open_vins rev-parse HEAD
git -C /home/he/uav_vio_px4/src/open_vins status --short --branch
```

重点核对当前 commit 中：

```text
ov_msckf/launch/subscribe.launch.py
ov_msckf/src/ros/ROS2Visualizer.cpp
ov_msckf/src/ros/ROS2Visualizer.h
src/open_vins/config/euroc_mav/
```

已通过官方当前源码预核对：

- launch 参数 `config:=euroc_mav` 有效，默认 namespace 为 `ov_msckf`。
- `pub_odomimu` 发布相对 topic `odomimu`，默认完整名称预期为 `/ov_msckf/odomimu`。
- `odomimu.header.frame_id = "global"`。
- `odomimu.child_frame_id = "imu"`。
- `odomimu.twist.twist.linear` 在源码中标注为 local frame 速度。
- 没有订阅者时，OpenVINS 可能不发布 `odomimu`，所以验证时必须保持 Adapter、`ros2 topic echo` 或 `ros2 topic hz` 至少一个订阅者在线。

最终以克隆下来的实际 commit 为准。

### 5.2 MAVROS2

本机 apt 当前候选版本：

```text
ros-humble-mavros         2.14.0
ros-humble-mavros-extras  2.14.0
ros-humble-mavros-msgs    2.14.0
```

Odometry plugin 位于 `mavros_extras`，因此只安装 `mavros` 不够。安装后必须用本机文件验证，而不是只按网页或 ROS 1 经验判断：

```bash
ros2 pkg prefix mavros
ros2 pkg prefix mavros_extras
ros2 pkg executables mavros
ros2 launch mavros px4.launch --show-args
```

当前官方 ROS 2 分支预核对结果：

- plugin 名称为 `odometry`。
- ROS 输入为插件私有名 `~/out`，常规完整 topic 为 `/mavros/odometry/out`。
- callback 为 `OdometryPlugin::odom_cb()`。
- 输出 MAVLink 消息为 `ODOMETRY`。
- 发送的 `frame_id` 为 `MAV_FRAME_LOCAL_FRD`，`child_frame_id` 为 `MAV_FRAME_BODY_FRD`，`estimator_type` 为 VISION。
- callback 使用 ROS 消息 `header.stamp` 生成 `time_usec`。
- plugin 要求 TF 能连接消息 parent frame 到 `<parent>_ned`，并连接 child frame 到 `<child>_frd`。

### 5.3 PX4 v1.17.0

本地 PX4 仓库只用于固定版本查源码：

```text
tag：v1.17.0
用途：核对 MAVLink ODOMETRY → uORB → EKF2 → vehicle_local_position 调用链
禁止：build、upload、flash、修改飞控源码
```

执行时重点确认实际 v1.17.0 中的符号与路径，不提前把 `main` 分支符号当成定论：

```text
src/modules/mavlink/mavlink_receiver.cpp
  MavlinkReceiver::handle_message_odometry()

src/modules/ekf2/EKF2.cpp
  EKF2::UpdateExtVisionSample()

src/modules/mc_pos_control/
  vehicle_local_position 的消费链
```

PX4 机内固件版本还需通过 MAVROS/QGroundControl 的 AUTOPILOT_VERSION 或 PX4 console 复核。USB 名称 `PX4 FMU v5.x` 只确认硬件接口族，不能单独证明固件为 v1.17.0。

---

## 6. 依赖准备（确认后执行）

先在干净 shell 中只 source 系统 ROS，避免 `/home/he/hno_vio_ws` 污染依赖判断：

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
printenv AMENT_PREFIX_PATH
```

目标 `AMENT_PREFIX_PATH` 在构建前只包含 `/opt/ros/humble`。随后：

1. 安装 ROS 2 Humble 的 `mavros`、`mavros_extras`、`mavros_msgs`。
2. 若 MAVROS 启动明确报告 GeographicLib geoid 数据缺失，再安装对应数据；不在没有错误证据时扩大安装范围。
3. 克隆 OpenVINS 官方仓库。
4. 克隆/检出 PX4 v1.17.0 源码，只做查阅。
5. 先运行 `rosdep check --from-paths src --ignore-src`，再安装明确缺失项。

所有实际版本、commit 和新增系统包写入阶段报告。

---

## 7. Gate 1：EuRoC bag 独立验证

无需转换，直接检查：

```bash
ros2 bag info /home/he/datasets/euroc/V1_01_easy_db
```

播放时仅选择 OpenVINS 必需 topic：

```bash
ros2 bag play /home/he/datasets/euroc/V1_01_easy_db \
  --topics /imu0 /cam0/image_raw /cam1/image_raw
```

第一轮使用 1.0 倍速。如果 OpenVINS 明确跟不上，再降低到 0.5 倍并在报告中记录，不先入为主修改 QoS 或算法参数。

验证：

```bash
ros2 topic hz /imu0
ros2 topic hz /cam0/image_raw
ros2 topic hz /cam1/image_raw
```

Gate 1 通过条件：三个 topic 都持续发布、双目时间序列存在、播放无致命类型/QoS 错误。

---

## 8. Gate 2：EuRoC → OpenVINS

构建：

```bash
cd /home/he/uav_vio_px4
source /opt/ros/humble/setup.bash
rosdep check --from-paths src/open_vins src/estimator_adapter --ignore-src
# 缺失项确认后再执行 rosdep install
colcon build --symlink-install --base-paths src/open_vins src/estimator_adapter
source install/setup.bash
```

先启动 OpenVINS：

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=euroc_mav \
  namespace:=ov_msckf \
  rviz_enable:=false
```

保持 odometry 订阅者，再启动 bag。验证：

```bash
ros2 node info /ov_msckf/run_subscribe_msckf
ros2 topic info -v /ov_msckf/odomimu
ros2 topic hz /ov_msckf/odomimu
ros2 topic echo /ov_msckf/odomimu --once
```

记录实际：

- OpenVINS commit。
- 实际 node/topic 名称。
- 输入 topic 是否正好为 `/imu0`、`/cam0/image_raw`、`/cam1/image_raw`。
- 输出频率。
- `header.frame_id`、`child_frame_id`。
- position、quaternion、velocity、pose/twist covariance 是否有限且更新。
- 初始化耗时和主要 warning/error。

Gate 2 通过条件：`/ov_msckf/odomimu` 连续有效输出，不要求此时连接 MAVROS/PX4。

---

## 9. estimator_adapter 设计

### 9.1 包结构

```text
src/estimator_adapter/
├── CMakeLists.txt
├── package.xml
├── include/estimator_adapter/adapter_core.hpp
├── src/adapter_core.cpp
├── src/estimator_adapter_node.cpp
├── config/{extrinsics,phase1,phase2}.yaml
├── launch/{phase1,phase2}.launch.py
└── test/test_adapter_core.cpp
```

### 9.2 输入与输出

```text
输入：/ov_msckf/odomimu        nav_msgs/msg/Odometry
输出：/mavros/odometry/out     nav_msgs/msg/Odometry
```

topic 名全部做成参数；默认值如上，但运行前仍以实际 discovery 为准。

### 9.3 第一版必须承担的职责

1. 检查 position、quaternion、选用的 covariance 是否为 finite。
2. 检查四元数非零并归一化；范数偏差超阈值时丢弃，而不是静默修复严重错误。
3. 检查源时间戳单调；检测跳变、回退和消息超时。
4. `replay_mode=true` 时将输出时间戳映射到当前系统时间，保证单调。
5. 将输出限制为 30 Hz，使用最新有效状态，不把约 200 Hz 的传播输出全部灌入 MAVLink。
6. 明确设置输出 `frame_id=odom`、`child_frame_id=base_link`。
7. 记录输入/输出/丢弃计数和最后一次丢弃原因。
8. 输出前打印一次醒目的 `DATA_CHAIN_TEST_ONLY / NOT_FOR_FLIGHT` 警告。

### 9.4 replay_mode

建议配置：

```yaml
replay_mode: true
output_rate_hz: 30.0
use_sim_time: false
```

规则：

- Adapter 和 MAVROS 必须使用系统时钟，不使用 `ros2 bag play --clock` 提供的历史仿真时间。
- 第一版可用消息到达时的 `node->now()` 重打时间戳，但必须验证输出严格单调。
- 保存原始 EuRoC stamp 到日志，报告中同时记录源 stamp 与发送 stamp。
- 真机模式必须改为 `replay_mode=false`，恢复真实测量时间并单独标定端到端延迟。

### 9.5 frame 与速度策略

OpenVINS 当前输出：

```text
pose：IMU 在 global 中的位姿
linear velocity：local/global frame
```

而 ROS `nav_msgs/Odometry`/MAVROS odometry plugin 要求：

```text
pose：在 parent frame 表达
twist：在 child/body frame 表达
```

因此不能复制 OpenVINS twist 后只把 `child_frame_id` 改为 `base_link`。

第一阶段默认：

```yaml
send_velocity: false
world_alignment_mode: data_chain_test_identity
body_extrinsic_mode: data_chain_test_identity
```

- `send_velocity=false` 时，把未验证的 linear/angular velocity 标成 unavailable（使用 MAVLink/PX4 可识别的 NaN 语义，并通过本机 MAVROS2 源码/实测确认），同时保证 EKF2 不启用速度融合。
- 如果本机 MAVROS 对 NaN 处理不符合预期，则 Adapter 正确计算 `v_body = R_body_world · v_world` 后再发送；不得原样伪装成 body velocity。
- 第一阶段允许把 OpenVINS `global` 与测试 `odom`、EuRoC `imu` 与虚拟 `base_link` 的安装关系暂设为 identity，但必须把它们标为**数据链测试的虚拟对齐**，不能用于真实机体。
- ENU→NED 和 FLU→FRD 的标准轴变换由 MAVROS odometry plugin/TF 链完成，不能再用 identity 代替。
- 真实 D435i/IMU 到飞行器 `base_link` 的安装外参属于第二阶段，未完成前禁止融合与飞行。

### 9.6 TF 原则

MAVROS ROS 2 当前 odometry plugin 会查找：

```text
odom_ned ← odom
base_link_frd ← base_link
```

执行时先查看 TF：

```bash
ros2 run tf2_ros tf2_echo odom_ned odom
ros2 run tf2_ros tf2_echo base_link_frd base_link
```

若 PX4 launch 已提供正确标准变换，不重复发布。若缺失，则按本机 MAVROS 2.14.0 配置补充标准 ENU↔NED、FLU↔FRD 固定旋转，并用基向量测试验证。禁止用两条全 identity static TF 只为消除报错。

### 9.7 单元测试

至少覆盖：

```text
[ ] NaN/Inf position 被丢弃
[ ] 零四元数被丢弃
[ ] 合法四元数归一化后保持旋转一致
[ ] 时间戳回退被检测
[ ] replay stamp 单调递增
[ ] 30 Hz 限频有效
[ ] send_velocity=false 不会生成可被误融合的伪速度
[ ] 已知姿态下 global velocity → body velocity 旋转正确
[ ] covariance 随旋转正确变换，或未启用字段明确标成 unavailable
```

---

## 10. Gate 3：OpenVINS → Adapter（不启动 MAVROS）

启动顺序：

```text
1. OpenVINS
2. estimator_adapter
3. ros2 bag play
```

验证：

```bash
ros2 topic hz /ov_msckf/odomimu
ros2 topic hz /mavros/odometry/out
ros2 topic echo /mavros/odometry/out --once
```

通过条件：

- OpenVINS 输入持续。
- Adapter 输出稳定在目标 30 Hz 附近。
- 输出 stamp 接近当前系统时间、严格单调。
- 输出 frame 为 `odom` / `base_link`。
- position/quaternion 有效。
- 默认模式下未验证速度不会作为有效 body velocity 发送。
- Adapter 无持续 warning、无异常消息泄漏。

---

## 11. Gate 4：MAVROS2 ↔ Pixhawk 连接

### 11.1 串口选择

优先使用：

```text
/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00
```

不要写死 `/dev/ttyACM0` 或当前的 `/dev/ttyACM1`。启动前确认 symlink 指向存在，并确认 QGroundControl 没有同时独占串口。

### 11.2 启动命令

安装 MAVROS2 后先检查 `px4.launch` 的实际参数，再形成最终命令。预期形态为：

```bash
ros2 launch mavros px4.launch \
  fcu_url:=serial:///dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:<实际波特率>
```

不在安装前硬编码波特率和 launch 参数。

### 11.3 连接验证

```bash
ros2 topic list | rg '^/mavros/'
ros2 topic echo /mavros/state --once
ros2 topic info -v /mavros/odometry/out
ros2 run tf2_ros tf2_echo odom_ned odom
ros2 run tf2_ros tf2_echo base_link_frd base_link
```

记录：

- MAVROS 实际版本。
- FCU connected 状态。
- 飞控 AUTOPILOT_VERSION/固件版本。
- odometry plugin 是否加载。
- `/mavros/odometry/out` 的实际订阅者。
- TIMESYNC 是否稳定、有无持续错误。
- 串口波特率和 by-id 路径。

Gate 4 只连接，不发布 Adapter 数据。

---

## 12. Gate 5：完整数据链注入

只有 Gate 0–4 全部通过，并确认 `EKF2_EV_CTRL=0` 后执行。

启动顺序：

```text
1. MAVROS2，确认 FCU connected 与 TIMESYNC
2. OpenVINS
3. estimator_adapter（先保持输出 disabled）
4. EuRoC bag
5. 确认 OpenVINS 与 Adapter 内部输出正常
6. enable Adapter 输出
7. PX4 console 观察 vehicle_visual_odometry
```

逐级证据：

### A. 数据集输入

```bash
ros2 topic hz /imu0
ros2 topic hz /cam0/image_raw
ros2 topic hz /cam1/image_raw
```

### B. OpenVINS 输出

```bash
ros2 topic hz /ov_msckf/odomimu
```

### C. Adapter 输出

```bash
ros2 topic hz /mavros/odometry/out
ros2 topic echo /mavros/odometry/out --once
```

### D. MAVROS 输入与发送

确认 odometry plugin 的订阅计数为 1，且没有持续 TF lookup、timestamp 或 send error。必要时开启该 plugin 的 debug 日志，但不全局刷屏。

### E. PX4 uORB

在 PX4 MAVLink Console/NSH 中：

```text
listener vehicle_visual_odometry 5
```

按 v1.17.0 实际字段验证：

- 数据持续更新，不是同一条缓存值。
- `timestamp` 与 `timestamp_sample` 合理，样本没有明显过期。
- pose frame/velocity frame 与预期一致。
- position/quaternion 有限。
- 默认禁用速度时，PX4 不把伪速度视为有效。
- 接收频率约 30 Hz，且无大量丢包/队列堆积迹象。

可补充：

```text
uorb top
ekf2 status
listener vehicle_local_position 1
```

但 `vehicle_local_position` 本身持续更新不是外部视觉融合证据；第一阶段也不要求 fusion flags 变为 active。

### F. 停止与回滚

验证到足够样本后按逆序停止：bag → Adapter → OpenVINS → MAVROS。若曾经经确认临时修改 PX4 参数，恢复原值并再次读回核对。

---

## 13. 第一阶段验收清单

### 13.1 必须全部满足

```text
[x] Gate 0 安全条件满足，全程 DISARM/裸板
[x] V1_01_easy_db 能按选择的三个 topic 播放
[x] OpenVINS 在 ROS 2 Humble 编译成功，commit 已记录
[x] /ov_msckf/odomimu 持续输出
[x] Adapter 单元测试通过（5 tests，0 failures）
[x] Adapter 输出约 30 Hz，时间戳实时且单调
[x] 输出 frame/TF 满足 MAVROS plugin，未使用伪 identity ENU/NED 变换
[x] MAVROS2 经 USB by-id 路径连接 PX4
[x] odometry plugin 实际订阅 /mavros/odometry/out
[x] MAVLink ODOMETRY 发出，无持续 TF/timestamp 错误
[x] PX4 vehicle_visual_odometry 持续收到新数据
[x] EKF2_EV_CTRL 在注入期间为 0
[x] 停止后系统和参数恢复到测试前状态（本次未修改 PX4 参数）
```

### 13.2 明确不算通过

- 仅看到 `/mavros/odometry/out`，但没有 PX4 uORB 数据。
- 只看到一次 `vehicle_visual_odometry` 缓存值。
- 用 identity TF 消除错误，却没有验证轴向语义。
- `vehicle_local_position` 有数据，就宣称 OpenVINS 已被 EKF2 融合。
- 用 EuRoC 历史数据进入 Position mode 或尝试 ARM。

---

## 14. 执行产物

确认执行后生成：

```text
logs/phase1_data_chain_report.md
logs/phase1_commands.log
logs/phase1_topics/
scripts/run_openvins_euroc.sh
scripts/run_estimator_adapter.sh
scripts/run_mavros_usb.sh
scripts/check_data_chain.sh
```

脚本在各 Gate 手工验证通过后再创建，不一开始封装复杂流程。所有脚本必须默认安全：

```text
- 不 ARM
- 不发 setpoint
- 不改 PX4 参数
- Adapter 默认 output_enabled=false 或需要显式开关
- 对错误串口/未 DISARM/缺少 TF 直接退出
```

阶段报告至少包含：

1. 系统、ROS、内核、MAVROS 版本。
2. OpenVINS 与 PX4 源码 commit/tag。
3. Pixhawk USB ID、by-id 路径与固件版本复核结果。
4. EuRoC bag 格式、topic、消息数、实测频率。
5. OpenVINS 实际输入/输出 topic、frame、频率。
6. Adapter 配置、测试结果、输入/输出/丢弃统计。
7. 时间戳映射方式与源/发送/FCU 样本时间证据。
8. TF 树与 ENU↔NED、FLU↔FRD 变换证据。
9. MAVROS odometry plugin 的本机源码路径、callback 和 topic。
10. PX4 `vehicle_visual_odometry` 连续样本证据。
11. PX4 参数原值、临时值、恢复值（如有）。
12. 遇到的问题、最小修复和未解决项。
13. 明确写出“第一阶段未证明 EKF2 融合或位置环可用”。

---

## 15. 第二阶段前置清单（本次不执行）

进入真实 OpenVINS → PX4 位置估计融合前，必须处理：

```text
[ ] D435i 图像与 IMU 时间同步/硬件时间基准
[ ] 相机内参、双目外参、IMU 噪声参数
[ ] Camera/IMU 到机体 base_link/FRD 的真实安装外参
[ ] OpenVINS global 到 PX4 local frame 的初始 yaw/原点策略
[ ] OpenVINS global velocity 转 body FRD 或正确声明 velocity frame
[ ] pose、velocity、orientation covariance 的语义和数值量级
[ ] VIO 初始化、失锁、重定位、reset_counter/quality 的传播策略
[ ] 真实 measurement timestamp 与 EKF2_EV_DELAY 标定
[ ] EKF2_EV_CTRL 融合位逐项启用，而不是一次全开
[ ] EKF2 innovation、reject flags、local position validity 验证
[ ] 地面手持轴向检查：前/右/上运动与 PX4 输出符号一致
[ ] VIO 中断/恢复的 failsafe 行为
[ ] 拆桨闭环或 SITL/HITL 验证
```

只有这些通过，才讨论 PX4 Position mode、位置 setpoint 或飞行测试。

---

## 16. 执行停止条件

遇到以下任一情况立即停止当前 Gate，不跨级补丁：

- 飞控意外进入 ARM 或电机有启动风险。
- `EKF2_EV_CTRL` 非 0 且未获确认允许临时关闭。
- PX4 实际固件不是 v1.17.0，导致接口/参数与计划不符。
- OpenVINS 输出 frame/速度语义与已核对源码不同。
- MAVROS odometry plugin 未加载或实际 topic 不同。
- TF 查找失败或变换方向无法证明。
- 时间戳回退、样本明显过期或 TIMESYNC 持续异常。
- 发送频率远离 30–50 Hz，或串口明显拥塞。
- 出现 NaN/Inf/零四元数进入 PX4。

定位顺序固定为：

```text
Dataset → OpenVINS → Adapter → TF/MAVROS → MAVLink/USB → PX4 uORB
```

在哪一级证据首次消失，就只处理该一级。

---

## 17. 核对依据

- OpenVINS 官方 ROS 2 launch：<https://github.com/rpng/open_vins/blob/master/ov_msckf/launch/subscribe.launch.py>
- OpenVINS 官方 ROS2Visualizer：<https://github.com/rpng/open_vins/blob/master/ov_msckf/src/ros/ROS2Visualizer.cpp>
- MAVROS ROS 2 odometry plugin：<https://github.com/mavlink/mavros/blob/ros2/mavros_extras/src/plugins/odom.cpp>
- MAVROS ROS 2 PX4 配置：<https://github.com/mavlink/mavros/blob/ros2/mavros/launch/px4_config.yaml>
- PX4 v1.17 外部位置估计文档：<https://docs.px4.io/1.17/en/ros/external_position_estimation.html>
- PX4 v1.17 MAVLink receiver：<https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/mavlink/mavlink_receiver.cpp>
- PX4 v1.17 EKF2：<https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/ekf2/EKF2.cpp>

---

## 18. 等待确认

在用户明确回复确认前，Codex 停在这里，不执行以下动作：

```text
- 不安装 MAVROS/依赖
- 不克隆仓库
- 不创建 Adapter 代码
- 不编译
- 不启动 MAVROS/OpenVINS/bag
- 不读取或修改 PX4 参数
- 不向飞控注入任何外部视觉消息
```

用户确认后，严格按 Gate 0 → Gate 5 顺序执行并逐 Gate 汇报。
