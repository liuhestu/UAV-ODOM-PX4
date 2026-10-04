# PX4 + OpenVINS 第一阶段执行报告

执行日期：2026-09-12（Asia/Shanghai）

## 结论

第一阶段通过。EuRoC 数据经过 OpenVINS、`estimator_adapter`、MAVROS2 和 USB，以 MAVLink ODOMETRY 进入 PX4 `vehicle_visual_odometry`。该结论不包含 EKF2 融合或位置环闭环可用性。

## 安全状态

- 用户确认 Pixhawk 是未连接电机的裸板。
- 注入前、注入中和停止前 `/mavros/state` 均为 `connected=true`、`armed=false`。
- `EKF2_EV_CTRL` 三次读取均为整数 `0`。
- 未发送 ARM、飞行模式、位置/速度/姿态或执行器设定值。
- 未修改 PX4 参数，未编译或刷写固件。

## 软件与硬件基线

- ROS 2 Humble。
- MAVROS：`2.14.0-1jammy.20260804.200257`。
- MAVROS extras：`2.14.0-1jammy.20260804.203703`。
- OpenVINS：`69488123ed9362dd44b6f28e7f4680abbff1442b`。
- 飞控 AUTOPILOT_VERSION：PX4 v1.17.0，flight software `011100ff`，commit `d6f12ad1c4000000`。
- 板卡：PX4 FMU v5.x，VID/PID `26ac:0032`。
- 串口：`/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:57600`。
- GeographicLib geoid：工作区 `.geographiclib/geoids/egm96-5.pgm`。

## 构建和测试

- OpenVINS 的 5 个 ROS 包全部构建成功。
- `estimator_adapter` 构建成功。
- Adapter 测试：5 tests，0 errors，0 failures，0 skipped。
- 覆盖：非有限位姿拒绝、四元数归一化、单调回放时间戳、未验证速度屏蔽、全局速度转 child/body frame。

## ROS 侧证据

- OpenVINS 输入：`/imu0`、`/cam0/image_raw`、`/cam1/image_raw`。
- `/ov_msckf/odomimu`：`frame_id=global`、`child_frame_id=imu`，原始输出约 175–185 Hz。
- Adapter `/mavros/odometry/out`：稳定 30.000 Hz，周期约 0.033 s，抖动标准差约 0.00009 s。
- 输出：`frame_id=odom`、`child_frame_id=base_link`，系统实时时间戳，姿态归一化。
- 默认 `send_velocity=false`：linear/angular velocity 及 twist covariance 为 NaN。
- MAVROS odometry plugin 对 `/mavros/odometry/out` 的订阅数为 1。
- `tf2_echo` 验证 MAVROS 标准转换：
  - `odom_ned ← odom` 矩阵为 `[ [0,1,0], [1,0,0], [0,0,-1] ]`。
  - `base_link_frd ← base_link` 矩阵为 `[ [1,0,0], [0,-1,0], [0,0,-1] ]`。

## PX4 uORB 证据

通过 MAVLink Shell 执行：

```text
listener vehicle_visual_odometry -n 5
```

连续收到 #1–#5。相邻 `timestamp_sample` 约 33 ms，接收延迟约 0.5–5.6 ms；其中：

```text
pose_frame: 2
velocity_frame: 0
velocity: [nan, nan, nan]
angular_velocity: [nan, nan, nan]
velocity_variance: [nan, nan, nan]
```

位置、四元数、position variance 和 orientation variance 为有限值，并随 OpenVINS 更新。

## 已知现象

- OpenVINS ROS 2 节点在收到 SIGINT 时偶发 `std::system_error`/退出码异常；数据运行阶段正常，不影响本次收包结论。后续工程化启动器应把它作为进程退出问题单独处理。
- MAVROS 2.14.0 对 PX4 v1.17 的部分 EVENT 文本无法完整解码，连接初期显示数字 EVENT；该现象发生在启用 Adapter 前，与本次 ODOMETRY 注入无关。
- rosbag 使用 `--loop` 回卷时 OpenVINS 会看到时间倒退，因此正式复现实验建议每轮重新启动 OpenVINS，或只单次播放，不依赖跨回卷连续估计。

## 停止与恢复

先将 `output_enabled=false`，再次读取 `armed=false` 和 `EKF2_EV_CTRL=0`，随后按 bag → Adapter → OpenVINS → MAVROS 停止。测试结束后未留下上述运行进程。
