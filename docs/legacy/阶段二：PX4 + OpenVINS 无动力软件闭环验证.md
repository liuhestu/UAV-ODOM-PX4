# OpenVINS → PX4 三维位置无动力验证计划

> 状态：Adapter 软件修改与单元测试已完成；Phase3D 实机验收尚未执行。  
> 安全边界：全程 DISARM，不发送 setpoint，不切换 Offboard。当前 \(T_{PV}\) 是未标定的单位 SE(3)，不可用于飞行。

## 1. 目标与非目标

目标是证明 OpenVINS 的 XY/Z position 经 MAVROS 进入 PX4 EKF2，并由主 EKF 实例反映到 `vehicle_local_position`；随后证明已启用的 `mc_pos_control` 消费 XYZ 状态并处于 landed/DISARM 安全输出状态。

本阶段不融合 external-vision velocity 或 yaw，不更换主高度参考，不证明悬停稳定性、控制误差收敛或可飞性。`src/px4_autopilot/` 只用于源码核对，不参与编译或刷写。

## 2. Adapter 坐标与外参契约

- `V`：OpenVINS `/ov_msckf/odomimu` 的 RealSense IMU frame。
- `P`：以 Pixhawk IMU 为原点的 ROS FLU frame。
- `W`：OpenVINS global/odom frame。
- \(T_{PV}\) 将 V 中的向量分量变换到 P；配置位于 `src/estimator_adapter/config/extrinsics.yaml`，phase1/phase2 launch 共用该文件。

```text
x^P = R_PV x^V + t_PV
T_WP = T_WV inverse(T_PV)
R_WP = R_WV R_PV^T
p_P^W = p_V^W - R_WP t_PV
q_WP = q_WV inverse(q_PV)
```

运行时输出仍为 `odom → base_link`，其中 `base_link` 的物理原点定义为 Pixhawk IMU；MAVROS 负责 FLU→FRD。相机到 RealSense IMU 的外参仍由 OpenVINS 内部负责。

外参参数只在启动时读取，顺序固定为 `translation_xyz_m=[x,y,z]`、`rotation_xyzw=[x,y,z,w]`。长度错误、非有限值、零/过小范数以及偏离单位范数超过 `1e-3` 的四元数必须使节点拒绝启动；合法四元数读取后归一化。运行中修改外参必须拒绝。

当前配置为：

```yaml
openvins_imu_to_pixhawk_imu:
  translation_xyz_m: [0.0, 0.0, 0.0]
  rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

`send_velocity=false`：linear/angular velocity 与全部 twist covariance 输出 NaN/unavailable。pose covariance 本阶段逐元素透传，不声称完成非单位外参下的完整 covariance 变换。

## 3. PX4 参数 Gate

唯一临时修改的融合开关：

```text
EKF2_EV_CTRL = 3
```

精确位定义：bit 0 (`1`) 开水平位置，bit 1 (`2`) 开垂直位置，bit 2 (`4`) 速度关闭，bit 3 (`8`) yaw 关闭。因此只接受十进制精确值 `3`，并通过 MAVROS 与 NSH 双重读回。

以下参数只记录原值，不为追求融合而修改：

```text
EKF2_HGT_REF
EKF2_EV_NOISE_MD
EKF2_EVP_GATE
EKF2_EVP_NOISE
EKF2_EV_POS_X
EKF2_EV_POS_Y
EKF2_EV_POS_Z
```

`EKF2_EV_POS_X/Y/Z` 保持 `0`，因为 Adapter 已将位姿换算到 Pixhawk IMU 原点。`EKF2_HGT_REF` 保持现场原值；EV Z 是高度辅助源，不被强制设置为主高度参考。

PX4 v1.17 的 `UpdateExtVisionSample()` 对 position、orientation、velocity 独立校验，因此有限 velocity 不是接收 pose 的必要条件；此前“必须发送有限 velocity”的结论作废，历史 velocity 与 covariance 结果不再属于验收证据。但本地 v1.17 的后续 `controlExternalVisionFusion()` 在 velocity frame 为 unknown 时会提前返回，所以 Gate 4 必须同时确认 `vehicle_visual_odometry.velocity_frame` 仍是 MAVROS 根据 `base_link` 设置的 `BODY_FRD`。即使 frame 枚举有效，velocity 数值及 covariance 仍应为 NaN/unavailable，且 `EKF2_EV_CTRL` bit 2 保持关闭。

## 4. 实机顺序

1. 确认裸板、`armed=false`，保存全部目标参数、两个 EKF 实例及 estimator flags 基线。
2. 以默认 `output_enabled=false` 启动 OpenVINS、Adapter 和单次 EuRoC bag。
3. 在 ROS 侧确认 position/quaternion 有限、twist 全部为 NaN、约 30 Hz、系统时间戳单调、stale input 会停止输出，再启用 Adapter。
4. 保持 `EKF2_EV_CTRL=0`，确认 `vehicle_visual_odometry` 收到有限 position/quaternion、velocity 数值 unavailable，且 velocity frame 枚举不是 unknown；否则不得开启融合。
5. 临时设置 `EKF2_EV_CTRL=3`，用 MAVROS 与 NSH 读回后采集第 5 节证据。
6. 先把 Adapter `output_enabled=false`，再恢复 `EKF2_EV_CTRL` 的基线值并双重读回。
7. 停止 bag、Adapter、OpenVINS；重启 Pixhawk。
8. 重连后确认 `armed=false`、`cs_ev_pos=false`、`cs_ev_hgt=false` 且没有残留测试进程。

任一时刻若 armed、参数读回不一致、frame/时间语义异常或无法确认回滚目标，立即停止注入并进入回滚。

## 5. 证据与通过条件

采集主 EKF 实例对应的：

```text
vehicle_visual_odometry
estimator_selector_status
estimator_status_flags
estimator_aid_src_ev_pos
estimator_aid_src_ev_hgt
estimator_event_flags
vehicle_local_position
ekf2 status
trajectory_setpoint
vehicle_local_position_setpoint
vehicle_attitude_setpoint
```

### Phase3D-A EKF2 EV XY processing

- `vehicle_visual_odometry` position/quaternion 有限，velocity 数值 unavailable，velocity frame 枚举有效。
- 主实例 `cs_ev_pos=true`、`cs_ev_vel=false`、`cs_ev_yaw=false`。
- `estimator_aid_src_ev_pos` 的 observation、innovation、test ratio 与 `time_last_fuse` 持续更新，并记录真实的 fused/rejected/reset 结果。
- 保存 `vehicle_local_position.x/y`、`xy_valid`、`delta_xy`、`xy_reset_counter`，其时间变化可与 EV aid-source 对应。

### Phase3D-B EKF2 EV Z processing

- 主实例 `cs_ev_hgt=true`，同时 `cs_ev_vel=false`、`cs_ev_yaw=false`。
- `estimator_aid_src_ev_hgt` 的 observation、innovation、test ratio 与 `time_last_fuse` 持续更新，并记录真实的 fused/rejected/reset 结果。
- 保存 `vehicle_local_position.z`、`z_valid`、`delta_z`、`z_reset_counter`，其时间变化可与 EV aid-source 对应。

EuRoC 与静止 Pixhawk IMU 并非同一物理运动，XY/Z innovation 不要求全部通过；不得调大 gate/noise 来制造 PASS。只要处理链证据完整，fused、rejected 或 reset 都应如实报告。

### Phase3D-C mc_pos_control XYZ runtime dataflow

源码链：

```text
vehicle_local_position
  → set_vehicle_states()
  → PositionControl::setState()
  → PositionControl::update()
  → vehicle_local_position_setpoint
  → vehicle_attitude_setpoint
```

运行时必须观察 `flag_multicopter_position_control_enabled=true`；当 `vehicle_local_position.z_valid=true` 时确认控制器消费 Z 状态，并同步读取三个 setpoint topic。预期为 landed/DISARM 安全态，不期待有效飞行 setpoint 或实际推力。

最终分别给出 `Phase3D-A`、`Phase3D-B`、`Phase3D-C` 的 PASS / FAIL / NOT_RUN / BLOCKED，不允许用源码推断代替运行证据。

## 6. 软件验收

- 单位、90°固定旋转、纯平移与组合 SE(3) 的 pose 结果符合矩阵公式。
- 非法数组、NaN、Inf、零四元数及异常范数被拒绝。
- pose-only 输出 position/quaternion 有限，全部 twist 字段与 covariance 为 NaN。
- 单位外参下 pose covariance 原样透传。
- EuRoC 离线输出保持约 30 Hz、系统时间戳严格单调并抑制 stale input。
