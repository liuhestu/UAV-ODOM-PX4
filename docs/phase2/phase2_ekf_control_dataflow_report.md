# OpenVINS → PX4 EKF/控制数据流报告

> 当前状态（2026-09-13）：Adapter 的 3D pose-only 改造已完成并通过软件测试；新的 Phase3D 实机测试尚未运行。  
> 历史 Phase2 的 XY-only 测试已经回滚，不能充当本轮 XY+Z 验收结果。

## 当前软件结果

- Adapter 实现 `T_WP = T_WV inverse(T_PV)`，输出 `odom → base_link`；`base_link` 原点是 Pixhawk IMU。
- phase1 与 phase2 共用 `src/estimator_adapter/config/extrinsics.yaml`。当前外参是未标定单位 SE(3)，不可用于飞行。
- phase1/phase2 均为 `send_velocity=false`；linear/angular velocity 及 twist covariance 均为 NaN/unavailable。
- 外参数组长度、有限性和四元数范数在启动时校验；xyzw 四元数归一化后使用，运行时修改被拒绝。
- pose covariance 当前原样透传；未实现非单位外参的完整 covariance 变换。
- `colcon build` 成功；Adapter core 的 12 个测试全部通过。
- 实际 ROS 2 phase2 launch 成功读取单位外参与 `send_velocity=false`；在线修改外参返回 `extrinsics are startup-only`。
- 隔离飞控的 EuRoC 实测输出为 29.997–30.000 Hz；抽样 pose 有限、twist 六个分量及全部 covariance 为 NaN；停止 bag 后输出按 stale gate 被抑制。
- Pixhawk 只读预检确认 `connected=true`、`armed=false`、`EKF2_EV_CTRL=0`、`EKF2_HGT_REF=1`、`EKF2_EV_NOISE_MD=0`、`EKF2_EVP_GATE=5`、`EKF2_EVP_NOISE≈0.1`、`EKF2_EV_POS_X/Y/Z=0`；随后已停止 MAVROS，未注入 EV、未写参数。

## 本轮待采集判定

```text
Phase3D-A EKF2 EV XY processing: NOT_RUN
Phase3D-B EKF2 EV Z processing: NOT_RUN
Phase3D-C mc_pos_control XYZ runtime dataflow: NOT_RUN
```

本轮 Gate 使用 `EKF2_EV_CTRL=3`：bit 0 水平位置、bit 1 垂直位置开启，velocity/yaw 位关闭。`EKF2_HGT_REF`、`EKF2_EV_NOISE_MD`、`EKF2_EVP_GATE`、`EKF2_EVP_NOISE` 和 `EKF2_EV_POS_X/Y/Z` 必须保持现场原值；其中 EV position offset 预期均为 0。

验收必须分别保存 `estimator_aid_src_ev_pos`、`estimator_aid_src_ev_hgt` 与 `vehicle_local_position` 的 XY/Z/reset/valid 字段，并读取三个控制 setpoint topic。EuRoC 与静止 IMU 运动不一致，因此真实 fused、rejected 或 reset 都可记录，不通过修改 gate/noise 追求融合通过。

## 历史证据的适用边界

2026-09-12 的 Phase2 曾以 `EKF2_EV_CTRL=1` 完成 XY-only、DISARM 测试并恢复为 0，重启后确认 `cs_ev_pos=false`。该轮证明过水平位置处理和 landed/DISARM 控制器运行，但没有启用或验证 EV Z。

该轮为绕过旧判断曾发送有限 body velocity，并记录 `velocity_frame=3`、velocity 与 covariance 有限。PX4 v1.17 源码复核表明 position、orientation、velocity 在 `UpdateExtVisionSample()` 中独立校验，因此有限 velocity 不是接收 pose 的必要条件，历史 velocity 数值/covariance 结果不再是 Phase3D 验收依据。

不过，本地 v1.17 的后续 `controlExternalVisionFusion()` 在 velocity frame 为 unknown 时仍会提前返回。MAVROS 应根据 `base_link` 把 ODOMETRY child frame 设置为 `BODY_FRD`，而 velocity 数值保持 NaN；实机 Gate 必须读回 `vehicle_visual_odometry.velocity_frame` 验证这一点。若仍为 unknown，本轮 pose-only 路径是 BLOCKED，不能设置 `EKF2_EV_CTRL=3`，也不能恢复发送伪速度来绕过。

## 安全与回滚

全程必须 `armed=false`，不得发送 setpoint 或切换 Offboard。顺序为先关闭 Adapter 输出，再把 `EKF2_EV_CTRL` 恢复基线并由 MAVROS/NSH 双重读回，停止测试进程，最后重启 Pixhawk 并确认 `cs_ev_pos=false`、`cs_ev_hgt=false`。
