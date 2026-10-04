# 阶段二部分执行记录（历史归档）

> 本文件只记录 2026-09-12 的旧 XY/velocity 试验，不是当前 Phase3D 的验收报告。当前方案使用 `send_velocity=false`、`EKF2_EV_CTRL=3` 并分别验证 XY 与 Z；历史 velocity 与 covariance 结果已移出验收依据。当前状态见 `phase2/phase2_ekf_control_dataflow_report.md`。

日期：2026-09-13（Asia/Shanghai）

## 已完成

- 新增 `phase2.yaml` 和 `phase2.launch.py`，阶段二使用有限 body-frame velocity，输出默认关闭。
- Adapter 增加线速度 covariance 有限性、对称性和非负对角线检查。
- 测试汇总：7 tests，0 errors，0 failures，0 skipped。
- EuRoC 离线输出：30.000 Hz；`odom → base_link`；position、quaternion、linear velocity 和 linear velocity covariance 有限。

## Gate 0 首次连接结果

```text
connected: true
armed: false
mode: AUTO.LOITER
PX4: v1.17.0 / d6f12ad1c4000000
```

已读取参数：

```text
EKF2_EV_CTRL      0
EKF2_EV_DELAY     0.0 ms
EKF2_EV_NOISE_MD  0
EKF2_EV_QMIN      0
EKF2_EVP_GATE     5.0
EKF2_EVP_NOISE    0.10000000149
EKF2_EV_POS_X     0.0
EKF2_EV_POS_Y     0.0
EKF2_EV_POS_Z     0.0
EKF2_MULTI_IMU    2
EKF2_MULTI_MAG    参数未导出
```

## 暂停原因

在建立 MAVLink Shell 后，MAVROS 报告 heartbeat timeout，随后串口发送队列溢出。USB 枚举仍存在，但 `/dev/ttyACM0` 无数据。测试在 Adapter 未启动、阶段二数据未注入、PX4 参数未修改时停止。

当前没有 MAVROS、OpenVINS、Adapter、bag 或 MAVLink Shell 进程。继续前需要物理拔插/断电重连 Pixhawk，并重新执行完整 Gate 0 快照。
