OpenVINS 输出 RealSense Odom 的 3D 位姿，Adapter 用一个固定外参把它转换成 Pixhawk IMU / PX4 控制所使用的位姿，再送给 PX4 EKF 融合。

```bash
硬件链路：
Ubuntu ──── Micro-USB ──── Pixhawk
     | ———— USB C3.0 ————— Realsense
     | ———— 数传 ————— Motion Capture
    

```

```bash
数据链路：
VIO/动捕：Odom 6DOF topic
  ↓
estimator_adapter：外参变换，发布数据
  ↓
MAVROS
  ↓
PX4 EKF2（EKF2_EV_CTRL = 3，XYZ融合）
  ↓
vehicle_local_position
  ↓
位置环
  ↓
悬停
```