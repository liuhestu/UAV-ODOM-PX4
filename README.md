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

## 编译

```bash
cd ~/uav_vio_px4
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths src/open_vins src/estimator_adapter
source install/setup.bash
```

## 实时启动

先启动 RealSense 和 OpenVINS：

```bash
source /opt/ros/humble/setup.bash
source ~/uav_vio_px4/install/setup.bash
ros2 launch ov_msckf subscribe_realsense.launch.py
```

再启动 Adapter 和 MAVROS：

```bash
source /opt/ros/humble/setup.bash
source ~/uav_vio_px4/install/setup.bash
ros2 launch estimator_adapter estimator_adapter.launch.py
```

默认使用实时相机时间戳并向 `/mavros/odometry/out` 发布：

```text
output_enabled:=true
replay_mode:=false
start_mavros:=true
fcu_url:=serial:///dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:57600
```

仅启动 Adapter、不占用 PX4 串口：

```bash
ros2 launch estimator_adapter estimator_adapter.launch.py start_mavros:=false
```

临时关闭向 MAVROS/PX4 发布：

```bash
ros2 launch estimator_adapter estimator_adapter.launch.py output_enabled:=false
```

该启动文件不会修改 PX4 的 `EKF2_EV_CTRL`、解锁飞控或发送控制设定值。当前
`config/extrinsics.yaml` 仍是未标定单位外参，只可用于地面数据链测试。
