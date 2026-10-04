# 本地 Codex 接续与验证

当前分支 `refactor/native-px4-ros1`。先读 `README.md`、`docs/ARCHITECTURE.md` 和本文件。
本轮云端只编辑并测试软件，没有连接真实 RealSense/Pixhawk，没有 ARM、模式切换、PX4 参数写入或固件构建/刷写。
本地后续先完成构建与只读/软件验证，再按用户当时授权决定是否做硬件或飞行操作。

目录已整理为根 `launch/`、`config/`、`test/` 与五个业务包；共享基础包集中在 `src/support/`。原 `uav_commander` 包已改名 `commander`；顶层任务入口为 `roslaunch uav_system commander.launch`。
如果你已有本地修改，先检查 `git status` 并保存自己的改动，再使用 `git pull --ff-only` 获取目录整理提交；不要覆盖本地标定配置。若已经构建旧的 `uav_commander` 包，整理后需清理该旧包的devel/install产物；必要时在保存修改后执行 `catkin clean`，再重新 `catkin build`，避免旧入口继续被ROS发现。

## 1. 构建门

```bash
git switch refactor/native-px4-ros1
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
python3 -m pip install -r test/requirements.txt
python3 -m unittest discover -s test -v
```

检查各本地包和 OpenVINS 的 ROS1 条件依赖。固件位于 `third_party/px4_autopilot`，不在 catkin 的 src 扫描范围内。
Jetson/Ubuntu 22.04 的 Noetic 安装、OpenCV/cv_bridge ABI 和 Ceres 版本需沿用本机可用环境，禁止用 Humble 环境编译这些节点。
先修复 build/launch/import 差异，把结果写到 `docs/VALIDATION.md`，不把静态测试当作 ROS 构建通过。

## 2. 无硬件软件门

```bash
roslaunch uav_system mock_system.launch
rostopic echo /uav/state/health
rostopic echo /uav/system/status
rosparam set /mock_state_source/mode timeout
rosparam set /mock_state_source/mode static
rosparam set /mock_state_source/mode nan
rosparam set /mock_state_source/mode jump
```

期望：static 时 source/adapter healthy；SYS/EKF/EV缺失使 system_ready/arm_ready 仍为 false。
超时/NaN 使 adapter unhealthy；jump 锁定错误直到重启 system。Backend output_enabled=false，不能输出飞控数据或转发 ARM。
同时启动 Commander 验证它停留 WAIT_SYSTEM；不要用假健康或固定姿态把它解锁。

## 3. 定位源只读门（获硬件验证授权后）

```bash
roslaunch uav_system system.launch state_source:=openvins start_mavros:=false start_ev_observer:=false output_enabled:=false
rostopic hz /ov_msckf/odomimu
rostopic echo -n 1 /uav/state/odom
rostopic echo /uav/state/health
```

确认没有另一个相机 driver/manager；核对 ROS1 RealSense 的 launch args、200 Hz IMU 配置和 IR emitter 动态参数。
RealSense ROS1 2.x 常见参数命名 `/camera/stereo_module/emitter_enabled`，应实际读取动态配置确认关闭，不仅看 rosparam 有一个值。
核对图像是 raw 还是 rectified、848×480 profile 与现有 Kalibr 相机模型是否匹配；继承的标定文件不能仅凭文件存在就当作当前相机已标定。
OpenVINS 初始化等待输出后，平移/旋转设备，确认位置、姿态、速度的轴向和 timestamp age。

填 source YAML 的 `T_SB`（base_link 在 sensor 中）和 `T_AW`（world→Z-up canonical），标定验证通过后才将 calibrated/verified 改为 true。
如果自己已有外部 RealSense driver，可设 source launch 的 `start_camera: false` 并相应移除 owned_nodes 中的相机项；不要重复占用。
NOKOV 模板需真实 ROS Odometry 发布者；SDK毫米、左/右手系、速度 frame 不会在本模板中猜测或自动纠正。

## 4. PX4 遥测与融合门（获硬件验证授权后，不启动 Commander）

记录实机 PX4、MAVROS、RealSense/OpenVINS 版本。按照实机 PX4 文档确认外部视觉位置/高度融合参数、延时、噪声、参考点和失效策略；代码不会写这些参数。
开启基础 system 后查看：

```bash
rostopic echo /mavros/state
rostopic echo /mavros/sys_status
rostopic echo /mavros/estimator_status
rostopic echo /mavros/extended_state
rostopic hz /mavros/local_position/odom
rostopic echo /uav/backend/ev_sent
rostopic echo /uav/px4/ev_status
rostopic echo /uav/system/status
```

`ev_sent=true` 仅表示本机发布成功。真正融合门要求 PX4-side observer 收到鲜活 visual odometry、EV pos/hgt fused、local validity 和稳定 reset counters。
Observer 目前只支持单 EKF，并通过参数读取验证。实机必须有四个 uORB topic 及相应字段。
若 ESTIMATOR_STATUS 等 MAVLink 流未启用，使用实机标准配置/消息频率接口补齐；缺失时 Supervisor 不伪造有效。
Observer 运行时关闭 QGC MAVLink Console；发生格式、输出截断/超时或带宽问题，修正桥或换成对应固件结构化桥，不删除融合 gate。
重复初始化/估计器 reset 锁定故障后重启整个 system，并检查新 session，不能继续旧任务。
对比 PX4 local pose 与 canonical pose 的变化、轴向和时间连续性；允许不同原点/航向，但 Commander 始终在 PX4 local ENU 下生成目标。

## 5. SITL 任务门

单独运行匹配版本的 PX4 SITL，接入真实随仿真机体运动的 Odometry，而非 static mock。
创建本地 `sitl.yaml`：source.simulated=true，正确的 launch/topic/frame/covariance、外参和轴对齐；先不启动 Commander。

```bash
roslaunch uav_system sitl_integration.launch source_config:=/absolute/path/to/sitl.yaml
```

检查实际 `/mavros/fcu_url` 为 `udp://:14540@127.0.0.1:14557`；Backend 会检查同一参数。
先完成 ready、EV pos/hgt 融合与 on-ground，然后在模拟环境启动第二终端 Commander。
检查服务只请求一次；服务响应和 FCU实际 mode/armed 分开；爬升限速、hover、AUTO.LAND、landed与disarmed顺序。
注入数据停止、NaN、frame错误、时间回退、源重启、坐标跳变、ARM拒绝、mode被接管、遥测中断；确认任务锁定退出且不续飞、不空中上锁。

## 已知需本地确认的范围

- catkin 编译、ROS消息生成、实时 launch 与节点通信尚未云端执行。
- PX4 shell observer 是首版只读桥，需实机/SITL验证 topic版本、输出格式、查询时延及串口占用；不能称为已验证的飞行监控器。
- 通用模板只接受 Odometry；NOKOV 私有 SDK 和 SITL 反馈发布者由源侧提供。
- source 重启、PX4估计器reset处理保守，需要重启整个 system。
- 世界轴/外参/协方差与现有相机标定必须按实机数据验证。
- failsafe 行为由实际 PX4配置决定；本轮没有验证失效后的真实运动行为。
