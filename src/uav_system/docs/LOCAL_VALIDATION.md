# 本地 Codex 接续与验证

当前分支 `refactor/native-px4-ros1`。先读 `README.md`、`docs/ARCHITECTURE.md` 和本文件。
最初云端阶段只编辑并测试软件；后续已按用户授权执行部分无桨实机检查，具体操作与结果见 `docs/VALIDATION.md` 的分阶段记录。
后续先完成构建与只读/软件验证，再按用户当时授权决定是否做硬件或飞行操作。

GitHub 仓库现在就是 catkin 工作区：真实业务包位于 `src/uav_system/`，OpenVINS 位于 `src/open_vins/`。本文件的配置路径相对业务包；构建命令在工作区根目录执行。

## 当前用户运行约定（2026-10-05）

飞行由程序控制，Commander 使用 `auto_arm: true`，遥控器只保留 KILL。用户计划自行在 QGC 关闭遥控器 ARM；在实际参数核对前不能声称已经关闭。后续操作不依赖遥控器 ARM、油门或模式拨杆，不为恢复遥控器解锁而切换 POSCTL/MANUAL，也不在测试结束时自动恢复这两种模式。

用户后续明确确认当前 OpenVINS 外参与世界对齐，并授权将 calibrated/verified 设为 true；包内 openvins.yaml 与实机临时 source.yaml 已同步。此为用户确认记录，软件未独立完成精确标定；其余健康、地面、模式、实际 ARM 门保持。源 YAML 和新 Supervisor 日志在下次完整 system 重启时加载，修改文件不改变正在运行节点的状态。Supervisor 在首次检查及结果变化时逐项输出 PASS/FAIL；可用 `rostopic echo /rosout` 查看，并用 `/uav/system/status.reasons` 核对失败项。

用户最新指定保留位姿、融合/估计器、通信、起飞前地面、实际模式/ARM 与重启禁止续飞等核心门；PX4 汇总状态和 SYS_STATUS 传感器位图仅保留诊断日志，不再单独阻止 ready/arm_ready。日志的 diagnostic only FAIL 与核心条件 FAIL 区分；PX4 自身解锁检查没有关闭。定位源健康、标定确认、仿真传输限制和故障锁定保持。

每轮实机验证先停止 Commander，完整停止并重启 system.launch，重新建立定位源 session 和健康证据，然后显式启动新的 Commander。当前 start_source=true 时 source manager 拥有 OpenVINS 子 launch，system 停止会关闭该子 launch，重启会重新初始化 OpenVINS；当前测试将相机和 MAVROS 独立启动，保持这两个进程并防止重复启动。start_source=false 才是不重启外部 OpenVINS、只监测其输入。运行中的任务遭遇飞控重启/断连则锁定终止，不能把恢复连接当作新的测试。

任务控制阶段由 Commander 提供持续目标并请求/确认 OFFBOARD。健康、标定、实际模式与解锁门保留；未就绪时不通过独立 MAVROS 服务脚本绕过门控。程序只能请求 OFFBOARD 和既有降落流程的 AUTO.LAND。飞控启动初始模式、目标流中断/失效保护可能导致退出 OFFBOARD，需按真实状态报告；此约定不意味着禁用 PX4 失效保护，也不意味着终止任务后持续抢回 OFFBOARD。修改飞控开机模式/RC/failsafe 参数须按用户当轮授权执行。

旧布局的 build/devel/logs/.catkin_tools 已保留在工作区 `.legacy_catkin/时间戳/`；不要 source 旧环境。新布局仍使用 uav_system.msg，且不再包含包内配置链接或 OpenVINS 发现链接。mock 配置位于包内 `config/test/mock.yaml`，仅由 mock launch 显式加载；实机配置位于 `config/state_sources/`。

## 1. 构建门

```bash
git switch refactor/native-px4-ros1
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
./scripts/build.sh
source devel/setup.bash
python3 -m pip install -r src/uav_system/test/requirements.txt
bash src/uav_system/scripts/run_checks.sh
```

检查各本地包和 OpenVINS 的 ROS1 条件依赖。固件位于 `third_party/px4_autopilot`，位于 uav_system 包内，catkin 不递归扫描已发现的包。
Jetson/Ubuntu 22.04 的 Noetic 安装、OpenCV/cv_bridge ABI 和 Ceres 版本需沿用本机可用环境，禁止用 Humble 环境编译这些节点。
先修复 build/launch/import 差异，把结果写到 `docs/VALIDATION.md`，不把静态测试当作 ROS 构建通过。

当前 Jetson 的只读运行环境需要已有 RealSense 工作区和 `/usr/local/lib`（log4cxx 兼容库）；本项目 overlay 最后 source，避免运行旧 OpenVINS 可执行文件：

```bash
source /opt/ros/noetic/setup.bash
source /home/jetson/jin_ws/uav_ws/devel/setup.bash
source /home/jetson/uav_odom_px4/devel/setup.bash --extend
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
```

`--extend` 保留外部工作区的 RealSense 包发现路径，同时使本项目新包优先；不带此参数会恢复新工作区构建时的 Noetic underlay。上述环境是当前机器记录，不是通用 Noetic 安装方案。在线 pip 未成功时，本轮测试使用现有 `px4_exp/.venv` 中的 pymavlink，详见验证记录。

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

可复跑隔离软件检查（自建 `localhost:11329` master，端口须空闲；只启动 mock 与 Commander，结束后清理本轮进程）：

```bash
python3 src/uav_system/test/verify_commander_wait.py
```

该脚本检查当前 YAML 默认 `auto_arm=true`、显式 true/false、旧参数映射和新旧参数冲突，确认缺失真实 PX4/融合证据时始终 WAIT_SYSTEM。Backend 服务回归使用 unittest 内存替身，不连接 MAVROS。系统 Python 缺 pymavlink 时可按既有环境记录临时设置 `PYTHONPATH=/home/jetson/px4_exp/.venv/lib/python3.10/site-packages` 运行 `run_checks.sh`，这不代表系统依赖已安装。

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
本地 OpenVINS 对成功的 ZUPT 记录完成更新的时间，使静态初始化后仍静止时也能发布 odomimu；此前首次 feature update 未发生会一直阻止发布。这个修正不改变估计器计算或项目健康门。静止输出恢复不代表运动或长期稳定已验证，仍须检查相机标定、漂移和 NaN/reset。

填 source YAML 的 `T_SB`（base_link 在 sensor 中）和 `T_AW`（world→Z-up canonical），标定验证通过后才将 calibrated/verified 改为 true。
当前 OpenVINS 模板使用镜头朝前、顶部朝上的 optical IMU（右、下、前）到机体 FLU 轴映射，T_SB 的 xyzw 为 [0.5, -0.5, 0.5, 0.5]；这是本机安装方向试验值。零平移与单位世界对齐仍未验证，其他安装方向/IMU frame 不应直接沿用。静止水平姿态检查通过不能替代运动轴向、平移、相机内外参与世界对齐验证。
如果自己已有外部 RealSense driver，可设 source launch 的 `start_camera: false` 并相应移除 owned_nodes 中的相机项；不要重复占用。
如果 RealSense 和 OpenVINS 都已在外部运行，复用 `openvins.yaml`，传 `start_source:=false`：manager 只监测输入心跳，不启动或接管外部源节点；外部源退出由数据超时检测。保持 `start_mavros:=false output_enabled:=false` 做只读诊断。
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
Observer 目前只支持单 EKF，并通过参数读取验证。实机必须有五个 uORB topic 及相应字段，包含 estimator_status_flags 的真实 EV/假位置/惯性推算状态。
标准配置为 EKF2_MULTI_IMU=0、EKF2_MULTI_MAG=0（固件提供时）、SENS_IMU_MODE=1、SENS_MAG_MODE=1，重启后读回并验证实际 instance 0。当前固件没有 EKF2_MULTI_MAG，观察器以必须成功读取的 EKF2_MULTI_IMU/SENS_IMU_MODE/SENS_MAG_MODE 核对单实例与传感器选择；不把参数读取失败当作零，不放宽实际 EV 融合门。参数写入和飞控重启仅在当轮授权、Commander 已停止且鲜活遥测确认上锁/地面时执行。
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
使用 `roslaunch uav_system commander.launch auto_arm:=true` 验证自动 ARM 请求最多一次，已 ARM 时不重复请求。提前 OFFBOARD 时仍须在预发送结束后请求一次模式，并收到本次请求之后的新 State 确认，才允许 ARM/起飞。以 auto_arm:=false 验证禁止 ARM 请求及长期等待实际解锁；本机 OFFBOARD 中不能用遥控器 ARM，该选项不是当前遥控器仅 KILL 的运行入口。两种配置均须完成连续就绪、地面确认和预发送后才起飞。
检查服务响应与 FCU 实际 mode/armed 分开；非零 local 原点下地面保持目标随位姿更新，进入 TAKEOFF 当轮保持地面目标，之后按速率爬升。初始 landed 未知/空中时只等待，恢复地面后重新完成稳定窗口；准备阶段再丢失地面、故障或人工上锁则停止输出并锁定。检查 hover、AUTO.LAND、landed 与 disarmed 顺序。
区分 PX4 解锁怠速与 Commander 任务执行，三重门控只约束后者。独立直接调用 MAVROS 的电机测试不能代替 Commander 门控验收，也不应绕过标定/融合门用于该验收。注入等待就绪、准备、飞行和降落期间的断连，以及启动时钟回退而未观察到 disconnected 的快速重启；检查重连后仍终止，服务重新出现时不发送旧 ARM/OFFBOARD 请求。
注入数据停止、NaN、frame错误、时间回退、源重启、坐标跳变、ARM拒绝、mode被接管、遥测中断；确认任务锁定退出且不续飞、不空中上锁。

## 已知需本地确认的范围

- catkin 编译、ROS消息生成、实时 launch 与节点通信尚未云端执行。
- PX4 shell observer 是首版只读桥，需实机/SITL验证 topic版本、输出格式、查询时延及串口占用；不能称为已验证的飞行监控器。
- 通用模板只接受 Odometry；NOKOV 私有 SDK 和 SITL 反馈发布者由源侧提供。
- source 重启、PX4估计器reset处理保守，需要重启整个 system。
- 世界轴/外参/协方差与现有相机标定必须按实机数据验证。
- failsafe 行为由实际 PX4配置决定；本轮没有验证失效后的真实运动行为。
