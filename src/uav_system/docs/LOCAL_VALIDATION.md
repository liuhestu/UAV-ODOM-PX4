# 本地环境与当前验证

架构、启动方式、任务/状态源扩展和终端状态见 [ARCHITECTURE.md](ARCHITECTURE.md)。本文件只保留本机环境、验证方法与当前结果，不累积旧版本流水。

## Jetson 环境

SSHFS 工作区直接编辑源码；构建和 ROS 检查在 Jetson 的 `/home/jetson/uav_odom_px4` 执行。使用 ROS1 Noetic，不混入 Humble，也不加载 `.legacy_catkin` 的旧构建环境。

```bash
ssh jetson-uav-odom 'cd /home/jetson/uav_odom_px4 && ./scripts/build.sh'
```

本机运行环境需要外部 RealSense 工作区和 `/usr/local/lib` 中的兼容库；项目 overlay 最后加载：

```bash
source /opt/ros/noetic/setup.bash
source /home/jetson/jin_ws/uav_ws/devel/setup.bash
source /home/jetson/uav_odom_px4/devel/setup.bash --extend
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
```

`--extend` 用于保留外部 RealSense 包路径。这是本机环境记录，不是其他机器的通用路径。依赖见 `requirements.txt`；本机曾使用 `px4_exp/.venv` 中的 pymavlink，若系统 Python 缺少该依赖，需明确所用解释器或 PYTHONPATH。

## 软件验证

在 Jetson 工作区根目录执行：

```bash
bash src/uav_system/scripts/run_checks.sh
./scripts/build.sh
source devel/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
python3 src/uav_system/test/verify_executor_wait.py
```

检查脚本包含数学、健康策略、协议、任务与执行状态机、服务替身、watchdog 和布局检查。隔离 ROS 脚本使用独立 master 11329，端口必须没有活动 listener；仅启动 mock 与 Executor，不启动 MAVROS，也不连接实际 FCU。预期 ready/arm_ready=false、两任务 WAIT_SYSTEM、重复启动被拒绝；结束后清理本轮子进程。

安装布局验证使用临时 DESTDIR 和隔离 Python 路径，确认模块来自安装目录、任务能加载、旧模块不被复制。源码只有 uav_core 保留 __init__.py；catkin 生成的开发空间入口不属于源码，不能据此判定源码目录整理失败。当前 setuptools 对命名空间包可能打印入口文件不存在的提示，需以实际导入结果判断。

## 硬件验证边界

启动 Executor 会在条件满足后自动请求 OFFBOARD 和 ARM。硬件/飞行动作、PX4 参数写入和固件刷写需要本次任务明确授权；只做软件检查时不要启动实机 Executor。

- 防止重复启动相机、OpenVINS 或 MAVROS。已有外部进程时按架构文档使用 start_source/start_mavros，或调整源 YAML 的相机启动及 owned_nodes。
- 当前 OpenVINS 的 calibrated/verified=true 来自用户对本机外参与世界对齐的确认，软件没有独立完成精确标定。新状态源或新安装必须重新验证；不能直接复制这些确认标志。相机图像尺寸、raw/rectified 与内外参匹配在 OpenVINS 源侧核对。
- 本地 OpenVINS 曾修正静态 ZUPT 完成后的里程计发布时序；静止时有输出不证明运动轴向、漂移或标定准确。
- PX4 Observer 只支持单 EKF，读取验证 EKF2_MULTI_IMU=0、SENS_IMU_MODE=1、SENS_MAG_MODE=1；不把缺失参数当零，也不自动写参数。运行 Observer 时避免同时使用 QGC MAVLink Console。
- 用户约定程序控制任务，遥控器只保留 KILL。是否已经在 QGC 关闭遥控器 ARM，需要实际读取确认；程序不为恢复遥控器 ARM 而请求 POSCTL/MANUAL。
- 任务运行中断连、飞控重启、源 session/估计器 reset 或人工接管后不续飞。故障定位并结束旧任务后，重新建立基础系统与新任务；start_source=true 管理的 OpenVINS 会随基础系统停止/启动，外部源不会。
- 后续 SITL 必须提供随模拟机体运动的实际反馈，并分别验证任务控制、降落判定和失效保护；固定 mock 不作为飞行反馈。

## 当前验证结论（2026-10-06）

| 范围 | 结果与边界 |
|---|---|
| 单元与结构检查 | 94 项通过，含实际模式/ARM 顺序、准备与飞行故障、断连/重启锁定、接管、正常降落和空中禁上锁 |
| catkin 构建 | 6 个包通过；保留 catkin distutils 弃用警告 |
| 隔离 ROS | 两任务 WAIT_SYSTEM、配置覆盖/冲突、健康诊断和重复启动拒绝通过；没有 MAVROS/FCU |
| 安装空间 | 合并后的七个领域文件导入、任务加载、FCU guard 和观察器封包通过，无被合并的旧模块 |
| SITL / 实际悬停或轨迹飞行 | 本轮未执行，软件验证不证明实际运动或落地判定成功 |

本轮证据位于 Jetson：`/tmp/uav_merge_modules_checks.log`、`/tmp/uav_merge_modules_build.log`、`/tmp/uav_merge_modules_ros.log`；隔离 ROS 目录 `/tmp/uav_executor_ros_s6hpootw`，安装空间 `/tmp/uav_merge_modules_install_PT5hNt`。这些是临时证据路径，不能保证长期存在。

后续验证更新本节的日期、当前代码验证结果及未覆盖范围，不恢复按版本累积的历史流水。旧 `VALIDATION.md` 已删除。

TaskUpdate 已合入 `mission_executor/execution.py`，任务与测试引用同步更新。Jetson 上 94 项单元/结构检查通过，`./scripts/build.sh uav_system` 及其依赖共 4 个包增量构建通过，工作区 `git diff --check` 通过。检查脚本最后的暂存区检查仍报告旧 `task_interface.py` 末尾空行；该文件已从工作区删除，暂存区未改动。本次未启动实机节点。日志：`/tmp/uav_task_interface_checks.log`、`/tmp/uav_task_interface_build.log`。
