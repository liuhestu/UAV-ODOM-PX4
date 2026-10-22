# 三项修正迁移记录

目标工作区：`/home/jetson/uav_odom_px4`。来源：`/home/jetson/zzx_ws/uav_odom_px4`。

## 1. EuRoC 离线回放退出顺序

`src/open_vins/ov_msckf/src/ros1_serial_msckf.cpp` 在最终可视化后显式执行
`viz.reset()`、`sys.reset()`、`ros::shutdown()`，使 ROS 可视化对象和估计器在 ROS 关闭前释放，避免依赖全局静态销毁及 image_transport/pluginlib 卸载顺序。

来源工作区历史验证：EuRoC V1_02_medium 完整 stereo 回放 clean exit，未见历史退出 SIGSEGV。迁移时不重新运行硬件，不把软件构建视为完整数据回放复验。

## 2. Adapter 高频消息队列

`src/uav_system/src/state_adapter/state_adapter.py` 的 Odometry Subscriber 和 Publisher 均从 queue_size=1 改为32，并启用 tcp_nodelay=True，容纳 TCPROS 批量到达及发送调度抖动。32条按200Hz约为160ms的消息容量，不是固定延迟；未修改时间戳、坐标/协方差数学或过期、跳变健康门。

历史输入约200Hz、初始输出约58Hz；修复后恢复约200Hz，不保证任意负载下无丢消息。Backend仍按约30Hz读取最新状态，不改变为逐条转发200Hz输入。本次迁移修正了源码中将32条误写成50ms的注释。

## 3. PX4 任务前通信恢复

新增 `scripts/prepare_px4.py`，CMake安装入口、7项 `test_px4_prepare.py` 测试及 `PX4_GROUND_PREPARATION.md` 用法。先验证目标未解锁心跳连续10秒；只有地面明确确认下才允许不稳定/无遥测时发送reboot，最多3次，每次释放串口、等15秒并重新验证。拒绝已有Mission、已解锁心跳、串口争用及重复基础链；串口I/O失败会终止。

这是显式执行的独立地面准备入口，`--start-base`成功后只启动基础链，固定不启动Mission。正常roslaunch不自动调用它；禁止在飞行中断连或旧任务续跑中调用。PX4恢复成功不代表根因已修复或任务已具备飞行条件。

## 迁移范围

仅迁移上述源码、安装入口、恢复测试与针对性说明。不迁移实验图/数据/执行日志、RealSense驱动包、测试fixture调整、机体外参或任务参数、高度打印等其他变化。原路径已有未提交任务YAML和未跟踪px4ctrl保持原样。

在原路径运行统一run_checks：134项全部通过（包括7项新增恢复测试）；单独Adapter测试也通过。来源工作区历史实机通信恢复/融合证据不等于本次迁移重新执行的硬件验证。
