# 工作空间结构

`/home/he/uav_vio_px4` 是 ROS 2 工作空间根目录：

```text
src/
├── open_vins/          # OpenVINS ROS 2 packages，参与 colcon build
├── estimator_adapter/  # 项目自有 ROS 2 package，参与 colcon build
└── px4_autopilot/      # PX4 v1.17.0 源码参考，不编译、不烧录
build/                  # colcon 生成目录
install/                # colcon 生成目录
log/                    # colcon 生成目录
scripts/
docs/
```

PX4 目录中的 `COLCON_IGNORE` 是工作空间级安全边界，避免直接执行 `colcon build` 时发现 PX4 工程。PX4 源码已直接纳入本仓库，仅用于源码查阅；项目脚本也会显式限定 `src/open_vins` 和 `src/estimator_adapter`。

适配器配置仍位于 `src/estimator_adapter/config/`，由 ROS 2 包安装，不使用根目录 `config/`。

从工作空间根目录构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/open_vins src/estimator_adapter
colcon test --packages-select estimator_adapter
colcon test-result --verbose
```
