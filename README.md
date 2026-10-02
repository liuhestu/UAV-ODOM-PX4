# UAV-ODOM-PX4 catkin 工作区

本仓库是完整的 ROS1 Noetic 工作区。源码保存为真实目录，不使用仓库内发现链接。

```text
uav_odom_px4/
├── scripts/build.sh
├── src/
│   ├── uav_system/        # 自有 ROS 包：构建清单、业务代码、配置、launch、测试与文档
│   │   └── third_party/px4_autopilot/  # 固件源码，不参与 catkin 构建
│   └── open_vins/         # OpenVINS 独立 ROS 包及配置
├── build/                # 本地产物，不上传
└── devel/                # 本地产物，不上传
```

## 构建与检查

使用已有 Noetic、catkin_tools 和依赖环境：

```bash
cd uav_odom_px4
./scripts/build.sh
source devel/setup.bash
python3 -m pip install -r src/uav_system/requirements.txt
bash src/uav_system/scripts/run_checks.sh
```

脚本默认限制为 2 个编译任务、1 个并行包，额外参数转交 `catkin build`，例如 `./scripts/build.sh --no-status`。已加载正确 Noetic 环境时也可以在工作区根目录直接执行 `catkin build`。不要混用 Humble 环境。

依赖检查使用 `rosdep check --from-paths src --ignore-src`。Jetson/Jammy 的 Noetic、RealSense 和 OpenCV 需沿用本机环境，具体记录见 [本地验证](src/uav_system/docs/LOCAL_VALIDATION.md)。

旧布局的构建产物已备份在本机 `.legacy_catkin/`，不上传、不作为新版构建结果使用。

## 项目文档

- [业务包说明与启动入口](src/uav_system/README.md)
- [架构与接口](src/uav_system/docs/ARCHITECTURE.md)
- [验证记录](src/uav_system/docs/LOCAL_VALIDATION.md)

先完成验证再执行任务。启动 Mission Executor 后，就绪时会自动请求模式与解锁；当前实机 VIO 和融合检查仍有未解决阻塞。
