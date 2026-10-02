# 测试目录用途

这里定义测试和模拟数据，`../scripts/run_checks.sh` 负责执行，不包含另一套测试实现。

| 文件 | 用途 |
|---|---|
| `test_state_adapter.py` | 外参、参考点、杠杆臂速度、世界/机体系转换及协方差的数学正确性 |
| `test_execution.py` | Mission Executor任务流程、单次请求、拒绝/超时、人工接管、故障退出与空中禁上锁 |
| `test_supervisor.py` | 所有就绪条件必须满足；mock硬件限制与未标定解锁限制 |
| `test_watchdog.py` | 数据心跳过期、旧测量时间、ROS时钟暂停/回退时的wall-time保护 |
| `test_evidence.py` | PX4融合证据解析、旧数据/拒绝/缺字段处理及MAVLink字节/CRC对照 |
| `test_structure.py` | YAML、manifest、Python语法、launch文件引用、单一自有ROS包、节点导出、真实目录布局和catkin包发现 |
| `support.py` | 测试公共路径设置，无测试用例 |
| `mock_state_source.py` | 软件测试运行时数据源，不是单元测试；通过mock launch启动 |
| `../requirements.txt` | 节点运行与软件检查共用的 Python 依赖 |

测试覆盖不同的故障点；执行结果与数量记录在 docs/LOCAL_VALIDATION.md。

```bash
cd src/uav_system
python3 -m pip install -r requirements.txt
bash scripts/run_checks.sh
```

直接运行 `python3 -m unittest discover -s test -v` 只执行单元/静态检查；统一入口还运行 `git diff --check`。
GitHub CI调用同一个统一入口。没有在这里运行硬件、ARM、飞行或SITL任务；mock数据源也不会自动连接飞控。


Mission Executor 提前 ARM 回归覆盖两种 `auto_arm`、各准备阶段的遥控器 ARM、提前 OFFBOARD、地面恢复/丢失、连续就绪/session、地面目标更新及起飞基准、手动无限等待、一次请求/拒绝/实际状态超时、人工接管/上锁和终止锁定。Backend 与 Mission Executor runtime 测试使用内存 ROS 替身，包含准备阶段跳变停止、飞行中跳变降落和服务 watchdog；不连接实际 FCU。

Noetic 环境下可运行 `python3 src/uav_system/test/verify_executor_wait.py`，在独立 master 11329 验证 launch 参数和 mock 等待。此为显式运行的 ROS 检查，不由 unittest 自动启动。

`test_mission_tasks.py` 检查任务选择、公共/任务配置分离、插件契约、单实例与故障处置；`test_executor_runtime.py` 检查 ROS 服务替身和运行器 watchdog；`test_fcu_guard.py` 检查断连及启动时钟回退锁定。领域 Python 包通过 test/support.py 加入源码路径，生产运行依赖 catkin 导出。

模块合并后，运行器测试直接加载 mission_executor_node.py，FCU guard 测试导入 execution.py，健康/融合测试导入 checks.py；观察器 CRC 对照测试以惰性的 ROS 替身导入真实 px4_ev_observer.py，不执行 main() 或发送命令。
