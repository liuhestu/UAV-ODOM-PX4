# 测试目录用途

这里定义测试和模拟数据，`../scripts/run_checks.sh` 负责执行，不包含另一套测试实现。

| 文件 | 用途 |
|---|---|
| `test_state_adapter.py` | 外参、参考点、杠杆臂速度、世界/机体系转换及协方差的数学正确性 |
| `test_mission.py` | Commander任务流程、单次请求、拒绝/超时、人工接管、故障退出与空中禁上锁 |
| `test_supervisor.py` | 所有就绪条件必须满足；mock硬件限制与未标定解锁限制 |
| `test_watchdog.py` | 数据心跳过期、旧测量时间、ROS时钟暂停/回退时的wall-time保护 |
| `test_evidence.py` | PX4融合证据解析、旧数据/拒绝/缺字段处理及MAVLink字节/CRC对照 |
| `test_structure.py` | YAML、manifest、Python语法、launch文件引用、目录链接和catkin包发现 |
| `support.py` | 测试公共路径设置，无测试用例 |
| `mock_state_source.py` | 软件测试运行时数据源，不是单元测试；通过mock launch启动 |
| `requirements.txt` | 软件检查依赖 |

以上六个测试模块目前共39个用例。它们保护不同的故障点，全部保留。

```bash
python3 -m pip install -r test/requirements.txt
bash scripts/run_checks.sh
```

直接运行 `python3 -m unittest discover -s test -v` 只执行单元/静态检查；统一入口还运行 `git diff --check`。
GitHub CI调用同一个统一入口。没有在这里运行硬件、ARM、飞行或SITL任务；mock数据源也不会自动连接飞控。
