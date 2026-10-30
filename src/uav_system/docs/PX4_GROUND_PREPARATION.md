# PX4 任务前通信检查与地面恢复

仅用于任务开始前已确认未解锁的地面准备。它是通信恢复措施，不代表已修复 PX4 内部静默的根因。正常稳定通信不重启。

```bash
cd /home/jetson/uav_odom_px4
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
python3 src/uav_system/scripts/prepare_px4.py \
  --ground-confirmed --start-base \
  --log /home/jetson/px4_preparation/prepare.jsonl
```

需要 Python `pymavlink`、pyserial，以及系统 `fuser`（psmisc）。先退出 QGC、MAVROS 和旧任务；脚本检查进程、串口占用，并使用进程锁和 TIOCEXCL。`--start-base` 也拒绝已有基础链，`--reuse-source` 可复用已运行的相机/OpenVINS。

先检查 sysid/compid=1/1 的 PX4 未解锁心跳连续稳定10秒；每次观察15秒，心跳间隔超过2.5秒重置稳定计时。不稳定且明确给出 `--ground-confirmed` 时，发送 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(param1=1)，最多3次。每次关闭串口、等待15秒、重新独占连接再验证；发送成功或ACK不能代替恢复后的心跳证据。串口I/O异常会终止，不盲目续发。已解锁心跳、占用或已有任务均禁止恢复。

无遥测不能证明未解锁；`--ground-confirmed` 是操作者对地面条件的明确确认。禁止把脚本放进飞行中断连 watchdog。不得自动续跑旧Mission。

`--start-base` 成功后启动正式基础链，显式 `start_mission_executor=false`；默认会管理OpenVINS（含相机）、MAVROS、Observer和Backend输出，不请求ARM/模式切换。仍需核查实际EV接收/融合与ready/arm_ready后再单独执行任务。直接roslaunch不自动调用此脚本。

来源工作区曾在一次reboot后取得连续12次未解锁心跳并恢复融合；这是历史硬件证据，本次迁移不发送reboot或连接飞控。
