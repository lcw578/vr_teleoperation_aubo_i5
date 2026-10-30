# AUBO 静止停止接口测试

日期：2026-10-19。用户已确认在机器人旁、周围清空，可以操作停止/急停。本轮只读检查后执行两次独立的静止停止诊断，没有发送运动目标、上电、切换模式、恢复运动或操作夹爪。

后续用户另行授权直接真机微动，已取得[单次 J6 运动及慢停证据](aubo_j6_micro_trial_2026-10-19.md)。下文保留静止测试当时的范围，不能将其“尚未运动”表述当成最新状态。

## 实测结果

设备 `192.168.1.5:8899`，服务端 `V4.5.111.71f2ee1-Alpha`，PC SDK 为此前已核对 SHA256 的 2.5.0 库。

| 测试 | 接口返回码 | 本次 stop 调用耗时 | 调用前后状态 |
| --- | --- | --- | --- |
| `robotMoveStop()` | **10001** | 约 0.0031 ms | 均为 RobotStopped=0，读取到的六关节角差为零 |
| `rootServiceRobotMoveControl(RobotMoveStop)` | **0** | 约 73.09 ms | 均为 RobotStopped=0，读取到的六关节角差为零 |

普通 stop 的 SDK 输出为 `Motion-Control command Undefined`，与此前静态分派分析一致。调用耗时很短，与客户端失败路径相符；不能解释为控制柜完成了停止。

慢停输出包含 `Motion-Control service finish. ret = 0`，以及事件 type 1300、描述 `robot controller enter stop state.`。这里记录的是终端观察到的 SDK 输出，未实现事件回调采集。JSON 中保存的是 API 返回、时间和前后状态/关节查询，不包含原始异步事件流。

两次均正常退出登录，测试后检查没有留下到控制柜的 TCP 连接。本次没有自动切换到 fast stop，也没有在失败后重试或继续发送运动目标。

**结果只验证静止时的接口返回路径。** 尚未测试运动中停止、停止距离/耗时、FollowMode 下缓存撤销、并发停止和网络中断。73 ms 是这一次静止调用的主机耗时，不是物理停止时间或最坏延迟保证。前后两次关节查询相同，也不证明整个期间具有连续物理静止测量。

原始报告实际位于 `/data/VR_teleoperation_runtime`：

- `aubo-pre-stop-readonly-20261019-164102.json`：本轮前置状态核对；
- `aubo-stationary-public-stop-20261019-164417.json`：普通 stop；
- `aubo-stationary-slow-control-20261019-164430.json`：候选慢停。

## 诊断入口与边界

增加独立 `_aubo_stop_probe` 扩展，源文件为 `teleop/aubo/native/stop_probe.cpp`。原 `_aubo_readonly` 的导出接口和只读定位保持不变。

诊断扩展只提供登录、退出、状态、关节查询和两种指定的 stop 方法；不导出 follow、joint move、startup、pause、continue、模式或参数写入。构造客户端仍不构造 SDK，只有显式登录才构造 SDK。每个客户端最多尝试一次 stop，且在调用前再次查询 RobotStopped；查询失败或未报告停止就拒绝发送。此串行诊断锁不是未来运动/停止并发方案。

Python 入口默认只打印离线计划；设备操作必须显式提供连接参数、stop 方法、操作者就绪标记和 `/data` 下新的报告路径。无自动重连/重试，也不自动尝试另一个停止接口。

调用前先保存“结果未知”标记，返回后保存返回值；随后查询状态和关节，退出并写入结果。运行异常保留未知标记，不将无返回解释成未发送。报告为本次有限诊断，不会开启持续 Logger。构建产物及清单均位于 `/data/VR_teleoperation_runtime/build-stop-probe` 及其父目录。

```bash
cd /home/lcw/VR_teleoperation

# 默认只打印计划，不加载 SDK、不连接
bash scripts/aubo_stationary_stop_probe.sh

# 编译单独的停止诊断扩展；不连接
bash scripts/build_aubo_stop_probe.sh

# 离线测试
bash scripts/aubo_offline_check.sh
/home/lcw/tomato_robot/.venv/bin/python tests/smoke_aubo_stop_probe.py
```

需要现场重新诊断时，采用以下模板并提供一个尚不存在的报告名；密码交互输入，不写入命令行：

```bash
bash scripts/aubo_stationary_stop_probe.sh --connect \
  --host 192.168.1.5 --port 8899 --user aubo \
  --method slow_control --operator-ready \
  --output /data/VR_teleoperation_runtime/<新的报告名>.json
```

## 离线验证与下一步

新增 7 项假 SDK 测试，覆盖：未报告停止时拒绝、无效关节反馈拒绝、仅一次 stop、保存调用意图、异常结果未知、不吞返回错误、默认无连接及显式现场条件。单独的原生边界检查验证未登录调用被拒绝、没有运动/恢复接口。既有只读符号隔离与 allowlist 检查仍通过。

`bash scripts/aubo_offline_check.sh` 完整通过：37 项离线接口、17 项模型/协调、5 项运行、7 项静止停止诊断、27 项仿真，共 **93 项** Python 测试，以及原生只读边界检查和两个入口的默认离线计划。原生停止诊断边界另行通过。回归日志为 `/data/VR_teleoperation_runtime/aubo-offline-with-stop-20261019.log`；shell 语法与 `git diff --check` 通过。

运动测试前仍需明确当前柜子的控制权/模式、MODEJ/TRACKING 停止和并发的适用说明；准备有依据的单轴幅度、速度/加速度、更新周期和停止判据。本轮没有借静止 stop 成功自动解除这些条件，也没有实现可向机械臂发送关节目标的入口。
