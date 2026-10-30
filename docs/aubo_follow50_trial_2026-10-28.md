# Follow 50 Hz 手感与实际调用频率试验

用户授权将正式六轴 Follow 基线临时以 50 Hz 请求运行，由用户操作并同时监测。保持 waypoint 反馈、world 映射、作者核心和 SDK profile 不写入策略。

新增显式 `--follow-hz 50`，只允许六轴 `follow`；不传选项仍为原 20 Hz。Python worker 与原生 prepare 报告选中的周期并做一致性检查。原生默认配置、J6 及透传节拍保留。选择项只改变请求调度，不设置物理关节速度或柜内伺服周期。

`hardware.send_metrics` 每秒更新最近 10 秒的完成 SDK 调用统计：

- `actual_within_engagement_hz`：同一接合段相邻调用开始时刻的间隔数除以间隔之和，释放时间不参与。
- `send_interval`、`sdk_call`：count、mean、P50、P95、P99、max（毫秒）。
- `calls`、`accepted`：窗口内已返回的调用数、成功数。
- `intervals_over_20ms/40ms/100ms`：长间隔计数；超过 20 ms 包括轻微调度偏差，不应都解释为丢了完整周期。

统计只对已经返回的调用可见。在途阻塞需看 `sdk_inflight` 和 age。不能从 SDK 返回成功推断柜内已执行目标，不能把该频率称作柜内伺服频率。

本轮入口：

```bash
bash scripts/aubo_quest_follow.sh \
  --execute-six-axis --command-mode follow --follow-hz 50 --waypoint-feedback \
  --host 192.168.1.5 --port 8899 --user aubo \
  --rotation world --duration 120 \
  --diagnostic-output /data/VR_teleoperation_runtime/aubo-follow50-20261028-190524/diagnostic.jsonl
```

为保留有限缓冲的完整数据，先做 120 秒。开始和结束由原会话流程处理，不自动重复试验或故障重启。建议至少一段 10–20 秒连续缓慢往复，另做正常离合；完整统计按接合段计算。状态页 `https://127.0.0.1:8443/aubo`；当前头显入口 `https://10.245.247.27:8443/`，网络改变时以本机地址为准。

启动脚本和日志均实际位于 `/data/VR_teleoperation_runtime/aubo-follow50-20261028-190524/`。桌面终端通过 getpass 等待用户输入 SDK 密码，密码不经聊天、不写入脚本或日志。`monitor.jsonl` 是只读 HTTP 状态每秒采样；`diagnostic.jsonl` 在退出时保存逐次 SDK 调用与反馈。监测不创建第二个 SDK 客户端。

构建与启动前完整离线检查：263 项 Python 测试、原生 fake SDK 及绑定离线检查通过，日志 `/data/VR_teleoperation_runtime/aubo-follow50-check-20261028.log`。另增加一项 50 Hz prepare/profile 一致性测试并单独通过，日志 `aubo-follow50-selected-profile-check-20261028.log`。启动进程持有独占锁后重复运行整个六轴测试时，模拟 profile 检查入口被该锁阻止；未绕过锁或对运行进程做测试注入。

本文件记录试验配置与监测口径；实际结果需待终端完成密码登录、用户操作和诊断正常落盘后补充。

## 本轮实测结果

会话在 19:08:21 左右就绪，19:10:21 按 120 秒时长退出。完整诊断保存 8054 条 worker 事件，`dropped_events=0`；1444 次 SDK 发送全部返回成功、无本地拒绝。16 个接合段中合计 1428 个段内发送间隔，时间跨度之和 64.797666 秒，合并实际提交频率 **22.037831 Hz**。该分母排除了释放/空闲间隔，不是用 1444 除以 120 秒。

| 指标 | 均值 | 中位数 | P95 | P99 | 最大 |
| --- | ---: | ---: | ---: | ---: | ---: |
| SDK 开始发送间隔 | 45.377 ms | 45.602 ms | 99.988 ms | 119.024 ms | 169.393 ms |
| Follow SDK 调用 | 33.855 ms | 41.599 ms | 57.933 ms | 65.627 ms | 118.012 ms |
| active 状态查询（460 次） | 21.629 ms | 5.509 ms | 56.982 ms | — | 86.234 ms |
| candidate → SDK 开始 | 2.451 ms | 2.323 ms | 4.401 ms | — | 80.152 ms |
| callback 接收间隔 | 30.362 ms | 30.379 ms | 50.580 ms | 60.611 ms | 165.183 ms |

546 / 1428 个间隔超过 50 ms（38.24%）；71 个超过 100 ms（4.97%）。有 497 个间隔不超过 25 ms，说明局部可以接近 50 Hz，但其余长间隔使总体仅约 22 Hz。最长的连续接合段约 10.43 秒，249 次调用，实际约 23.79 Hz。各段实际频率约 17.42–24.41 Hz，短段受边界影响更大，不能用最快短段宣称稳定上限。

按段内相邻 SDK 开始时刻划分，64.798 秒总间隔时间中：

- 前一次 Follow 调用占 48.122 秒，约 **74.26%**。
- 期间 active 状态查询与这些间隔重叠 9.949 秒，约 **15.35%**。
- 其余约 6.727 秒，约 **10.38%**，包含 20 ms 调度节拍的等待、Python/native 包装和线程调度等，不能全部归因于操作系统。

包含 active 查询的 460 个发送间隔平均 **68.932 ms**；不含 active 查询的 968 个间隔平均 **34.183 ms**。这给出了当前串行查询对发命令节拍的直接证据，同时表明仅去除查询占用也不能据此保证 50 Hz。

控制循环最后一条周期报告约 200 Hz，`stalls=0`，12 个过期时点跳过。最大 `runtime_step_s` 1.279 ms，最大反馈阶段 36.288 ms，最大循环工作 39.614 ms；没有 100 ms 真实 stall。当前数据的主要耗时集中在 SDK 请求/响应，而非作者 IK 工作段。

停止 16 次：15 次为 `grip_released`，最后 1 次 `websocket_disconnected` 发生在定时结束的 Relay/receiver 收尾阶段（界面关闭日志 19:10:20.998，本机 WebSocket 关闭 19:10:21.079），不是中途丢流。最终 `fatal=false`、`stop_result_unknown=false`、`stop_confirmed=true`，SDK 线程正常退出并 logout。停止请求至慢停开始平均 32.479 ms、最大 55.862 ms；慢停调用平均 853.313 ms、最大 1314.799 ms；请求至软件反馈确认平均 1120.216 ms、最大 1597.874 ms。物理停止时刻仍需现场观测配合。

结论：**请求 50 Hz 后，这台设备通过当前串行 Follow 链路的本轮实际提交约 22.04 Hz，且节拍明显不均匀；这不是已经找到了控制柜硬件的 22 Hz 极限。** 需要优先定位 Follow 响应的 40–60 ms 聚类、同步查询占用，再比较手感与目标/实测反馈响应。本轮不继续自动提频、切透传或重开运动会话。当前正式默认仍为 20 Hz，50 Hz 只在显式选项下启用。

完整数字：`/data/VR_teleoperation_runtime/aubo-follow50-20261028-190524/summary.json`；逐次记录 `diagnostic.jsonl`；独立只读采样 `monitor.jsonl`；终端输出 `stdout.log`。用户手感评价尚待反馈，不能由 SDK 返回码替代。
