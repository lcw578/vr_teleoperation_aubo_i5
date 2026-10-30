# 25 Hz 持续按住 Grip 中断：关节反馈更新空档

本次记录对应 2026-10-29 10:25:51.637 左右的中断。停止触发点已定位；关节反馈空档在控制柜、网络或 SDK 内部的具体来源仍未确定。没有修改控制代码、有效期或自动恢复规则。

证据快照：`/data/VR_teleoperation_runtime/aubo-input-recurrence-20261029-102652.json`。
会话日志：`/data/VR_teleoperation_runtime/follow25-wifi-input-20261029-102428.log`。
运行参数：六轴 Follow、25 Hz、waypoint feedback、parallel state query。

## 首次停止触发

`hardware.first_stop_trigger.reason = hardware_feedback_unavailable`，触发单调时间为 `34683.919206786`，此前状态为 active。

| 项目 | 触发时证据 | 含义 |
| --- | --- | --- |
| 最新手柄输入年龄 | 1.253 ms | 输入正在到达 |
| 关节反馈年龄 | 约 209.95 ms | 超过当前 200 ms 有效期 |
| 独立状态查询 B 最新结果年龄 | 40.678 ms | 查询线程正常，返回码 0，state=1 |
| 当前 Follow 同步调用最终耗时 | 283.270 ms | 停止触发时仍未返回，返回码最终为 0 |
| SDK 单次调用预算 | 500 ms | 本次未触发 SDK 调用超时锁定 |
| 停止确认 | 触发后约 1061.22 ms | 已确认停止，fatal=false |

触发前 500 ms 内，保存了 48 个收到的 XR 帧，全部右手柄存在且 Grip=true；28 个核心处理记录全部 accepted。最大接收间隔约 54.30 ms。因此，本次没有证据支持手柄松开、右手柄丢失或输入有效期超限是首次停止原因。

`refresh_feedback()` 捕获反馈读取异常后统一报告 `hardware_feedback_unavailable`。完整异常字符串没有保存在本次冻结快照中；反馈年龄超过 200 ms 且 `FeedbackCache.require_recent()` 在此条件下会抛出 `feedback_reception_expired`，与本次停止证据相符。

## 时间顺序

以首次停止触发为 0：

| 相对时间 | 事件 |
| --- | --- |
| -209.951 ms | Follow 发送前快照中的最近关节反馈回调时间 |
| -209.754 ms | Follow 原生同步调用开始 |
| -40.678 ms | 独立状态查询 B 返回成功，查询本身耗时 19.370 ms |
| -1.253 ms | 收到最新 XR 帧，Grip=true |
| 0 | 关节反馈年龄约 210 ms，触发 `hardware_feedback_unavailable` |
| +73.517 ms | Follow 返回成功，记录 `revoked_during_call=true` |
| +73.562 ms | 开始 slow_control_stop |
| +806.513 ms | slow_control_stop 返回成功，该调用耗时 732.951 ms |
| +1061.222 ms | 软件确认停稳 |

冻结的 `core_interruption=waiting_for_stop_confirmation` 是硬件反馈停止之后的核心门控结果，不是最初原因。用户随后放下设备造成的 `invalid_xr_frame` / 右手柄缺失，也不能倒推为本次中断原因。

## 已核对的实现边界

- `teleop/aubo/quest_follow.py:99`：核心循环刷新反馈，异常进入停止流程。
- `teleop/aubo/follow.py:220`：`measured_feedback()` 从原生推送缓存读取最新关节样本，按真实回调接收时间检查有效期。
- `teleop/aubo/feedback.py:71`：当前关节反馈有效期为 0.2 s。
- `teleop/aubo/native/follow.cpp:85`：关节回调直接写入有界 C++ 缓存，不需要 Python GIL，也不获取发送使用的 `io_` 锁。
- `teleop/aubo/native/follow.cpp:172`：缓存读取只获取推送缓存锁，不等待同步 SDK 网络 I/O。

独立状态查询 B 在 A 的 Follow 调用尚未返回时已完成一次查询，说明本次查询确实与发送并行。触发记录里 `reception_age_s.state=268.9 ms` 来自 A 先前快照；应使用同时冻结的 B 最新结果时间计算实际年龄 40.678 ms，不能把旧快照解释为 B 查询卡住。

## 尚未证明的来源与后续定位方向

283 ms 的 Follow 调用与关节反馈更新空档重叠，但仅凭重叠不能证明 Follow 阻塞了 SDK 回调。当前没有控制柜采样时间、对应网络包记录或连续保留的故障前后原生回调序列，尚不能区分：控制柜没有及时推送、以太网/TCP 到达延迟、SDK 收包或派发回调延迟。

下一轮定位应同时保留关节回调序列及时间间隔、Follow 起止时间，以及必要的主机侧网络到达时间；停止时冻结故障前后的有界记录。若网络数据按时到达而回调暂停，继续查 SDK 收包/派发；若网络包也出现空档，再分析控制柜发送与网络。当前证据不足以据此关闭反馈有效期检查或承诺增大发送频率可以解决中断。
