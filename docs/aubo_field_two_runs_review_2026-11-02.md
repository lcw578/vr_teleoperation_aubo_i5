# 车载两段现场录制复核（2026-11-02）

两段录制和故障现场均已保存。**本轮复现了左 Y 跟踪保护，以及一次停止 SDK 超时；没有形成有效的 QUICKACK B 组。** QUICKACK 显式开启但 SDK 目标接收命中为零，不能用本轮结果判断该网络干预是否有效。

用户已确认机械臂静止并保持待机。复核仅通过 SSH 读文件和本地离线分析进行，没有新建 SDK 连接或发送运动/停止命令。

## 1. 会话与数据完整性

| 项目 | 第一段 | 第二段 |
| --- | --- | --- |
| 现场目录 | `home-field-20261102-142813-h6u3YJ` | `home-field-20261102-143821-PcvRsv` |
| 数采目录 | `20261102-142813-f9f62b5e` | `20261102-143821-ec9dd0bf` |
| 入口退出码 | 1 | 0 |
| PCAP 包数 | 26434 | 19567 |
| tcpdump 内核丢包 | 0 | 0 |
| 诊断 event_sequence | 1–24900 连续 | 1–19302 连续 |
| 数采 written_records | 22492 | 15566 |
| 数采 dropped_records | 0 | 0 |
| 数采 session_status | closed / complete | closed / complete |

数采文件正常闭合只表示文件写入完成，不表示机器人控制验收成功。第一段控制最终状态为 `fatal=true, stop_result_unknown=true, stop_confirmed=false`；第二段为 `fatal=false, stop_result_unknown=false, stop_confirmed=true`。

`session_end.dropped_events` 分别为 20804 / 15206。源码 `FollowWorker.diagnostic_report()` 中该值是总事件数减去当前内存环形历史长度（4096），不是磁盘诊断流丢失。本轮另行核对实际 event_sequence，二者均无缺口。PCAP 配对 Follow 数也分别与诊断记录的 2301 / 1704 次调用一致。

两轮各用自己的单调时钟和 `clock_sync`；没有把跨进程/主机运行期的单调时间直接拼接。运动轨迹和操作不相同，不据此推导 A/B 因果改善或总体故障率。

## 2. 左 Y 中断：已捕获触发判据

| 左 Y 生命周期 | 第一段 | 第二段 |
| --- | --- | --- |
| 有实际运动的请求 | 6 | 5 |
| 完成且确认停稳 | 1 | 2 |
| `home_tracking_error` | 3 | 3 |
| 主动取消 `home_abort_pressed` | 1 | 0 |
| 接近原位进入停止后失败 | 1 | 0 |

第一段三次跟踪保护的最大误差为 8.178°、8.210°、8.048°，最差轴为 J1、J1、J5；触发判据所用关节反馈年龄约 11.43ms、11.38ms、9.36ms。

第二段三次为 8.662°、8.680°、8.426°，最差轴均为 J1；反馈年龄约 7.64ms、10.86ms、2.75ms。

这些是决策时实际使用的反馈和最后一次 SDK 接受目标之间的差值，不是“离最终原位还有多少度”，也不是发送前的旧快照。它们证明本次中断是 8° 跟踪保护触发。反馈仍新，不支持把这六次事件解释为头显暂停、左 Y 松开或单纯使用了过期反馈。

这还不能确定误差增长的最终根因。需要对齐归位目标斜坡、真实提交间隔、SDK Follow 接口对各目标的规划/跟随语义、实际关节反馈。短行程重试后可以完成也不能证明可以去掉保护。

## 3. 停止：一次 10007，线缆上存在停止相关响应

第一段记录 17 次停止调用，16 次确认停稳；最后一次因到位触发停止，在 SDK 中等待 5000.221ms 后返回 10007，最终进入 `stop_unknown`。归位生命周期为 `waiting_for_stop → ready → moving → stopping → failed`。

用最近的 `clock_sync`（距离开始约 5.74ms）将 SDK 单调时钟映射到 PCAP 后，该次命令 89：

- 上位机完整请求在 SDK 开始后约 0.129ms 上线，20 字节。
- SDK 开始后约 144.760ms，控制柜方向有两帧命令 89 报文，各 36 字节，同一捕获到达时刻。
- SDK 等待期间仍有柜端关节推送和其他报文。
- SDK 到约 5 秒返回超时；随后没有完成软件停稳确认。

因此不能把 10007 描述成“控制柜整整 5 秒未发出任何停止响应”。目前只解码帧命令和时序，没有解释响应载荷的成功/失败含义，也没有观察 SDK 内部事件消费。两帧命令 89 到达不等于物理停止成功。其他成功停止中同样存在两帧同刻到达的情况，不能只凭合包认定 SDK 粘包错误。

下一步重点是核对命令 89 响应的语义、SDK 请求响应等待与完成事件等待、事件处理先后。保留当前超时后的故障锁定；不自动重试，不将超时视为成功。

第二段的 17 次停止均确认成功；这只是该段观测结果。

## 4. 延迟：两段均为无效干预，应保留为现场故障证据

| 指标 | 第一段 | 第二段 |
| --- | --- | --- |
| SDK Follow 调用数 | 2301 | 1704 |
| 同次接合内聚合提交频率 | 18.530Hz | 16.787Hz |
| SDK Follow P95 | 57.830ms | 61.707ms |
| 活动时串行状态查询 P95 | 58.373ms | 64.017ms |
| 40–60ms Follow 线缆往返数 | 1161 | 986 |
| 其中满足延迟 ACK 后 <1ms 应答特征 | 1160（99.91%） | 982（99.59%） |
| 关联 ACK 等待中位数 | 40.685ms | 40.739ms |
| ACK 到应答中位数 | 0.336ms | 0.323ms |
| QUICKACK matched_receives | 0 | 0 |
| QUICKACK setsockopt_success | 0 | 0 |

`enabled=true` 只表示开关被解析；两段均无 `first_target_receive`，退出汇总中所有接收路径命中为零，不能证明 SDK socket 上设置过 QUICKACK。

SDK 的 `readonly.load_native()` 以 `RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND` 加载厂家库。该隔离用于避免厂家 `re_match` 与 libc 不兼容同名符号冲突，不能为拦截接收而直接删除。

新增纯回环实验用一个依赖 libc `recv` 的小型 C 动态库，普通加载时接收命中 1、设置成功 1；同库用 DEEPBIND 加载时数据正确收到，但接收命中和设置成功均为 0。这复现了普通预加载拦截在 SDK 加载方式下的限制；回环实验没有加载 SDK 或联系机械臂。

本次仍然强烈支持 ACK 与应答发送门控相关，未直接观测柜内 Nagle 选项。干预没有实施成功，不能据此确认或否定 QUICKACK 的实际改善，也不能声称已经消除 40ms 延迟或获得 200Hz。

## 5. 本轮修正与验证

已在 `aubo_home_field.sh` 增加入口拒绝：`--check --quickack` / `--execute --quickack` 均在任何 sudo、USB 配置、SDK 登录前返回 2，并提示 SDK DEEPBIND 绕过预加载。默认无 QUICKACK 的路径保留；预览仍可用。旧实验代码保留用于离线研究。

新增入口回归检查及 DEEPBIND 回环复现实验。两个测试模块合计 19 项，本地与车载均已通过；两端 `bash -n` 通过。车载首次按包名运行 unittest 因 tests 不可导入而失败，改用 discovery 后全部通过，未修改业务代码规避测试。未改运动参数、跟踪门限、SDK 隔离策略、停止路径或全局网络配置。

## 6. 下一步执行顺序

1. 离线分析本次六次跟踪保护的目标/反馈时间线，区分轨迹推进过快、SDK Follow 语义及网络/状态查询引入的影响；保留现有 8° 保护。
2. 结合本次命令 89 报文核查 SDK 停止等待路径，解决“到位后请求超时而无法确认”的独立问题。未经证据不改为自动恢复。
3. 设计兼容 DEEPBIND 的可关闭网络干预。先做同加载方式的纯回环验证，再做现场不运动的目标 socket 生效验证；确认设置成功后才安排小范围 B，随后 A2 回退对照。
4. 左 Y 的修正与停止处置完成后，再做分级现场复验。左 X 继续等待实际示教点、部署路径、工具/负载与夹持判据，不因本轮录制成功就开始带物自动放置。

当前无需用户继续运动或重复录制。原 B/A2 操作命令已暂停，不能沿用之前文档里的待执行步骤直接启动。

## 7. 原始证据与复算

车载原始文件均在 `/home/xc/code/lcw/VR_teleoperation_runtime/` 内，未删除或覆盖。笔记本副本在 `/data/VR_teleoperation_runtime/home-audit-20261102/field/`，包括上述两份 field 目录及两份 recordings 目录。

各 field 目录包含 `analysis-follow.json`、`analysis-home.json`、`analysis-ack.json`、`analysis-stop-wire.json`。Follow/Home 使用仓库离线分析器；ACK 使用 `/data/VR_teleoperation_runtime/aubo_ack_reaudit_20261102.py`，停止使用 `/data/VR_teleoperation_runtime/home-audit-20261102/analyze_stop_wire.py`。两者只适用于本次记录条件，不是任意会话的通用协议解码器。

PCAP SHA256：

- 第一段：`9cecda8fbf8eb1041c4a299371c38e36437aa3d8f30b0a14c745cd5788f72cbb`
- 第二段：`bd2800fd4be6baa5a4870f65f475781e2391907de743ce700eceb18b160e1ee8`
