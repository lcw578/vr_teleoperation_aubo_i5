# A/B 与 A/B/C 隔离部署及保持验收

2026-11-08，Asia/Shanghai。本轮按用户授权先部署通信隔离并做有限当前姿态保持，再以验收通过作为运动连续性改造的前置条件。沿用已经确认的现场监督与实体急停条件；没有接自由 VR、没有生成增量轨迹、没有调整保护阈值或网络参数。

## 执行结果

**两连接与三连接隔离均已实现、部署并进行一次保持验收，但均未通过。** 两轮都在约 0.18 秒活动观察窗口中触发 `state_reception_expired`，各完成 4 次发送；随后退出透传、慢停、新鲜状态及稳定关节反馈确认均成功，A/B/C 正常注销，无残留 SDK 连接或诊断进程。

此前实验确认的“发送连接自身路点推送足以导致 P50 约 52ms”仍成立，但**它不是全部长阻塞的完整解释**。本轮在 A 的三种可选遥测推送全部关闭、B 查询和 C 推送分别位于独立 SDK 进程的情况下，仍记录到 A/B 同窗口超过 300ms 的调用。

两轮各只有四次发送，不能据此前两个约 5–6ms 调用宣称 A 已稳定恢复约 6ms，也不报告具有稳定性能含义的 P50/P95 或控制频率。

**第一步的验收前提未满足，因此第二步没有执行：原有 Python EMA 和 `passthrough.hpp` 运动策略保持原样；未进行小幅轨迹验收或自由 VR 遥操。**

## 部署内容与当前版本

车载项目 `/home/xc/code/lcw/VR_teleoperation`；运行目录 `/home/xc/code/lcw/VR_teleoperation_runtime`；当前 SSH `xc@10.89.248.164`。控制柜 `V4.5.111.71f2ee1-Alpha`，SDK 仍为固定的 Linux x64 2.5.0；共享库哈希与前轮一致。

已部署：

- A 在挂接独立反馈缓存时显式关闭 RoadPoint、JointStatus、EndSpeed 三类可选推送，并检查各次关闭调用的返回码与预算。A 的连续发送路径只读本地反馈缓存，不进行额外状态/关节主动查询。启动检查及退出/停止调用仍存在，SDK 自动事件和命令应答不属于这三类可选遥测推送。
- B 合并方案：独立进程拥有一个 SDK，接收路点推送并查询整机状态。状态 RPC 与关节遥测使用不同 IPC 通道；B 查询阻塞不应在应用层阻塞关节遥测转发。回调原接收时间、序号、会话代次一直传到 A，不以 IPC 读取时刻刷新。
- 三连接方案：`SplitProcessStateObserver` 将 B 纯状态查询与 C 纯路点订阅分给两个独立 SDK 进程。C 不启动状态查询调度器。A 的关节缓存明确标为 C，状态缓存来自 B。
- `quest_follow.py` 增加显式 `--separate-waypoint-connection` 选择三连接方案；它要求启用独立状态查询。默认选项没有自动切为三连接，也没有自动重连或在失败后自动改架构继续运动。
- 反馈接收新鲜度仍为 **200ms**，单次 SDK 调用预算仍为 **500ms**。原有接合前新查询、撤销许可、退出透传、慢停与停稳确认保留。

当前车载 `follow-build.json` 指向三连接能力候选绑定：

```text
/home/xc/code/lcw/VR_teleoperation_runtime/aubo-channel-isolation-20261108/abc-candidate/build-follow/python/_aubo_follow.cpython-310-x86_64-linux-gnu.so
SHA256: 8637c53ffb833d31ae88ae8b0f05d0fe4ef6cc2491e5897cf43a69ef97655ca5
```

该二进制支持两种方案，**构建成功与能力存在不代表真机验收通过**。当前清单副本：vehicle-follow-build.json（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/vehicle-follow-build.json`）。

最终车载源码哈希已与本地核对：

| 文件 | SHA256 |
| --- | --- |
| `native/follow.cpp` | `9d4c766eb43d5a5d4a8a3042ec72aed70fdaabcea390933a28d79dea82e71d06` |
| `process_state.py` | `c551adcc68d8159e9a8afb3b352f841de2845610825ba66021b43c516cb19b05` |
| `follow.py` | `f6901dc72898dde6408698dffef2fd54f0571ea555561b95f6c47cf8ee6f1415` |
| `quest_follow.py` | `8be64e7945d4defd1570020f7580c267e2202be7bb7446abbd95d634660d9b3d` |
| `native/passthrough.hpp`，本轮未改 | `5c720af66aed8222ad5736b4cd9d21979f3b8ae77a5870736ff86a0090cca94c` |

部署前源码备份：pre-isolation-sources.tar.gz（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/pre-isolation-sources.tar.gz`）；两连接部署后、三连接改动前的备份：pre-ABC-sources.tar.gz（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/pre-ABC-sources.tar.gz`）。原生产二进制未覆盖；初始 manifest 备份位于车载隔离目录 `follow-build-before.json`。

## 20:38 两连接保持

本轮调用生产 `J6FollowWorker`、原生发送绑定与独立 B 进程，目标固定为 prepare 实测姿态，首帧沿用生产实测保持。原计划保持 3 秒，首个保护触发后终止，没有同条件自动重试。

| 项目 | 实测 |
| --- | --- |
| A 四次发送耗时 | **8.248 / 6.052 / 17.388 / 302.963ms**，均 rc=0 |
| 运动窗口开始的 B 查询 | **292.038ms**，rc=0 |
| 首个停止原因 | `state_reception_expired` |
| 触发时状态接收年龄 | **206.015ms** |
| 触发时关节推送接收年龄 | 159.963ms |
| 退出透传 | rc=0，9.943ms |
| 慢停 ACK | rc=0，176.220ms |
| 停稳确认 | `stop_confirmed=true`，`stop_result_unknown=false` |
| 活动观察期间最大实测姿态偏差 | 约 0.000205° |
| 网卡错误/碰撞增量 | 已检查字段全部 0 |

触发保护时，B 在途查询已运行约 161ms，而上一次状态接收到本次查询开始已有约 45ms。因此 **200ms 接收新鲜度可以在单次查询耗时还没达到 200ms 时先失效**。该查询后来成功返回 292ms，也不能反过来补足此前已经缺失的状态证据。

原始报告：ab-hold1.json（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/ab-hold1.json`）。

## 无运动发送的只读分离对照

两连接保持失败并确认停稳后，分别运行三组各约 10 秒的只读对照。此处**没有 A 透传发送负载**，因此只能评估 B/C 的基础观测开销，不能代替运动期验收。查询目标为开始到开始 100ms、单一在途、不追赶；实际发起频率均约 9.99Hz。

| 配置 | 查询次数 | 查询 P50 | 查询 P95 | 查询最大值 | 采样到的最大状态接收年龄 | 最大关节接收年龄 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| B 同连接推送 + 查询 | 100 | 41.313ms | 56.366ms | 58.689ms | 151.727ms | 63.164ms |
| B 仅查询 | 100 | 6.607ms | 16.314ms | 20.894ms | 105.704ms | 无推送 |
| B 查询，C 独立推送 | 101 | 7.023ms | 20.104ms | 27.998ms | 112.358ms | 38.299ms |

三组查询均成功，监测样本没有状态年龄超过 200ms，网卡错误/碰撞无新增。年龄由约 20ms 主机采样记录计算，是采样到的最大值，不冒充连续时间精确峰值。

结果支持“B 自身携带推送会使其状态查询的典型耗时变慢；拆出 C 能降低只读基础开销”。它没有证明“只要拆出 C，透传负载下所有共享阻塞就会消失”。

数据：readonly-summary.json（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/readonly-summary.json`）、combined（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/readonly-combined.json`）、query-only（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/readonly-query-only.json`）、split（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/readonly-split.json`）。

## 20:44 三连接保持

根据只读对照，增加一次不同架构的有限保持。A 只发透传，B 只查询，C 只接收路点。B 与 C 的进程 ID 分别为 11065、11068；A 的 SDK 在父进程中。

| 项目 | 实测 |
| --- | --- |
| A 四次发送耗时 | **5.286 / 5.496 / 26.186 / 309.668ms**，均 rc=0 |
| 运动窗口开始的 B 查询 | **307.782ms**，rc=0 |
| 首个停止原因 | `state_reception_expired` |
| 触发时状态接收年龄 | **206.241ms** |
| 触发时 C 路点接收年龄 | 160.855ms |
| 退出透传 | rc=0，93.307ms |
| 慢停 ACK | rc=0，20.074ms |
| 停稳确认 | `stop_confirmed=true`，`stop_result_unknown=false` |
| 活动观察期间最大实测姿态偏差 | 约 0.000205° |
| A/B/C 注销 | 全部 rc=0 |
| 网卡错误/碰撞增量 | 已检查字段全部 0 |

这次分离没有通过原有 200ms 状态保护。因此三连接方案目前只是已实现、已测试的候选，不能作为已验证修复，更不能据此开始动态轨迹。

两个保持窗口都在活动期开始后很快退出，没有形成稳定 10Hz 的运动期查询样本。查询调度保留生产的空闲 50ms、运动 100ms以及接合前刷新；首个活动查询可能继承空闲期已安排的节拍。表中慢查询依据**请求开始时刻**落在进入透传与首个停止触发之间判定；其返回时已在停止阶段，因此不能仅按日志里的 `phase=idle_or_stopping` 排除它。

原始报告：abc-hold1.json（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/abc-hold1.json`）；两轮独立校验：hold-summary.json（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/hold-summary.json`）、verify_acceptance.py（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/verify_acceptance.py`）。

## 对保护逻辑和下一步的判断

现有接收年龄保护正确地区分了“读到缓存”与“收到新状态”。不应以查询线程仍存活、请求在途、IPC 正常或后来 rc=0 来刷新旧状态的时间戳。单次 SDK 耗时预算与状态接收年龄是两项不同约束，不能简单把 200ms 与查询最大值对比后放宽。

三连接仍出现跨进程 A/B 同窗口长阻塞，说明“B 自身推送争用其 SDK 接收分发”不足以解释全部现象；不支持继续靠拆出第四条连接解决。下一轮应关联同一窗口中的 A 请求、B 请求、各连接推送、TCP ACK 和业务应答，检查请求何时真正发出、柜端何时确认交付及应答何时到达，并区分入模早期瞬态与持续发送期行为。当前证据尚不能将主机 TCP 时序、柜端共享应答路径或 SDK 内部行为定为唯一机制。

本轮没有抓取匹配这些新会话的逐包报文，也没有读取每条新 socket 的 `TCP_INFO`；**不能把前轮无重传的结果移用成这两轮也已证明无重传**。本轮可确认的是 NIC 错误和碰撞计数无新增。网口、路由、QUICKACK、NODELAY和实时调度均未修改。

后续运动连续性改造仍需真实发送间隔 dt、速度/加速度状态、长阻塞后停止及重新实测锚定规则；它不能填补已经发生的 300ms 通信空档。本轮依照验收门槛暂未实施。

## 验证与产物

- 首次两连接部署前：相关离线检查 **28 项通过**。
- 三连接能力部署前：含 B/C 原生缓存时间戳与代次拒绝、路由/清理、发送及启动相关检查，**169 项通过**。离线日志：offline-step1-C.log（本机存档路径：`/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/offline-step1-C.log`）。构建运行的原生生命周期检查亦通过；这些均不代替保持验收。
- 两轮实机保持均未通过；没有放宽阈值、自动重试、清故障、重启控制柜、小幅位移轨迹或自由 VR 操作。
- 诊断数据、备份与脚本位于 `/data/VR_teleoperation_runtime/aubo-channel-isolation-20261108/`；车载复用既有 runtime 下同名目录。两轮完整保持 JSON 在车载 runtime 顶层分别为 `ab-hold1.json`、`abc-hold1.json`，本地已归档到该诊断目录。
