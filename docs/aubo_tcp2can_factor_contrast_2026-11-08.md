# TCP2CAN 发送耗时的单变量实机对照

2026-11-08，Asia/Shanghai。执行范围：用户授权监督下的当前姿态保持对照、发送路径代码审计、`eno1` 只读检查。没有接入 VR，没有增量运动目标，没有部署生产代码，没有修改网络参数；未执行第三步状态观测和运动约束改造。

## 结论与证据边界

**在本机 V4.5.111 控制柜上，仅在发送连接 A 开启 `RealTimeRoadPointCallback` 路点推送，就足以把 `SetRobotPosData2Canbus` 的 P50 从约 6.5ms 推高到约 52ms。关闭推送后恢复，独立重复仍成立。** 这复现了完整遥操的主要耗时特征，不需要 VR、Python、IK 或完整应用的互斥锁参与。

单独加入 B 连接、以目标 10Hz 查询 `GetRobotCurrentState`，本轮没有复现 A 的中位数恶化。该结果不能证明 B 查询永不影响性能，也没有覆盖 A 推送与 B 查询的组合效应；目前不支持以“B 查询已经证实是主因”为由删除状态监测。

本轮锁定的是**同连接路点推送这个充分触发条件**，尚未把 SDK 接收分发、控制柜推送/应答调度、TCP ACK/Nagle 交互中的具体内部机制分离。不能将 52ms 直接解释为某一把锁等待、固定 50ms 控制柜周期或 TCP 重传超时。

后续 20:38/20:44 的[通信隔离保持验收](aubo_channel_isolation_acceptance_2026-11-08.md)显示：关闭 A 的可选遥测推送、甚至将状态查询和路点推送进一步拆到独立 B/C 进程后，仍有跨连接约 300ms 长阻塞并触发状态过期。本页对 P50 恶化的触发条件归因不等于全部长尾已被解决，隔离方案尚未通过持续保持验收。

## 环境和实验方法

- 车载电脑：`xc-pc`，当前 SSH 地址 `xc@10.89.248.164`，复用原 `10.245.247.164` 的已知主机密钥校验身份。
- 控制柜：`192.168.1.5:8899`，实测版本 `V4.5.111.71f2ee1-Alpha`。
- SDK：`aubo_sdk-2.5.0-linux64.bf7070c`；共享库 SHA256 为 `a04ad0aa05b2d46fea5a863fe30698950e6434afeafd5a3eaf605c3f0df0ff1b`。
- 使用从 minimal 派生的独立 C++ 诊断程序；本轮不是原 minimal 的 J6 累计增量测试。每轮先测得六关节当前角度，整轮重复发送该固定目标。
- 同一二进制、普通调度策略 0/优先级 0；以发送开始时刻为基准，调用不足 5ms 时等待到 5ms，超过则不额外等待。不启用 QUICKACK、NODELAY、实时调度或路由调参。
- 所有组先统一关闭 RoadPoint、JointStatus、EndSpeed 推送；A 推送组再注册与遥操同类的路点回调。回调只复制关节角、记录真实接收时间、发布到有界队列，不反调 SDK。
- B 空闲组与 B 查询组均使用第二个独立 SDK 对象；查询组由独立线程执行，目标每 100ms 发起一次、同一时刻只有一个在途查询。此处 B 在同进程中，完整遥操的 B 可在独立进程中；未把两者当成完全相同的应用环境。
- 发送耗时只包围 `robotServiceSetRobotPosData2Canbus`，回调缓存读取及检查在计时外；发送循环内不逐条打印或写盘。
- 每组有限 2 秒，补充复验各 3 秒，均设置最多 512 次调用。保留入模前状态/诊断检查、事件故障监测、退出透传、兼容慢停和停稳确认。发送耗时预算 500ms；启用推送时检查 200ms 接收新鲜度及相对固定目标的 0.1° 偏差；B 查询组另检查状态新鲜度。
- 基线连续发送期间没有额外主动状态轮询，这是此次有限、有人监督的隔离诊断条件，**不是已完成生产状态监测替换验收**。
- 百分位使用 `(n-1)*p` 线性插值；全部统计已从 CSV 独立复算。历史 minimal 的统计实现与窗口不同，不能把本轮均值/频率与历史数字直接视为同条件比较。

诊断源文件：tcp2can_factor.cpp（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/tcp2can_factor.cpp`）。车载副本位于 `/home/xc/code/lcw/VR_teleoperation_runtime/tcp2can-factor-20261108/`，未覆盖生产二进制。

## 第一轮对照结果

运行时间：20:17:31–20:17:50。表中频率由相邻发送起始时间间隔的均值计算，并非 `1000/P50`。

| 条件 | 发送次数 | SDK P50 / ms | SDK P95 / ms | 最大值 / ms | 实际发送频率 / Hz |
| --- | ---: | ---: | ---: | ---: | ---: |
| N0：单连接、无路点推送 | 133 | 6.435 | 64.594 | 141.669 | 66.68 |
| A_push：仅 A 路点推送 | 56 | 51.986 | 68.896 | 179.044 | 27.76 |
| N1：关闭 A 推送恢复基线 | 159 | 6.552 | 22.756 | 172.406 | 79.07 |
| B_idle_before：A 无推送、B 只连接 | 145 | 5.877 | 60.625 | 154.351 | 72.09 |
| B_query10：A 无推送、B 目标 10Hz 查询 | 149 | 5.602 | 45.313 | 165.925 | 74.14 |
| B_idle_after：B 停止查询、保留连接 | 215 | 5.907 | 17.405 | 124.604 | 107.13 |

开推送后，超过 40ms 的发送从 N0 的 8/133 上升为 31/56，关闭后 N1 为 3/159。这说明慢调用占比上升，不只是所有调用统一增加某个固定耗时。

B 查询实测共 19 次，平均发起频率 **9.274Hz**，自身 P50 13.411ms、P95 141.128ms、最大 199.154ms。目标节拍是 10Hz，但不能写成实测稳定 10Hz；B 自身也存在长尾。本窗口中 A 的 P50 没有恶化至 52ms，P95 仍有 45.313ms，不能据此宣称 B 完全无影响。

原始报告：report.json（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/vehicle/report.json`）。

## 较长推送/恢复复验

首次 A 推送窗口仅 56 次，耗时分布快慢并存；去掉前六次后剩余 50 次，恰好 25 次超过 40ms，P50 变为 28.810ms。为避免把短窗口的中位数敏感性误判为稳定的 52ms，补充执行一次 3 秒推送、一次 3 秒恢复，未改变其他设置。

运行时间：20:19:55–20:20:03。

| 条件 | 发送次数 | SDK P50 / ms | SDK P95 / ms | 最大值 / ms | 实际发送频率 / Hz |
| --- | ---: | ---: | ---: | ---: | ---: |
| A_push_repeat：A 路点推送 | 65 | 52.241 | 120.588 | 273.879 | 21.36 |
| N2：关闭推送 | 255 | 6.833 | 23.474 | 132.841 | 84.74 |

同样去掉前六次后的敏感性核查：

- 推送开启：剩余 59 次，P50 **50.813ms**，P95 75.123ms，34/59 超过 40ms。
- 推送关闭：剩余 249 次，P50 **6.632ms**，P95 22.797ms，最大 34.482ms，无超过 40ms 的调用。

因此，50ms 级恶化不只来自入模后的前几次调用。但不同窗口仍有明显长尾，不应把推送开启后的所有调用描述成固定 52ms，也不能声称关闭推送后已具备稳定 100Hz/200Hz 控制能力。

复验报告：repeat-report.json（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/vehicle/repeat-report.json`）。八轮 CSV 的独立复算及哈希：all-eight-independent-verification.json（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/all-eight-independent-verification.json`）；校验脚本：verify_evidence.py（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/verify_evidence.py`）。

## 代码与锁审计

车载抓回源码的 `send_joints` 在 deployed-follow.cpp:397（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/deployed-follow.cpp:397`）。`io_` 加锁、状态缓存刷新和发送前检查发生在 `s=now_s()` 之前；第 416 行（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/deployed-follow.cpp:416`） 才开始计时、调用 SDK、记录结束。

本地当前源码保持同样边界：[follow.cpp:456](../teleop/aubo/native/follow.cpp#L456)，调用计时位于第 475 行。[passthrough.hpp:18](../teleop/aubo/native/passthrough.hpp#L18) 的发送包装仅检查透传模式状态后直接调用 SDK。

据此可区分：

- **应用层等待 `io_`、进入 GIL 释放区、读取缓存锁的时间不计入原生 `duration_s`。** 它们可能影响端到端发送间隔，但不能直接解释该指标的 52ms。
- 路点回调 [follow.cpp:44](../teleop/aubo/native/follow.cpp#L44) 使用反馈缓冲区自己的 mutex，不取得 A 的 `io_`，不调用 SDK。B 状态观察对象也有自己的 SDK 和 I/O mutex。没有发现这三者共用 A 的应用层 I/O mutex。
- Python [follow.py:430](../teleop/aubo/follow.py#L430) 的 `_call` 另有较宽的看门狗诊断计时，但原样返回 native result；发送指标读取的是原生 `result['duration_s']`，不能把两类 duration 混为一谈。
- **SDK 内部的等待仍计入原生 `duration_s`。** 所用 SDK 头文件中存在按通信类型划分的 `m_robotCommandMutexSet`，声明是对象成员；这不足以排除其他全局资源或柜端共享资源。
- 反汇编确认 `requestServiceOnlyCheckSendResultMode` 调用了 `sendRequestWaitResponse`。因此“同步 SDK 调用”指调用线程必须等 SDK 此次调用完成后才能继续发下一点，不能理解为只把字节写到 socket 就立即返回，也不等于电机已到达目标。

不依赖完整应用 mutex 的独立 C++ 已复现推送效应，说明完整应用的锁不是产生这一现象的必要条件。SDK 内部锁等待是否参与，需要进一步按请求、接收和唤醒时间点观测，当前没有足够证据确定。

SDK 请求路径证据：sdk-request-only.asm.txt（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/sdk-request-only.asm.txt`）。

## 车载源码与本地源码差异

本轮抓回的车载 `native/follow.cpp` SHA256：`33bc2eb6ab7b96d490b7670aacf684fc537d7f1b1a9aef450c0fb169822ab7af`；本地为 `25df0234a47e3b206a0e203e188672510878a7df74fdc5aa04026450dfe8efaa`。

车载 `start_waypoint_feedback` 仍在发送对象的 SDK 上注册 A 路点推送；车载 `process_state.py` 仅转发状态查询，没有本地新版本的独立关节遥测通道。本地已有的 B 独立反馈路径，不能视为已经在车载端部署并通过实机验收。这里核对的是源码快照，并未将源码哈希冒充当时加载的二进制哈希。

抓回源码：deployed-follow.cpp（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/deployed-follow.cpp`）、deployed-process_state.py（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/deployed-process_state.py`）。本轮没有部署它们的替代版本。

## eno1 只读检查

车载原先未安装 `ethtool`。本轮将本机现有工具复制到诊断运行目录执行，没有安装软件包或改动网络配置。主要结果如下：

```text
Supported link modes: 10baseT/Half Full, 100baseT/Half Full, 1000baseT/Full
Supports auto-negotiation: Yes
Advertised link modes: 10baseT/Half Full, 100baseT/Half Full, 1000baseT/Full
Advertised auto-negotiation: Yes
Link partner advertised link modes: 100baseT/Half
Link partner advertised auto-negotiation: No
Speed: 100Mb/s
Duplex: Half
Auto-negotiation: on
Link detected: yes
```

驱动为 `e1000e`。工具对一个附加 netlink 查询打印 `Operation not permitted`，上述核心链路字段和 `-S` 驱动计数均已读取。原始输出：initial-link.txt（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/vehicle/initial-link.txt`）、initial-driver.txt（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-factor-20261108/vehicle/initial-driver.txt`）。

NetworkManager 保存的 `Wired connection 1` 配置中 `auto-negotiate=no`、`speed=0`、`duplex` 未指定；这与当前 PHY 的 `Auto-negotiation: on` 不一致，应区分保存配置与实际生效状态。

本机支持全双工和千兆，实际却是 100Mbps 半双工；对端报告未开启自协商。这与对端采用固定速率、主机通过并行检测获得速率并落入半双工的情况相符。**仅凭本端输出，不能确认柜端实际配置是 Full 还是 Half，也不能确认连接中间是否有其他端口，因此尚不能把双工不匹配定为事实。** 后续需要核对实际链路对端端口配置和拓扑，再一致地修复两端设置，不应单方面强制 Full。

初始累计计数有 `tx_errors=1`、`collisions=2`、`tx_window_errors=1`、`tx_abort_late_coll=1`。八轮各自前后比较，这些计数及其他已读取 NIC 错误计数**均无新增**。各测试进程内每条控制柜 socket 的 `TCP_INFO.tcpi_total_retrans` 前后均为 0。

因此，半双工是已证实、需要继续处理的链路问题，但**本轮复现的 52ms 不能归因为测试期间新增迟碰撞或 TCP 重传**。半双工下的争用/时序影响并未被这些计数完全排除，TCP 延迟 ACK 也不是 TCP 重传，不应混写。

## 收尾及下一步依据

八轮共 **1177 次发送**，发送返回码全部为 0；每轮进入/退出、慢停、注销均成功，`stop_confirmed=1`，事件回调未记录配置中定义的故障事件。逐轮结束和复验后的连接快照无残留控制柜 TCP 连接，最终另行只读核对无诊断发送进程。停稳确认使用 `RobotStopped` 加连续采样关节角稳定判据；这并非对整套遥操安全系统的替代验收。

下一轮优先验证“**A 只发透传、真实路点推送由独立 B 连接接收**”，完整保留真实接收时间戳和现有状态保护，并在相同保持对照条件下验证发送间隔与反馈新鲜度。此次未测 B 路点推送，也未测 B 推送与 B 查询的组合，不能把该方向写成已验证修复。它是否足以消除发送长尾、B 自身的查询阻塞是否影响路点新鲜度，都要单独验收。

现阶段不依据本轮结果删除 B 状态查询，不实现第三步二阶限幅，不开始自由 VR 遥操。网络侧按用户当前范围仅做了只读检查。
