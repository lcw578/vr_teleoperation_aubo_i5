# AUBO TCP2CAN：多连接与反馈接口核对

日期：2026-11-08。本轮仅核对现有 SDK、历史官方文档与官方源码；未登录机器人、未运行保持或运动测试、未修改生产代码和网络。本文是公开资料核对结果，**不是厂家对本固件的兼容性确认**。

后续修正：用户明确当前系统是 **PE，而非 ARCS**，并提供 master 分支链接。新增 [PE/ARCS 与故障范围调研](aubo_pe_passthrough_research_2026-11-08.md)。master 固定提交采用双连接、单点发送及诊断水位控制；本文原核对的 Noetic 提交采用三连接、vector 批量补点。因此下文“官方批量路径”特指 Noetic 该实现，**不能推导所有官方版本都批量，或单点路径不受支持**。需要补齐的是适用于 PE 的接口行为说明，不要求文档标题必须逐字对应 V4.5.111 补丁版本；覆盖该版本的 PE 文档同样有效。

## 1. 结论与架构决定

1. **官方旧版 ROS 驱动存在多连接透传的明确先例。** 不能把“两条 SDK 连接”本身判为不支持，不能把现有长阻塞直接归因于多连接。
2. **当前 SDK 的公开回调不足以直接替代完整 RobotState 查询。** 关节、路点、末端速度和事件回调存在；未找到完整运行/停止/暂停/恢复状态的公开周期推送订阅接口。保留现有状态保护，不通过删掉 B 查询让 A 测速变快。
3. **所查 Noetic 驱动的持续轨迹路径采用轨迹整形与缓冲水位管理、批量送点；master 也有水位控制下的单点路径。** 不能把逐点发送本身判错；200Hz 轨迹采样不等于每个同步 RPC 必须在 5ms 内完成。
4. **本固件的点消费周期、允许断流、缓冲单位和停止清队列语义仍缺资料。** 不能直接照抄官方示例的水位 400 或除以 6，也不能据两次 step=3 尖峰推断队列深度。
5. 进程隔离实现保留，但只算已部署的隔离措施，未证明解决根因。保持验收仍未通过；动态 VR 验收不推进。下一步应先取得本版本接口说明，再决定保留双连接、改用厂家支持的反馈接口，还是实现有明确时延上限的短批量供点。

## 2. 固定版本和证据

| 项目 | 本次依据 |
| --- | --- |
| 已测柜端版本 | V4.5.111.71f2ee1-Alpha；引用已有现场记录，本轮未重新读取 |
| 当前 SDK | `aubo_sdk-2.5.0-linux64.bf7070c` |
| SDK 库 SHA256 | `a04ad0aa05b2d46fea5a863fe30698950e6434afeafd5a3eaf605c3f0df0ff1b` |
| 官方 ROS1 源码固定提交 | `AuboRobot/aubo_robot`，`32210c0b465bc4eef4de01a0917a878c3b24e799` |
| 驱动文件 SHA256 | `5c3d2f3be73f4dbba4e3bf270c32ad684ee00291f27ce283c05d4ea369354d1b`；本轮重新下载与原存档一致 |
| 新证据存放 | `/data/VR_teleoperation_runtime/tcp2can-contract-confirmation-20261108/`；含源码、guide 文档及 URL/字节数/SHA256 清单 |

SDK 实际目录：`/data/robot_SDK/新版SDK/aubo_sdk-2.5.0-linux64.bf7070c/`。下文头文件路径均相对此目录 `include/aubo_sdk/`。

## 3. 多连接：官方做过，但没有本固件的时延保证

官方固定源码：[aubo_driver.cpp](https://github.com/AuboRobot/aubo_robot/blob/32210c0b465bc4eef4de01a0917a878c3b24e799/aubo_robot/aubo_driver/src/aubo_driver.cpp)。

| 源码位置 | 可确认行为 |
| --- | --- |
| 601、606、622 行 | `robot_send_service_`、`robot_receive_service_`、`robot_mac_size_service_` 分别登录同一 server_host/port |
| 643 行 | send 对象进入 TCP2CAN |
| 147–158 行 | timer 在 receive 对象上查询当前路点 |
| 936–964 行 | mac_size 对象查询诊断水位，并通过 vector 重载批量发点 |
| 72、93–95 行 | OTG 采样周期 1/200 秒，设置速度、加速度和 jerk 上限 |
| 974 行 | 该补点循环另有 4ms sleep，并非每次 RPC 等于一个 5ms 伺服周期 |

这证明历史官方实现曾组合“多连接 + 查询 + 透传”。没有找到针对 `V4.5.111.71f2ee1-Alpha + SDK 2.5.0` 的连接数上限、跨连接调度或查询最大响应时间保证。

receive 路径的 `GetRobotCurrentState` 在 164 行被注释；该处注释和软件 `start_move_` 标志不能当成厂家禁止查询，也不能当成完整柜端状态反馈的替代方案。

SDK CHANGELOG 中“多台机器人接入主机时分别设置运动属性”指多机器人，不能引用为同一控制柜多连接支持声明。

## 4. 异步反馈能提供什么

| 接口/结构 | 已有数据 | 不能直接替代的内容 |
| --- | --- | --- |
| `RealTimeRoadPointCallback`；`wayPoint_S` | 关节角、笛卡尔位置、姿态 | 无完整 RobotState、碰撞及全部诊断字段 |
| `RealTimeJointStatusCallback`；`JointStatus` | 关节电流、速度、位置、电压、温度、目标值、关节错误码 | 无运行/停止/暂停/恢复枚举；不能把关节错误码为 0 当成完整状态正常 |
| `RealTimeEndSpeedCallback` | 末端速度 | 速度接近零不等价于控制状态为停止 |
| `RobotEventCallback` | 状态变化和异常等事件 | 不是已确认有周期、有完整快照、可检测遗漏的 RobotState 流；无事件不能刷新状态新鲜度 |

依据：`serviceinterface.h:130–188`；`AuboRobotMetaType.h:462–470`、`799–805`、`861–875`。完整 RobotState 为 stopped/running/paused/resumed；公开查询入口在 `serviceinterface.h:908`，诊断查询在 941 行。

在已安装 SDK 的公开接口中未找到 RobotState 或 RobotDiagnosis 完整周期回调注册入口。协议枚举 `CommunicationType_RealTimeRobotDiagnosis` 的存在，只说明内部协议定义存在，不能证明应用有可用、受支持的订阅 API。

`serviceinterface.h:182–188` 明确：机械臂事件默认推送且“不允许取消”；取消应用回调不等于关闭网络事件。因此先前抓到 DO 变化事件本身不能作为多程序冲突依据，也不能通过取消回调就声称降低了柜端推送流量。

## 5. 送点契约仍缺哪些内容

本地 Linux API 手册的 TCP2CAN 章节仅描述进入、单点/vector 发送关节弧度和退出。旧 Python/Linux 示例包含进入透传后查询关节状态，但并非持续发送与并发查询的时延保证。

当前头文件对 `macTargetPosBufferSize`、`macTargetPosDataSize`、`macDataInterruptWarning` 标注“预留”；官方历史驱动却使用其中的水位字段。这个差异必须按固件确认，不能自行把数值解释成点数、毫秒数或固定缓存深度。

另外，官方 `aubo_driver_guide` 的 README 明列 AuboServer 4.10/4.11 对应另一套 driver 0.x；RTDE 示例使用 `createAuboDriver` 和 recipe API，升级说明涉及柜端系统、控制器和接口板固件。它不是已证实能直接接到本机 V4.5.111 上的替代反馈 API，本轮不安装、不升级、不切换。

## 6. 对已有失败证据的约束

- 16:07 PCAP：第二次发送 SDK 217.966ms，请求出现后 35.958ms 已被柜端 TCP ACK 确认收齐，匹配成功应答在 217.706ms 才到达，随后 SDK 约 0.198ms 返回。请求收齐到业务应答到达之间约 181.748ms；不能凭主机数据再细分为柜端哪个模块，也不是纯物理网络 RTT。
- 同窗口 B 状态查询也出现 ACK 后长等待，未见有效载荷重传、序号缺口、零窗口或新增 NIC 错误。两个连接一起慢是共同现象，不等于已证实同一把锁。
- 17:27 已把 B 放进全新进程，IPC 最大约 0.246ms，保持仍因状态过期终止。说明同进程 SDK 共享资源不是解释失败的必要条件，尚不能排除柜端多连接交互。
- 单连接最小测试平均 100Hz 以上不等于具备稳定供点能力，也不等于端到端运动延迟低于 10ms。位置 EMA 无法消除已经发生的通信断流。

详见 [进程隔离记录](aubo_process_isolation_2026-11-08.md) 和 [保持验收记录](aubo_passthrough_hold_acceptance_2026-11-08.md)。本轮没有新增实机测量。

## 7. 下一步只收敛这五项

给厂家核实的具体问题已整理为 [厂家确认问题单](aubo_tcp2can_vendor_questions_2026-11-08.md)，尚未对外发送：

1. 本固件与 SDK 的兼容版本、是否有已知透传/多连接响应阻塞问题。
2. 同柜双连接的支持范围、允许查询与推荐查询节拍，以及反馈是否有完整异步替代接口。
3. 单点/vector 的消费周期、同步返回含义、推荐批量和最大允许送点间隔。
4. 水位字段单位和有效性、欠载/积压处理、退出及停止是否清空剩余轨迹。
5. 结合已有 ACK 后等待证据，厂家需要的柜端只读诊断及应修复的版本/配置。

取得回复后先判断是否已有明确可实现的路径，再做必要改造及一次有区分目的的有限验收；不原样重复保持、不调整保护阈值、不预填深队列。
