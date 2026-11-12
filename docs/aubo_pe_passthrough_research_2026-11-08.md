# PE TCP2CAN：模式名称、系统区别和故障定位范围

2026-11-08。用户明确当前系统为 PE，要求核对提供的三个链接并回答四个问题。本轮只查源码、文档及已有证据，没有新增实机测试、修改控制方式或安装 ARCS。

## 1. 当前使用的模式

当前项目透传入口 `teleop/aubo/native/passthrough.hpp` 调用：

```cpp
robotServiceEnterTcp2CanbusMode();
robotServiceSetRobotPosData2Canbus(q);
robotServiceLeaveTcp2CanbusMode();
```

PE 旧 Linux SDK 手册 V1.0.1，第 52–53 页：模式叫“TCP 转 CAN 透传模式”；发送接口说明为“发送坐标数据到关节 CAN 总线”“通过透传将关节角度信息发送至驱动器”，角度单位 rad。

因此本项目讨论的“透传”“CAN 透传”“TCP2CAN”指向同一条当前接口路径，不是尚未开启另一种更底层模式。PC 通过网口发送六轴目标，控制柜负责向关节驱动传递；PC 没有通过 CAN 适配器直接发原始 CAN 帧。“透传”这个名字不能证明无缓存、无调度、无安全逻辑或零延迟。

实际模式由启动入口决定：原生类仍兼容 Follow；上述结论特指本次已经测过的 passthrough 路径，不声称当前有某个控制会话正在运行。

## 2. PE 与 ARCS 的依据

| 来源 | 本轮实际读到的内容 | 适用判断 |
| --- | --- | --- |
| 用户现场说明、当前 SDK 和代码 | PE；`ServiceInterface`、`robotService...`、SDK 2.5.0；已有版本 V4.5.111.71f2ee1-Alpha | 当前分析基线 |
| [SDK 在线文档版本管理](https://docs.aubo-robotics.cn/aubo_sdk_docs/guide_sdk/4_dev_preparation/3_version_control.html) | 明文“ARCS SDK 开发包遵循 语义版本控制(SemVer)” | 用户给的 SDK 在线文档属于 ARCS 文档体系 |
| [SDK ServoJ 说明](https://docs.aubo-robotics.cn/aubo_sdk_docs/guide_sdk/7_servoj_func_test.html) | `servoJoint`、`setServoMode`、t 参数及超时退出；列举 ARCS 0.29/0.31 等版本 | 不能把 t×5 规则或 servo 参数套到 PE TCP2CAN |
| [47 ARCS 安装](https://docs.aubo-robotics.cn/application_notes/47-install-arcs/) | 安装系统、初始化环境、ARCS 包及按硬件选择配置；涉及固件和新旧协议 | 是另一系统的部署说明，不是当前 PE TCP2CAN 的操作手册 |
| [98 PE 迁移 ARCS](https://docs.aubo-robotics.cn/application_notes/98-PE_migration_ARCS/index.html) | 明确对比两者的工程、脚本和配置；例如 PE `move_joint` 对照 ARCS 运动接口 | 有助于确认系统区别；未给出本 PE TCP2CAN 的消费与应答细则 |
| [55 SDK 资源汇总](https://docs.aubo-robotics.cn/application_notes/55-aubo-sdk-resources-guide/index.html) | 接口手册指向 `arcs_api/zh/`，Python 包为 `pyaubo-sdk` | 不能把此资源页当作当前 PE 2.5.0 的适配声明 |

PE 与 ARCS 的系统区别，也不能简单等同于机械臂内部 CAN 的“新/老协议”。ARCS 安装页本身区分新旧协议配置；不能仅凭用了 CAN，就判断系统是 PE 或 ARCS。

## 3. 用户给的 master 驱动：重要补充

已读取用户提供链接对应的 master 固定快照：

- 提交 `3b3b9b4f156a2f43585748b2adc536abeabff8e8`。
- [aubo_driver.cpp](https://github.com/AuboRobot/aubo_robot/blob/3b3b9b4f156a2f43585748b2adc536abeabff8e8/aubo_driver/src/aubo_driver.cpp)。
- 文件 SHA256：`0de70a1d1b64f75e39ecd50b802c76daf54be48e890f423d894c441d63802dc1`。

| 行号 | 行为 |
| --- | --- |
| 523、528 | send、receive 两个对象分别登录同一控制柜 |
| 558 | send 连接进入 TCP2CAN |
| 102、111–112 | receive 连接查询当前路点和诊断；取 `macTargetPosDataSize` |
| 308 | 使用与我们同名的单点 `SetRobotPosData2Canbus` |
| 378–392、432–446 | 主机轨迹队列、启动条件和柜端水位控制；不是收到一个最新目标就不顾水位地发送 |
| 39、59–61、282–293 | 200Hz OTG 参数、速度/加速度/jerk 设置及停止时速度轨迹生成 |
| 114 | `GetRobotCurrentState` 被注释；不能据此认定禁止查询，也不能把软件运动标志当成完整状态反馈 |

上一轮读到的 Noetic 固定提交 `32210c0b465bc4eef4de01a0917a878c3b24e799` 是三连接及 vector 批量补点。两个分支均有官方多连接先例，单点与批量都有官方实现；**不再将“官方都批量”作为架构前提**。源码存在仍不构成本机固件时延保证；这些队列数值也不宜直接复制到最新目标遥操中。

## 4. 故障实际定位到了哪里

按当前接口可以描述为：

```text
PC 目标/执行层 → SDK → 以太网/TCP → 柜端 TCP 接收
                                    ↓
                         柜端业务处理/透传队列
                                    ↓
                         内部接口板/CAN → 关节驱动

柜端业务应答 → 柜端 TCP 发送 → 以太网/TCP → PC SDK 返回
```

这是定位用的逻辑划分，不代表已取得本柜端完整内部线程或板卡架构。尤其不能假设 SDK 成功应答必须等到关节完成执行。

16:07 既有 PCAP 的一次慢调用：

| 阶段 | 已测时间 |
| --- | ---: |
| SDK 调用开始至请求出现在主机抓包 | 约 0.063ms |
| 请求出现至柜端 TCP 确认收齐 | 35.958ms |
| 已确认收齐至匹配业务成功应答到主机 | 181.748ms |
| 业务应答到达至 SDK 返回 | 约 0.198ms |

本窗口无有效载荷重传、序号缺口、零窗口或新增 NIC 错误。因此优先查柜端请求交付后的业务/应答路径及连接交互，比继续泛查 VR、USB 相机或重复网络测速更有依据。

但 TCP ACK 只确认柜端 TCP 收到字节，不确认业务程序已读取、点已入队、CAN 已发出或关节已执行。181.748ms 仍可包含柜端业务调度、队列等待、底层处理、应答生成、发送及回程交付；目前不能把这段全部归到“控制器转 CAN”。另一条状态查询也慢，但尚不能确定两者具体共享哪个模块。16:23 既有复查的 CAN 原始状态、碰撞、关节报警和六轴错误码均无异常，不构成已经确认的 CAN 硬件故障，也不能用事后无报警排除瞬态异常。

另外必须区分两个症状：

- 按住 Grip 中断：已有 17 次记录均由状态反馈超过 200ms 触发主机保护；不是已经证明 TCP 或 VR 断线。反馈为何迟到仍未彻底定位。
- 机械抖动/异响：PC 执行层确有缺口，实际送点不均匀、仅固定 alpha=0.15 位置 EMA，没有相邻实际下发点的速度/加速度连续性保证。它是有证据的抖动机制，但尚未证明为异响唯一物理原因；不能因为网线交付正常就把这项 PC 软件问题排除。

## 5. “需要接口说明”具体指什么

不是必须找到标题逐字等于 V4.5.111 的文档，也不是要求升级 ARCS。需要的是**适用于当前 PE TCP2CAN 的行为规则**，来源可以是覆盖本版本的 PE 手册、官方示例说明或技术支持回复：

1. 消费周期：一个关节点按什么节拍执行；单点/vector 是否相同；发送 0 返回表示收到、入队还是已执行。
2. 缓存及断流：水位字段单位、本版本是否有效、欠载与满载如何处理、建议点间隔及允许空档。
3. 查询交互：当前 RobotState/诊断查询是否支持与透传并用，推荐查询节拍；是否有适用 PE 的完整异步状态源。
4. 停止规则：Stop、Leave、断线是否清除尚未执行的点，恢复后会不会执行旧点。

已有旧手册和 master 源码足以确认“接口存在、单点和多连接有先例”，但还不足以精确定义上述规则。先复用这些 PE 依据；只对公开资料仍未回答的事项请厂家补充，不把获取某个同补丁号手册当成人为前置门槛。

## 6. 证据存档与范围

本轮 HTML、提取文本、链接、master 源码与来源清单存于 `/data/VR_teleoperation_runtime/aubo-pe-doc-research-20261108/`。master 原始下载有不完整/超时，最后通过 GitHub contents API 按固定提交取得完整内容，解码核验长度 33678 字节；未将失败下载算作已审阅源码。SDK 首页无 sitemap 不影响已读取的具体章节。

另读了官方 [83 CAN 故障排查](https://docs.aubo-robotics.cn/application_notes/83-can_bus_error_software_troubleshooting_manual/index.html)：该页包含现代 CAN FD/固件及硬件诊断内容，没有给本 PE TCP2CAN 同步调用的应答规则，也未提供本机发生 CAN 硬件故障的证据。本轮未据此安排拆机、接 CAN 分析仪或升级。
