# PE 异步反馈、第二连接与平滑器核对

2026-11-08。用户提出 Issue #89、多连接/半双工重传及二阶限幅方案。本轮读取实际代码和 Issue 网页，未修改生产逻辑、未执行实机测试。

## 当前实现的准确描述

- `native/follow.cpp:235–241`：初始化注册 `RealTimeRoadPointCallback`；回调中的 `wayPoint_S.jointpos` 是当前关节反馈，不是主机目标。回调只发布带原接收时间的缓存，没有主动 SDK 查询。
- `quest_follow.py:1066,1134–1137`：透传入口启用 `ProcessStateObserver`，全新解释器持有 B SDK；A 绑定本地原生状态缓存。
- `native/follow.cpp:116–123`：B 主动调用的是 `GetRobotCurrentState`，用于运行/停止/暂停/恢复状态，不是每帧 GetJointStatus/GetCurrentWaypoint。
- `follow.py:68–69,175–179`：活动查询目标周期 100ms，空闲 50ms，即目标 10/20Hz；实际可能因阻塞变慢。每次仅一个在途查询。初始化、接合和停止确认另有刷新需求，不能将目标周期写成全程固定实测频率。
- 现有 200ms 保护同时依赖运行状态和关节反馈各自的新鲜度。JointStatus 回调虽可提供更多关节字段，但没有完整 RobotState，不能只换回调就声称原监测等价。

## Issue #89 原文

来源：[Physical robot movement stutters a lot.](https://github.com/AuboRobot/aubo_robot/issues/89)。GitHub API 多次 504，最后取得完整网页并提取可读正文及讨论，存于 `/data/VR_teleoperation_runtime/aubo-pe-doc-research-20261108/issue89/issue.html` 和 `issue.txt`。

2023-09-06，用户 ian-chuang 报告关闭 `publishIOMsg` 线程并降低 `timerCallback` 频率，有助于避免缓冲归零、抖动减轻，但原文仍说“still get stuttering though not as bad”。讨论还包含主机实时性、轨迹变换等其他假说，没有将“后台 Get 必抢柜端锁”作为证实结论或厂家保证。

这给“查询负载影响持续供点”增加相关先例，值得定向核查；不证明本机 101→27Hz 的确切因果，也不说明关掉第二连接一定恢复 100Hz。

## 与已有本机证据的关系

- B 查询/柜端共享处理仍是嫌疑。进程隔离只能隔开同进程资源，不能隔离柜端，因此隔离后失败不能排除此嫌疑。
- 半双工与累计碰撞值得核实两端配置，但已有失败抓包窗口没有有效载荷重传、序号缺口和新增 NIC 错误，不能认定该窗口由 Linux 200ms 重传造成；TCP 连接数量也不直接决定以太网碰撞次数。
- 不能通过只关闭 B、只看关节角回调的方式宣称完整状态保护保留。需要适用于 PE 的替代状态源，或在覆盖新鲜度及响应预算的范围内设计查询节拍对照。

## 所提平滑器能与不能保证的内容

带速度/加速度状态的执行层是正确改进方向，优于仅位置 EMA；但示例只约束内部变量 `cmd_v` 的每步变化。它未完整定义离散停车距离和目标越过处理，不能保证接近目标一定不超调；目标突变/反向会持续减速后再转向。未约束 jerk。

若将真实 dt 截断为 20ms，积分参考时间已不同于真实通信/执行时间；SDK 阻塞期间没有新点发给柜端，限幅代码不会消除空档。柜端如何给位置点定时仍需 PE 规则，因此内部变量限加速度不等于真实关节加速度得到同一上限。

该代码不是 ARCS servoJoint 的数学等价实现；不能保证任意网络抖动下无异响，也不能修复同步 SDK 阻塞或保证恢复 100Hz。1rad/s、3rad/s² 是提议示例值，未经本机限制与试运行约束核实，不直接作为实机参数。
