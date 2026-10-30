# AUBO 遥操链路再次审查

日期：2026-10-13。本轮仅审查、构建与离线测试，没有登录或查询机器人，没有打开夹爪串口，没有修改控制源码。临时传输测试录制在 `/data/VR_teleoperation_runtime`，测试退出后清理。

## 结论

已实现的是经过仿真验收的 Quest 控制链、可构建的 SDK 只读通信入口，以及作者核心与假设备的离线组合演练。真实持续运动通信控制没有完成，不能描述为“代码都写好了，只等真机验证”。

只读扩展仅导出 `login/logout/query/fk`。`ReadOnlyFollower.send_action()` 和 `stop()` 明确抛出 MotionUnavailable。OfflinePositionSession 只接受精确的 FakePositionBackend；AuboShadowCoordinator 内部建立假设备，dispatch_fake 不发送给控制柜。

## 逐段状态

| 链路 | 代码和离线证据 | 尚未完成／未验证 |
| --- | --- | --- |
| Quest 网页 → Relay → Quest 核心 | 腕部校准测试、输入序列/年龄回归、本机实际 WS 传输通过 | 本轮无真实头显；无线长时可靠性不由本机测试证明 |
| 核心 → 作者 mapper/IK → MuJoCo | world/RG/kinematic 基线回归与作者数值基准通过 | 运动学显示不模拟真机跟随或真实抓取 |
| 开发电脑 → SDK → 控制柜 | 只读绑定重新构建、加载、符号与未登录访问边界通过 | 没有网络登录成功证据、控制柜兼容证据 |
| SDK 反馈 → 缓存 → 模型/核心 | 有限串行 Get 和显式缓存转入接口；假反馈组合回归通过 | 无持续反馈服务、推送回调和机器人事件订阅 |
| Quest 网络输入 → AUBO 协调 | shadow.accept_xr_frame 可由调用者驱动 | 无实际真机运行入口；原 core WS 接收器不会调用 shadow 接口 |
| 候选目标 → 控制柜跟随 | 假设备检查最新槽、年龄、会话、跳变及发送未知结果 | FollowModeJointMove 未绑定，没有真实发送调度 |
| 超时/释放/退出 → 停止 | 假 stop 故障与恢复门控通过 | 无真实停止通道、在途调用协调、独立 watchdog |
| Trigger → RS-485 RG | 仿真二态行为通过；既有 robot_ws_dev 驱动可作参考 | 未接入新链；退出扭矩策略未确认 |
| 相机/记录 → LeRobot | 仿真三路 WebRTC 与 opt-in 原始日志通过 | 真实相机、发送/反馈时间对齐、LeRobot 数据构建未完成 |

## 具体接线缺口及复现

`QuestTeleoperator._ws_runner()` 对 xr_frame 直接调用自己的 accept_xr_frame。AuboShadowCoordinator 同名入口另行负责严格校验、input_time、Grip、参考空间变化和会话撤销。

离线复现：建立 shadow，给入有效假反馈，然后用 shadow.core.accept_xr_frame 喂入有效输入（与原核心 WS 接收器相同的调用路径）。结果 core_accepted=true，但 shadow.input_time=null，shadow.tick() 返回 null。这说明仅启动 shadow.core.connect() 不会完成网络接线。当前没有真机启动器这样调用，属于待实现接线，不是已运行真机链中的运动故障。

应保持一条输入入口，并明确核心离合与设备故障状态的职责；不能新增第二套手柄映射或 IK。现有 shadow/session 以单调用者测试为前提，不能直接用于多线程运行。

## 本轮验证

- test_aubo_offline.py：34 项通过，包括此前三处错误的回归。
- test_aubo_model_shadow.py：17 项通过。
- test_mujoco_teleop.py：27 项通过；合计 78 项 Python 测试。
- 网页腕部拟合 JS 检查通过；新入口脚本 bash -n 与 git diff --check 通过。
- 只读扩展重新构建成功；绑定加载、allowlist、未登录拒绝与 FK 输入边界通过。ELF 引用 SDK 构造/析构、Login/Logout、Get 与 FK，没有引用 Follow/Move/Stop/配置写入或注册推送接口。
- 约 10 秒本机端到端 WS/WebRTC 测试：688 输入帧、1982 状态、2004 录制控制周期；640×480 两路及 1280×720 一路有效视频；运动学显示/应用命令最大差 0 rad。输入为程序生成，不是真实 Quest。

## 后续工作边界

接机前仍可以完成：单一 XR 输入接线、单控制者运行框架、反馈/发送/故障职责划分、可替换假传输的调度与 watchdog 故障注入测试；按厂家已知接口声明补绑定并保持运动禁用。这些是代码工作，不需要等待机器人证明才开始，也不能把接口存在当作设备语义已确定。

接机后才可确认：SDK/固件兼容、关节顺序方向零位、法兰/TCP 与模型一致、实际状态推送行为、跟随覆盖/排队和周期、真实 stop 与断网行为、连接并发、真机限位及误差阈值、RG 退出策略。未确定参数不能沿用仿真 200 Hz 或人工测试值作为真机默认值。

robot_ws_dev 的现有 /move_joints 仍调用 robotServiceJointMove(..., true)，是既有阻塞点到点服务；新原生链没有调用它，不能将它当作已完成的连续跟随后端。
