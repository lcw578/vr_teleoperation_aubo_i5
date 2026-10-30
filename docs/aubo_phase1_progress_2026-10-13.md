# AUBO 真机部署阶段一进度

日期：2026-10-13。用户已授权先完成阶段一离线代码工作。本轮没有连接控制柜、登录 SDK、调用运动/停止接口或打开 RS-485 夹爪。

## 本轮完成

### 1. 增加单一离线运行适配器

`teleop/aubo/runtime.py` 增加 `OfflineRuntime`。它不是硬件 follower，而是阶段一的控制所有者演练：

- 接收线程只发布最新手柄运动帧，不在接收线程执行 IK；
- 输入格式错误、接收时间错误和乱序接收事件不会被下一帧静默覆盖；
- `step()` 是唯一推进 `AuboShadowCoordinator` 和假发送的位置；
- watchdog 可以独立于 IK tick 检查输入/反馈失效；
- 首次接合的候选不会在同一时刻立即发送，等待显式控制周期；
- 仍然只使用 `FakePositionBackend`，`motion_available=False`。

### 2. 明确 Shadow 的公共故障入口

`AuboShadowCoordinator` 增加 `reject_input()` 和 `watchdog_tick()`，运行适配器不再需要访问撤销私有方法。Quest 核心控制算法、映射、IK、归位姿态和仿真入口未改变。

`accept_xr_frame()` 可接收主机接收时间，便于后续区分 Quest 源时间、网络接收时间和控制执行时间。它仍会拒绝未来、过期或不可解析的时间戳。

### 3. 回归测试

新增 [tests/test_aubo_runtime.py](/home/lcw/VR_teleoperation/tests/test_aubo_runtime.py)：

- 接收线程不会提前执行核心；
- latest-only 运动帧与输入故障不会静默丢失；
- 首次候选等待一个显式控制周期；
- 独立 watchdog 在输入断流后锁住 Grip 恢复；
- 乱序接收事件会撤销会话。

执行入口：

```bash
cd /home/lcw/VR_teleoperation
bash scripts/aubo_phase1_check.sh
```

本轮该入口以及原有离线、模型/核心和仿真回归均通过：34 + 17 + 5 + 27 项 Python 测试，原生只读绑定边界检查也通过。

## 阶段一仍未完成的部分

这些工作可以继续离线完成，但本轮尚未实现：

- 把现有 Quest WebSocket 接收线程正式路由到 `OfflineRuntime` 的网络启动入口；当前仿真入口继续直连作者核心，以保护已验收手感；
- 可注入阻塞的假发送任务，以及发送期间 watchdog 的结果未知/退出状态；
- 为未来 AUBO follower 固化反馈、发送、停止和断开协议接口，但不填入未经确认的 FollowMode/stop 默认实现；
- 夹爪独立任务的 fake-only 生命周期演练。

## 必须等接机或厂家资料的内容

- FollowMode 的目标覆盖/排队/拒绝语义；
- 正常调用周期、尾延迟和控制柜反馈含义；
- 当前固件的正确停止入口、并发要求、在途目标处理和通信丢失行为；
- 实时回调频率、采样时间与注销生命周期；
- 关节零位/正方向、法兰/TCP、实际范围和 RG 负载配置。

阶段一代码不得因为缺少这些信息而自造真机默认值。真实运动入口必须继续保持未配置或显式拒绝状态，直到只读核对和受控跟随测试完成。
