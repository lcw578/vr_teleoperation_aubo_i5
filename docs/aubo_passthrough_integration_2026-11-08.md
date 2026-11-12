# Quest 3 六轴 TCP2CAN 遥操接入（2026-11-08）
> 现场状态更新：本次实机遥操出现明显抖动与频繁保护中断，当前版本暂停作为实机使用基线。27.04Hz段内实发、17次状态过期及SDK最长535ms阻塞的证据与处理建议见 [本次排查](aubo_passthrough_incident_2026-11-08.md)。下方为接入记录，不代表实机验收通过。

已停止 `tcp2can_minimal.cpp` 的底层测速，使用现有本地 IK、最新目标槽和独立 SDK 发送线程接入 `robotServiceSetRobotPosData2Canbus`。

## 车载启动

在车载电脑桌面终端执行：

```bash
cd /home/xc/code/lcw/VR_teleoperation
bash scripts/aubo_passthrough.sh --execute
```

输入原有 SDK 密码。Quest 已通过 USB 连接并授权调试时，启动脚本会建立 adb reverse。头显浏览器打开 `http://localhost:8443/`，点击 Start Teleop。右 Grip 接合遥操，松开 Grip 撤销发送并执行原有退出透传、停止和反馈停稳确认流程。右食指 Trigger 保留实体夹爪切换，录制沿用原有按键管理流程，启动时只处于 READY，尚未录制。

默认运行到退出；可用 `--duration 120` 指定时长，`--no-gripper` 禁用实体夹爪，`--no-record` 禁用录制。`--check` 只校验本地运行环境和构建产物，不连接控制柜。

## 发送行为

- 原 VR 输入和本地 IK 继续更新最新目标槽；不积压旧目标。
- 每次 Grip 新接合需要新鲜停稳反馈；进入透传后第一条位置指令发送该次实测六关节角。
- 后续每次发送按 `cmd = previous_cmd + 0.15 * (latest_ik - previous_cmd)` 平滑，`previous_cmd` 来自上一条成功下发指令。重新接合时从新实测角度重置，不继承上次滤波状态。
- 发送周期按 SDK 实际调用开始时间计算，最短请求周期 5ms。SDK 耗时超过 5ms 时不额外等待，也不突发补发。
- 新入口使用独立只读 SDK 连接和线程查询机械臂状态；运动连接发送时读取状态与关节推送的内存缓存，不主动发同步状态查询。
- 保留现有输入/目标/反馈过期、SDK 失败及停止结果不确定等撤销规则。低通滤波不是关节速度/加速度限制，不能消除 SDK 阻塞，也不保证端到端延迟。

本次只接 VR 遥操。左 Y 回零、左 X 放置仍属于原 Follow 入口；新入口不会自动执行这些动作。原 Follow 入口保留。

录制 metadata 标记 `command_mode=passthrough`、`target_policy=ik_ema`、`passthrough_alpha=0.15`、请求频率 200Hz。实际下发仍记录 `sdk_command`；SDK 接受不代表机械臂已执行到位。

## 验证与回滚

本地 Python AUBO 回归 502 项通过；生产 C++ 绑定连接离线 SDK 的并发与 CLI 检查 26 项通过；原生 SDK 绑定编译通过。覆盖首帧实测保持、平滑、最新目标、慢调用不额外等待、进入/首帧期间松手、重新接合滤波重置，以及透传状态查询分离、录制接线。

本地集成本轮备份及日志：`/data/VR_teleoperation_runtime/passthrough-integration-20261108/`。
车载部署前源码与原生模块备份：`/home/xc/code/lcw/VR_teleoperation_runtime/passthrough-integration-20261108/before-source.tar`、`before-native.tar`。

以上为软件离线验证。接入后的实机 VR 手感、实际发送频率和端到端延迟尚未测量；不将之前独立测速的 100–123Hz 视为本次整套遥操的实测性能。

车载部署已完成：原生绑定编译、`--check`、26 项离线原生并发/CLI 检查和 74 项六轴回归通过。车载桌面已打开 `AUBO TCP2CAN VR Teleop` 终端并启动新入口，USB 8443 转发已建立；启动后由操作者在终端输入 SDK 密码，再在头显网页开始遥操。
