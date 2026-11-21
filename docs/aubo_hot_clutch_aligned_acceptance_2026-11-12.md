# 热离合参数对齐、实机验收与 C-1 启动（2026-11-12）

> 当前版本已由用户确认为[真机基线 aubo-c2-refined90-2026-11-12](aubo_teleop_baseline_2026-11-12.md)。本文保留该轮实验/诊断的原始时序与状态；“待验收”“当前未启动”等措辞属于当时记录，不覆盖后续基线结论。

本次按用户三项参数决策更新，**最终同一源码/native 的 Stage A 与 Stage Clutch 全部通过**。车载默认 `follow-build.json` 已原子切换至 `hot-aligned-20261112` 构建，并启动 C-1 真机 USB WebXR 多轮自由离合测试。C-1 启动检查就绪、idle、fatal=false、stop_result_unknown=false、submitted=0，等待新的 Grip 松开/按下；这代表 C-1 已开始，不代表自由遥操阶段已验收完成。

## 最终参数

- 透传 ACTIVE/BRAKING/HOLDING 的 State Age、Joint Age、单次 A 调用统一 ≤500ms；超过任一硬底线闭锁。连续三次 A 调用各 >250ms 仍闭锁，中间一次 ≤250ms 即清零。VR shadow 的关节反馈时效同步为 500ms。
- 运动期 B 查询目标周期 **200ms（5Hz）**，PRIMING、空闲、接合前与停止确认保留 **50ms（20Hz）**。保持单次在途、起点到起点调度、无追赶补发。
- 冷预热与热重接合的快帧条件统一为连续 **5 次 ≤40ms**；native 冷预热同步执行。冷预热仍至少 12 帧、总上限 2.5s、单次/年龄 400ms，冷准入 State ≤100ms / Joint ≤60ms。
- 热重接合要求 State Age ≤250ms、Joint Age ≤120ms、距保持点 ≤0.1°，并保留连续静止确认及最新手柄重新锚定。反馈暂未达标时保持 HOLDING，刷新达标后自动准入，首帧为精确保持点。
- 轨迹上限仍为 10°/s、30°/s²、300°/s³，dt 推进封顶 50ms。它们约束主机参考曲线，不是实测电机导数结论。
- 启动检查同时核对 native/Python 的反馈及快帧常量，避免旧 native 与新 Python 混用。Follow/旧 J6 仍使用原先 200ms 反馈门槛。

## 软件检查

本机 **553 项完整 AUBO 回归通过**；两端 native 模拟各 **31 项通过**；车载新增 native 门槛测试 **5 项通过**，热离合 Python **16 项通过**。两端绑定离线检查、Shell 与差异格式检查通过。

更新覆盖：302.580ms / 318.9ms 原失败年龄现允许；499ms 放行与 501ms 拒发；三次慢调用计数/复位；30ms 帧计入快帧、41ms 帧重置连续计数；冷预热 400ms 与热阶段 500ms 的独立边界；B 的阶段切换及真实调度时间；热准入 State 250ms / Joint 120ms 边界；握住 Grip 时等待刷新自动重接合。

## 最终实机结果

两阶段均为生产 J6FollowWorker、A 发送 / B 状态查询 / C 独立关节推送；源码哈希和 native 清单相同。无自由 VR 输入参与程序验收，未追加 PCAP 或柜端抖动归因。

| 指标 | Stage A | Stage Clutch |
| --- | ---: | ---: |
| 结果 | 通过 | 通过，5 次完整运动制动/重接合 |
| 有限 ACTIVE/热阶段窗口 | 5.000s | 12.230s |
| 冷预热发送帧数 | 13 | 20 |
| ACTIVE 发送数 | 376 | 683 |
| ACTIVE A P50/P95/Max | 7.110 / 25.893 / 166.717ms | 6.259 / 23.962 / 48.609ms |
| State Age 连续保守上界 | 275.657ms | 235.004ms |
| Joint Age 连续保守上界 | 218.340ms | 57.789ms |
| Enter 次数 | 1 | 1 |
| 热 resume 次数 | 0 | 5 |
| 首次 ACTIVE/重接合首帧增量 | 精确 0° | 6 帧均精确 0°，包括冷接合首帧 |
| 最大关节变化（相对初始点） | 0.001755° | 0.840230° |
| 最终停止 | 已确认，结果非未知 | 已确认，结果非未知 |

Clutch 使用 ±0.8° 正弦目标。五次松手瞬间参考速度分别为 **1.544、1.347、1.464、1.477、1.396°/s**，均非零，实际完成 ACTIVE → BRAKING → HOLDING → ACTIVE 五次循环；结束时另做一次最终归锚后的制动/保持，再 Leave、慢停、停稳确认及登出。BRAKING 共 101 次发送，P50/P95/Max=11.780/22.004/27.875ms；HOLDING 共 162 次，9.749/18.526/39.051ms。

## 默认版本与 C-1

车载默认清单已切换为 `/home/xc/code/lcw/VR_teleoperation_runtime/hot-aligned-20261112/build-follow/python/_aubo_follow.cpython-310-x86_64-linux-gnu.so`，SHA256 `945d808aade8a6e9235a12c0e03d45f95ea8c2b5e88c713d82f6e7f399894b90`。切换前旧清单已备份；SDK 库未变更。

C-1 正式进程 PID **8399**，2026-11-12 **13:35:36（Asia/Shanghai）**进入启动流程，13:35:37 反馈与 Relay 就绪；有限时长 **1800s（30分钟）**，预计约 14:05 自动受控退出。Quest USB 页面 **http://localhost:8443/** → Start Teleop；先松开 Grip 再按住接合，松手制动后保持。world 旋转与原坐标映射保持，数据集录制和夹爪未启用，仅保存诊断流。

C-1 启动器第一次因项目导入路径缺失在连接 SDK 前退出；补上项目路径后正式进程正常启动。没有修改通过验收的控制源码/native，没有因此重跑运动验收；首次启动日志单独保留。

启动快照确认 `motion_available=true`、`feedback_ready=true`、`idle`、`fatal=false`、`stop_result_unknown=false`、`submitted=0`、`require_release=true`、A/B/C 连接3条、热反馈上限 state/joints 均 0.5s、运动/空闲 B 周期 0.2/0.05s。C-1 运行结果由后续用户操作和诊断评估，本记录不提前宣称通过。

## 证据位置

本机 `/data/VR_teleoperation_runtime/hot-aligned-20261112/`，车载 `/home/xc/code/lcw/VR_teleoperation_runtime/hot-aligned-20261112/`。`stage-A/report.json`、`clutch-5/report.json` 保存原始推送、状态、发送、哈希与清单；`summary.json` / `analyze_acceptance.py` 保存统计。车载 `C1/run.log`、`C1/diagnostics.jsonl`、`C1/process.json`、`C1/startup-status.json` 保存本次真实 VR 运行记录。

实机验收证据包 `acceptance-evidence.tar.gz` SHA256：`39f3715ef0e4da9408ffb6bd821e912cae24cb1cb5054a5be9220932f4ea389d`。


## 13:42 头显连接故障复核

用户报告头显网址无法连接。确认 PID8399 和本机 HTTP 服务仍运行，`http://127.0.0.1:8443/` 返回200；Quest USB已连接，但 `adb reverse --list` 为空。C-1 自定义启动器绕过了包含 USB reverse 的 shell 入口，遗漏端口映射，是本轮启动检查遗漏。已补 `adb -s 2G97C5ZJ3002SP reverse tcp:8443 tcp:8443` 并确认列表含 `tcp:8443 tcp:8443`；后续启动器也已加入明确单设备检查和反向映射。头显应使用 HTTP 的 `http://localhost:8443/`，不能使用 HTTPS 或车载热点地址访问此 loopback 服务。

独立发现 C-1 在空闲、XR输入0帧、submitted=0时已触发 `state_observer: query_over_budget` 并锁定。首因记录的主机B在途年龄564.895ms；其后迟到结果记录SDK原始调用227.062ms、返回到主机消费另有延迟，不能将564.895ms直接当柜端SDK调用时长。当前未清锁、未重启机器人会话、未放宽阈值；网页恢复与遥操准入是两个不同状态。快照保存于 `C1/usb-repair-fault-status.json`，待后续处理。此前程序 A/Clutch 通过结果保持为有限实机验收事实，不能据此写为C-1通过。


## 13:47 用户授权清除主机锁定并重启 C-1

用户明确要求清除锁定后，先保存 `C1/before-authorized-reset-status.json`，对已核实的旧 PID8399 发 SIGINT，确认退出、8443 监听关闭、A/B/C SDK 连接全部断开，再从当前反馈重新建会话；未修改保护参数或清控制柜报警。新进程 PID9405，13:47:53 启动，30分钟上限预计14:18受控结束。记录目录 `hot-aligned-20261112/C1-reset-20261112-134753/`；`latest-C1.json` 指向新会话。启动检查 idle、fatal=false、fault=null、feedback_ready=true、stop_result_unknown=false、submitted=0，USB reverse tcp:8443 已存在，等待先松开再按 Grip。


## 后续 C-1 结果

用户实际操作报告连续运动与制动均顿挫。第六次松手后 hot_braking_settle_timeout 闭锁，停稳确认后已关闭。C-1未通过，真实VR当前不继续；程序小幅验收不替代执行质量验证。相关历史试验记录已从当前基线工作区清理。
