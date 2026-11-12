# 热离合分级容忍实现与实机验收（2026-11-12）

> 当前版本已由用户确认为[真机基线 aubo-c2-refined90-2026-11-12](aubo_teleop_baseline_2026-11-12.md)。本文保留该轮实验/诊断的原始时序与状态；“待验收”“当前未启动”等措辞属于当时记录，不覆盖后续基线结论。

> 后续用户已授权对齐为 Joint 500ms、B 运动期 200ms、快帧 40ms；新版本已通过两项实机验收并启动 C-1。见[最新记录](aubo_hot_clutch_aligned_acceptance_2026-11-12.md)。本文保留此前 Joint 300ms 轮次历史。

按用户本次工程决策完成 Python/native 分级保护、热重接合等待与验收门槛更新。软件检查全部通过；本次实机静止验收在 ACTIVE 4.634s 时触及 **Joint Age >300ms** 的硬底线，未满 5s。五次运动中热离合未执行，车载默认 `follow-build.json` 未切换，C-1 未启动。没有追加 PCAP 抓包或对柜端长尾进行归因。

## 实现规则

- 仅透传 ACTIVE / BRAKING / HOLDING 采用 State Age ≤500ms、Joint Age ≤300ms 的硬边界。超过 200ms 的瞬态反馈年龄不再立即撤销许可；用户描述中 Joint Age 容忍区与硬底线重叠，执行明确的二级 Joint >300ms 底线。
- 单次 A 发送 ≤500ms 允许返回；连续三次 A 发送各 >250ms 时闭锁，任一次 ≤250ms 重置计数。Python 与 native 同步计数，连续计数跨 ACTIVE/BRAKING/HOLDING 保留，冷接合重置。
- 硬底线触发即撤销许可并闭锁；同步 SDK 调用无法中途取消，返回后串行 Leave → 慢停 → 新鲜反馈确认停稳。
- 热重接合保持连续 5 次 ≤25ms 发送、State Age ≤150ms、Joint Age ≤80ms、距保持点 ≤0.1°，并保留连续静止确认和精确首帧保持点。按住 Grip 时状态帧暂时不达标会留在 HOLDING；新状态达到门槛后，用最新手柄位姿重新锚定再自动接合。
- 冷 PRIMING 仍为 2.5s 总时限、400ms 调用/年龄边界、12 帧及连续 5 次 ≤25ms，准入 State ≤100ms / Joint ≤60ms。Follow 和旧 J6 路径仍使用原 200ms 反馈门槛。
- 主机轨迹保持 10°/s、30°/s²、300°/s³，dt 推进最多 50ms；本次采用 dt 封顶持续推进，没有启用可选的 ACTIVE >250ms 暂停轨迹功能。
- VR shadow 的反馈门槛与透传 Joint 300ms 对齐；启动要求 native 导出 `supports_graded_hot_feedback`，避免新 Python 与旧 native 混用。
- 验收脚本按 State 500ms / Joint 300ms 判定，并继续要求同一源码/native 的 A 通过报告才能运行 ±0.8° 正弦、五次非零参考速度松手制动。

## 软件验证

- 本机 AUBO 完整回归 **549 项通过**；本机与车载 native 模拟各 **31 项通过**。
- 车载热离合 Python **14 项通过**，新增实际 C++ 绑定分级保护 **4 项通过**。
- 覆盖：201ms 瞬态反馈保留许可、State 499ms / Joint 290ms 放行、State 501ms / Joint 301ms 拒发、单次 >500ms 调用、连续慢调用计数与复位、跨热阶段计数、硬故障锁存。
- 实际 runtime XR 路径覆盖：按住 Grip 且 State Age 180ms 时留在 HOLDING；状态刷新后自动重接合；离合期间不运行 IK，热接合首帧精确保持点。
- 两端无连接绑定检查、native 制动/透传生命周期检查及 Python/Shell 语法检查通过。软件检查不代表实机运动验收通过。

## 实机结果

第一次运行在 prepare 阶段发现机械臂未上电：只读诊断 `power_on=false`、`pose_read_enabled=false`，六轴为零；未 Enter、未发送目标。用户随后明确回复“已使能，可以继续”。此设备未就绪记录保留在 `stage-A/`。

恢复使能后，用相同候选源码/native 执行 `stage-A-ready/`：

| 指标 | 结果 |
| --- | ---: |
| PRIMING 时长 / 帧数 | 0.928s / 22 |
| ACTIVE 至硬保护 | 4.634s，未达到 5s |
| ACTIVE 调用数 | 219 |
| A P50 / P95 / Max | 17.410 / 32.898 / 333.935ms |
| 首次硬保护 Joint Age | **302.580ms** |
| 同时 State Age | 318.892ms，低于 500ms |
| 当时 A 在途年龄 | 约 290.923ms |
| 该次 A 最终耗时 | 333.935ms，返回记录已被撤销许可 |
| Leave / 慢停调用 | 17.455 / 255.006ms，均 rc=0 |
| 保护触发至停稳确认 | 613.385ms |
| 相对初始点最大关节变化 | 0.000701° |
| 最终停止结果 | stop_confirmed=true；stop_result_unknown=false |

该窗口此前一次 **266.591ms** A 发送成功返回，`revoked_during_call=false`，慢调用连续计数为 1，随后正常调用清零。这证明单次 >250ms 调用已按新规则容忍。最终触发来自 C 原始反馈接收时间戳的 **Joint Age 302.580ms**，并非 State 200ms 门槛，也不是连续三次慢调用。

验收异常保留 fatal 锁存；没有清锁、自动重复运动、进一步放宽阈值或开启自由 VR。默认清单仍指向先前版本。A/B/C 均正常登出，现场没有遥操或验收进程、没有残留 SDK 连接。不能把这轮报告写为 5s 静止通过或五次运动制动通过。

## 证据

本机 `/data/VR_teleoperation_runtime/hot-graded-20261112/`；车载沿用既有 `/home/xc/code/lcw/VR_teleoperation_runtime/hot-graded-20261112/`。新构建与记录均在这些运行目录，未迁移既有项目/SDK。

`summary.json` / `analyze_acceptance.py` 保存统计；`stage-A-ready/report.json` 保存原始事件、状态/反馈时间戳、native 清单及源码哈希；`readiness-blocker.json` 保存未使能时的只读结果；两端测试日志保留各自结果。车载修改前源码备份为 `pre-change-vehicle.tar.gz`；C-1 启动器已准备，要求同一源码和 native 的 A/CLUTCH 两份报告通过后才允许运行。

原始实机证据包 `evidence.tar.gz` SHA256：`2d1a83e0024cacffc8c81a16e12791b237cf453761211d3be611aab2a14e71e4`。
