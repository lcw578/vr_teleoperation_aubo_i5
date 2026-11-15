# AUBO 番茄串多模态采集与 LeRobot v3

> 本文描述字段和详细操作。当前验收值、三相机并发风险、完整按键表、π0.5边界及数据集回放命令见[数采基线与验收记录](aubo_collection_baseline_2026-11-15.md)。

本入口实现 30 Hz 原始状态快照、6 轴规划器准入目标、夹剪绝对状态/目标及 1 至 3 路 30 fps RGB 的独立采集。策略训练和 RTC 不属于此入口。

## 相机配置

从 [示例 JSON](../environment/aubo_cameras.example.json) 复制为 `/data/VR_teleoperation_runtime/config/aubo-cameras.json`，将实际接入相机的序列号、分辨率与视角名核对无误。配置允许 `scene_left`、`scene_right`、`wrist` 中任意非空子集；也可保留某项并设置 `"enabled": false` 暂时禁用。启用相机必须为 RGB@30 fps 且序列号唯一。本次实际启用列表会写入 raw manifest 的 `active_cameras`。

采集程序由一个 owner 为每台相机打开一个 RealSense pipeline；原始 RGB 视频写入 `cameras/{scene_left,scene_right,wrist}.avi`，同名 JSONL 保存设备帧号、设备时戳域和主机单调时钟接收时间。每路相机有独立的 60 帧有界队列，30fps 下约可缓冲 2 秒；相机断开、写盘失败、缺配置和队列溢出都会使 raw session 标记不完整。示例配置下，两台 1280x720 L515 和一台 640x480 D405 的队列图像数据约占 369 MiB，不含编码器及临时拷贝。相机帧在控制线程之外采集，不跨设备等待同步。

`aubo-recording-writer` 在队列空闲时常规 `flush()`，只在 episode 结束确认保存和 session 关闭时对状态流执行强制 `fsync()`；episode 摘要与 session 状态文件仍以原子写入方式落盘。相机索引在相机 writer 关闭时 `flush/fsync`。

## 现场录制

现场只应在完成设备、线路与急停检查后使用原授权流程。透传启动器的新增参数为：

```bash
bash scripts/aubo_passthrough.sh --execute \
  --task "采摘下画面里面所有的成熟番茄串并放入收集筐" \
  --record-cameras /data/VR_teleoperation_runtime/config/aubo-cameras.json
```

此选项要求启用录制和夹剪工具；配置加载并核对无误后才初始化已启用的相机。三相机采集、导出和数据集逐帧校验已完成有限真实轨迹验收；控制欠供与最终安装视角仍需后续现场验收。遥操记录根目录使用`/data/VR_teleoperation_runtime/recordings/aubo/`新会话目录；车辆无`/data`时使用已存在的相邻运行时目录。视频数据直接写入所配置运行时，不应先落家目录再搬运。

episode 仍由既有按键开始、结束、保存或标无效。操作状态暂停、Grip 制动与重锚定不再停止原始流。录制样本继续带 `pause_type` 与 `task_stage`，episode 事件另写 pause/resume 边界。

### 正式录制手柄按键（2026-11-15）

- 左 Y 单击：加载有效原位文件后，先停止遥操并确认静止，再自动回到保存的六轴关节姿态；无需按住 Y 或 Grip。回位准备/预热/运动期间再次新按 Y 取消。完成或取消后需重新接合右 Grip 才能遥操。断连、输入超时、反馈故障和停止确认保护保留。
- 正式启动器固定传入 `--home-trigger oneshot`，默认读取运行目录 `config/aubo_home_20261029T091225103094Z.json`；可用 `--home-reference /绝对路径/home.json` 显式选择有效参考。没有有效参考时回位禁用，状态页显示原因。回位使用同一个 passthrough owner、静止预热和规划器，速度上限取当前 profile 与 20°/s 中较小值，加速度取当前 profile 与 6°/s² 中较小值，jerk 仍受当前 profile 约束；到位容差 0.15°，随后确认停稳。此动作是关节空间回位，不会反向重放采摘路径。
- 右 A 按住：平移与旋转倍率各为普通模式的 0.5，松开恢复，不累计减半。切换沿沿用核心的重新锚定，避免把既有手柄位移按新倍率重新解释。头显显示普通 1.0×／精细 0.5×。
- 右摇杆按下：原始 `operator_event` 写入 `command=mark` 与会话内 `mark_id`。头显在后台入队成功后显示最近标记编号和本段计时，绿色突出约三秒，之后保留编号；失败则显示“标记未保存”。这是入队确认，episode 保存确认才表示落盘。状态卡约 2 Hz 更新，与控制线程独立。

自动回位期间原始采集持续，并标记 `pause_type=automatic_motion` / `control_source=home_return`，可在离线导出时按需排除；原始 episode 不会因此自动结束。回位不会自动切换夹剪工具目标。

阶段标签通过本机遥操页可访问的接口写入，参数是下列枚举之一：

```text
select, approach, align, clamp_cut, withdraw, transport, release, verify, retry, reset, unknown
```

例如切换到搬运阶段：

```bash
curl -X POST http://127.0.0.1:8443/aubo/recording/stage/transport
```

`pause_type` 为 `none`、`clutch_reanchor`、`task_wait`、`automatic_motion`、`operator_pause` 或 `input_or_system`。Grip 松开/等待首次接合标记为 `clutch_reanchor`；夹剪与释放等待可调用 `pause_episode('tool_wait')`/`release_wait` 后持续采集。阶段和暂停信息是元数据，原始 episode、attempt 与生命周期事件仍保留。

## 动作与状态语义

每个 30 Hz tick 输出 7 维 `observation.state` 和 7 维 `action`：

| 维度 | `observation.state` | `action` |
|---|---|---|
| 1–6 | 当前 6 轴实测关节反馈，rad | 实际送入平滑规划器的 6 轴准入绝对目标，rad |
| 7 | 实测夹剪位置，按打开标定=0、闭合标定=1 归一化 | 同一编码下的绝对工具开/闭目标，`0=open, 1=closed` |

归一化所用开闭位置及工具配置写入 session manifest。工具故障、目标不明确或反馈过期时，不伪造样本。SDK 回执、5 ms 轨迹、TCP2CAN 批量路点、夹剪电流/扭矩等仍作为 raw 审计事件保留，但**不从旧 `sdk_command` 回执重建 LeRobot action**。

## 导出与筛选

退出采集、确认 raw `session_status.json` 完整后，以真实 LeRobot 0.6.2 环境离线导出：

```bash
bash scripts/aubo_export_lerobot.sh \
  --input /data/VR_teleoperation_runtime/recordings/aubo/SESSION \
  --output /data/VR_teleoperation_runtime/datasets/tomato_bunch_v001 \
  --repo-id local/tomato_bunch_v001
```

导出使用 LeRobotDataset v3 writer，并按 manifest `active_cameras` 创建图像视频 feature。只对本 session 启用的相机执行因果选帧和 150 ms freshness 检查，数据集 fps=30。所有成功保存的业务 episode、人工/placement/home 来源及搬运释放阶段都纳入候选；无效和未正常关闭的 episode 不导出。状态、工具和图像 freshness 默认阈值分别为 200、250、150 ms；超龄或无效 tick 切开片段并写进 `rejected_ticks.jsonl`。这些阈值是首版质量门槛，现场数据出来后应据分布审查。

默认不排除已标暂停类型。按阶段或暂停元数据筛选时，导出拆成多个 LeRobot episode，但 `alignment.jsonl` 保留各个片段原始 episode、帧和时间对应关系。必须显式 `--stitch-filtered` 才会压缩仅由该元数据过滤造成的时间缺口；每次拼接会记录到 segment discontinuities。滤掉时长较长的等待后，拼接数据集用于学习的物理时间含义可能改变，不能不经审查就作为默认训练版。

### 离线打标映射与自动回位过滤

导出的总任务提示词默认统一为 **采摘下画面里面所有的成熟番茄串并放入收集筐**。`--task` 可以显式覆盖；原始各 episode 提示词保存在 `export_report.raw_tasks`，不改写 raw。

`--infer-task-stages` 开启 `harvest_retry_v2` 离线注释协议，假设一条 raw episode 对应同一串番茄，允许任意次数原地重试，不再要求四次 mark，也不把多次开闭视为异常：

| 条件 | 阶段 / attempt_id |
|---|---|
| 开录至首次对准线索 | approach / 1（没有闭合尝试时为0） |
| 首次闭合前右 A（XR 按钮索引4）或 mark | align / 1 |
| 前 N−1 次闭合 | retry / 当前尝试编号 |
| 非末次张开至下次闭合 | align / 下一尝试编号 |
| 末次闭合 | clamp_cut / N |
| 末次闭合后的 mark，或到位反馈后关节相对移动≥0.02rad | transport / N |
| 末次张开 | release / N |
| 左 Y 回位生命周期 | reset / 0 |

只解析工具 `command_sent` 的绝对目标变化，去除重复目标命令；`trigger_toggle` 和 `target_reached` 不增加尝试次数。每条原始 episode 的 `attempt_id` 从1重新计数，写入 Parquet 的 `source.attempt_id`、alignment 与 rejected tick；先张开但无先前闭合、末次未张开等情况以不完整序列警告保留，不伪造最终释放。单纯关节回位不会跨 episode 拼接。

缺少 A/mark 时，首次闭合前最多1秒（且不超过此前时长一半）作为 **假设的** align 窗口；缺少闭合后 mark/运动证据时，若最终确有张开，transport 从闭合到位时刻开始，或从闭合后最多0.5秒（不超过闭合区间一半）开始。所有假设写入边界的 `assumed`、`inference_source` 和警告，不是识别出了真实接触或搬运。极短阶段可能没有30Hz帧，检查器不造帧凑阶段。已有非 unknown 在线标签优先保留，缺少的部分才填入推断结果。

“前面的循环是失败重试、最后的循环是最终释放”依赖完整 episode 的未来事件，是 **离线回溯标签**；它不改变动作和图像的因果前值对齐，也不能当作在线可用观测。夹剪到位只证明执行器到位，不能确认夹稳、剪断、原地重试或已放入筐；报告固定 `physical_success_verified=false`，有无果串、成功与否仍需人工/视觉核验。若同一 episode 包含不同果串，上述协议不适用，应先拆清任务边界。

示例：

```bash
bash scripts/aubo_export_lerobot.sh \
  --input /data/VR_teleoperation_runtime/recordings/aubo/SESSION \
  --output /data/VR_teleoperation_runtime/datasets/tomato_retry_test \
  --repo-id local/tomato_retry_test \
  --infer-task-stages --stitch-filtered \
  --exclude-pause-type automatic_motion --exclude-pause-type input_or_system
```

业务数据显式添加 `--exclude-pause-type automatic_motion` 排除左 Y 回位；按需添加 `--exclude-pause-type input_or_system`。`--stitch-filtered` 本身不自动排除任何 pause 类型。若松开 Grip 后仍操作工具，全面排除 `clutch_reanchor` 会连同有效释放记录一起丢弃；应先审查工具事件，再选择过滤规则，保留的 tick 仍必须满足全部状态、规划器、工具和图像检查。质量报告增加逐条导出 episode 的阶段/暂停/控制来源帧数及开闭转换次数，切段边界不计作工具转换。

`quality_report.retry_analysis` 按 raw episode 列出尝试数、重试数、命令循环数、最终释放候选、每次尝试的时间和导出帧覆盖；每条导出 episode 的 `attempt_ticks` 列出尝试帧数。检查器验证尝试序号、开闭先后、transport 所在闭合区间、Parquet 与 alignment / 推断边界一致性，不以“必须一闭一开”作为数据完整性规则；筛选后某次尝试零覆盖也会如实显示。

## 导出后校验

```bash
bash scripts/aubo_check_dataset.sh \
  --dataset /data/VR_teleoperation_runtime/datasets/tomato_bunch_v001
```

脚本用真实 `LeRobotDataset` 读取 Parquet 并逐帧解码所有 MP4。完成前验证 v3 标识、帧距、30 Hz、实际图像 feature 集合、七维状态/动作、与 planner 原始值一致性、时间先后、图像年龄、工具编码和对齐 sidecar。单路数据不计算多视角时间差。float64 monotonic 时间从 Parquet 原始值审计；不会用 Hugging Face 默认 float32 tensor 转换后的低精度值算毫秒年龄。

生成 `quality_report.json` 与 `tick_quality.jsonl`，包含启用相机的帧号缺口/队列丢帧、原视频可解码帧数、图像年龄 P50/P95/Max、状态/工具年龄、多视角选帧时间差（至少两路时）、每路已选独立帧和重复率、开闭目标转换统计、阶段/暂停行数，以及过滤原因。

合成离线验收（不连接任何设备）：

```bash
/data/lerobot/.venv/bin/python tests/smoke_aubo_recording_lerobot.py \
  --output /data/VR_teleoperation_runtime/multimodal-smoke-$(date +%Y%m%d-%H%M%S)
```

该测试生成三路和单路合成 RGB、搬运/释放阶段、工具事件和暂停标签，验证真实 v3 写入/读回、动态图像 feature、相机丢帧报告、暂停过滤切段和显式拼接；其成功不等于真机或 RealSense 联合验收。

详细实现约束见 [HarvestFlex 精读报告](harvestflex_deep_review_2026-11-13.md) 与 [完整采集方案](tomato_bunch_collection_and_pi05_plan_2026-11-12.md)。旧 [无相机导出说明](aubo_recording.md) 只描述 `aubo_state_action_v1` 历史 schema；它不能用于生成多模态训练集。

2026-11-15 单相机联调后，图像默认 freshness 从 100 ms 调整为 150 ms，容纳实测 114.56–124.28 ms 的孤立接收延迟；仍只使用 tick 之前已收到的帧，超过 150 ms 的质量缺口继续切段。可用 `--image-max-age-s 0.1` 复现旧标准。此选项只影响离线导出，不改变在线机械臂安全时限。

相机capture和AVI writer是每路独立的同进程Python线程，不是相机子进程；有限队列可以缓冲短暂I/O延迟，但没有实测证明它对5ms控制调度零影响。
