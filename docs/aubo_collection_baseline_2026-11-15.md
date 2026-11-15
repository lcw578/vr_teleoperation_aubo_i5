# AUBO 番茄串数采基线与验收记录

更新日期：2026-11-15。本文定义当前能复现的采集和数据处理基线，注明证据与未完成验证。遥操控制的45/180/1800参数基线仍以[aubo-c2-refined90](aubo_teleop_baseline_2026-11-12.md)为准。

## 当前状态

| 项目 | 当前实现 / 验收 | 边界 |
|---|---|---|
| 采样协议 | 30 Hz、7D实测状态与7D绝对目标；L515/L515/D405以30 fps采集 | 视频与动作时间轴按主机 monotonic 接收时间因果对齐，不是硬件曝光同步 |
| 机械臂 | 已验收透传基线；底层固定tick目标为5 ms、批量路点1–18点、45°/s、180°/s²、1800°/s³ | 5 ms是控制tick/调度目标，不代表RPC或实体反馈每5 ms稳定一次 |
| 相机采集 | 三路并发实录、AVI解码和LeRobot视频均已通过；每路60帧有界队列 | 同进程Python线程。没有证明GIL、编码或磁盘I/O不会增加控制抖动 |
| 数据闭环 | 一条经审计恢复的业务轨迹导出为1047帧、1个v3 episode，真实LeRobotDataset逐帧校验通过 | 格式/对齐通过不表示原始tick全接收，更不表示采摘成功 |
| π0.5 | 30 Hz、50步动作chunk、q01/q99、224输入及轻量投影/loss接口已冒烟 | 完整Policy.forward、PaliGemma/Action Expert Transformer、训练与真机推理未验证；尾部padding未被本地原生loss屏蔽 |

业务任务提示词统一为：**采摘下画面里面所有的成熟番茄串并放入收集筐**。第一版采集范围是单串完整的接近、对准、夹剪、撤出、搬运、释放，可包含原地失败重试。每条episode应只覆盖一个业务目标；左Y安全回位是单独的自动动作，不能混入业务示范。

## 设备与部署

当前相机配置见[`aubo_cameras.example.json`](../environment/aubo_cameras.example.json)：`scene_left` L515 `f1150496`、`scene_right` L515 `f1422449`，均1280×720 RGB@30；`wrist` D405 `230322274449`，640×480 RGB@30。配置支持这三个键的任意非空子集，并可用`enabled:false`禁用设备。连接后仍须现场核对序列号和视角；当前联调位置下右L515主要看墙面，wrist闭合时未清楚显示剪口，因此还没有最终安装后的可见性验收。

车端Wi-Fi地址会随DHCP变化。最近确认的地址是`10.34.196.164`，SSH账号`xc`；控制柜有线侧地址为`192.168.1.5`，车端有线口为`192.168.1.100`。先从笔记本检查当前地址/连通性再SSH，不要把历史DHCP地址当成永久地址。车辆已有工程目录是`/home/xc/code/lcw/VR_teleoperation`；运行时数据优先落`/data/VR_teleoperation_runtime`，若车端无`/data`则复用已有`/home/xc/code/lcw/VR_teleoperation_runtime`。不复制或迁移大型录制到家目录。

车载Python 3.10无LeRobot。在线遥操使用车端VR环境及已安装RealSense Python模块，离线导出和检查在有LeRobot 0.6.2的`/data/lerobot/.venv/bin/python`环境执行。不要在在线机械臂控制进程导入LeRobot/Torch来做离线视频处理。头显USB反向端口为8443，页面为`http://localhost:8443/`。

## 在线录制与操作

在车端原有checkout执行，使用实际业务task和三相机配置路径：

```bash
cd /home/xc/code/lcw/VR_teleoperation
TELEOP_RUNTIME_ROOT=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_RUNTIME_DIR=/home/xc/code/lcw/VR_teleoperation_runtime \
TELEOP_EXTRA_PYTHONPATH=/home/xc/.local/lib/python3.10/site-packages \
bash scripts/aubo_passthrough.sh --execute --headless --duration 1800 \
  --task "采摘下画面里面所有的成熟番茄串并放入收集筐" \
  --record-cameras /home/xc/code/lcw/VR_teleoperation_runtime/config/aubo-cameras.json \
  --diagnostic-output /home/xc/code/lcw/VR_teleoperation_runtime/diagnostics/SESSION.jsonl
```

启动器会检查设备/SDK并建立Quest USB反向端口；SDK密码在终端交互提示输入，不应写入命令行、配置或文档。先确认机械臂、夹剪、急停、视角和工作区安全，再开始录制。READY时相机已开始采集预热，但30 Hz业务快照在episode开始后才记入样本。

| 按键 | 当前语义 |
|---|---|
| 右Grip | 遥操离合。松开后制动；raw相机、状态与工具记录连续，标注`clutch_reanchor`；再次接合时重新锚定 |
| 右A | 按住时平移和旋转均为0.5倍，松开恢复；可辅助精调 |
| 右Trigger | 边沿切换夹剪的绝对开/闭目标；不是按住持续控制。Trigger事件本身不代表工具到位或夹持成功 |
| 右摇杆按下 | 添加带`mark_id`和单调时戳的事件；头显状态卡确认已入队。不会在线改写`task_stage` |
| 左摇杆向右 | 保存并结束当前业务episode，不自动开始下一条 |
| 左摇杆向左 | 作废当前episode并重启循环。误操作可能使有效业务段标成invalid |
| 左摇杆向上 | 保存当前episode并结束Session |
| 左Y | 有效home reference下单击开始回安全原位，再按取消；通过原透传规划/发送路径运行并记为`automatic_motion`。建议在业务段保存后单独执行 |

需等头显或状态接口确认episode已保存，再开始下一条。单次Session中多条episode要逐条确认保存状态；结束后检查`session_status.json`、episode摘要及相机status文件。按钮具体状态及安全细节见[多模态采集操作说明](aubo_multimodal_recording.md#正式录制手柄按键2026-11-15)。

## 数据语义与异步结构

Raw schema为`aubo_multimodal_v2`。30 Hz采样由Quest遥操`step()`主循环按时限触发，不是单独SNAP线程；读取已有反馈缓存和规划器/工具状态，不等待相机帧或串口I/O。在线状态为当前6轴实测角(rad)+工具实测位置归一值；action为真正准入平滑规划器的6轴绝对目标(rad)+工具绝对开闭目标（0=open,1=closed）。不从旧单点`sdk_command`回执拼动作。录像保留XR、控制诊断、阶段/暂停/生命周期、相机索引与原始记录。

每台相机有独立捕获线程和AVI/索引写盘线程，视频编码调用OpenCV；这些线程与主控制循环在**同一个Python进程**。每路捕获到写入间使用容量60帧队列（30 fps约2秒）；队列满会记`capture_queue_overflow`并让Session报错，不能当作无损完成。相机索引在关闭时flush/fsync；状态流在episode结束确认保存、Session结束时fsync，普通循环仅flush。此设计有队列吸收短暂磁盘抖动，但Python GIL和同步编码调用仍可能影响调度，三相机并发长期控制抖动尚待现场测量。录制器/相机写入错误可能撤销操控许可并触发停止保护。

暂停样本仍连续记录并带`pause_type`，例如`none`、`clutch_reanchor`、`task_wait`、`automatic_motion`、`operator_pause`及`input_or_system`。规划器准入目标缺失、状态不安全、反馈过期等会在离线拒绝流中明确列出。

## 导出、检查与阶段标注

```bash
bash scripts/aubo_export_lerobot.sh \
  --input /home/xc/code/lcw/VR_teleoperation_runtime/recordings/aubo/SESSION \
  --output /data/VR_teleoperation_runtime/datasets/DATASET \
  --repo-id local/DATASET --infer-task-stages

bash scripts/aubo_check_dataset.sh \
  --dataset /data/VR_teleoperation_runtime/datasets/DATASET
```

导出动态读取manifest的`active_cameras`，执行因果前值选帧及150 ms图像、200 ms关节、250 ms工具年龄门槛。LeRobot v3中Parquet为分片表、MP4为分片视频；校验器真实加载LeRobotDataset并解码所有帧。原始单调时间与alignment sidecar保留溯源。质量门槛拒绝造成的缺口会切分episode；阶段/暂停元数据筛选默认也切段。`--stitch-filtered`只压缩显式元数据筛选造成的时间轴间隔，必须有意选择过滤规则；它本身不会过滤任何内容。使用`--exclude-pause-type clutch_reanchor`可能把松Grip期间操作夹剪的释放样本一并删掉，导出前应检查工具事件。

`--infer-task-stages`启用`harvest_retry_v2`离线回溯：支持多次开闭，将前N−1个闭合/张开循环标为retry/align，末次闭合后标transport、末次张开标release；右A/mark可提供align/transport线索。在线mark不会改变快照stage；证据不足时标注assumed边界并留下警告。到位反馈不证明果梗剪断、果串夹稳或入筐，报告`physical_success_verified=false`，仍需人工看图审核。episode边界应按同一串作业定义，重置/自动回位单独保留或显式排除。

## 已完成的真实数据验收

原始Session为`20261115-163133-9413599e`。操作者确认Episode 0在结束时误触restart方向，原始记录因此保留为invalid；仅episode 1为自动回位。按用户授权生成了独立恢复副本，恢复episode 0、排除episode 1，未改原始车端目录或本地镜像。恢复副本附修改理由、原始/副本hash与字段记录。

| 结果 | 默认导出 | 过滤换握并Stitch |
|---|---:|---:|
| verified episodes / frames | 1 / 1155 | 1 / 1047 |
| clutch_reanchor帧 | 108 | 0 |
| 导出帧闭合/张开转换 | 1 / 1 | 1 / 1 |
| 相机丢帧队列溢出 | 原始帧号缺口0/0/28；队列0 | 同左 |

Stitch通过真实LeRobotDataset逐帧7D state/action及三路MP4解码检查，1047/1047输出帧通过。图像年龄P50/P95/Max(ms)：scene_left 16.01/27.79/111.35，scene_right 13.99/30.75/63.41，wrist 15.01/31.60/112.59；关节49.81/66.45/85.27，工具60.97/88.91/107.20；三视角接收时间spread 18.87/28.33/112.20。0图像/工具超龄拒绝。此处是输出格式质量，不是原始tick 100%接收：Stitch候选1325 tick中，1047保留、266由显式metadata过滤、另有规划器缺失/unsafe拒绝。D405原始设备帧号有28个缺口；队列溢出为0。

控制诊断流记录到2次active欠供恢复，均在张开附近。该有限轨迹及之前的透传验收没有证明三相机写盘下5 ms调度无抖动；部署前需要足时压测，检查RPC延迟尾部、欠供和停止行为。分辨率224模型预处理正确，但当前临时场景右相机偏向墙面，wrist剪口在此次样本中不可清晰确认，须在最终安装视角重做现场视觉验收。

恢复审计及完整分布报告在车端运行数据目录：`/data/VR_teleoperation_runtime/audits/20261115-163133-9413599e/recovery/REPORT.md`。PI0.5加载接口冒烟报告和预览图在`/data/VR_teleoperation_runtime/audits/pi05-smoke-20261115-163133/REPORT.md`及同目录。此等大文件与真实录制/数据集保存在运行数据盘，不进Git。

## π0.5冒烟检查与回放

审计环境`/data/lerobot/.venv`用50步`delta_timestamps`和batch=2读入clamp及episode末尾padding，视觉预处理到224×224，检查7D状态/动作及逐维有效q01/q99。按显存/依赖边界，没有完整PaliGemma/Action Expert checkpoint；轻量投影路径loss和梯度有限，但不是PI05Policy完整前向、模型质量或部署预测验证。另已用真实`lerobot_dataset_viz`工具验证本机可交互回放：

```bash
/data/lerobot/.venv/bin/python -m lerobot.scripts.lerobot_dataset_viz \
  --repo-id local/tomato_bunch_3cam_recovered_20261115_163133_stitch \
  --root /data/VR_teleoperation_runtime/datasets/tomato_bunch_3cam_recovered_20261115_163133_stitch \
  --episode-index 0 --batch-size 2 --num-workers 0
```

查看各camera面板和动作/状态时间序列；此工具是本地viewer，不执行机器人动作。也可直接查看LeRobot episode的MP4分片，但viewer便于同时核对三视角、动作和阶段。更换数据集时同步替换root与repo-id。

## 复现检查

```bash
# 项目离线AUBO Python回归
PYTHONPATH=. /data/VR_teleoperation_runtime/.venv/bin/python \
  -m unittest discover -s tests -p 'test_aubo_*.py'
node --test tests/test_recording_hud.cjs tests/test_xr_input_gate.cjs

# 合成相机和实际LeRobot v3 writer/readback闭环（不连接设备）
/data/lerobot/.venv/bin/python tests/smoke_aubo_recording_lerobot.py \
  --output /data/VR_teleoperation_runtime/audits/multimodal-smoke-$(date +%Y%m%d-%H%M%S)
```

合成测试通过不等于现场相机、机械臂与夹剪的并发资格。研究背景及旧结论沿革见[HarvestFlex对齐审查](harvestflex_alignment_review_2026-11-02.md)、[HarvestFlex精读](harvestflex_deep_review_2026-11-13.md)及[全流程数据计划](tomato_bunch_collection_and_pi05_plan_2026-11-12.md)；其历史现场状态以本文及当前实现为准。
