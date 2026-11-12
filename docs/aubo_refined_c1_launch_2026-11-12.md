# refined-stream 晋升及C-1启动

> 当前版本已由用户确认为[真机基线 aubo-c2-refined90-2026-11-12](aubo_teleop_baseline_2026-11-12.md)。本文保留该轮实验/诊断的原始时序与状态；“待验收”“当前未启动”等措辞属于当时记录，不覆盖后续基线结论。

2026-11-12，北京时间。用户明确批准晋升已通过受控程序验收的refined-stream，并启动真实Quest 3自由离合遥操。C-1处于等待用户操作阶段，不标记为真实VR验收通过。

车载默认 `follow-build.json` 已通过同目录临时文件、flush/fsync和os.replace原子切换。模块仍为已验收的 `refined-stream-20261112/build-follow/python/_aubo_follow.cpython-310-x86_64-linux-gnu.so`，未重新编译或替换控制模块。切换前清单备份在该版本目录 `default-follow-build-before-promotion.json`。晋升后默认清单SHA256：`baeb2656719e068719f8b70dd0c8d69e3001c243d0f45354ba7272293d609f73`。

`qualification.json.execution_quality_qualified=true`，候选清单 `refined_stream_candidate.default_enabled=true`、`execution_quality_qualified=true`。程序验收、现场平顺性确认及原始证据保持；`c1_vr_acceptance_passed=false` 等待实测。

VR入口现在读取该默认配置，自动选择fixed-tick、wall-clock、refined-stream、A/B/C三连接和NODELAY。A首次登录后立即开启并核对NODELAY，随后才登录B/C，避免socket识别歧义。生产worker接收12点名义预充、2点触发、单批最多8点、入模暂停B及欠供零速恢复配置，复用已验收控制实现。仅VR启动集成代码改变；已核验follow.py、trajectory.py、state_query.py、native/follow.cpp及passthrough.hpp与通过程序验收时的哈希相符。

本地完整628项回归通过，两端启动专项各27项通过，包含默认清单选择、未合格清单拒绝、A NODELAY先于B/C登录、worker参数传递及清理。原用户选择的world映射、Grip离合和重新建锚规则保持。

## 运行信息

- PID：**19031**。
- 启动：**2026-11-12 16:58:19（Asia/Shanghai）**，有限1800秒，约17:28受控退出。
- Quest USB serial：`2G97C5ZJ3002SP`；`adb reverse tcp:8443 tcp:8443`设置并回读成功。
- 头显页面：**http://localhost:8443/**；服务HTTP 200。
- 状态页面：`http://localhost:8443/aubo`，JSON接口 `/aubo/status`。
- 就绪快照：idle、fatal=false、fault=null、feedback_ready=true、stop_result_unknown=false、submitted=0、require_release=true、motion_available=true、sdk_connections=3；refined_stream=true、wall_clock_batch=true、名义预充12点。诊断流实测A NODELAY=1。
- 参考上限保持 **10°/s、30°/s²、300°/s³**，启动器逐项核验。没有程序运动、上电或回零命令，等待用户新鲜Grip松开/按下。
- 只保存诊断流，未开启数据集录制、相机或夹爪控制。

车载日志和快照目录：`/home/xc/code/lcw/VR_teleoperation_runtime/refined-stream-20261112/C1-20261112-165819/`，含 `run-metadata.json`、`diagnostics.jsonl`、`startup-status.json`。父目录 `C1-launch.log`、`C1-process.json`、`latest-C1.json`定位本会话。新大文件存储按既有车载目录运行，本机归档继续使用 `/data/VR_teleoperation_runtime/refined-stream-20261112/`。

## C-2准备

两端保存 `C2-plan.json`，当前enabled=false。用户确认C-1多轮离合与连续六轴运动平顺、无异常声音后，再逐档评估15/45/400、20/60/500、25/75/600（速度/加速度/jerk）。每档需同步核对native单步上限、制动收敛、供点/欠供与跟踪情况；这些档位尚未应用到运行进程。
