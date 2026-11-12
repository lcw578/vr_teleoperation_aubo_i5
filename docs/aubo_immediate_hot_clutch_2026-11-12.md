# 即时热离合修改与 C-1 启动（2026-11-12）

> 当前版本已由用户确认为[真机基线 aubo-c2-refined90-2026-11-12](aubo_teleop_baseline_2026-11-12.md)。本文保留该轮实验/诊断的原始时序与状态；“待验收”“当前未启动”等措辞属于当时记录，不覆盖后续基线结论。

用户明确要求回到直接 Grip 交互：热会话中允许 BRAKING/HOLDING 立即重新建锚，取消快帧/120ms反馈/100ms静止窗口作为热准入条件；保留首次冷启动 PRIMING 和 500ms 硬保护。此次车载默认构建为 refined-immediate-20261112，原 refined-stream-20261112 的二进制及证据保留。

## 改动

- 静止参考（HOLDING 或参考 v/a 均为零）在 due>4 时将账本截至2点，不进入欠供锁定或静止轮询。进入运动后用小批量建立名义缓冲。
- 非零运动参考的恢复判定同时要求预充已完成、扣除名义垫后的净积欠>0、距上一批发送起点>60ms。名义预充账面不足不再单独触发恢复。这是主机估算，未声称读取到了实时柜端FIFO排空证据。
- 恢复立即在最后提交的 q 上将主机参考 v/a 清零；首批2点，恢复预充期每批最多4点，逐步将名义垫从2增加至12点。没有100ms静止等待，也不把主机参考清零描述为已测得电机静止。C原始反馈有效性及500ms硬底线继续由native核对。
- 热接合允许从 BRAKING/HOLDING 直接进入 ACTIVE。运行线程在同一参考锁下捕获 q_cmd，回调重新建立手柄锚点，保留当前 q/v/a；取消旧制动意图，后续五次曲线从该状态接续。热接合仍要求有效新鲜手柄输入、已有有效C反馈≤500ms、无停止/硬故障。精确原子建锚避免在采样线程推进前后拿到不同参考位置。
- 旧非refined诊断路径保留原热准入策略；当前默认 refined 使用即时策略。HUD v9 在制动中明确提示可再次捏住Grip，保持中不再显示旧快帧门槛。

## 验证与部署

本机610项AUBO回归通过，车端25项原生/worker测试与4项启动测试通过；HUD和XR输入门控Node测试通过，git diff --check通过。新增测试覆盖静止90ms间隔、慢8点首包的部分预充、零等待小批量重起步、500ms反馈拒绝、减速中非零q/v/a连续性，以及真实作者核心/手柄输入下连续12次快速捏放再接合。native测试使用fake SDK，不发送真机运动。

新native SHA256：94eb6f3d83fb0d93f172dec00c2683f5cc953799e1d934aa9e7cee3882a22989。默认清单于北京时间17:58:51原子切换；旧清单备份在新构建 default-follow-build-before-promotion.json。新qualification明确 regression_passed=true、c1_vr_acceptance_passed=false、execution_quality_qualified=false，使用 supervised_c1_trial_ready=true 启动用户本轮明确要求的实测，不沿用旧程序通过结论冒充新版本已通过真机验收。

C-1 进程PID22830，于北京时间17:59:11启动，1800秒有限会话（约18:29结束），无程序启动运动，等待人工Grip。输出：车端 /home/xc/code/lcw/VR_teleoperation_runtime/refined-immediate-20261112/C1-20261112-175911。就绪快照 idle、fatal=false、fault=null、feedback_ready=true、stop_result_unknown=false、submitted=0、immediate_hot_clutch=true。USB映射 tcp:8443→tcp:8443 已回读，头显网址 http://localhost:8443/。参考上限仍为10°/s、30°/s²、300°/s³，C-2未开启。

后续按本轮真实VR记录核验 Grip按下到hot_clutch_resumed的时间、制动中接合连续性、静止账本截断与运动恢复事件；现场手感与异响仍待用户实测确认。

## C-1 现场确认与结束

北京时间18:23之后，用户反馈“可以了，感觉没啥问题”。本轮实际运动窗口131.715s，9529次发送，1次冷预热、29次松手制动、28次热重接合；Grip主机接收到模式接合耗时P50=3.439ms、P95=4.916ms、最大4.999ms。六轴命令累计活动范围分别约44.28°、39.06°、49.71°、34.59°、31.57°、29.83°，不代表某个时刻的偏差。HOLDING的113次静止账本截断没有造成旧式恢复自锁。仍有5次运动参考欠供零速恢复，不能宣称零欠供。所有28次重接合记录中的参考速度为零；非零速度制动中重接合本轮仅有离线覆盖，不能扩大成实机认证。用户没有单独对声音作陈述，本次只记录其总体体验反馈。

正常松手后处于HOLDING，随后输入丢失请求停止；停止确认成功，无fatal或未知停止。读取该状态后以SIGINT结束本轮，最终idle、fatal=false、stop_confirmed=true、stop_result_unknown=false；SDK三连接全部退出。与本轮owner绑定的USB映射恢复守护也已结束，网页服务随会话关闭。

车载qualification与默认清单已登记C-1在10/30/300保守档的监督遥操与热Grip响应验收通过，默认仍为refined-immediate-20261112，C-2=false。本机摘要：/data/VR_teleoperation_runtime/refined-immediate-20261112/C1-20261112-175911/C1-acceptance-summary.json；完整原始诊断留在车载同名会话目录，摘要包含原始文件SHA256。C-2仍需另起有限会话按档位验证，未修改当前速度上限。
