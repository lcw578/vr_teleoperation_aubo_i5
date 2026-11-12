# C-2：90ms对齐缓冲、精确规划与两档实机验收

> 当前版本已由用户确认为[真机基线 aubo-c2-refined90-2026-11-12](aubo_teleop_baseline_2026-11-12.md)。本文保留该轮实验/诊断的原始时序与状态；“待验收”“当前未启动”等措辞属于当时记录，不覆盖后续基线结论。

2026-11-12，按用户要求消除50ms垫/90ms恢复的未保护区间。精确多项式规划器保留；新候选`c2-aligned90-20261112`。所有大文件在本机`/data/VR_teleoperation_runtime/c2-aligned90-20261112`，车端沿用既有`/home/xc/code/lcw/VR_teleoperation_runtime/c2-aligned90-20261112`。

## 修改与回归

显式运动档默认18点/90ms垫，2点触发、单批最多8点；Python与Native均拒绝垫和恢复阈值不相等的配置（支持8~20点，兼容原12点路径）。预充完成后只要墙钟净欠供>0就触发零速重起步，移除了“还必须上一调用间隔>阈值”的额外条件，因此连续短慢调用也不能逃过检查。HOLDING/静止仍丢弃账本积欠、继续同一保持点。热转换沿用同一epoch，不增发第二份缓冲。

冷预充18点拆批；恢复沿用2点起步、最多4点小批，逐步建立18点垫。预充完成前不按稳态欠供二次恢复。Native在send_batch的SDK提交前再次检查净欠供，处理Python采样期间缓冲过期的竞态；SDK未调用时Python丢弃未发送的采样，回到Native最后已提交点q_last_sent，v=a=0，重新规划。

643项AUBO回归通过，车端27项motion-profile/Native专项通过，实际SDK绑定编译及Native生命周期检查通过，git diff --check通过。故障注入用生产C++绑定+离线SDK double，覆盖105ms单空窗、多个65ms空窗积欠、计划后提交前过期、延迟部分预充、恢复小批重建、四档批内/跨批步长、拒绝10点垫/18点阈值。没有在实体高速机械臂上人为注入断流；物理零速恢复仍为偶发事件处理，不能把离线注入冒充实机故障验收。

500ms A/C硬底线、连续三次A>250ms闭锁、B透传期暂停、A NODELAY、即时热Grip均保留。25档动态参考/反馈包络7.5°、45档13.5°，最终HOLDING静止准入始终0.1°。

## 两档受控验收

两档先各做5秒Stage A，再各做五次非零速度制动/热接合，同源码、同Native、同manifest。25档J1±8°/24°/s，45档J1±12°/43.2°/s（避免±8°下正弦加速度超过180）。全部退出后stop_confirmed=true，无fatal或unknown。

| 指标 | 25/75/600 | 45/180/1800 |
| --- | ---: | ---: |
| Stage A静止窗口 | 5.000296s | 5.001048s |
| 五次热离合程序窗口 | 26.798426s | 22.794519s |
| 参考峰速 | 23.999942°/s | 43.199855°/s |
| 参考峰加速度 | 73.651694°/s² | 172.027909°/s² |
| 最大相邻步长 | 0.119998° / 上限0.125° | 0.215996° / 上限0.225° |
| 最大批起点间隔 | 57.236638ms | 55.319837ms |
| 制动方向越界最大值 | 0.003750° | 0.009610° |
| HOLDING准入偏差最大值 | 0.003394° | 0.007507° |
| 稳态净欠供批次仍提交 | 0 | 0 |
| 欠供零速重起步 | 0 | 0 |

25档用户反馈“很平滑，没啥问题”。45档先反馈“感觉还行吧”，补充确认“已消失，运动和回程均可接受”（针对咔哒声）。据此受控执行质量合格。不能仅凭小于90ms间隔或主机账本断言真实FIFO绝无排空；未进行FIFO实时仪器测量，physical_fifo_certified=false。

原始report.json保留程序有限指标，分析selfcheck-analysis.json与现场确认写qualification.json。25-high-clutch/c2-five-clutches.png、45-high-clutch/c2-five-clutches.png为参考/反馈及批间隔图。

## 离线规划时间

参考q/v/a初始为零，整段端点+导数实根检查，含1%时长裕量。5ms最终tick向上取整不超过额外5ms。

| 位移 | 旧凸包25档 | 精确25档 | 精确45档 |
| --- | ---: | ---: | ---: |
| 5° | 1.380154s | 0.801638s | 0.555824s |
| 10° | 2.384905s | 1.010000s | 0.700295s |
| 20° | 4.121117s | 1.515000s | 0.882316s |

短位移受jerk限制不会全达到45°/s；新45档对应峰速16.8668/26.7744/42.5018°/s。planning-table.json与planning-before-after.png保留独立复现结果。

## 默认构建和VR启动

两档客观指标及现场确认后，原子切换根follow-build.json到新候选，旧manifest备份`default-follow-build-before-promotion.json`。Native SHA256 `f836d7bd0583a5919598e059dcc14c33475fe9d3f6cab04cd89f1e80f69276c7`。practical45/180/1800默认；conservative10/30/300、balanced25/75/600、advanced35/105/900保留CLI切换。qualification中controlled_selfcheck_passed/execution_quality_qualified=true；c2_vr_acceptance_passed=false，等待本轮自由VR体验，不能继承程序通过当作VR通过。

北京时间20:19:42启动有限1800秒人工Grip会话，PID8321，约20:49:43结束。无自动程序运动、无自动上电、无自动点击Start Teleop；尚未提交运动点。USB序列2G97C5ZJ3002SP，adb reverse tcp:8443 tcp:8443就绪；守护PID8542只在本轮owner存活时恢复映射。网页`http://127.0.0.1:8443/`，头显旧页面已连到relay。启动后hardware IDLE、fatal=false、feedback_ready=true、profile45、step0.225、cushion18、gap90ms；无右手柄帧时页面require_release=true，可能保留hardware_observation_pending提示。戴好头显、页面Start Teleop、完全松开右Grip后人工捏合接合。未启用数据集录制、回家、放置或物理夹爪控制。

## 20:31 急停后会话恢复

用户反馈误按急停，要求恢复，并表示新手感丝滑很多。旧会话软件锁存hot_braking_settle_timeout；20:30核对stop_confirmed=true、stop_result_unknown=false后正常SIGTERM退出PID8321，保留emergency-stop-status-before-restart.json和原始诊断。无遗留SDK连接后，只读诊断power_on=true、collision=false、joint_error=false、buffer_data_raw=0。未发上电/解除急停命令，未修改保护或运动参数。

20:31:07重启原验收配置C2，PID9174（1800秒，约21:01:08结束）。启动prepare检查包含softEmergency/remoteEmergency，成功；状态fault=null、IDLE、fatal=false、feedback_ready=true、submitted=0、require_release=true，等待右手柄帧。USB reverse8443核对通过；新owner映射守护PID9279。用户需完全松开右Grip再捏合，不恢复旧目标、不自动接合。车端保存emergency-stop-recovery-status.json。此次用户手感认可未据此将自由VR45整体验收改为通过。
