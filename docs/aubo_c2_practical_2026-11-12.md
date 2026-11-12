# C-2 practical 档实现、自检与启动

> 当前版本已由用户确认为[真机基线 aubo-c2-refined90-2026-11-12](aubo_teleop_baseline_2026-11-12.md)。本文保留该轮实验/诊断的原始时序与状态；“待验收”“当前未启动”等措辞属于当时记录，不覆盖后续基线结论。

2026-11-12，用户明确要求默认25°/s、75°/s²、600°/s³，保留10/30/300及35/105/900参数切换；自检通过后启动真实USB WebXR。C-1原构建与28次热接合验收记录保留。

## 实现

新增 teleop/aubo/motion_profile.py 定义conservative、practical、advanced三档。JointTrajectory每实例选择限速，不修改类级全局默认。worker将同一档参数交给native的configure_motion_limits，native仅允许未arm、未入透传时配置白名单档位。批内、跨批步长检查统一采用v_max×5ms+浮点裕量：0.05°、0.125°、0.175°。CLI `--motion-profile conservative|practical|advanced`，未指定时从follow-build.json的motion_profile读取；35档仅离线验证，未实际启用。

指定档位采用16点/80ms名义垫，单批上限8，静止积欠截断至2点，运动重起步仍用2～4点。运动欠供必须预充完成、净积欠>0且距上一批起点>80ms；70ms容忍与90ms触发有native回归覆盖。这是主机时基估计，未把它当实时FIFO读数。首次PRIMING、500ms反馈/A调用硬底线和连续三次A>250ms闭锁不变。

制动动态参考/反馈差包络按3°×v/10缩放，25档7.5°；最终HOLDING≤0.1°保持。25档实机曾验证原3°门槛不足，未通过轮次全部保留。

## 未通过轮次与关键修复

- high-clutch：完成第一次约24°/s制动后，第二次测试正弦入口不能找到符合导数限制的五次曲线，已确认停稳。测试源改为每次停止后通过限速曲线回中心，再入下一轮正弦；生产VR规划器未因此改变。
- high-clutch-certified-entry：刚进入第一次BRAKING时触发原3°动态差保护。最大单步0.119998°，触发前参考与反馈相差约3.0006°；不是旧0.05°步长误拦截。按用户授权适配到7.5°。
- high-clutch-scaled：PRIMING时C反馈超过500ms，未进入运动即退出并确认停止；不放宽硬底线。
- high-clutch-final：前三轮完成，第4次制动触及7.5°。发现static→moving的epoch重置丢失已有名义信用，重复补垫可导致多轮前瞻累积。去掉跨阶段重置，首次小垫/恢复以外保持同一账本；静止截断只丢弃积欠，不凭状态转换增发一份垫。新增20次跨状态epoch守恒回归。没有继续扩大7.5°。

## 最终构建验证

本机632项AUBO测试通过；车端22项motion-profile专项通过；native lifecycle/profile顺序测试通过。新测试覆盖三档批内与跨批步长的允许/拒绝、运行中禁止换档、70/90ms缺供边界、25档峰速/加速度与整条制动曲线导数界、静止0.1°独立保留、反复测试入场及跨阶段单账本信用守恒。

最终stage-A-ledger：5.000878s通过。high-clutch-ledger：31.128123s完成1次冷启动、5次运动中制动、5次热接合及最终回程。J1正弦±8°，参考峰速23.999942°/s，参考最大加速度71.999999°/s²，五次松手23.986～24.000°/s。最大相邻步长0.119998°<0.125°，最大参考/反馈差3.143703°。最大方向性越界和HOLDING准入偏差均0.002197591°；五次释放至保持0.785～0.833s。运动欠供重起步0次，无硬闭锁，退出/停稳确认成功。该轮支持受控程序客观指标通过，不能代替VR手感、音频或电机导数实测。

原始报告车端 `/home/xc/code/lcw/VR_teleoperation_runtime/c2-practical-20261112/`，本机最终报告及分析 `/data/VR_teleoperation_runtime/c2-practical-20261112/`。selfcheck-analysis.json含指标，c2-five-clutches.png显示反馈与提交参考及调用间隔。先前失败原始报告留在车端，不覆盖。

## 默认构建与C-2会话

北京时间18:53:43原子切换follow-build.json至c2-practical-20261112。旧默认备份位于新构建default-follow-build-before-promotion.json。native SHA256 f95a895a51f6a34ca9601e2fac812bcc0c160b5ea9ae03c3e4c45aaabe522864。manifest记录practical、三档参数、16点、7.5°动态包络、0.1°静止容差；qualification记录controlled_selfcheck_passed=true、c2_vr_acceptance_passed=false、execution_quality_qualified=false，允许用户明确请求的监督试验，不冒充VR已通过。

C-2 PID27248，启动于18:53:43之后，1800秒有限会话，约19:24结束。日志C2-launch.log，latest-C2.json定位当轮目录。就绪状态idle、fault=null、fatal=false、feedback_ready=true、submitted=0、immediate_hot_clutch=true、profile=25/75/600、step=0.125°、cushion=16。A NODELAY和USB8443映射核对，守护PID27352只随本轮owner存活恢复映射，owner退出即结束。头显打开 http://127.0.0.1:8443/?c2=25 。启动不发送程序运动；人工Start Teleop、释放后Grip接合。保持作者world映射，未开启数据集录制或夹爪控制。
