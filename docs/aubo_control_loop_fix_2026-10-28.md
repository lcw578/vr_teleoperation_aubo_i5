# 透传200Hz循环误停修正

本次只修改本项目Python协调层的循环调度、真实停顿检查和诊断，不修改作者滤波/离合/解耦IK、方向映射、原生透传接口、关节范围或设备/输入/反馈保护。不自动启动实体会话。

旧实现每轮将计划时刻增加5ms，累计落后100ms就报`preview_control_overrun`。连续6ms或8ms的工作即使没有一次100ms停顿，也会累计触发。因此不能将该原因理解为单次IK或SDK耗时超过100ms。

修正后错过的计划时点直接跳过，只计算下一次未过期的时点，不追赶旧周期，也不把累计落后作为故障。200Hz继续表示期望节拍，不表示实际可达频率。

原有100ms试验预算现在检查真实的单次循环工作时间和循环之间的间隔。恢复运行时先检查间隔，再刷新反馈和运行核心；刷新、IK、状态生成和显示后均检查耗时。`J6QuestRuntime.step()`还会在作者核心返回之后、向硬件工作线程提供候选之前检查本次耗时，阻止长时间IK返回后的目标重新获得发送机会。真实超时使用`preview_control_stall`，撤销目标并沿用原停止与重新接合规则。在途SDK请求仍无法被Python检查中途打断。

状态页`control_loop_timing`提供周期数、错过时点数、真实停顿数及最近/最大耗时：`feedback_s`、`runtime_step_s`、`status_s`、`display_s`、`work_s`、`idle_gap_s`。`runtime_step_s`包含核心与协调工作，不等同纯IK求解耗时。统计覆盖本次会话已完成循环，不含SDK线程耗时。循环结束后的状态发布/诊断时间会计入下一轮间隔。

显式启用`--diagnostic-output`时，已有有界内存诊断每秒增加一条`control_loop_timing`事件；真实停顿也记录事件。正常退出时与原关节诊断共同落盘。未启用该参数不写文件，数据集Logger仍关闭。

## 已完成会话证据

`/data/VR_teleoperation_runtime/aubo-passthrough-200hz-20261028-152650.jsonl`已正常落盘。完整文件有12条以`preview_control_overrun`为原因的`stop_requested`，另有1条`hardware_feedback_unavailable`。这次修正针对前一项，不能据此声称反馈问题也已解决。40次透传返回中最长SDK调用约222.686ms；现有串行SDK路径实际发送频率不能仅由5ms配置推定。

## 离线验证与现场复测

新增7项回归覆盖持续6/8/20ms工作不累计误停、5ms快速节拍、显示长停顿、调度挂起、反馈刷新长停顿、核心长停顿后不提供候选，以及诊断接入。`bash scripts/aubo_offline_check.sh`完整通过：252项Python测试、原生假设备生命周期检查、绑定和默认不连接入口检查。日志为`/data/VR_teleoperation_runtime/aubo-loop-timing-fix-20261028.log`。本轮没有调用设备或发送运动目标。现场复测仍需启动新进程；旧进程不热更新。

```bash
cd /home/lcw/VR_teleoperation
bash scripts/aubo_quest_follow.sh \
  --execute-six-axis --command-mode passthrough --waypoint-feedback \
  --host 192.168.1.5 --port 8899 --user aubo \
  --rotation world --duration 60 \
  --diagnostic-output "/data/VR_teleoperation_runtime/aubo-loop-fix-$(date +%Y%m%d-%H%M%S).jsonl"
```

头显入口为`https://10.245.247.27:8443/`（以当前电脑热点IP为准）。先松开Grip，再按住缓慢测试。检查是否仍中断；如中断，使用新的首个停止原因和分段耗时分析，不继续取消其他保护或把真实停顿归为累计误停。
