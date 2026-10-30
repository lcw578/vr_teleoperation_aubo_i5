# 透传接合后中断：现场只读排查

后续已按用户授权完成[循环修正](aubo_control_loop_fix_2026-10-28.md)。下文保留修正前排查时的状态；会话退出后完整诊断统计为12次累计超时停止、1次反馈不可用，详见修正文档。

本轮仅读取运行状态、检查代码、无SDK的时间循环复现；未修改运行代码/阈值，未重启或发送目标。现场进程为PID26860，六轴透传、world、期望200Hz、300秒诊断，启动文件`aubo-passthrough-200hz-20261028-152650.jsonl`。诊断仍在内存，未受控关闭前不从只有头部的JSONL推断所有中断。

状态快照保存于`/data/VR_teleoperation_runtime/aubo-passthrough-interruption-20261028-152650-status.json`。

## 已确认的最近一次接合首因

`hardware.first_stop_trigger.reason=preview_control_overrun`，触发时state=active，主机单调时刻9528.620180575。该字段每次arm重置，所以只代表最近一次接合的首个停止，不代表本轮所有中断。

- 输入年龄1.923895ms，候选5.889639ms，最近观察年龄63.247111ms，距上次发送起点14.728205ms。
- 在途名沿用`follow`（工作线程通用标签），实际SDK结果operation=`passthrough_six_axis_target`，并未误用跟随API。
- 最后透传返回0，耗时52.439945ms，无local_refusal，调用期间被撤销。原3°目标误差阈值仍null。
- 退出透传返回0，约52.840444ms；慢停返回0，约53.041404ms；组合105.882078ms。后续反馈stop_confirmed=true，fatal=false、stop_result_unknown=false。
- 全会话累计40次提交；后续状态读到缺右手柄的invalid_xr_frame，发生在上述停止之后，不能据此把本次首因说成头显掉流。

## 直接触发代码与机制

`teleop/aubo/quest_preview.py`共用循环运行反馈刷新、IK、JSON状态和MuJoCo显示。尾部执行`next_tick += runtime.control_dt`，当`now-next_tick > .1`时`runtime.submit_fault("preview_control_overrun")`并将计划时刻重置。这是本项目协调层逻辑，不是厂家拒绝或作者IK关节限位。

该检查衡量相对初始5ms计划节拍的累计落后，不等于当前一次循环耗时超过100ms。无SDK、按同一公式复现：每循环固定6ms，约100次/0.6秒积累到100ms落后；每循环8ms，约34次/0.272秒触发。单次远小于100ms也会不断停止，符合“轻轻动就取消”的可能机制。它说明判定机制有缺陷；现场尚无分阶段耗时，不能断言具体耗时是IK、viewer.sync、GIL或其他线程。

## 建议修正范围（本轮未实施）

1. 调度晚到时跳过过时计划时点，不追赶、不因累计节拍偏差直接判定设备/输入故障；200Hz明确为期望节拍。
2. 保留真实输入、候选、反馈年龄和SDK返回/停止检查；真实长停顿仍撤销目标，不能把本项改成无条件忽略。
3. 添加循环总耗时、反馈/IK/显示分段时间和错过周期数量，用证据确定瓶颈；将显示对控制的阻塞减小或分离，避免在尚无测量时归因。
4. 对比实际SDK发送频率；本次最后透传调用52ms，故切换透传本身不保证5ms稳定。正式判断须待本轮完整诊断落盘。

目前主机报告idle且反馈已确认停止，仍需现场观察配合；未从状态页代替物理现场确认。
