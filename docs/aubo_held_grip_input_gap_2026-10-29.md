# 持续按住 Grip 时中断：现场排查

本轮仅做只读排查，没有修改控制代码、阈值、恢复规则，也没有重启真机会话。

## 现场证据

进程 12963，25 Hz Follow、waypoint push、parallel state query，1800 秒。现场快照保存到 `/data/VR_teleoperation_runtime/aubo-held-grip-status-20261029-094346.json`。

快照保留的本次接合首次停止事件：

| 项目 | 数值 |
| --- | --- |
| 原因 | `core_input_unavailable` |
| 触发时刻 | 主机单调时间 32032.610165594 s |
| 触发前状态 | `active` |
| 距最近接收端接受的有效输入 | 207.971 ms |
| 关节接收年龄 | 51.844 ms |
| 运行状态接收年龄 | 38.501 ms |
| 最近后台查询耗时 | 8.175 ms，rc=0，state=1 |
| 最近候选年龄 | 4.819 ms |
| 触发时在途 Follow | 之后于 32032.639356977 返回，40.862 ms，rc=0，`revoked_during_call=true` |
| 后续慢停调用 | 604.723 ms，rc=0 |
| 最终停止确认 | 32033.620374169 s |
| fatal / stop_result_unknown | false / false |

`first_stop_trigger` 在新的接合时重置，因此它是所保存的那次接合的首次原因，不是永久会话历史。后续现场状态已出现新接合和正常 `grip_released` 停止；应使用保存快照复核此前异常。

用户确认中断时一直佩戴头显，之后才放下手柄。取证时的 `right_controller_missing` 是后续状态，不能归为异常中断起因。快照中主控制循环 stalls=0，最大 runtime step 约17.1 ms。反馈与后台查询均未接近200 ms，SDK调用未超过500 ms。

## 代码因果链

复用核心 `quest_teleop.py` 的 `XR_FRAME_STALE_TIMEOUT_S=0.2`；`get_action` 使用每只手最后接受时间，`_update_arm` 在超过200 ms时将 paused 置为 `input_timeout`，不再求解。AUBO shadow 在检测到 paused/未接合/ramp 时用通用 `core_input_unavailable` 撤销。硬件 runtime 将该原因传给 worker，撤销在途许可、清空候选并串行慢停。该理由不是正常Grip释放，停稳后要求新的释放/按下，因此一直按住不会自动续跑。

对应现场输入年龄207.971 ms，判断为输入更新空档触发核心过期。现有事件没有保存当时核心 paused 字段和逐帧来源时序，不能将更上游原因精确归给无线网络、Quest渲染暂停、追踪缺失或延迟帧拒绝。

核心还会拒绝相对最佳源时差超过200 ms的帧。快照累计 `queued=15`，说明本轮存在过此类拒绝，但没有事件时间，不能断言这15帧恰好导致该次中断。累计 max_gap 同样不是这次中断独有的间隔。

前端每个XR渲染帧发送 `performance.now()`、epoch、seq、手柄与Grip；`getPose` 为空会省略手柄。前端WebSocket发送缓冲超过64 KiB会断开重连。PC输入线程使用独立本地WebSocket接收，不调用SDK或IK。以上提供下一步诊断位置，不构成已证实根因。

## 进一步取证

本轮另接只读Relay订阅，短时记录接收单调时间、间隔、源时间、epoch、seq、右手存在和Grip，文件前缀 `/data/VR_teleoperation_runtime/aubo-held-grip-passive-20261029-`。不发送应用消息或运动指令。不能从中恢复订阅前的异常；需要与一次新的中断同步才可精确拆分源帧间隔与传输延迟。

后续若修改诊断，应在每次撤销时保留核心 paused、最后源/接收/接受时间、Grip、拒绝理由和最近短窗间隔，以避免当前只保留通用原因和累计计数的局限。当前不以放宽200 ms或自动恢复来代替原因定位。

## 本轮 Wi-Fi 继续定位与诊断实现

用户无法接USB，要求继续Wi-Fi。已新增不含位姿的输入时间记录，各保留最多128条原始接收/核心处理事件，包括seq、epoch、源时间、主机接收时间、接收间隔、源间隔、Grip和核心拒绝原因。`relative_delay_change_s`为相邻帧的接收间隔减源间隔，仅代表传输/主机处理延迟变化，不是绝对单向网络延迟。

核心拒绝可区分`source_delay_exceeded`和`source_or_sequence_out_of_order`；在核心reset前保留paused/engaged/ramp/Grip状态。真机首次异常停止快照保存为`last_input_stop`，在后来手柄缺失、正常释放或重新接合后仍保留，直到下一次异常停止。完整短窗在`/aubo/status`返回；终端转移日志只输出摘要，避免在200Hz主循环中输出整个短窗增加I/O负担。输入线程不写磁盘、不做SDK调用。此修改只是诊断补全，不宣称已修复输入空档。

303项离线回归通过（含新增3项：源间隔与接收间隔的分离/有界缓存、核心延迟帧拒绝证据、按住Grip输入过期及证据不被后来缺手柄覆盖），日志`/data/VR_teleoperation_runtime/aubo-input-stop-evidence-check-20261029.log`。测试CLI使用独立临时锁目录，不清除/绕过真机会话锁。无需重建原生绑定；正在运行的旧Python会话不会热加载新诊断。

本机网络只读核查：

- 控制柜路由仍经`enp3s0`，`192.168.1.100→192.168.1.5`。
- 电脑Wi-Fi为`lcw`，信道11（2.4GHz），信号93。没有修改网络或代理设置。
- Mihomo本地只读`/connections`记录了两条局域网连接：`10.245.247.115→10.245.247.27:8443`，入口`HTTPS`代理，`chains=[DIRECT]`，规则`GeoIP/lan`。开始时间09:40:36和09:40:37，与本轮头显页面会话一致。说明局域网遥操流量经本机代理转发，并未走远程代理节点；尚无证据说明代理就是中断原因。
- 对该客户端做20次ICMP探测：0%丢包，RTT min/avg/max为5.251/18.092/51.858 ms。它是此刻的往返延时，不能代表之前200ms空档时的WebSocket表现。

Wi-Fi下下一次受控比较优先只改头显代理：让`10.245.247.27:8443`直连，优先使用代理绕过列表，若头显没有此功能则在测试期间将该Wi-Fi代理改为“无”，再重开原局域网页面。电脑上其它应用代理不需改动。先保持25Hz、2.4GHz、映射和保护阈值一致；之后若仍有空档再单独对比5GHz热点，避免多个因素同时变化。

新会话可用下列命令保存摘要日志，保持Wi-Fi，无`--usb`；先正常退出原会话，不能同时运行两路控制：

```bash
cd /home/lcw/VR_teleoperation
set -o pipefail
bash scripts/aubo_quest_follow.sh \
  --execute-six-axis --command-mode follow --follow-hz 25 \
  --waypoint-feedback --parallel-state-query \
  --host 192.168.1.5 --port 8899 --user aubo \
  --rotation world --duration 1800 \
  2>&1 | tee "/data/VR_teleoperation_runtime/follow25-wifi-input-$(date +%Y%m%d-%H%M%S).log"
```

判定：源seq连续且源间隔大，定位到源产生/发送暂停；源间隔正常但接收间隔大，定位到传输/Relay/主机接收一段（还不能直接认定Wi-Fi）；接收持续正常但`host_wait_s`大，查主控制消费；`source_delay_exceeded`持续出现，查延迟积压与源时差判断。依证据修复对应段，再以同条件长会话验收。
