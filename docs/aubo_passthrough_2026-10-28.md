# 六轴透传后端与200Hz期望发送

同日后续已修正控制循环累计落后误停，新增真实停顿与分段耗时诊断，见[循环修正与复测指令](aubo_control_loop_fix_2026-10-28.md)。

按用户明确要求，正式六轴默认后端由FollowMode切换为TCP→CAN透传，发送周期由50ms改为5ms（期望200Hz）。旧J6入口仍为FollowMode/10Hz，未连接默认入口仍只打印计划。作者IK、滤波、离合目标管理、world默认、HMD yaw关闭、实际状态锚定、关节范围、输入/反馈时效、查询调度和日志默认关闭均保留。

## 实际接口与生命周期

- prepare仍只核对设备并读取参数，不进入透传；`--inspect-profile`也不进入透传。
- Grip接合、取得新实测状态、原生许可建立后，调用`robotServiceEnterTcp2CanbusMode()`；进入成功之后才允许发送。
- 发送调用`robotServiceSetRobotPosData2Canbus(double jointAngle[6])`，原IK六轴顺序、弧度单位，控制柜范围内最新目标直接提交，无历史队列或新增位置整形。
- 正常释放/故障/正常退出先撤销许可，再调用`robotServiceLeaveTcp2CanbusMode()`，之后保留原`RobotMoveStop`及反馈确认。退出失败也尝试原慢停；任一错误不能自动重新接合。退出成功不被视为停止证明。
- 进入过程中被撤销，不发送任何目标；进入返回失败/异常被视为可能已经切换，走退出/停止处理；不自动重复进入。停止确认后下一次有效接合重新进入。
- 停止结果`leave_passthrough_result`和`slow_stop_result`分别保存原始返回/耗时，外层停止耗时包含退出+慢停，不应拿它直接与旧纯慢停耗时比较。退出未确认不允许正常logout；析构不重试未知退出。SDK仍串行，无法在阻塞期间独立发停止。

厂家资料核对：已有文本`/data/robot_SDK/analysis/tmp/linux_api.txt`第2.3.47–49节确认进入、六轴弧度角发送、退出；`learning.txt`第65节给出每次极小增量和5ms休眠例子。这只是接口及例子，不能证明当前控制柜200Hz持续性能、缓冲/停流语义或任意IK跨度可直接执行。

透传不能假定继续执行FollowMode全局速度/加速度规划；SDK内存25°/s、25°/s²不作为透传限速保证。保留作者IK步长限制不等于对实际发送跨度做了5ms轨迹规划：通信延迟时会跳过中间IK点。未自动增加整形、批量点缓冲或切换其他停止方式。

## 启动与诊断

本轮只构建/离线测试，没有登录控制柜、发送目标或启动服务。现场首次透传验证仍未完成，尤其要核对进入/退出、Grip停止、断流及重新接合。先退出其他运动会话，操作者在场可停止/急停，从小范围动作检查。显式模式命令：

```bash
cd /home/lcw/VR_teleoperation
bash scripts/aubo_quest_follow.sh \
  --execute-six-axis --command-mode passthrough --waypoint-feedback \
  --host 192.168.1.5 --port 8899 --user aubo \
  --rotation world --duration 60 \
  --diagnostic-output "/data/VR_teleoperation_runtime/aubo-passthrough-200hz-$(date +%Y%m%d-%H%M%S).jsonl"
```

头显`https://10.245.247.27:8443/`，电脑`https://127.0.0.1:8443/aubo`，无需另外启动Relay。启动日志应显示`COMMAND MODE=passthrough`、`requested 200Hz`，网页标题为六轴实体透传，JSON`hardware.command_mode=passthrough`。正常Ctrl+C或时长结束后执行退出/停止并落盘。诊断仍手动开启、无图像、只写/data；数据集Logger关闭。

实际频率用退出后的文件分析：`python3 -m teleop.aubo.analyze_follow /data/VR_teleoperation_runtime/实际文件名.jsonl`。发送为5ms期望节拍，不保证硬实时；查询仍可能阻塞发送，超时不补发中间点。

`--command-mode follow`可显式选择原六轴接口进行回退对照，其六轴发送期望现在同为200Hz；旧J6频率不变。模式声明加入Python/原生prepare一致性校验，旧绑定或不匹配模式拒绝启动，不能静默退回跟随。

此前空闲`54.5ms原生状态查询/556ms外层耗时`导致500ms预算锁定的问题没有在本轮修正，切换API不能证明已解决。若新会话启动后锁定，不应盲目增大预算或反复按Grip。

验证完成：原生模块重建成功；245项Python完整离线回归通过；不链接厂家SDK的原生透传进入/精确六轴发送/退出/失败/重复接合检查通过，profile无写入及绑定未连接拒绝检查通过。覆盖5ms最新目标替换、不补发、进入期间释放、失败进入和停止锁定、退出后重新接合。日志为`/data/VR_teleoperation_runtime/aubo-passthrough-build-20261028.log`及`aubo-passthrough-fullcheck-20261028.log`。离线检查不代表已完成本机透传运动/停止验收。
