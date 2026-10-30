# 关节反馈空档：同步取证操作

本轮保持六轴 Follow 25 Hz、独立状态查询 B、200 ms 关节反馈有效期、500 ms SDK 调用预算和原有离合恢复规则。诊断只增加记录，不增加 SDK 查询连接或运动调用。原生绑定不需要重编译。

## 启动

确认上一会话已退出后，从项目目录运行：

```bash
cd /home/lcw/VR_teleoperation
bash scripts/aubo_feedback_diagnosis.sh
```

终端先询问电脑的 sudo 密码，用于网卡抓包；随后 SDK 单独询问机械臂登录密码。两者都在本机终端输入。抓包在收到 prepare 记录（SDK 登录及反馈订阅完成）后启动，不抓取登录请求。

此脚本启动 300 秒有限诊断会话。先观察未接合时的反馈，再接合保持手柄基本不动，最后正常小范围遥操。记录每段操作的大致时间；不要为了复现增加运动幅度。出现中断后保留会话数秒，以便记录反馈恢复及停止确认，然后报告发生中断。

所有证据放在 `/data/VR_teleoperation_runtime/feedback-gap-时间/`：

- `events.jsonl`：后台线程每约 100 ms 刷新，含 Follow 开始/结束、每批原生回调序号与时间戳、内存读取起止、具体反馈异常、状态查询、输入冻结证据和故障后两秒状态。流式记录内存历史为 4096 条，待写队列为 8192 条；队列覆盖用 `diagnostic_stream_gap` 显式报告。新增 `python_gc_pause` 记录至少 5 ms 的垃圾回收停顿，用于与控制循环停顿对时。
- `session.log`：启动及状态转换日志。
- `wire.pcap0` 等：仅控制柜 192.168.1.5:8899 双向 TCP；8 MB 每卷、最多四卷，独立硬超时 330 秒。
- `capture.log`：抓包启动结果、退出信息及内核丢包统计。

`/data/VR_teleoperation_runtime/latest-feedback-diagnosis.txt` 指向最近目录。PCAP 有容量上限；如果轮转覆盖了早期数据，只能分析实际保留的时间窗。后台事件队列也有上限；写盘跟不上时输出 `diagnostic_stream_gap`，不能将记录缺口误判为真实反馈缺失。

## 时间与消息身份

`clock_sync` 定期同时记录主机 monotonic 和 wall clock，用于把 PCAP 时间对齐控制日志。主机抓包时间是网卡捕获时间，SDK 回调时间是回调入口时间；两者都不是控制柜采样时间。

当前 SDK 的 `communicationmatetype.h` 中 `CommunicationType_RealTimeRobotJointAngle=7`。已核对当前 release 库 `RobotCommunicationClient::encodeAndSendCommunicationPackage`、`recvRobotCommunicationPackage` 及 `RobotControlServices::responseProcess`：帧头 `ee cc ff ff`，小端长度、命令号，尾部 CRC 和 `ff ff ff ff`。离线解析器重组 TCP 后按命令号区分关节推送与其他应答；需用正常时段的消息数与回调数核对对应关系后再做 SDK 派发延迟判断。

```bash
/data/VR_teleoperation_runtime/.venv/bin/python -m teleop.aubo.wire_timing \
  /data/VR_teleoperation_runtime/feedback-gap-时间/wire.pcap0 \
  --output /data/VR_teleoperation_runtime/feedback-gap-时间/wire-summary.json
```

多卷需按实际捕获时间顺序传入；轮转后文件编号不保证时间顺序。解析器输出按 TCP 连接的命令计数、关节推送最大间隔，以及逐帧首次/完整到达时间和 TCP 可按序交付时间。后续消息即使完整到达，也可能等待前方缺失的 TCP 字节，不能将到达时间误当成 SDK 已经可以读取的时间。不解码关节值、不验证协议 CRC、不将重叠 TCP 段直接宣称为网络丢包。截断抓包或不支持的链路类型会报错，抓包丢包统计需另查 `capture.log`。

## 判定顺序

1. 先用冻结的首次停止原因和具体异常确认触发条件，不用随后摘下头显的状态倒推。
2. 比较原生回调序号、时间间隔与主循环读取序号，区分回调暂停与缓存/调度问题。
3. 核对关节推送网络消息是否在对应空档内持续完整到达，同时检查抓包自身丢包、TCP 缺口与重组完整性。
4. 网络消息持续、回调暂停时重点查 SDK；网络也出现空档时继续分辨控制柜推送与网络路径。仅有主机侧证据不能直接证明控制柜未发送。
5. 对齐 Follow 调用与 B 查询，检查关系是否反复出现，不凭一次时间重叠断言因果。

后台写盘失败会在终端报错，停止导出时也会报告，不改变运动权限。输入、反馈超时与慢停规则保持原样。
