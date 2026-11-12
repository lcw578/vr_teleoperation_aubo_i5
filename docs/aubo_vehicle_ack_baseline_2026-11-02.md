# 最新现场复核：两段实验未命中 SDK 接收

14:28 / 14:38 两段已完成并保存故障现场。两段 QUICKACK 目标接收和设置成功数均为 0，DEEPBIND 绕过普通预加载已用回环实验复现；不能当成有效 B 组。入口现已拒绝 `--check/--execute --quickack`，下文原 B/A2 命令暂停。左 Y 两段共复现六次跟踪保护、一次停止 10007；详见 [两段复核](aubo_field_two_runs_review_2026-11-02.md)。


---

# 车载首轮基线与 QUICKACK 对照准备

日期：2026-11-02（北京时间）。本报告区分已经完成的 A 基线与尚未执行的 B 干预，不提前宣称修复成功。

## 1. 已完成的两轮现场会话

| 会话 | 用途与结果 |
| --- | --- |
| `home-field-20261102-134856-ufkN94` | 只观察待机；0 条运动命令，未收到 XR；300 秒正常结束，退出码 0 |
| `home-field-20261102-135516-ViqpnD` | USB 接入后收到有效输入并有遥操/归位；1750 条运动提交；300 秒正常结束，退出码 0 |

第一轮网页登录短暂成功后失效的直接原因是网页服务随有限会话正常退出。USB 随后实查为 `device`，端口转发仍存在；不能把该次服务退出算作 USB 断线。第一轮之前还发现过头显尚未接入及端口转发缺失，已分别核实并配置 `adb reverse tcp:8443 tcp:8443`。

第二轮用户确认右手柄不再被识别时已摘下头显或放下手柄，因此该片段先按正常暂停解释，不作为佩戴中的异常掉流证据。网页连通、XR 新鲜、机器人接合是三个独立状态。

车载完整证据分别在 `/home/xc/code/lcw/VR_teleoperation_runtime/` 下的上述目录；已复制到笔记本 `/data/VR_teleoperation_runtime/home-audit-20261102/field/`。第二轮录制目录为 `20261102-135516-a212550b`，亦已复制。原始文件保留。

## 2. 第二轮 A 基线结果

配置：USB、Follow 请求 25Hz、串行状态查询、waypoint 推送、oneshot 归位；未加载 QUICKACK。没有改变速度、跟踪阈值、停止接口或柜端网络配置。

| 指标 | 实测 |
| --- | ---: |
| SDK 运动调用 / PCAP 命令 22 配对 | 1750 / 1750 |
| 同一接合内聚合提交频率 | 18.916Hz |
| SDK 调用时长中位 / P95 / 最大 | 12.851 / 56.853 / 187.544ms |
| 线侧应答 <15ms | 882 次，50.40% |
| 线侧应答 15–40ms | 7 次，0.40% |
| 线侧应答 40–60ms | 833 次，47.60% |
| 慢应答中：延迟 ACK 后 <1ms 收到应答 | 831 / 833，99.76% |
| 上述 ACK 等待中位 | 40.723ms |
| ACK 至应答中位 | 0.340ms |
| 第二轮 tcpdump | 30125 packets captured；0 packets dropped by kernel |
| 独立诊断流 | written_events=27546；missing_events=0；error=null |

这次在**车载**复现了笔记本旧抓包中的 ACK 门控特征。PCAP 范围为单条客户端 TCP 流；分析工具按同流命令 22 FIFO 配对，报文重组统计没有列出重复/乱序异常。工具限制仍适用：不支持任意端口复用场景，未直接读取柜端 socket 选项，不将 ACK 特征单独当作柜端 Nagle 已证实。

`session_end.dropped_events` 来自有限内存历史环的裁剪计数；存在独立诊断写流时，不能把它直接当磁盘流丢失。本轮独立流的序号缺口为 0。普通录制段由操作按钮管理，READY 不等于开始了训练 episode；此处没有宣称数据集质量验收。

## 3. 左 Y 与停止的实际结果

按 `requested_monotonic_s` 分组，共 2 次归位请求：

1. **实际运动归位 1 次**：起点距原位最大关节差 18.824°，75 个归位 SDK 调用，进入 moving 后约 4.55 秒完成；最终最大关节差 0.001475°。到位后的停止调用 104.398ms、rc=0，并独立完成停稳确认。
2. 第二次按 Y 时已在原位，没有归位 SDK 运动，显示 complete；不计入运动成功样本。

本轮 3 次停止均有返回成功和停稳确认：

| 触发 | 停止调用时长 | 触发至停稳确认 |
| --- | ---: | ---: |
| Grip 松开 #1 | 482.289ms | 759.927ms |
| Grip 松开 #2 | 415.915ms | 676.564ms |
| 归位到位 | 104.398ms | 466.638ms |

最终 fatal=false、stop_result_unknown=false、stop_confirmed=true，无存活控制进程。没有复现昨日的 10007。**一次约 18.8° 归位成功不能排除昨日约 57° 归位中的累计跟踪滞后，也不能证明偶发停止超时消失。** 尚未验收归位中第二次按 Y 的主动取消。

## 4. 可复核的分析产物

笔记本第二轮证据目录内已生成：

- `analysis-follow.json`：调用分布、同接合频率、停止分段、独立诊断流状态；
- `analysis-home.json`：每次请求的生命周期、停止调用、源文件哈希；
- `analysis-ack.json`：原始 PCAP 哈希、ACK 关联统计和报文实例。

对应笔记本完整路径：`/data/VR_teleoperation_runtime/home-audit-20261102/field/home-field-20261102-135516-ViqpnD/`。Follow 分析使用仓库 `analyze_follow.summarize`，归位使用 `analyze_home.audit`；ACK 使用 `/data/VR_teleoperation_runtime/aubo_ack_reaudit_20261102.py`，其适用范围与报告 caveats 一并保留。

## 5. B 干预工具的实现与验证

新增 `teleop/aubo/native/quickack.c`，是 Linux 可选预加载库，不链接 SDK。只有显式 `AUBO_QUICKACK=1` 才工作，按**实际 socket 对端 IPv4、端口和 TCP 协议**筛选，当前实验固定为 `192.168.1.5:8899`。不缓存 fd，避免关闭后复用为文件/管道仍被当作旧 socket。

在成功接收数据后调用 `TCP_QUICKACK=1`，覆盖 `recv/read/recvfrom/recvmsg/readv`。当前 SDK 动态导入列表已核对含 `recv` 和 `read`。没有 send/write 拦截，没有调整 TCP_NODELAY、全局 sysctl、负载数据、SDK 请求、运动参数或控制柜配置。QUICKACK 是临时提示；每次匹配的正长度接收后重置，实际效果仍以抓包为准。

库只输出配置、首次目标接收及退出计数，记录匹配接收数、选项成功/失败数、失败 errno、路径计数和首次 fd；不打印接收内容或密码。SDK 调用返回值、接收字节和 errno 保持原值；选项设置失败会被计数，不伪装为成功。此工具依赖当前 SDK 的正常生命周期，即关闭 fd 前先结束接收线程；不能据此保证任意进程中跨线程关闭/复用同一 fd 的行为。

启动入口增加显式 `--quickack`，仅为 SDK Python 进程设置 LD_PRELOAD，sudo/tcpdump 不继承实验库；普通入口默认关闭。执行前验证源码和实验库 SHA256。构建清单：车载 `/home/xc/code/lcw/VR_teleoperation_runtime/quickack/build.json`。

本地与车载均通过：**10 项 QUICKACK 回环测试、7 项现场入口测试**。回环测试覆盖五种接收路径、非目标 TCP 地址/端口、UDP、关闭开关、非法配置、fd 复用、错误码、多线程，以及注入 setsockopt 失败仍保留接收结果。实验库用 `-Wall -Wextra -Werror` 编译成功。车载 `--check --quickack` 实际通过，未据此启动 SDK 或机器人。

截至本报告，**B 尚未实机运行**；这 17 项为新增工具验证，没有重新宣称既有 203 项或历史 313 项全套重跑。

## 6. 下一轮执行与回退

用户现场执行以下命令；它会检查恰好一台已授权 USB 设备并自动建立 8443 转发，再分别提示电脑 sudo / SDK 密码。头显地址为 `http://localhost:8443/`，进入 Start Teleop 后先保持 Grip/Y 松开：

```bash
TELEOP_RUNTIME_ROOT=/home/xc/code/lcw/VR_teleoperation_runtime AUBO_RUNTIME_DIR=/home/xc/code/lcw/VR_teleoperation_runtime AUBO_ALLOWED_ROOTS=/home/xc/code/lcw bash /home/xc/code/lcw/VR_teleoperation/scripts/aubo_home_field.sh --execute --quickack --duration 300
```

先确认日志的 `first_target_receive` 出现、enabled=true、setsockopt_success>0、setsockopt_failed=0，诊断/抓包正常；先看待机查询，再以与基线相似的小范围遥操/归位动作对照。固定 Follow25，不能将 B 的 SDK 应答改善等同实际运动跟随完全解决。停止未知立即结束该轮，不用重复按键恢复。

B 完成后取消 `--quickack`、重新启动同样配置进行 A2 回退复验；退出当前实验进程后，QUICKACK 不会全局驻留。需要比较慢峰比例、ACK 等待、SDK P95、提交间隔、反馈和停止结果。保持与 A2 尽可能相同的起点及动作；之前 A1 是现场自由操作，不声称三轮轨迹完全一致。

启动入口更新前已归档至车载 `…/home-audit-20261102/launcher-before-quickack/`。若需恢复入口本身，先确认没有运行中的会话，再只恢复该快照的入口和测试文件；无需改动 SDK 或网络。
