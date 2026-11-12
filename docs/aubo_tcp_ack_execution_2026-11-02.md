# 8899 ACK 验证：静止与 Follow A/B/A 已闭合

日期：2026-11-02，北京时间。用户已同意按“离线验证 → 车载静止对照 → 小范围遥操”推进。

更新：现场静止 A1/B/A2 与 Follow A1/B/A2 均已完成。静止 ACK 等待中位为 34.003→0.161→34.120ms；Follow 40–60ms 慢峰比例为 47.86%→0.85%→47.03%，QuickACK 组实际发送频率由基线约 19.1Hz 增至 24.24Hz。A2 回退结果见第 10 节，之后实际 Follow 如需使用应显式指定 `--ack-mode quickack`。

原始各轮日志、抓包均在 `/data/VR_teleoperation_runtime/tcp-ack-validation-20261102/field/` 保存并校验。下面前 1–9 节记录验证路径及前期结果；Follow A2 与闭环结论见第 10 节。

## 1. 已实现的方法

`teleop/aubo/tcp_ack.py` 在 SDK 所在进程内识别既有 IPv4 TCP 连接，不加载 SDK、不建立新连接，不读取或改写业务报文。它复制文件描述符以固定 socket 身份，使用实际两端地址、端口、TCP 类型及 ESTABLISHED 状态核对目标。发现多个候选（包括同一连接已有别名）时拒绝猜测；每个周期检查源 fd 的 inode，源 fd 关闭/复用或 TCP 退出 established 后结束并关闭自有引用。

QUICKACK 模式每约 5ms 设置一次临时提示，记录成功/失败、实际调度间隔和工作线程 CPU 时间。不依赖 LD_PRELOAD，不改变 RTLD_DEEPBIND，不访问 SDK 私有成员或修改厂家库。线程可能被调度延迟，5ms 不是逐包 ACK 延迟保证。退出前先停止线程，再由调用者退出 SDK。

四种模式：baseline 只做同样的周期身份检查；quickack 只设置 QUICKACK；nodelay 只观察/开启 NODELAY；combined 设置两者。NODELAY 原本为 1 时不重复作为有效干预，改变过的值退出时恢复。QUICKACK 不通过设为 0 来“恢复”，各对照组必须新进程、新连接。

旧的 `aubo_home_field.sh --quickack` 拦截方案仍被禁止使用；本次没有将实验接入遥操运动入口。

## 2. 离线实测

`tcp_ack_loopback.py` 硬编码 127.0.0.1，以 A1/B/A2 三条新连接运行。为演示机制，双方有意构造条件：接收端每笔实验前设置 QUICKACK=0，发送端 NODELAY=0 并分两次小写入。这不代表真实 SDK 正在采用同样写法，也不代表控制柜已经修复。

每组 12 笔，SDK 未加载、控制柜未连接：

| 主机 | A1 中位耗时 | QUICKACK B 中位耗时 | A2 中位耗时 |
| --- | ---: | ---: | ---: |
| 笔记本 | 40.994ms | 5.093ms | 41.005ms |
| 车载 | 41.008ms | 5.395ms | 40.987ms |

两台机器都是 A1 12/12、B 0/12、A2 12/12 超过 30ms。两台 B 均记录 QUICKACK 成功设置 13 次。效果与设定约 5ms 重置周期相符。

新测试 28 项（连接工具 18、静止入口 10），加既有拦截方案回环 11 项，两端同样 39 项全部通过。本地另有 5 项既有双路推送测试通过。新测试包含：DEEPBIND 接收库兼容、完整数据不变、多连接/别名拒绝、UDP 排除、fd 复用、远端关闭、NODELAY 恢复、选项失败、工作线程失败、无 SDK 的默认计划、静止状态判据、采样失败清理及抓包未启动即失败。DEEPBIND 用例只加载小型本地接收库，没有加载厂家 SDK。

原始实验报告：

- 本地 `/data/VR_teleoperation_runtime/tcp-ack-validation-20261102/loopback-aba.json`。
- 车载 `/home/xc/code/lcw/VR_teleoperation_runtime/tcp-ack-validation-20261102/loopback-aba.json`。
- 车载报告本地副本 `…/tcp-ack-validation-20261102/vehicle-loopback-aba.json`。

## 3. 独立静止入口

`scripts/aubo_ack_stationary.sh` 启动 `teleop.aubo.ack_probe`。它仅实例化当前 SDK 的 ReadOnlyClient，调用 login、Get、logout，不启动遥操控制器或暴露运动入口。登录后先检查 RobotStopped=0，再选择真实连接；抓包在认证完成后启动，只捕获这条 TCP 四元组。没有重连。

采样为串行、请求总计 25 次 Get/秒，每五次中四次 joint_angles、一次 robot_state；超时不补发。状态非停止、读取失败、干预线程失败或抓包异常则结束。该方案没有注册 waypoint 推送，通信负载与实际 Follow 不同，若不出现慢峰，只能记“该静止工作负载未复现”，不可否定 Follow 的 ACK 假设。

每次建立独立证据目录 `ack-stationary-A1-*` / `B-*` / `A2-*`，包含 summary.json、events.jsonl、wire.pcap、capture.log。summary 记录原始 NODELAY、socket inode/地址、选项成功数及周期统计。抓包缺少内核丢包统计或丢包不为零时不标完整通过。JSONL 查询包含 SDK 时间戳，离线抓包需按实际 Get 命令识别；旧的“命令 22 FIFO”分析器不能直接用于这些查询。

默认无参数只打印计划；`--check` 检查 manifest、eno1、抓包工具，不加载 SDK、不执行 sudo 认证。两个 stationary 会话用 flock 互斥；启动前另查已有已知机器人会话，发现时要求正常结束，不杀进程。这不能排除其他主机或非标准客户端，现场仍须保持单一操作者。

车载离线 `--check` 已通过。所有车载新增文件在 `/home/xc/code/lcw` 内，原有运动/相机入口未改。

## 4. 下一步：先做一组 A1

用户在车载终端执行，保持机器人静止，结束已有控制会话；无需头显、Grip、左 Y、左 X 或相机：

```bash
TELEOP_RUNTIME_ROOT=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_RUNTIME_DIR=/home/xc/code/lcw/VR_teleoperation_runtime \
bash /home/xc/code/lcw/VR_teleoperation/scripts/aubo_ack_stationary.sh \
  --execute --mode baseline --label A1 --duration 90
```

终端先提示电脑 sudo 密码，再提示 SDK 密码；输入密码时没有字符回显正常。进入采样后输出 READY，每约 5 秒输出进度，约 90 秒结束并打印报告路径。若提前退出，保留屏幕错误与报告，不自动重复。

拿到 A1 后先核对完整性、原始 NODELAY 及是否复现慢峰，再安排 B。B 参数为 `--mode quickack --label B`；A2 为 `--mode baseline --label A2`，其余相同。真实实验不设置 QUICKACK=0 来人为制造慢峰。

验收顺序：真实连接设置有效 → ACK 等待改变 → Get/Follow 应答慢峰变化 → 最后单独验收实际遥操手感。网络应答改善不等于 200Hz、位置精度或数据集质量验收。

## 5. 现场 A1 与抓包收尾修复

2026-11-02 19:22:19 开始（北京时间），目录 `ack-stationary-A1-ya8ieuya`。90.038 秒内 2238 次 Get 全部成功：448 次 robot_state、1790 次 joint_angles；状态采样均为 RobotStopped。SDK 原始 NODELAY=0，未设置 QUICKACK。

| 指标 | A1 |
| --- | ---: |
| SDK 调用中位 / P95 | 6.449 / 17.500ms |
| 线侧往返中位 / P95 | 6.208 / 17.240ms |
| 40–60ms 线侧往返 | 1 / 2238 |
| 请求/应答配对 | 2238 / 2238 |
| 请求拆分 / 应答拆分 | 0 / 0 |
| 带数据的 ACK / 纯 ACK（覆盖柜端数据的首个确认） | 2236 / 2 |
| 柜端数据至电脑 ACK 的中位 / P95 | 34.003 / 35.678ms |
| tcpdump 抓包 / 内核丢包 | 4484 / 0 |

抓包只见查询 10 / 26，没有 waypoint 推送。多数 ACK 由下一笔 Get 请求捎带，因此存在较长 ACK 等待却没有大量慢应答：本轮柜端数据基本已完整返回，并无后续推送/应答被证实卡在 ACK 后。这不能代表实际 Follow 的混合通信负载，不能据这次 1/2238 推翻先前约半数慢应答的观测。

原入口 `_stop_capture` 向 sudo/timeout 外层 PID 发送 SIGINT，未正确转发到 tcpdump，随后等待超时，故原始 summary 保留 `completed=false`、`capture_cleanup_error` 和未及时读到的丢包统计。tcpdump 随后在有限 timeout 下正常收尾，capture.log 显示 4484 包、零丢包。SDK 已正常 logout，基线工作线程正常退出。事后独立配对验证全部 2238 次，无缺失/重组异常，数据可用于比较；不回写原 summary 为成功。

修复改为只查找本用户、tcpdump 程序名、与本轮完全相同 `-w` 文件路径的写进程，对其发 SIGINT，再等待外层进程回收。新增无 sudo 的回归，使用不转发信号的外层等待程序和已初始化信号处理的子写进程，确认确切目标被停止并正常返回。新工具/入口测试共 29 项，加旧 QUICKACK 测试 11 项，两端 40 项全部通过。原两文件已备份至车载 `tcp-ack-validation-20261102/before-capture-cleanup-fix/`。

事后分析保存在本地 `tcp-ack-validation-20261102/field/ack-stationary-A1-ya8ieuya/analysis-A1.json`，只处理本次一个 TCP 四元组、无序号回绕、查询 10/26 的记录。4 个原始文件两端 SHA-256 一致，PCAP SHA-256 为 `53bda99b4485ade8a4bf0b155298d7113535bc129256a91714165be0b11da09e`。

下一轮 B 命令：

```bash
TELEOP_RUNTIME_ROOT=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_RUNTIME_DIR=/home/xc/code/lcw/VR_teleoperation_runtime \
bash /home/xc/code/lcw/VR_teleoperation/scripts/aubo_ack_stationary.sh \
  --execute --mode quickack --label B --duration 90
```

B 先检查真实 socket 成功设置数，再检查 ACK 等待分布；A1 没有大量慢查询，因此即使 B 查询耗时改善很小也不构成方案无效。Follow 效果仍需后续匹配负载验证。

## 6. 现场 B：QUICKACK 生效与抓包尾部限制

B 于 2026-11-02 19:31:28（北京时间）开始，目录 `ack-stationary-B-jovg163n`。90.023 秒共 2237 次 Get（448 状态、1789 关节），全部返回成功，状态采样均为 RobotStopped。真实连接为 192.168.1.100:32900 → 192.168.1.5:8899，QUICKACK 成功 17690 次、失败 0，NODELAY 保持原值 0。重置间隔平均 5.092ms，最大 25.537ms；线程 CPU 时间 0.465 秒，约占该轮单核时间 0.52%，不是整机峰值 CPU 利用率。

| 指标 | A1 | B |
| --- | ---: | ---: |
| SDK 成功查询 | 2238 | 2237 |
| 完整捕获的查询/应答对 | 2238 | 2214 |
| ACK 等待中位 | 34.003ms | 0.161ms |
| ACK 等待 P95 | 35.678ms | 0.196ms |
| ACK 小于 1ms | 2/2238 | 2212/2214（99.91%） |
| 纯 ACK（覆盖柜端数据的首个确认） | 2 | 2214 |
| 捎带在数据中的 ACK | 2236 | 0 |
| SDK 调用中位 / P95 | 6.449 / 17.500ms | 6.163 / 16.776ms |
| 捕获范围内 40–60ms 线侧应答 | 1 | 2 |

结论：已确认在真实机器人 SDK 连接中 QUICKACK 干预有效，ACK 行为发生明显变化。尚未完成 A2 回退，因此不把所有差异归因过程视为最终闭合；本轮静止查询没有复现 Follow 的慢峰，也不能把 34→0.161ms 描述成“运动命令延迟已降低 34ms”。没有读取柜端 socket 选项，不能据此独立确认柜端 Nagle。

### B 抓包的尾部缺失

入口这次正常结束、capture_exit_code=0、capture_kernel_dropped=0、SDK logout=0，原 summary 显示 completed=true。但逐笔比对发现 PCAP 仅包含 2214 个完整往返，日志有 2237 个成功查询，最后约 0.926 秒（23 次查询）未进入 PCAP。已捕获前段无悬空请求、未确认柜端数据或重组异常。tcpdump captured=6646、received by filter=6715，差 69 包，恰与 23 次交互每次请求/应答/ACK 的数量相符，与停止时捕获缓冲尾部未排空一致。

“0 packets dropped by kernel”不保证采集边界覆盖所有日志查询。此次 ACK 统计仅使用已完整捕获的 2214 次交互；SDK 调用统计使用日志全部 2237 次。原始记录不回写，不能宣称整轮抓包完整，也不据此认定机器人网络丢包。

入口补充 `tcpdump --immediate-mode`，车载版本确认支持。采集退出后对查询 10/26 两个方向的完整帧计数与日志逐项比对，缺少请求、应答或有残留帧均不标 completed=true。离线验证已用真实 A1/B 文件分别得到 complete=true/false；新增用例覆盖缺少尾部请求/应答、完整数据，以及立即捕获参数。两端相关回归各 42 项通过。修改前两文件备份于车载 `tcp-ack-validation-20261102/before-capture-immediate-fix/`。

四个 B 原始文件两端哈希一致；PCAP SHA-256 为 `0bfe777945d36c9ef6c003d6f45b885fed140b641ec26e6479e24caca28b5b6d`。分析文件位于本地 `tcp-ack-validation-20261102/field/ack-stationary-B-jovg163n/analysis-B.json`；对 A1 使用同一独立分析器复算输出 `analysis-A1-v2.json`。离线分析脚本 `tcp-ack-validation-20261102/audit_stationary_capture.py` 仅支持本次单 TCP 流、Get 10/26、无序号回绕的抓包。

### 下一步 A2

```bash
TELEOP_RUNTIME_ROOT=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_RUNTIME_DIR=/home/xc/code/lcw/VR_teleoperation_runtime \
bash /home/xc/code/lcw/VR_teleoperation/scripts/aubo_ack_stationary.sh \
  --execute --mode baseline --label A2 --duration 90
```

保持静止，不使用头显，不按 Grip/Y/X。新进程、新连接，不主动设置 QUICKACK=0 或 NODELAY。先检查 ACK 是否恢复较长等待，再安排匹配 Follow 流量的后续验证。

## 7. A2 回退与静止阶段结论

A2 于 2026-11-02 19:41:24（北京时间）开始，目录 `ack-stationary-A2-0gxoig8t`。90.003 秒完成 2236 次查询（448 状态、1788 关节），全部成功，状态采样均为 RobotStopped。QUICKACK 设置数为 0，NODELAY 保持原值 0。

所有 2236 次请求/应答均完整捕获，内核丢包 0，抓包与 SDK 正常退出，原 summary completed=true。末次柜端应答的后续 ACK 在抓包停止时尚未出现，ACK 统计只用其余 2235 条，不能把它当作网络丢包。

| 指标 | A1 | B | A2 |
| --- | ---: | ---: | ---: |
| SDK 成功查询 | 2238 | 2237 | 2236 |
| 完整捕获的请求/应答 | 2238 | 2214 | 2236 |
| ACK 等待中位 | 34.003ms | 0.161ms | 34.120ms |
| ACK 等待 P95 | 35.678ms | 0.196ms | 35.691ms |
| ACK 小于 1ms | 2/2238 | 2212/2214 | 4/2235 |
| SDK 查询中位 | 6.449ms | 6.163ms | 6.354ms |

该对照支持结论：**在此真实 SDK 的静止读取工作负载中，客户端 QUICKACK 干预能显著缩短 ACK 等待，撤回后恢复；不是单纯插件配置显示开启。** 由于三轮本就没有明显的查询慢峰，不能宣称已解决 Follow 的 40–60ms 慢应答或运动跟随问题。A1 与 B 的采集收尾限制仍如实保留，不把它们伪装为完全相同的软件版本；A2 已使用立即捕获，观察到基线 ACK 行为仍恢复。

PCAP SHA-256：`710281d2bea4002ae2f0e23de9bc3fcd0fd90be3fcd97d285ec09822d7ac96c5`。离线分析在本地 `tcp-ack-validation-20261102/field/ack-stationary-A2-0gxoig8t/analysis-A2.json`，三个阶段汇总另存 `stationary-aba-comparison.json`。原始文件两端哈希一致。

### Follow 接入准备

`quest_follow.py` 新增显式 `--tcp-ack-mode`，仅允许现有单连接、六轴 Follow 路径，拒绝并行第二连接、J6 试验和透传模式。登录后启动干预，注册推送/prepare 仍走原流程；关闭顺序为运行时 → 运动 worker 的停止/确认 → ACK worker → SDK logout。ACK worker 清理卡住时拒绝并行关闭 SDK。默认不创建选项 worker，旧 `--quickack` 预加载入口继续拒绝使用。

`aubo_home_field.sh --ack-mode baseline|quickack|nodelay|combined` 将模式转交现有程序，baseline 也运行相同身份检查工作线程但不设置选项。trace 记录 tcp_ack_start/tcp_ack_end，field-result 记录模式。抓包加入立即捕获；按钮、速度、运动算法、Follow25、串行状态查询及原数据采样方式不变。本轮不启用 Placement。

新增回归检查无选项时不创建线程，显式模式在登录后启动，prepare 失败时仍先停止 ACK 再 logout，启动失败不订阅/运动，线程清理失败禁止 logout，多连接/其他模式拒绝；双端各 186 项相关 Python 回归全通过，脚本语法及车载 --check 通过。未据此执行真机运动。原文件备份在车载 `tcp-ack-validation-20261102/before-follow-ack-integration/`；比较后确认除本次接入外无其他两端差异才部署。

下一次先做短时 Follow 原始基线，120 秒，操作者保持现场物理停止可用：

```bash
TELEOP_RUNTIME_ROOT=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_RUNTIME_DIR=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_ALLOWED_ROOTS=/home/xc/code/lcw \
bash /home/xc/code/lcw/VR_teleoperation/scripts/aubo_home_field.sh \
  --execute --ack-mode baseline --duration 120
```

该入口会恢复头显网页与按 Grip 的实际遥操能力。先松开 Grip，进入 `http://localhost:8443/` 的 WebXR，待就绪后按右 Grip 做熟悉的小幅动作，再松开确认停稳。重复两三次即可，不做左 Y 归位或左 X 放置。记录按钮可按原方式使用，连续诊断/抓包独立保存；没有相机采集。然后保持相近动作和 25Hz 不变做 QUICKACK B，再回退复验；依实际基线是否复现慢峰决定后续，不提前声称 200Hz。

## 8. 实际 Follow A1：慢峰复现，可进入 QUICKACK B

2026-11-02 20:01:03（北京时间）启动，目录 `home-field-20261102-200103-NLOIQH`，120 秒，`--ack-mode baseline`，请求 25Hz、单连接串行查询。QUICKACK 成功设置数为 0，NODELAY 原值与活动值均为 0，工作线程正常退出，无选项错误。

| 指标 | Follow A1 |
| --- | ---: |
| SDK 成功 Follow / 抓包完整配对 | 1914 / 1914 |
| 按住 Grip 的各段内合计发送频率 | 19.059Hz |
| SDK 调用中位 / P95 / 最大 | 39.390 / 56.158 / 263.841ms |
| 线侧往返 <15ms | 946 / 1914（49.43%） |
| 线侧往返 15–40ms | 11 / 1914（0.57%） |
| 线侧往返 40–60ms | 916 / 1914（47.86%） |
| 慢应答紧随延迟 ACK（间隔 <1ms） | 913 / 916（99.67%） |
| 上述 ACK 等待中位 / ACK 后应答间隔中位 | 40.760 / 0.330ms |
| 停止调用成功 / 停止确认 | 6 / 6 |
| tcpdump 抓包 / 内核丢包 | 11664 / 0 |

其余 41 条线侧往返不在表中三个区间内，未假定全部小于 60ms。1914 条请求均为完整的单帧 480 字节请求，未观察到请求拆分。典型慢交互为：电脑发 Follow → 柜端发 82 字节状态推送 → 电脑约 40.8ms 后发纯 ACK → 柜端约 0.33ms 后发 Follow 应答及推送。该证据支持 ACK 门控发送，仍不直接等于读取到柜端 Nagle 配置。

5 次停止由 Grip 松开触发，末次停止原因是会话末尾 `websocket_disconnected`；停止调用均返回 0，最终 `fatal=false`、`stop_result_unknown=false`、`stop_confirmed=true`。末次断连与本轮结束相邻，单凭该记录不认定 USB 再次发生独立故障，也不把该次写成 Grip 松开。停止请求到确认约 0.699–1.209 秒；这是 SDK 状态确认时间，并非用抓包测得的机械运动停止时刻。

磁盘诊断事件序号 1–14110 连续，stream missing_events=0、error=null。报告中的 dropped_events=10014 是内存环形缓存淘汰数，不能解释为磁盘丢失 10014 条。全部 5 个原始文件两端 SHA-256 一致，PCAP 为 `e2d6711d1ed1ad5a41c4c7dd94670a7f9ecca0310426b1529f83e0e80706c715`。副本与 `analysis-follow.json`、`analysis-ack-gating.json`、`analysis-integrity.json` 存在本地 `/data/VR_teleoperation_runtime/tcp-ack-validation-20261102/field/home-field-20261102-200103-NLOIQH/`。

下一轮仅将模式换成 QUICKACK，保留请求频率、运动算法、串行状态查询和 120 秒时长。比较对象为相近动作的不同会话，不宣称轨迹完全一致；先看 ACK 干预实际生效，再看慢峰、SDK P95、发送频率与停止结果。

```bash
TELEOP_RUNTIME_ROOT=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_RUNTIME_DIR=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_ALLOWED_ROOTS=/home/xc/code/lcw \
bash /home/xc/code/lcw/VR_teleoperation/scripts/aubo_home_field.sh \
  --execute --ack-mode quickack --duration 120
```

本阶段尚未执行 Follow B，也尚未证明运动跟随延迟改善。保持右 Grip 小幅动作并松开确认停稳，不做左 Y / 左 X，不开启相机。B 完成后核对实际效果，再安排 Follow A2 回退。

## 9. 实际 Follow B：QUICKACK 显著降低慢峰，待 A2 回退

B 于 2026-11-02 20:10:01（北京时间）启动，目录 `home-field-20261102-201001-EfbNLc`，120 秒，`--ack-mode quickack`。25Hz 请求、单连接、串行查询与 A1 相同。两轮人为运动轨迹和 Grip 段数不同，不作为完全相同轨迹的重复实验。

| 指标 | Follow A1 | Follow B |
| --- | ---: | ---: |
| SDK 成功 Follow / 抓包完整配对 | 1914 / 1914 | 2347 / 2347 |
| 各段 Grip 内合计实际发送频率 | 19.059Hz | 24.238Hz |
| SDK Follow 调用中位 / P95 | 39.390 / 56.158ms | 10.610 / 26.554ms |
| 线侧 Follow 往返中位 / P95 | 39.102 / 55.852ms | 10.301 / 26.236ms |
| 线侧 Follow 往返 40–60ms | 916/1914（47.86%） | 20/2347（0.85%） |
| 线侧 Follow 往返 >60ms | 41/1914（2.14%） | 7/2347（0.30%） |
| 所有柜端数据段的 ACK 等待中位 / P95 | 12.643 / 41.324ms | 0.174 / 0.271ms |
| 上述 ACK 等待 <1ms | 2274/4952（45.92%） | 7424/7612（97.53%） |
| 慢应答紧随约 40ms 延迟 ACK | 913 | 0 |
| 停止调用成功 / 停止确认 | 6 / 6 | 2 / 2 |

此处全体 ACK 统计覆盖柜端推送和应答，与第 8 节仅对慢 Follow 关联 ACK 的 40.760ms 中位数口径不同。累积 ACK 可以同时覆盖多个数据段，统计单位为被确认的数据段。离线分析仅支持这两次单连接、端口不复用、无序号回绕的捕获。

B 的实际 socket 为 192.168.1.100:53804 → 192.168.1.5:8899。QUICKACK 成功设置 23748 次，失败 0；NODELAY 保持 0，未做 NODELAY 干预。提示间隔平均 5.132ms、最大 216.435ms；worker CPU 时间 0.694 秒，约占 121.864 秒工作时长的单核时间 0.57%，并非硬实时 5ms 保证。抓包中仍有 2 个数据段的 ACK 等待在 35–50ms，不能声称每个包都立即确认。

本轮 SDK Follow 最大耗时 225.160ms，线侧最大 224.835ms，仍有残余长尾。QUICKACK 显著减少了原有慢峰，尚未证明消除所有延迟，也没有改变 25Hz 设定或验证 200Hz。端到端操作感和机械跟随误差亦不能用 SDK 往返数值代替。

第一轮停止由 Grip 松开触发，第二轮为会话末尾 WebSocket 断连触发，两次均返回 0 并确认停止；停止请求到状态确认为约 0.774 / 0.838 秒，最终无 fatal、无停止结果未知。一次 `push_refresh_failed: feedback_reception_expired` 出现在第一轮已请求停止后约 0.598 秒：当时最新推送年龄约 0.202 秒，略超 0.2 秒门槛；它不是本轮停止的起因，随后确认停止并重新进入遥操。该短暂过期保留为待观察项，不据两次成功停止宣称长期稳定性已经完成。

抓包 18773 包，received by filter 同为 18773，内核丢包 0；全部 Follow 请求/应答完整配对，无悬空 Follow。磁盘诊断序号 1–17133 连续，stream missing_events=0、error=null；内存环形缓存淘汰 13037 条不属于磁盘日志缺失。5 个原始文件两端哈希一致，PCAP SHA-256 为 `2ef8da0ae66936831ab8f52261cd05abb9c749948c714945f55c36217c57c8f7`。

分析保存在本地 `/data/VR_teleoperation_runtime/tcp-ack-validation-20261102/field/home-field-20261102-201001-EfbNLc/`。全体 ACK 与完整 Follow 配对统计由 `tcp-ack-validation-20261102/audit_follow_ack_comparison.py` 离线生成，两轮汇总为 `follow-a1-b-comparison.json`；不修改原始日志。

当前证据支持：客户端 QUICKACK 能在现有旧版 SDK Follow 路径中显著降低应答慢峰，原先“只能由厂家改固定 40ms 固件节拍”的归因不能维持。柜端发送选项没有直接读取，不能把柜端 Nagle 作为已读取确认的配置；现阶段也没有证据要求升级控制柜才能达到当前 25Hz 请求目标。下一步使用新进程、新连接回到 baseline 完成实际 Follow A2，不用设置 QUICKACK=0 模拟回退。

```bash
TELEOP_RUNTIME_ROOT=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_RUNTIME_DIR=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_ALLOWED_ROOTS=/home/xc/code/lcw \
bash /home/xc/code/lcw/VR_teleoperation/scripts/aubo_home_field.sh \
  --execute --ack-mode baseline --duration 120
```

仍做相近的小幅慢动作与 Grip 松开停稳检查，不做 Y/X、不采集相机；结束前先松开 Grip。实际 Follow A2 尚未执行，默认模式尚未切换为 QUICKACK。

## 10. 实际 Follow A2：回退复现，三轮验证完成

A2 于 2026-11-02 20:20:12（北京时间）启动，目录 `home-field-20261102-202012-oJOOGK`。新进程、新连接 192.168.1.100:42844 → 192.168.1.5:8899，baseline 模式，QUICKACK 设置数 0，NODELAY 始终为 0，选项线程正常退出。1816 次 Follow 全部成功，抓包全部配对，无悬空 Follow。

| 指标 | A1 关闭 | B 开启 QUICKACK | A2 关闭 |
| --- | ---: | ---: | ---: |
| Follow 成功调用 | 1914 | 2347 | 1816 |
| 段内实际发送频率 | 19.059Hz | 24.238Hz | 19.136Hz |
| SDK Follow 平均耗时 | 27.138ms | 13.366ms | 26.703ms |
| SDK Follow P95 | 56.158ms | 26.554ms | 56.029ms |
| 40–60ms 线侧往返 | 916（47.86%） | 20（0.85%） | 854（47.03%） |
| 全体柜端数据段 ACK 等待 P95 | 41.324ms | 0.271ms | 41.425ms |
| 成功停止 / 确认停止 | 6 / 6 | 2 / 2 | 7 / 7 |

A2 的 854 次慢应答中，850 次（99.53%）紧随约 40ms 延迟 ACK，关联 ACK 等待中位 40.763ms、ACK 后应答中位 0.334ms，恢复 A1 的特征。B 对应关联慢应答为 0。三轮使用相同软件、25Hz 请求与串行查询，但人为轨迹、Grip 段数不同；改善与回退同时出现在 TCP ACK、SDK 耗时和有效发送频率三个层面，支持 QUICKACK 对这条链路有实际因果作用。

双峰各占约一半时，中位数对两侧比例敏感：A2 SDK 中位数为 12.407ms，A1 为 39.390ms，不能单看中位数误判回退未复现。两轮平均耗时约 27ms、P95 约 56ms、慢峰约 47%、频率约 19.1Hz，均一致恢复。

7 次停止均成功并确认，其中 6 次 Grip 松开、末次会话结束时 WebSocket 断连。最终 fatal=false、stop_result_unknown=false。状态确认等待 0.588–1.490 秒，仍与机械实际停止时刻区分。A2 也出现一次停止期间推送年龄约 0.201 秒的反馈过期，随后确认停稳；该现象并非 QUICKACK 组独有，当前保留观察，不修改门槛或保护。

抓包 captured/received 均为 11658，内核丢包 0。磁盘诊断序号 1–13894 连续，stream missing_events=0、error=null；5 个原始文件两端哈希一致。PCAP SHA-256 为 `164f7d4ff378b4399f808fa94bdf93ab0fefbb008726971ed504ce4ac2680831`。A2 分析与完整性记录在对应 `field/` 子目录，三轮汇总为 `/data/VR_teleoperation_runtime/tcp-ack-validation-20261102/follow-aba-comparison.json`。

### 结论与后续启动

该阶段完成。推荐当前六轴单连接 Follow 使用显式 `--ack-mode quickack`，保持 25Hz 请求。客户端延迟确认是原有约 40ms 慢峰的重要组成；“固件固定 40ms 节拍且客户端无法改善，只能找厂家”的原结论应撤回。未直接读取柜端 Nagle 配置，未验证 200Hz，也未完成端到端机械响应与长时间稳定性验收。QUICKACK 组仍有最长约 225ms 的少数长尾，后续另行分析。

现有入口已支持 QUICKACK，不需要再改运动代码或 SDK；不带 `--ack-mode` 的旧命令不会自动启用。后续车载现场 1800 秒启动命令为：

```bash
TELEOP_RUNTIME_ROOT=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_RUNTIME_DIR=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_ALLOWED_ROOTS=/home/xc/code/lcw \
bash /home/xc/code/lcw/VR_teleoperation/scripts/aubo_home_field.sh \
  --execute --ack-mode quickack --duration 1800
```

1800 秒是已支持的会话时长，不代表本次完成了 30 分钟持续稳定性验证。命令不启动相机；保留既有录制按钮逻辑、诊断和抓包。此次三轮没有测试左 Y 归位或左 X 放置，不能据此宣称两项功能已验收。无需再重复相同 A/B/A，后续围绕剩余具体问题推进。

## 11. Combined 复测启动故障与启动顺序修正

2026-11-02 21:02:21，现场 combined 会话为 `home-field-20261102-210221-ND0Xru`。此前端口预检查已统一使用与 Uvicorn 相同的 SO_REUSEADDR 规则；活跃监听仍拒绝，另可显式选择 relay port。本轮 8443 服务成功启动，因此不能继续按端口冲突解释“网页正常但控制无响应”。

本轮 socket 为 192.168.1.100:40250 → 192.168.1.5:8899，NODELAY 从 0 成功设为 1，QUICKACK 设置成功 23677 次、选项失败 0，退出时 NODELAY 恢复。左 X 放置配置已加载。首次 fatal 发生于 idle、尚无头显输入或运动意图，原因是 feedback_reception_expired，实际提交和 send_returned 均为 0；这轮不能用于判断 NODELAY 的运动延迟收益。

PCAP 中，故障前状态查询（命令 26）在线侧耗时 6.479ms。原生返回记录也只有 6.978ms，但 Python 快照直到约 205ms 后才继续检查，整个快照表观耗时 212.331ms，状态年龄 205.370ms、关节年龄 228.647ms，超过 200ms 反馈门槛。独立 ACK 提示线程最大间隔为 212.201ms。柜端期间继续发送关节推送。该轮两次可观测 GC 停顿约 7.7/20.1ms，不能用它们解释这次 212ms 停顿。

证据支持这是状态回复到达后的主机处理停顿，而非本笔柜端应答等待 212ms。停顿发生在网页/桌面预览初始化窗口内；没有采样当时 GIL/调度器堆栈，因此不把具体的 GL 调用或系统调度原因写成已确认事实。

最小修正：先完成 relay、输入接收和桌面预览初始化，再通过 on_ready 启动受监督 SDK 查询线程，取得新鲜反馈后才进入控制循环。日志增加 `Control feedback ready after relay/preview initialization`，诊断增加 `control_startup_ready`。初始反馈若失败则启动直接报错退出，避免继续展示已永久锁定的控制会话。没有改变 follow.py、200ms 反馈门槛、运动中停止规则、X/Y 操作或运动参数。

本地 234 项相关 Python 回归与 24 项模拟 SDK 回归通过；车载 44 项离线回归、语法编译和 combined + placement 的 --check 通过。均不连接真实机器人。源文件两端 SHA-256 一致；原车载两文件备份在 `/home/xc/code/lcw/VR_teleoperation_runtime/tcp-ack-validation-20261102/before-control-startup-order/`。待现场复测，尚不宣称启动故障已实机闭环。

原始 PCAP SHA-256：`a66cabb12b837baaef5271e05e7e552b2e9d6a7cfb48dbb81c17bbac6c13d42c`；派生分析为本地该会话目录的 `analysis-startup-fault.json`，原始日志未修改。

下一轮继续 combined、120 秒，并保留左 X 的篮筐配置；QUICKACK 正式基线有效，NODELAY 的新增收益仍待这轮有实际 Follow 发送后与既有 QUICKACK 数据比较。

## 12. Combined 实际 Follow 复测完成：无明确 NODELAY 增量收益

2026-11-02 21:15:23 启动的 `home-field-20261102-211523-rg5Q3M` 正常结束。日志先完成 relay/预览初始化，再于 21:15:27 输出 Control feedback ready；诊断的 control_startup_ready 早于首条 SDK 查询。本轮无 fatal、无 stop_result_unknown，未再出现启动反馈锁定；控制主循环记录 stalls=0。仅证明本轮修正后的启动和运行正常，不推断所有未来主机停顿都已消除。

combined 的 NODELAY 实测由 0→1，QUICKACK 设置成功 23625 次、选项失败 0，退出时 NODELAY 正常恢复。2200 条 Follow 全部 SDK 接受，PCAP 中 2200 个请求与 2200 个应答完整配对，无悬空请求。13 次停止均由 grip_released 触发，慢停接口均成功并确认停止；停止请求至状态确认为约 0.582–1.494 秒，这是软件确认时长，不是机械实际刹停时长。

| 指标 | 既有 QUICKACK | 本轮 QUICKACK + NODELAY |
| --- | ---: | ---: |
| Follow 成功调用 | 2347 | 2200 |
| 同接合段实际发送频率 | 24.238Hz | 24.184Hz |
| SDK 平均耗时 | 13.366ms | 13.187ms |
| SDK P95 | 26.554ms | 26.745ms |
| SDK 最大耗时 | 225.160ms | 223.910ms |
| 线侧 40–60ms 应答 | 20/2347（0.85%） | 38/2200（1.73%） |
| 全体柜端数据段 ACK 等待 P95 | 0.271ms | 0.268ms |

两轮人工动作不同，Grip 停止次数为 2 对 13，本轮还调整了启动顺序。因此这些差异不能证明 NODELAY 改善或导致退化；现有结果只支持“组合模式可运行，但没有明确额外收益”。保留 QUICKACK 为正式基线，不自动将 NODELAY 纳入正式基线，也不重复强制安排 QUICKACK 单独测试。约 224ms 的偶发线侧长尾仍存在，TCP 设置没有解决全部延迟；端到端手感须另看主端目标到实际反馈的链路。

左 X 的 placement 配置与篮筐路径已在录制 manifest 核实；本轮所有提交 control_source 都为 teleop，录制中没有 placement 生命周期事件，因此本轮不是实际放置动作复验。配置里的 route_validated=false 属于原有 commissioning 标志，不据此否定此前现场验证。

抓包 captured/received 均为 18381，内核丢包 0；诊断序号 1–16618 连续，stream missing_events=0、error=null。内存历史环淘汰 12522 条不是磁盘日志丢失。录制会话闭合且 complete=true、dropped_records=0；未据此声称采集了相机或有效任务分段。5 个原始取证文件两端 SHA-256 完全一致，PCAP SHA-256：`d80cb2e23b3253065e349308e2eded8336b3264afcdb37200227fa5a926d8306`。

可复核报告：`/data/VR_teleoperation_runtime/tcp-ack-validation-20261102/follow-quickack-combined-comparison.json`；本轮源文件及派生分析位于该目录的 field/home-field-20261102-211523-rg5Q3M/。此次只读取分析和归档，没有修改运行代码、网络配置或正式启动参数。
