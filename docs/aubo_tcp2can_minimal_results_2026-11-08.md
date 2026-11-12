2026-11-08 已在车载电脑 xc-pc 执行两轮独立 TCP2CAN 测试。控制柜地址 192.168.1.5:8899，SDK 登录报告系统版本 V4.5.111.71f2ee1-Alpha。两轮均从当前实测关节角开始，J6 每次增加 0.0001 rad，发送 1000 次，累计请求增加 0.1 rad（约 5.73°）。

循环内只有 SetRobotPosData2Canbus 一个 SDK 调用，没有 VR、IK、状态查询线程或用户添加的网络选项干预。循环外有登录、停止状态检查、一次当前位置读取、进入/退出透传和登出。停止状态检查失败则不进入透传。没有另外执行停后角度查询，累计角度不是实测到位结果。

| 指标 | 固定休眠，13:39 | 分项计时与条件补齐，13:44 |
| --- | ---: | ---: |
| 成功发送 | 1000/1000 | 1000/1000 |
| 循环耗时 | 17.376918144 s | 9.836285582 s |
| 实际平均发送率 | 57.547604 Hz | 101.664393 Hz |
| 每次 SDK 后等待 | 固定 usleep(5000) | 仅调用返回早于 t0+5ms 时 sleep_until(t0+5ms) |
| SDK 独立计时 | 未记录 | 已记录 |
| 进入、发送、退出及登出返回码 | 均为 0 | 均为 0 |

后一轮实际平均频率约为前一轮的 1.767 倍。两轮是顺序执行的不同会话，没有第一轮的逐次 SDK/休眠计时，因此不能精确把 7.5406 秒总耗时差全部归因于删除固定休眠。

后一轮按用户指定的 `target_wake_time=t0+5ms` 实现，每次以本次 SDK 开始时间为基准补齐；它不是以整个会话起点维护一个全局 200 Hz 时间网格。超期后不额外补睡，不追赶补发。

| 后一轮耗时分布，nearest-rank | SDK 调用 | 相邻发送开始间隔 |
| --- | ---: | ---: |
| 样本数 | 1000 | 999 |
| 平均 | 9.833512 ms | 9.839323 ms |
| 最小 | 4.903841 ms | 5.000715 ms |
| P50 | 6.476634 ms | 6.476727 ms |
| P95 | 19.431929 ms | 19.491251 ms |
| P99 | 26.561157 ms | 26.568768 ms |
| 最大 | 171.084511 ms | 171.084688 ms |

SDK 超过 5/10/20/40/100 ms 的次数分别为 971/334/39/2/1。29 次调用不足 5ms，执行了条件补齐，补齐等待区间实测累计 2.637934 ms。SDK 计时累计 9.833512294 秒，已占几乎全部循环时间；热循环无打印或写盘，1000 条记录预留内存，退出模式和登出后才落盘。

本轮支持“去掉固定附加等待后，平均吞吐达到约 101.66Hz”。它没有证明稳定 100Hz、200Hz 能力或控制柜底层固定 10ms 节拍。P50 约 6.48ms、P95 约 19.43ms、最大约 171ms，存在明显波动。SDK 调用时长本身也不能证明 Nagle/Delayed ACK 归因，或推导手柄到实体响应达到 10ms。本轮没有改 LD_PRELOAD、TCP_NODELAY、TCP_QUICKACK 或路由配置。

源码：[tcp2can_minimal.cpp](../teleop/aubo/native/tcp2can_minimal.cpp)。本地与车载编译通过，使用 `-O2 -Wall -Wextra -Werror`。后一轮运行日志记录的源码 SHA256 与本地源码一致；CSV 已独立重算样本数、成功返回、分位数、条件睡眠次数和总频率。未新增自动化测试。

数据文件：

- 固定休眠原始日志（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-minimal/run-20261108-133909.log`）。
- 条件补齐原始日志（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-minimal/conditional-20261108-134428.log`）。
- 条件补齐逐次计时（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-minimal/conditional-20261108-134428.csv`）。
- 条件补齐统计汇总（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-minimal/result-conditional-20261108-134428.json`）。

车载日志与逐次计时位于 `/home/xc/code/lcw/VR_teleoperation_runtime/tcp2can-minimal/` 的同名文件。上一轮源码与可执行文件在两端各自运行目录的 `original-fixed-sleep/` 下备份。除本独立测试外，没有修改现有遥操控制方式。

2026-11-08 13:56:47，用户在车载终端完成 sudo 认证并执行了 `aubo_tcp2can_quickack_rt.sh`。该轮使用同一可执行文件，临时增加仅针对控制柜的 `/32 quickack 1` 路由，并通过 `chrt -f 90` 启动进程。

| 指标 | 条件补齐基线 | QUICKACK 路由 + FIFO90 |
| --- | ---: | ---: |
| 成功发送 | 1000/1000 | 1000/1000 |
| 循环总耗时 | 9.836286 s | 8.105061 s |
| 实际平均频率 | 101.664393 Hz | 123.379703 Hz |
| SDK 平均 | 9.833512 ms | 8.100115 ms |
| SDK P50 | 6.476634 ms | 5.393543 ms |
| SDK P95 | 19.431929 ms | 16.507107 ms |
| SDK P99 | 26.561157 ms | 29.390229 ms |
| SDK 最大 | 171.084511 ms | 125.002154 ms |

组合干预组平均频率提高约 21.36%，P99 则变差，仍有 125ms 长尾，不能声称抖动已经消除。最大尖峰同样在 step=3，下一次 step=4 为 48.599ms；单凭相同 step 不足以定位模式切换或其他原因。两个因素同时变化，且每组仅一次会话，不能分离 QUICKACK 与调度的贡献；没有抓包证明逐包 ACK 行为。

该轮所有发送、退出透传及登出均返回 0。日志记录 `temporary_host_route_removed=1`，事后只读查询确认 `/32` 临时路由已删除，到控制柜的 SDK 连接已关闭。独立复核 CSV 的 1000 个样本、成功返回码、均值及分位数与日志一致。

组合组证据：原始日志（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-minimal/quickack-fifo90-20261108-135647.log`）、逐次计时（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-minimal/quickack-fifo90-20261108-135647.csv`）、统计汇总（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-minimal/result-quickack-fifo90-20261108-135647.json`）。

2026-11-08 14:02:45，默认路由、普通调度执行 10000us 条件补齐测试。程序新增显式 `--period-us`（默认仍为5000），并在循环前记录 scheduler_policy=0、priority=0。旧源码与二进制备份于两端运行目录的 conditional-5ms-version。循环不添加资源探针、查询或抓包；SDK 后不足10ms才补齐至本次调用开始后10ms，超期不补发。

| 指标 | 5ms 基线 | 10ms 普通调度 |
| --- | ---: | ---: |
| 成功发送 | 1000/1000 | 1000/1000 |
| 循环耗时 | 9.836286 s | 11.847391 s |
| 实际平均频率 | 101.664393 Hz | 84.406769 Hz |
| SDK 平均 | 9.833512 ms | 9.380042 ms |
| SDK P50 | 6.476634 ms | 6.855771 ms |
| SDK P95 | 19.431929 ms | 18.390439 ms |
| SDK P99 | 26.561157 ms | 27.756944 ms |
| SDK 最大 | 171.084511 ms | 88.659040 ms |

10ms 组315次SDK调用超过10ms；发送开始间隔P50=10.083949ms、P95=18.501828ms、最大88.659279ms。剔除前6次后，5ms基线/10ms组的SDK P95分别为19.393692/18.386833ms，超过20ms的次数分别为37/33（各994样本）。没有出现主动降低至100Hz后运行长尾明显消失的预期效果。不能由单轮结果否定任何缓冲的存在，但“5ms发太快导致缓冲满，改10ms就能消除慢峰”的具体预测未得到支持。

最大尖峰位置为step=4（88.659040ms），其余早期尖峰包括step=3（78.895663ms）、step=9（87.239430ms）、step=10（60.996783ms）。测试期间RX/TX errors和dropped增量均为0，累计TX errors仍为17。SDK进入、发送、退出及登出均成功。CSV样本数、均值、来源源码哈希与日志独立核对一致。

10ms组证据：原始日志（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-minimal/period10ms-20261108-140245.log`）、逐次计时（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-minimal/period10ms-20261108-140245.csv`）、统计汇总（本机存档路径：`/data/VR_teleoperation_runtime/tcp2can-minimal/result-period10ms-20261108-140245.json`）。
