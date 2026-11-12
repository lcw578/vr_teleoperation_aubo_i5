# 左 Y 归位取证：首批执行结果与现场复测步骤

日期：2026-11-02。对应已确认方案的工作包 0、1；工作包 2 尚未做网络干预。

**后续现场进度**：两轮车载基线已完成，其中一轮包含实际归位和 3 次成功停止；车载 PCAP 复现 ACK 等待特征，QUICKACK 可选工具已完成双端离线测试并部署，尚待 B 实验。详见 [车载 ACK 基线与对照准备](aubo_vehicle_ack_baseline_2026-11-02.md)。本页“未启动／未发送”等句子保留为首批执行时的历史状态。

**当前交付是可回退的车载基线、历史故障复原、已部署的故障取证补丁及双端离线验证。归位稳定性和停止可靠性尚未完成实机验收。** 本次没有启动机器人 SDK 会话，没有发送运动/停止指令，没有调整网络、速度、跟踪阈值或超时预算。

## 1. 本次已经执行的工作

| 项目 | 实际结果 |
| --- | --- |
| 车载入口 | SSH `xc@10.245.247.164`；车载新增产物均在 `/home/xc/code/lcw` |
| 基线 | `/home/xc/code/lcw/VR_teleoperation_runtime/baselines/before-home-audit-20261102T051619Z` |
| 快照 | 16 个文件，包含控制源码、按键/HUD、启动脚本、原位配置、构建清单、原生模块；另存 Git HEAD/status/diff |
| 原生模块指纹 | `6e8e383587c2d546a8c2a5f40cd0dcb3e0903af6aca3391363f629da121e9c58`，与构建清单一致 |
| SDK 指纹 | `a04ad0aa05b2d46fea5a863fe30698950e6434afeafd5a3eaf605c3f0df0ff1b`，与构建清单一致 |
| 历史证据 | 三份车载日志及对应录制已复制到笔记本 `/data/VR_teleoperation_runtime/home-audit-20261102/evidence/vehicle`，保留源文件 |
| 离线分析 | 新增 `teleop/aubo/analyze_home.py`，按请求分组、重复快照去重、保存源文件 SHA256；三份 JSON 报告已生成 |
| 控制补丁 | `quest_follow.py` 只增加归位故障现场记录和每轮清空旧现场；保留原运动/停止决策 |
| 车载部署 | 两个已有文件逐文件更新，两个离线工具/测试新增；替换前核对快照哈希并检查控制进程 |
| 本次验证 | 本地 203 项、车载 203 项定向离线测试均通过；不是完整回归或现场成绩 |

补丁清单和部署 SHA256：车载 `/home/xc/code/lcw/VR_teleoperation_runtime/home-audit-20261102/deployment.json`，笔记本 `…/home-audit-20261102/deployed-hashes.json`。

本次没有改动车载 launcher、原生绑定、SDK、网络配置、相机脚本、原位点文件或其他工作区；没有提交或推送 Git。

## 2. 历史归位要按请求分组

`home_return` 每个状态变化都会产生一条事件，事件条数不是按键次数。分析以 `requested_monotonic_s` 作为一次请求的身份，保留 waiting/ready/moving/stopping/complete/failed 的完整序列；未进入 moving 的“本来就在原位”不能计作成功运动。

| 会话 | 请求 | 真正进入 moving | 最终 complete | 跟踪超限 | 人工/按键取消 | 硬件类失败 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `20261101-154932-5f1156df` | 19 | 19 | 1 | 15 | 2 | 1 |
| `20261101-164502-c8c6a1ea` | 13 | 11 | 4 | 8 | 1 | 0 |
| `20261101-171430-a4d76e6d` | 2 | 2 | 0 | 1 | 0 | 1 |

三份会话中途改过触发方式、阈值和步长，**不能合并成一个“当前版本成功率”**。前两份用于回顾；当前问题优先看最后一份的两次真实运动。三份 `session_status.json` 均为 closed/complete、dropped_records=0；这说明写入没有报告丢记录，不代表运动成功，也不代表暂停期间被策略过滤的 SDK 事件存在。

## 3. 最新第一轮：运动中跟踪超限，停止调用返回成功

所有时刻为同一车载单调时钟，单位秒。

| 事件 | 时刻 / 数值 |
| --- | --- |
| 请求归位 | `30506.211048276` |
| 进入 moving | `30506.251937760` |
| 起点距最终原位最大关节差 | `56.936795°` |
| 已保存的归位 SDK 返回 | 55 次；调用中位 `10.660ms`，P95 `54.957ms` |
| 跟踪错误触发 | `30509.161948944`，`home_tracking_error` |
| 停止调用 | `30509.163918730 → 30510.005832984` |
| 停止调用结果 | `rc=0`，`841.914ms` |

归位参考在向最终原位推进，实际关节跟随落后逐渐增大。已保存的命令前快照中，J1 参考/实测差从约 1.27°、2.61°、5.31°增至 7.05°。这些值是各自命令时刻的诊断值，不能直接充当最后一次保护触发的精确误差。

最后一次命令前快照的 J1 差为约 `9.676°`；其反馈时刻 `30509.031399587`。随后原生日志显示新回调时刻 `30509.155777362`，比前快照新约 124ms。触发归位保护使用的是新反馈，但旧录制没有保存该反馈的六轴 q，因此不能把 9.676° 写成“触发现场实测值”。

同理，旧 `home.max_error_deg=39.560957°` 是**距最终原位的最大关节差**，不是 8° 保护所比较的“实测与最近已接受参考之差”。

可以确认：当前 Follow 归位确实触发了主机跟踪保护。这不是该次 Y 松开，也不是因为未调用透传接口。尚需用完整现场数据区分：真实运动滞后、命令/反馈时间对齐偏差、通信节奏和柜端跟随行为各自的贡献。先保留 8° 阈值，不据此放宽到 12°。

## 4. 最新第二轮：已到位，但停止请求超时

| 事件 | 时刻 / 数值 |
| --- | --- |
| 请求归位 | `30510.210769787` |
| 进入 moving | `30510.226926147` |
| 起点距最终原位最大关节差 | `32.580095°` |
| 已保存的归位 SDK 返回 | 89 次；调用中位 `41.801ms`，P95 `59.951ms` |
| 进入 stopping | `30515.311988297`，距原位 `0.011235661°` |
| 当时仍在途的最后一次 Follow | `30515.308857035 → 30515.404698610`，rc=0；返回时许可已撤销 |
| 停止调用开始 | `30515.404742369` |
| 最后可见关节回调 | `30515.418596749` |
| watchdog 首次 fatal | `30517.410766339`，`stop_completion_over_budget_operator_stop_required` |
| 归位外层失败 | `30517.411919728`，旧外层仅报 `hardware_fault_or_session_finished` |
| 停止调用返回 | `30520.404938849`，`rc=10007`，`5000.196ms` |
| 最终 | `stop_unknown` / `stop_request_failed`，没有完成停稳确认 |

SDK 2.5.0 静态证据与此相符：`sendRequestWaitResponse` 在地址 `0x249874` 将等待截止时间加 5 秒，经 `pthread_cond_timedwait` 等待；超时路径 `0x249b5b` 返回 `0x2717=10007`。参考笔记本 SDK 分析：`/data/robot_SDK/analysis/aubo_follow_timing_stop_review_2026-10-13.md`。

**该 5 秒高度符合 SDK 请求应答等待超时，不能解释为“机械臂需要慢停 5 秒”。** 2 秒是我们自己的异常判定预算；把它改到 6 秒不能解释或消除 rc=10007。此次缺少同步 PCAP，尚不能定位是请求/应答网络、柜端处理，还是 SDK 接收派发/通知链出了问题。停止开始后关节回调停止更新也是下一次必须对照线侧流量的线索。

本次核对当前停止绑定仍调用 `rootServiceRobotMoveControl(RobotMoveStop)`，不是直接 `robotMoveStop()`。SDK 静态分析显示后者在当前库中存在 0x86 分派不匹配风险，因此没有仅凭函数名字替换停止入口，也没有引入同实例跨线程 SDK 调用。

## 5. 取证补丁具体补了什么

`home.failure_evidence` 在中断时生成，下一次请求清空，旧字段及错误字符串保持兼容。

- 跟踪超限：保存**判断使用的**六轴反馈 q、接收时刻、来源与年龄，比较目标、逐轴误差、最大误差、最差关节、8° 门限；不使用旧 pre_send 替代新反馈。
- 同时保存当前 home session、SDK 命令 session/开始/返回时刻及其旧 pre_send，供后续对齐检查；本次未改变 session 判定逻辑。
- 硬件故障：保存原 `first_stop_trigger` / `first_fatal` / 在途调用 / last_stop，避免外层笼统错误盖住首因。watchdog 触发时停止可能尚未返回，最终返回码仍从后续硬件日志/诊断事件读取。
- 将现场嵌入 `home_return`，并在启用独立诊断流时产生 `home_return_interrupted`。即使归位暂停了训练录制，管理事件仍保存；没有恢复暂停期间的普通 XR/SDK 样本。
- 深拷贝故障现场，避免后续硬件快照更新或新请求清空导致历史证据被修改。没有额外 SDK 查询，没有把每周期诊断写盘放入运动线程。

新测试覆盖新旧反馈区分、现场不可变、录制暂停期间保存故障、新请求清空旧证据，以及“到位→停止超时”始终不报告 complete。离线分析测试覆盖重复快照去重、已在原位不算运动、未完成生命周期、不虚构缺失反馈和原样保留 10007。

## 6. 本次测试与复现

| 测试文件 | 本地 | 车载 |
| --- | ---: | ---: |
| `test_aubo_home.py` | 29 PASS | 29 PASS |
| `test_aubo_home_analysis.py` | 4 PASS | 4 PASS |
| `test_aubo_follow.py` | 55 PASS | 55 PASS |
| `test_aubo_six_axis.py` | 72 PASS | 72 PASS |
| `test_aubo_recording.py` | 37 PASS | 37 PASS |
| `test_aubo_follow_analysis.py` | 6 PASS | 6 PASS |
| 合计 | **203** | **203** |

这些测试全部使用假 SDK、合成时钟或文件回放。未运行完整 `aubo_offline_check.sh`、未重新编译原生模块、未做物理运动/停止验证，也没有进行 QUICKACK A/B。历史“313 项”与本次范围不同，不互相替代。

笔记本测试日志和 JSON 汇总：`/data/VR_teleoperation_runtime/home-audit-20261102/`。车载对应日志：`/home/xc/code/lcw/VR_teleoperation_runtime/home-audit-20261102/`。

笔记本复现最新历史会话：

```bash
cd /home/lcw/VR_teleoperation
/data/VR_teleoperation_runtime/.venv/bin/python -m teleop.aubo.analyze_home \
  --session /data/VR_teleoperation_runtime/home-audit-20261102/evidence/vehicle/recordings/20261101-171430-a4d76e6d \
  --log /data/VR_teleoperation_runtime/home-audit-20261102/evidence/vehicle/sixaxis-30min-20261101-171430.log \
  --output /data/VR_teleoperation_runtime/home-audit-20261102/20261101-171430-a4d76e6d-audit.json
```

工具默认只读输入并向 stdout 输出；指定 `--output` 才写目标文件。每份报告包含原始源路径、大小和 SHA256。运行时不导入机器人 SDK、不打开网络连接。

## 7. 下一次现场复测必须补齐的证据

### 7.1 后续补充：单终端取证入口

用户确认在机器人现场后，新增并部署 `scripts/aubo_home_field.sh`。默认仅显示计划；显式 `--execute` 才启动固定的 USB / Follow25 / serial / oneshot 会话，自动启用普通录制和独立诊断，并在 SDK prepare 后启动有限时长、限定目标的 `eno1` 抓包。SDK、速度、阈值、停止路径均未修改。运行时电脑 sudo 密码与 SDK 密码分别在本机终端输入。

新增 `tests/test_aubo_home_field.py` 的 4 项测试在本地和车载通过，检查默认无副作用、参数拒绝和预览边界；两端 `bash -n` 通过。尚未将这些启动边界测试当成完整实机流程验证。

现场首次启动发现车载未安装 `rg`，原入口在 `command -v rg` 检查失败后静默退出；确认当时无控制进程、无取证目录。已改用现有 `grep`、增加明确的缺依赖提示/错误行号和 `--check` 无副作用检查。修复后车载实际 `--check` 通过，扩充后的 6 项启动测试双端通过；旧脚本和测试保存在车载 `…/home-audit-20261102/launcher-before-missing-rg-fix/`。此修复不涉及机器人控制程序。

车载桌面终端执行：

```bash
TELEOP_RUNTIME_ROOT=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_RUNTIME_DIR=/home/xc/code/lcw/VR_teleoperation_runtime \
AUBO_ALLOWED_ROOTS=/home/xc/code/lcw \
bash /home/xc/code/lcw/VR_teleoperation/scripts/aubo_home_field.sh --execute --duration 300
```

登录后先不按 Grip/Y，确认诊断和抓包运行后再进入小行程验证。目录写入 `/home/xc/code/lcw/VR_teleoperation_runtime/latest-home-field.txt`；`field-result.json` 保存会话退出码和 PCAP 文件列表。PCAP 无有效数据时入口返回非零，即使会话本身正常结束；文件存在也不表示捕获完整，仍需核对 capture.log 的 dropped 数、时间覆盖及 SDK 首因。

以下保留手工双终端步骤，便于检查或绕过启动入口的工具问题。

### 7.2 捕获要求和手工步骤

本次已通过 SSH 确认 `192.168.1.5 dev eno1 src 192.168.1.100`，`tcpdump` 和 `timeout` 可用；`sudo -n -l` 返回“需要密码”，尚未启动捕获。以下为**现场有人、可操作物理停止措施时**的下一轮步骤，当前未执行。

1. 保持 USB / Follow25 / serial / oneshot / 现有 8°、20°/s、6°/s² 配置。使用单一遥操进程，先核对当前关节姿态与工作区、原位路径和停止条件，不并行启动徐灿运动服务。
2. 同一会话同时启用 `--record` 与 `--diagnostic-output`；归位会暂停普通训练采样，但新故障管理事件仍写入。
3. **等 SDK 登录及 prepare 完成后再抓包**，避免把登录凭据写进 PCAP。确认 events.jsonl 中出现 `"event": "prepare"`，再在第二个车载本机终端捕获 `eno1` 的目标 8899 流量。不要在捕获期间重新登录另一连接。
4. 首先只观察已登录的待机推送/查询与 ACK 节奏；物理停止处置安排就绪后再做小行程归位/主动取消。停止未知出现后结束本轮，不通过重复按 Y 清故障或继续运动。
5. 保留 PCAP、tcpdump 的 dropped 统计、events.jsonl、session.log、录制目录及配置指纹；先复核停止请求是否上网、应答是否入网、SDK 是否返回，再讨论停止 API/线程或柜端问题。
6. 基线取证后才引入可关闭的 QUICKACK 单变量 A/B/A。PC `TCP_NODELAY` 不能关闭柜端 Nagle；25Hz 下慢峰改善也不能证明实际命令率达到 200Hz。

车载本机启动参数模板（会开启运动能力，不是只读命令；仅供满足上述条件后的现场执行）：

```bash
export TELEOP_RUNTIME_ROOT=/home/xc/code/lcw/VR_teleoperation_runtime
export AUBO_RUNTIME_DIR=$TELEOP_RUNTIME_ROOT
export AUBO_ALLOWED_ROOTS=/home/xc/code/lcw
cd /home/xc/code/lcw/VR_teleoperation
AUDIT_RUN=$TELEOP_RUNTIME_ROOT/home-field-$(date +%Y%m%d-%H%M%S)
mkdir -m 700 "$AUDIT_RUN"
printf '%s\n' "$AUDIT_RUN"
set -o pipefail
bash scripts/aubo_quest_follow.sh \
  --execute-six-axis --command-mode follow --follow-hz 25 \
  --waypoint-feedback --serial-state-query --usb \
  --host 192.168.1.5 --port 8899 --user aubo \
  --rotation world --home-trigger oneshot --duration 120 \
  --record --task '左Y基线取证；不做Placement或ACK干预' \
  --diagnostic-output "$AUDIT_RUN/events.jsonl" \
  2>&1 | tee "$AUDIT_RUN/session.log"
```

第二终端将 `AUDIT_RUN` 设置为第一终端实际输出的同一目录；确认 prepare 事件后，执行有限时长的被动捕获（此命令不控制机器人）：

```bash
# AUDIT_RUN 必须是本次刚打印的 /home/xc/code/lcw/... 路径，不能留空。
: "${AUDIT_RUN:?先设置第一终端输出的取证目录}"
rg -q '"event": "prepare"' "$AUDIT_RUN/events.jsonl" && \
sudo /usr/bin/timeout --signal=INT --kill-after=5 150 \
  /usr/bin/tcpdump -i eno1 -nn -s 0 -B 4096 -U -C 8 -W 4 \
  -Z "$(id -un)" -w "$AUDIT_RUN/wire.pcap" \
  'host 192.168.1.5 and tcp port 8899' \
  2> "$AUDIT_RUN/capture.log"
```

`timeout` 到期可能返回 124，需结合 capture.log、文件存在及 tcpdump 统计判断；不能把“命令结束”当作抓包完整。滚动 PCAP 需保留全部片段并按时间分析，不只取最后一片。sudo 提示输入的是车载电脑密码，与 SDK 密码不同，不把密码写到命令/文档里。

## 8. 回退

前提：结束控制会话、确认没有控制进程，保留故障证据。仅恢复本次改动的两个已有文件即可恢复原控制程序及原归位测试，不恢复整个旧 Git 工作区：

```bash
AUDIT_BASE=/home/xc/code/lcw/VR_teleoperation_runtime/baselines/before-home-audit-20261102T051619Z
cp -p "$AUDIT_BASE/files/repo/teleop/aubo/quest_follow.py" \
  /home/xc/code/lcw/VR_teleoperation/teleop/aubo/quest_follow.py
cp -p "$AUDIT_BASE/files/repo/tests/test_aubo_home.py" \
  /home/xc/code/lcw/VR_teleoperation/tests/test_aubo_home.py
```

新增的 `analyze_home.py` / `test_aubo_home_analysis.py` 是离线分析模块，未加入控制启动入口，可保留用于回放。回退后核对原哈希分别为 `667983db551367864c1a21bb3039e0fdddc3c83a411622f5ec1036303228adb7` 和 `01755a94b309a647ccf51a32844d5ae0ae3ba244fbb6855829269e502f600109`。

## 9. 左 X 的并行准备状态

本轮没有抢先绑定左 X 或加入未经示教的路径。此前已阅读徐灿的 `motion_service.cpp` 与 `auboi5_path_check.cpp`；柜端 P2P 是候选后端，既有固定过渡点不等于可从任意当前姿态安全直达蓝框。

需要现场同徐灿核对的最小信息：当前真正运行的节点/SDK 与模式；果箱过渡点和蓝框释放点如何示教、用什么基座/TCP/负载；JointMove/LineMove 的速度和中途停止/恢复方式；夹剪一体夹爪的实际安装、协议及夹持/释放反馈；“分离且稳固夹持”的确认方式。没有向徐灿发送消息。

这些信息决定 P1 空载到位的配置和路径，后续 P2 才加入夹持门控与释放。停止请求超时先取得同步证据，避免把现有停止问题复制到左 X 自动动作。
