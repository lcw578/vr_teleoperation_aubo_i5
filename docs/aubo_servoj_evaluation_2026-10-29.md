# ServoJoint 独立评估（2026-10-29）

目标：与现有单连接串行 Follow 对照接口调用耗时、发送间隔和实测关节响应。现有 Follow 入口保持不变，不把状态查询并行化和运动接口切换混为一次变更。

## 已核实的边界

- 当前正式绑定使用 ServiceInterface SDK 2.5.0，不包含 ARCS 的 `servoJoint`。
- 本地官方 ROS2 驱动引用 ARCS SDK `0.24.1-rc.3+318754d`，通过 RPC 调用 `setServoMode` / `servoJoint`，反馈使用 RTDE。这不是当前柜端 `V4.5.111.71f2ee1-Alpha` 已支持它的证明。
- 新 SDK 和下载包放在 `/data/robot_SDK` 的独立子目录，不替换旧 SDK 或其软链接；测试输出放 `/data/VR_teleoperation_runtime`。
- 默认仅输出离线计划。只读兼容检查只能调用连接、登录及 Get/查询方法；进入伺服、发送、停止和退出属于后续显式运动测试。
- SDK 示例可能包含上电、归位、运行程序和无限重试，不直接运行厂家示例。

## 验证顺序

1. 核实 SDK 实际签名、返回码、版本、停止和断流语义，完成离线模拟。
2. 显式运行只读兼容检查，记录柜端版本、机器人名称、模式、关节限位和真实反馈。
3. 兼容检查通过后，独立短程测试先以实测姿态建立首点，再执行有限平滑往复；所有发送和停止由唯一 owner 串行调用。
4. 使用同一轨迹定义分别测 Follow 与 ServoJoint；记录不同 SDK 和反馈路径差异，不能只比较请求频率。
5. 运动及停止验证后才考虑接回 Quest 和 30 分钟稳定性验收。

厂家资料（2026-10-29 查阅）：

- https://developer.aubo-robotics.cn/guide_sdk/7_servoj_func_test.html
- https://developer.aubo-robotics.cn/application_notes/53-servoj/index.html
- https://developer.aubo-robotics.cn/guide_sdk/4_dev_preparation/3_version_control.html

公开文档中的 `t >= 0.005` 和约 `5*t` 无指令退出描述，必须结合实际柜端/SDK版本核实；不能当作已测通信时延或当前设备的停止保证。

## 本次已实现的入口

`scripts/aubo_servoj_probe.sh` 是独立进程的只读兼容检查，不是遥操运动入口。默认不加载 SDK、不连接控制柜。其原生绑定只暴露 `connect`、`inspect`、`sample_joint_positions`、`logout`，不暴露伺服模式切换、运动、停止、上电或参数写入。

构建和 SDK 文件实际存放位置：

- SDK：`/data/robot_SDK/arcs-servoj-0.24.1-rc.3/`
- 原生扩展：`/data/VR_teleoperation_runtime/build-servoj-probe/`
- 构建记录及库、扩展 SHA256：`/data/VR_teleoperation_runtime/servoj-probe-build.json`
- 接柜报告默认目录：`/data/VR_teleoperation_runtime/servoj-reports/`

本地重建及离线检查：

```bash
cd /home/lcw/VR_teleoperation
bash scripts/build_aubo_servoj_probe.sh
bash scripts/aubo_servoj_probe.sh
/data/VR_teleoperation_runtime/.venv/bin/python tests/test_aubo_servoj_probe.py
/data/VR_teleoperation_runtime/.venv/bin/python tests/smoke_aubo_servoj_probe_binding.py
```

首次接柜只读取版本、机器人名称、运行与安全状态、伺服模式编号、实测关节角和速度、物理关节限位。多机器人时要求显式传入 `--robot`，不会默认选第一个。

```bash
cd /home/lcw/VR_teleoperation
bash scripts/aubo_servoj_probe.sh \
  --connect --host 192.168.1.5 --port 30004 --user aubo
```

这里 `30004` 是 ARCS SDK 的 RPC 端口；原 Follow 的 `8899` 属于原协议，原入口没有修改。ARCS 登录账户是否与旧 SDK 相同仍需实测。程序在终端隐藏输入密码，不通过命令行或环境变量传递密码；SDK 原始日志和异常文字不输出到报告。程序使用与现有遥操相同的会话锁，已有遥操会话时先拒绝检查，避免同时登录测试。

每个读取方法记录调用开始、返回的主机单调时钟，以及调用耗时。它们是**逐项串行查询，不是同一时刻的状态快照**。物理限位不是已核实的示教器安全配置。SDK 请求超时默认 1000 ms，可通过 `--request-timeout-ms` 调整到 100..5000 ms；这是 SDK 请求超时设置，不是已经验证的连接/整个进程硬截止时间。

只读检查完成后，可另做有限的 getter 耗时采样：

```bash
bash scripts/aubo_servoj_probe.sh \
  --connect --host 192.168.1.5 --port 30004 --user aubo \
  --samples 100 --sample-hz 25
```

该采样只调用 `getJointPositions`，计算中位数、P95、最大耗时，以及按真实调用开始时间计算的实际请求频率；慢调用后跳过已经错过的周期，不补发积压请求。采样最多 200 次，计划时长最多 30 秒，读取失败后结束，不自动重试。**这些数值衡量只读 RPC 往返，不代表 ServoJoint 发送频率、运动响应延迟或最高可用频率。**

报告自动使用唯一文件名，不覆盖已有报告；也可用 `--output /data/.../new-name.json` 指定新文件。报告包含版本、构建来源、逐项查询计时、失败阶段、数据校验结果和注销/断开结果。即使查询失败，也保留此前读取的部分结果；非有限数据会被标为无效并在 JSON 中记为 `null`。

## 如何判读并继续

- `connection.return_code=-8`：按 SDK 定义是连接被拒绝；没有进入 Servo 模式。
- `connection.return_code=-15`：按 SDK 定义是 SDK/服务版本不兼容；不得据此要求用户升级控制柜。
- `status=connection_or_login_failed`：查看 `connection.phase`，区分连接和登录。
- `status=inspection_failed`：查看 `inspection.queries` 最后一个查询及 `validation_errors`。这类错误不能直接归咎于网线或遥操手速。
- `read_only_compatibility_passed=true`：本次只读方法成功且数据通过校验。**仍不能证明控制柜接受 `servoJoint`**。
- `cleanup_failed=true`：注销或断开失败，即使之前读取成功也返回非零退出码。

报告保留 `servo_joint_supported_by_controller=null` 和 `ready_for_motion_test=false`。尚未实现或执行的环节是：实柜 RTDE 反馈验证、与当前柜端匹配的伺服进入/停止/断流行为核实、有限运动测试，以及同轨迹 Follow/ServoJoint 对照。不能在缺少这些证据时把现有 Quest 遥操直接切换到 ServoJoint。

## 本地验证结果

2026-10-29：独立 C++ 扩展编译成功；17 项新增 Python 测试通过；真实 ARCS 扩展在未连接状态下的 API、超时参数、拒绝未登录读取、注销以及新旧 SDK 隔离检查通过。现有 `scripts/aubo_offline_check.sh` 全部通过，包括 Follow 原生假 SDK 测试。本次没有连接控制柜，没有测得实际 ServoJoint 耗时或支持频率。

## 随后的实柜兼容检查：连接被拒绝

用户在北京时间 2026-10-29 15:07 运行只读检查，报告为：

`/data/VR_teleoperation_runtime/servoj-reports/probe-20261029T070726-de360b91.json`

- 失败阶段是 `connection.phase=connect`，返回 `-8`，没有进入 SDK 登录、版本读取、模式切换或运动阶段。
- 连接调用耗时约 0.253 ms；这是失败建连的耗时，不能当作控制延迟。
- 随后使用标准 TCP socket 分别连接三个已知服务端口，没有发送应用协议或运动数据：`8899` 建连成功，`30004` 与 `30010` 均为 `ECONNREFUSED`。
- 路由检查显示目标 `192.168.1.5` 经 `enp3s0`，源地址 `192.168.1.100`；此次服务连接与 Quest 的 Wi-Fi 输入链路分开。

结论：从当前电脑访问控制柜时，原 SDK 服务端口可达，ARCS RPC/RTDE 端口拒绝连接。失败不在密码认证、查询并行化、目标发送频率或 ServoJoint 运动阶段。当前这条 ARCS ServoJoint 路径不可用，增加请求超时或修改运动频率没有依据。

这些证据尚不能单独区分“当前控制软件不提供该服务”“服务没有启用/采用其他地址端口”或“访问规则主动拒绝”，也不能证明该控制柜硬件永久不支持 ServoJoint。后续应依据这台柜端软件版本的厂家兼容/服务说明确认 ARCS 支持条件，不把 ARCS 协议直接改发到旧协议的 `8899`，不擅自升级或更换控制软件。

旧 SDK 另有 TCP→CAN 透传接口，项目中也已有 `--command-mode passthrough` 代码。它与 ARCS `servoJoint` 不同，已有代码不等于已完成运动、停流及退出的实柜验收；本轮没有切换该模式，也没有测得其延迟收益。
