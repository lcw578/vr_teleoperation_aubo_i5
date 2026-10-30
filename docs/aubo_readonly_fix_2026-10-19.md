# AUBO 只读入口修复与接机验证

2026-10-19：当前 AUBO i5 未安装末端夹爪。本轮仅修复 SDK 加载和只读检查，不新增运动、停止、上电、清错、参数写入或夹爪接口。作者控制核心及已验收的 RG 仿真入口未修改。

## SDK 同名符号冲突

SDK 2.5.0 发布库导出 tiny-regex 的 `re_match(pattern, text, int*)`，系统 libc 也导出参数定义不同的同名函数。首次默认加载时，SDK 内部的 `re_match` 动态重定位绑定到了 libc，接机触发段错误或登录内部字符串越界异常。

`teleop/aubo/readonly.py` 现在先校验原有构建清单中的 SDK SHA256，再以绝对路径和 `RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND` 装载该 SDK，随后导入只读扩展。保留库句柄，核对实际映射路径与查询 allowlist；不修改 SDK 二进制、Python 全局 dlopen 标志、系统配置或共享环境变量。

同一进程若已经通过其他入口装载 AUBO SDK，不能靠后续 dlopen 修复既有绑定，因此在构造 SDK 客户端之前拒绝继续，并要求重启 Python 进程。重复调用经过该加载器的同一个模块则允许。加载步骤由锁串行化，避免本模块的并发调用观察到半加载状态。

加载器异常现纳入 CLI 的错误报告；此时不询问密码、不登录。默认无 `--connect` 时仍只输出计划，不加载 SDK。

## 默认查询与结果含义

- 原生 `QUERY_NAMES` allowlist 保持不变。
- 默认探测排除 `joint_position_limits`。证据为 2026-10-19、服务端 `V4.5.111.71f2ee1-Alpha` 上该查询返回 `10007`，耗时 5.000143 秒。这是当前设备的实测结果，不能推广为所有 AUBO 控制柜均不支持。
- 离线计划和连接报告包含 `query_selection`、`requested_queries`、`default_excluded_queries`，明确列出排除项及原因；其他查询仍按原有失败策略处理，不自动重试。
- 显式指定 `--query joint_position_limits` 仍会执行该诊断。若它超时，报告保留返回码，停止该会话后续查询并退出；本轮没有在真机上重新调用这项已知超时查询。
- `tool_kinematics` 增加 `data_source=sdk_client_move_profile` 及说明。该 SDK getter 读取客户端运动配置，不能将零偏移解读成示教器 TCP 已核实。
- SDK FK 增加 `data_source=sdk_local_fk`，继续记录其输入关节；不能把本地计算耗时当成控制柜响应或采样周期。

## 验证结果

离线执行 `bash scripts/aubo_offline_check.sh`：

- 37 项离线接口测试、17 项模型/核心协调测试、5 项离线运行测试、27 项仿真测试，共 86 项通过。
- 原生绑定检查通过；从 ELF 重定位表读取 SDK 内部 `re_match` 槽位，验证其确实指向 SDK 自身实现，不只检查导出符号地址。
- 独立子进程验证：未经隔离预先装载 SDK 会被拒绝，期间不构造 SDK 客户端、不连接设备。
- Python 全局 dlopen 标志未改变；未登录时继续拒绝所有查询及有效输入的 FK，无运动 API 开放。

修复后在新的 Python 进程调用正式 `teleop.aubo.probe.main`，仅为无人值守输入复用已有成功部署源码中的账户；未覆写加载器或查询流程，密码不写入报告、文件或命令行。实际结果：

- 地址 `192.168.1.5:8899`；SDK 发布库与既有构建清单一致。
- 登录、17 项默认查询、1 项 SDK FK、退出均返回 0。
- 无异常、无反馈验证错误、无非有限数值；未调用默认排除项。
- 网络 Get 本次约 5.3–15.4 毫秒；登录约 374 毫秒；从开始登录到退出返回约 494 毫秒。客户端状态、工具运动配置和本地 FK 的微秒级耗时不混入网络延迟统计；本次耗时不代表高频控制或实时保证。
- 查询时机器人状态为停止，诊断未报告急停、碰撞或关节错误。

原始 JSON：`/data/VR_teleoperation_runtime/aubo-readonly-fixed-20261019-155921.json`。
首次接机及示教器照片核对：`/data/VR_teleoperation_runtime/aubo-readonly-review-20261019.md`。

SDK 自身仍输出 `Get movep joint range failed` 和 `Ikfunc::SetDhParaLen unkown robottype[0]`，随后识别 i5 并完成查询。本轮没有消除或隐藏这两条初始化日志，原因仍需单独核对。

## 使用

```bash
cd /home/lcw/VR_teleoperation

# 离线查询计划；默认不连接、不录制
bash scripts/aubo_readonly_probe.sh

# 当前设备的一次有限只读检查，交互输入 SDK 密码
bash scripts/aubo_readonly_probe.sh --connect \
  --host 192.168.1.5 --port 8899 --user aubo

# 离线回归，不连接设备
bash scripts/aubo_offline_check.sh
```

显式保存报告时加 `--output /data/VR_teleoperation_runtime/<新文件名>.json`，禁止覆盖已有报告。这是一次性诊断报告，不是开启持续 Logger。

后续已完成[裸法兰数学对照和有限反馈采样](aubo_flange_and_follow_review_2026-10-19.md)。
控制权限、物理基座约定及运动中的关节方向仍需核对；未实现或验收真实跟随和停止，不能用本次只读成功替代运动测试。
