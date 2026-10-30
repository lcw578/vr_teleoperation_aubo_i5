# AUBO i5 离线接入准备

本文记录 2026-10-13 的只读 SDK 接入和假后端验证；当日没有连接控制柜或执行真实运动。

**2026-10-19 更新：已完成当前控制柜的只读接机验证，详见[入口修复与验证](aubo_readonly_fix_2026-10-19.md)。**
正式加载器隔离 SDK 的同名符号；默认探测显式排除已实测超时的 `joint_position_limits`，报告保留原因，
仍可使用显式 `--query` 诊断。工具运动学结果标注为客户端配置，FK 标注为本地计算。
本次未执行真实运动，跟随和停止仍未验收。下面的“本轮”与离线核验记录均指 2026-10-13。

作者 IK、归位姿态、相机挂载与控制参数不变。仿真入口唯一行为改动为显式录制的存储位置：新录制必须在 `/data`，默认 Logger 仍关闭。

## 已实现范围

- 一个小型 pybind11／C++ 绑定，链接现有 Linux x64 SDK 2.5.0 发布库；只有 Login、允许的 Get 查询、FK 和 Logout，没有 startup、跟随、透传、stop、清错或参数写入入口。
- Python `ReadOnlyFollower`，查询结果保留返回码、耗时和主机接收单调时间；失败不把结构体初始化的零值伪装成测量。
- `FeedbackCache`，检查六关节 rad 数据、有效性、主机接收时间、过期／乱序。它不宣称控制器采样时间已知。
- 默认不访问网络的只读探测 CLI；必须明确指定连接开关、地址、端口和账户才登录。密码交互输入，不放在命令行或报告中。
- 假后端下的最新目标槽、实测锚定、输入／反馈失效、会话撤销、故障后松开再按 Grip、发送跳变检查与响应未知时不重试。
- `AuboModel` 按名称把六个臂关节填入完整模型；其余关节明确是显示模板。FK 对照须显式提供坐标变换及其来源，不自动猜测法兰/TCP关系或宣布验收通过。
- `AuboShadowCoordinator` 直接复用作者核心并接入反馈有效性和假会话检查，生成候选但没有真实运动入口。可转入只读 follower 已取得的有效反馈，不在控制 tick 中调用阻塞 SDK 查询。输入必须含有效右手、按钮、epoch/seq/t_client；首次及故障后要求松开再按 Grip，屏蔽摇杆归位。
- 异常时间戳会撤销会话目标；不可解析/未来反馈时间使缓存失效，合法较旧包拒绝并保留更新样本。假 stop 失败后锁住会话，不因松开 Grip 就复位。
- SDK 原始查询结果与主机验证结果分开保留；验证失败时不执行后续采样或 FK，不重试。

`OfflinePositionSession` 强制只接受 `FakePositionBackend`，不会发送真实运动。它是离线协议演练，`check()` 由调用者触发；目前没有真机 watchdog 线程或实际停止通道，不能将其称为已完成的真机后端。

本轮反馈缓存接入的是串行查询结果，尚未注册控制柜推送回调；不承诺轮询能够得到比控制柜更高频的新测量。推送频率、事件处理、回调生命周期和连续运动并发，留待设备资料与只读接机核对后完善。

`AuboShadowCoordinator` 是单个调用者驱动的协调模块，尚不是带网络接收线程/独立 watchdog 的真机运行入口。只读观测可以来自控制柜，但 `dispatch_fake()` 永远只提交给假设备，不会把模型或假设备输出伪装成控制柜反馈。夹爪没有串口连接、自动开爪或退出扭矩指令。

## 文件与存储

| 内容 | 位置 |
| --- | --- |
| 小型源码 | `teleop/aubo/` |
| 构建产物与清单 | `/data/VR_teleoperation_runtime/build-readonly/`、`/data/VR_teleoperation_runtime/readonly-build.json` |
| SDK 实体 | `/data/robot_SDK/新版SDK/aubo_sdk-2.5.0-linux64.bf7070c` |
| 测试 | `tests/test_aubo_offline.py`、`tests/test_aubo_model_shadow.py`、`tests/smoke_aubo_readonly_binding.py` |

使用已安装的系统 pybind11 2.9.1、CMake 和 C++ 编译器，以及已有 Python 3.10 环境，没有向共享虚拟环境安装或升级包。构建只链接发布库，不同时链接调试库。

构建清单包含库路径、SHA256、解释器和模块目录。加载时核对库文件哈希与实际映射路径，避免运行另一份同名 SDK。这个清单证明构建来源，不证明控制柜兼容。

## 现在可运行的全部离线指令

```bash
cd /home/lcw/VR_teleoperation

# 只显示查询计划：不加载 SDK、不连接设备、不写报告
bash scripts/aubo_readonly_probe.sh

# 一次运行离线接口、组合模型/核心、仿真回归与绑定检查；不访问设备
bash scripts/aubo_offline_check.sh

# 重新构建；输出仍在 /data，不登录设备
bash scripts/build_aubo_readonly.sh

# 假后端与只读入口逻辑测试，无硬件调用
/home/lcw/tomato_robot/.venv/bin/python tests/test_aubo_offline.py

# 真实作者核心与AUBO模型的组合故障验证，测量/发送均使用假设备
/home/lcw/tomato_robot/.venv/bin/python tests/test_aubo_model_shadow.py

# 加载已编译绑定，检查ABI和访问边界；不构造SDK client、不调用login
/home/lcw/tomato_robot/.venv/bin/python tests/smoke_aubo_readonly_binding.py
```

可通过 `AUBO_RUNTIME_DIR`、`AUBO_SDK_ROOT`、`TELEOP_PYTHON` 指定其他已存在目录或解释器。构建脚本要求输出目录解析后位于 `/data`，不会自动回退到主目录，也不自动申请管理员权限。

## 接机后才使用的只读指令模板

下面是模板，不是本次已执行的命令。先确认实际地址、SDK端口、账户和控制柜状态；将三个占位项替换成真实值。

```bash
cd /home/lcw/VR_teleoperation
bash scripts/aubo_readonly_probe.sh --connect \
  --host '<实际控制柜地址>' \
  --port '<实际SDK端口>' \
  --user '<SDK账户>'
```

默认在终端打印一次有限长度报告，不持续录制。若只查询部分信息，可多次使用 `--query`，例如 `--query server_version --query joint_angles`。

明确要求保存时，在一个已有 `/data` 目录里指定不存在的 JSON 文件：

```bash
# 附加在上面的实际连接命令末尾；不覆盖已有报告
--output /data/VR_teleoperation_runtime/read-only-device-report.json
```

读到通信断开、请求超时或发送失败返回码时停止后续查询，不自动重试或重连。每项返回码原样保存；查询失败不自动 startup 或写配置。辅助查询失败会将反馈缓存标为无效，属于当前只读实现的保守处理，不能直接作为未来高频控制的全部故障策略。

如果正常查询均成功且有六关节位置，则调用 SDK FK 记录比较材料。报告中的 SDK 路点／FK 法兰或TCP定义仍须核对；这不是自动宣布模型FK一致。模型对照、示教器角度核对和真实安装检查在下一阶段完成。

## 显式只读采样和离线 FK 对照

只读接机时可添加 `--feedback-samples <次数>` 和 `--feedback-period <主机查询间隔秒>`，两项必须一起提供，没有默认采样周期。它只进行有限次串行 `GetJointAngleInfo`，报告调用耗时、主机接收间隔与原始角度；不订阅推送、不运行后台服务。失败后停止采样，不自动重连；不能将查询频率当作控制柜新测量频率或跟随发送周期。

保存报告后，FK比较完全离线运行，不加载SDK、不连接设备：

```bash
bash scripts/aubo_fk_compare.sh \
  --report /data/VR_teleoperation_runtime/read-only-device-report.json \
  --alignment /data/VR_teleoperation_runtime/frame-alignment.json
```

alignment文件格式如下；null必须根据对应基准变换替换为实际4×4刚体矩阵，不能直接运行本模板：

```json
{
  "model_frame": "wrist3_Link",
  "provenance": "填写基准约定、资料来源或实际标定记录",
  "sdk_base_from_model_base": null,
  "model_frame_from_sdk_pose_frame": null
}
```

变换公式为 `T_sdk_base_sdk_pose = A × T_model_base_model_frame × B`。模型帧可选 `wrist3_Link` 或 `tool0`。SDK FK记录保存本次输入关节，比较必须使用这同一组关节；旧报告缺少输入关节时拒绝比较，不从其他时刻的反馈猜测。报告只输出位置/朝向残差，不预置“合格阈值”，单姿态一致不能证明整机标定正确。

## 录制存储

正式仿真启动不带 `--record` 时不创建录制目录。显式 `--record` 默认使用 `/data/VR_teleoperation_runtime/recordings/mujoco/<时间>`；自定义录制路径也要求解析后在 `/data`。已有项目数据没有迁移。录制图像仍须额外启用 `--record-images`，本轮没有接入 LeRobot数据集或实际相机。

## 不在本轮确定的参数与行为

- 实际控制柜型号、固件、模式和控制权，以及接口适用性。
- 跟随最新目标覆盖／排队、允许发送周期、实际调用延迟。
- 停止入口、并发连接方式、断流／断网行为。
- 真机速度／加速度、跟随误差与各超时阈值。
- RG 夹爪退出是否保持扭矩、反馈编码与物理开度标定。
- 相机和 LeRobot 采集接口。

`SessionPolicy` 没有真机默认数值，测试使用的是明确的人工数值，仅用于边界验证。运动方法在只读 follower 中直接拒绝，不能添加一个启动参数就启用真机遥操。

## 本轮核验记录（2026-10-13）

- 只读扩展编译和链接成功，产物约 1.8 MB（含构建目录），位于 `/data/VR_teleoperation_runtime`。
- 离线接口、模型/核心组合与仿真回归通过，详见本轮[实现与验证记录](aubo_offline_completion_2026-10-13.md)。
- 绑定加载、查询 allowlist、未登录拒绝、FK 输入边界和重复加载检查通过。
- 默认探测入口输出 `offline_plan`，不加载 SDK，不访问设备。
- ELF 导入符号核查：绑定只引用 SDK 构造／析构、Login／Logout、列出的 Get 和 FK；没有运动／配置写入函数。
- SDK 动态库在绑定加载检查中被加载；未构造 `ServiceInterface`、未登录、未查询或控制真实设备。不能将这些离线结果描述成 SDK 正常响应耗时或真机测试通过。
- 原作者控制核心未改动；仿真录制路径改动后重新运行仿真回归与本机WS/WebRTC传输核验。

部署决策与作者差异的详细证据见 `/data/robot_SDK/analysis/aubo_deployment_full_audit_2026-10-13.md`。下次接机首先执行只读核对，再依据设备证据完善运动接入。
