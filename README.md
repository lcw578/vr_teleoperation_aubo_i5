# Quest 3 / AUBO i5：独立 MuJoCo 遥操作

当前基线 `sim-world-rg-2026-10-12` 已获操作者手感验收，链路与参数／文件快照见
[仿真遥操基线](docs/simulation_baseline.md)。

真机当前进度统一见 [接入真机后的工作总览（2026-10-19）](docs/aubo_real_integration_summary_2026-10-19.md)：
已完成只读与模型核对、J6 连续/改目标实验、Quest 只读验收及两轮 J6 实体联调。
最新慢停时序修正已构建并通过 146 项离线回归，尚待实体复测；六轴遥操未开放。
下文各阶段介绍保留历史进展，当前完成度与证据边界以总览为准。

新增 [Quest + AUBO 只读预览入口](docs/aubo_quest_readonly_preview.md)：
复用原网页和作者 IK，以真实关节反馈建锚，桌面显示未发送候选；原生仅加载只读绑定，实体机械臂禁止运动。
先验收头显映射与恢复门控，再开放后续受限运动阶段，Logger 默认关闭。

新增 [J6 受限跟随后端与现场验证说明](docs/aubo_quest_follow.md)：复用同一作者核心，
只开放 J6 目标，2026-10-26 按用户要求采用控制柜本次返回的完整关节范围、允许双向转动；
其他五轴仍保持启动角度。SDK 串行发送及慢停，预览速度与实体限速分别配置。
代码和离线检查已完成，本入口的实体运动尚未验收。默认脚本只打印方案，不登录、不运动。

AUBO 真机接入已完成[离线只读准备](docs/aubo_offline_preparation.md)及
[首次接机后的只读入口修复与验证](docs/aubo_readonly_fix_2026-10-19.md)：SDK 登录、17 项默认查询、
SDK FK 与退出已在当前控制柜成功执行。连续跟随、故障停止和 Quest 真机运动尚未验收；
[阶段一运行准备](docs/aubo_phase1_progress_2026-10-13.md)中的控制协调仍只使用假后端。
当前真机未安装夹爪。构建产物与设备报告实际存放于 `/data/VR_teleoperation_runtime`。
后续已完成[裸法兰 FK 对照与有限反馈检查](docs/aubo_flange_and_follow_review_2026-10-19.md)：
模型与 SDK 之间确认了 Z 方向 −23.5 mm 的数学基座换算；文档列出了停止接口疑点、
裸法兰适配和独立跟随测试顺序。该结果不表示真实运动或物理标定已验收。
[静止停止诊断](docs/aubo_stationary_stop_test_2026-10-19.md)实测普通 stop 返回 10001，
候选慢停返回 0；该独立诊断入口不导出运动接口，后续运动结果见下。

后续已执行[两次真机 J6 微动与慢停](docs/aubo_j6_micro_trial_2026-10-19.md)：
每次只发送一个 +1° 目标，J6 分别约 +0.54°、+0.55° 后停止，跟随和慢停均返回 0。
首次停后记录问题修复后，第二次完整保存 12 次停后停止状态及最终查询，无预算超限；操作者确认静止、附件线缆正常。
这是监督下的单目标实验，连续跟随、并发停止及 Quest 真机接入仍未验收。

随后完成[一次 J6 低频连续往返实验](docs/aubo_j6_sequence_test_2026-10-19.md)：
21 个小目标全部返回成功，实际发送约 4.8 Hz，J6 从起点转动约 +0.800°并回到起点附近，
慢停和停后观察成功，操作者确认静止且附件、线缆正常。
反向存在可见的跟随滞后；本轮只验证这一小范围序列，不代表已确认目标覆盖语义、正常遥操频率或独立故障停止。

再完成[+10°目标在运动中改回起点的实验](docs/aubo_j6_reversal10_test_2026-10-19.md)：
实际约 +3.009°时改目标，J6 仅继续到约 +3.240°就反向，随后回到起点附近并停止，现场确认无异常。
本轮证实新目标能在旧目标未完成时改变运动；尚未验证高频输入、多目标积压边界或通信故障停止。

随后完成[约 10 Hz 的 J6 连续往返](docs/aubo_j6_sequence10hz_test_2026-10-19.md)：
101 个目标全部成功，实测平均约 9.43 Hz，J6 在起点到 +2°附近往返并停止，现场确认无异常。
最大间隔约 183 ms，其中前一轮 follow 和查询合计约 171 ms；串行诊断入口尚不能提供稳定周期的正式遥操发送。

又完成[20 Hz 设定的 J6 连续往返](docs/aubo_j6_sequence20hz_test_2026-10-19.md)：
201 个目标全部成功，实测平均约 16.19 Hz，J6 在起点到 +2°附近往返并停止。
本轮实际频率较上一轮提高；发送、查询与写盘串行时仍有周期波动，不能将设定值直接视为控制柜更新率。

仿真入口直接复用 vr-teleop-kit 作者的遥操循环、目标管理、腕部校准、
3＋3 解耦 IK、网页和 WebRTC Relay，使用 AUBO／RG 模型直接显示 IK 关节位置。
默认是作者式运动学预览：`qpos → mj_forward`，不执行动力学或新增的加速度整形。
ROS 不再是这个入口的运行前提。默认 RG，AG95 可选；完整记录默认关闭。

```bash
cd /home/lcw/VR_teleoperation
bash scripts/run_mujoco_teleop.sh --backend kinematic --rotation world --gripper rg
```

新入口自带 relay，不需要另外运行旧 run_relay.sh。启动前退出旧 ROS 会话和
旧 relay；工作区锁与端口检查会阻止冲突，不会强制结束其他程序。
LAN 头显地址是 `https://电脑当前局域网IP:8443/`。先完成页面右手腕部校准。
USB、记录开关、按钮、时间戳与完整步骤见 [独立 MuJoCo 使用说明](docs/mujoco_teleop.md)。

两种模式共用作者的腕部枢轴目标管理与解耦 IK；HMD 朝向不参与控制映射。
相机安装保持原值；RG 起始／摇杆归位采用用户选定的预览姿态：上臂前倾、
前臂水平、夹爪摆正、腕部相机在上方，腕 3 为零位，双向关节余量约 168°。
此姿态改变了 TCP 起始位置；具体参数和验证边界见使用说明。
当前完成运动学、原作者数值基准与本机传输核验，
world与当前姿态已获实际Quest手感验收；tool_local局部轴对应、长时无线可靠性
仍需单独验收，未验证真机。

作者代码固定于 `8d37f3d6bd44c8ce646f7bc110ac98e802e0ab57`，Apache-2.0。
完整小仓库保存在 `third_party/vr-teleop-kit/`；
[上游与改动清单](third_party/vr-teleop-kit/LOCAL_CHANGES.md)记录必要适配。

`--backend physics` 显式选择保留的动力学实验入口；其参考整形已复现暂态命令振荡，
不作为当前手感验收基线。运动学模式不模拟物体抓取、重力下落或接触力反馈。

旧 ROS Route B 入口保持可运行作为回退，使用方法见 [ROS Route B 文档](docs/ros_route_b.md)。
旧 authoritative_session.sh 仍指向 ROS，不代表当前基线入口。
路线 A 的历史代码与实验文档见 [归档](docs/archive_route_a/README.md)。

## 本机环境入口（2026-10-26）

VR 已使用独立 Python 与干净 ROS overlay，不再借用 tomato_robot 或农场工作区。启动命令和复现方法见 [环境说明](environment/README.md)。旧 ros2_ws/install 仅保留回退；本机使用 scripts/teleop_env.sh 中的 teleop_ros_env 或 project-env vr-ros。
