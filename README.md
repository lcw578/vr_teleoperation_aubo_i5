# Quest 3 / AUBO i5：独立 MuJoCo 遥操作

当前基线 `sim-world-rg-2026-10-12` 已获操作者手感验收，链路与参数／文件快照见
[仿真遥操基线](docs/simulation_baseline.md)。

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
