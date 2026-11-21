# AUBO i5 遥操基线

本仓库只保留两条正式入口

- 真机六轴遥操，入口为 `scripts/aubo_quest_follow.sh`
- MuJoCo 仿真遥操，入口为 `scripts/run_mujoco_teleop.sh`

## 真机六轴基线

当前固定版本是 [`aubo-c2-refined90-2026-11-12`](docs/aubo_teleop_baseline_2026-11-12.md)。该基线使用 AUBO i5 六轴控制、Quest 输入和受保护的 TCP2CAN 透传链路，默认关闭录制、实体夹爪和数据采集。

正式启动参数见[真机基线文档](docs/aubo_teleop_baseline_2026-11-12.md#复现与启动)。

源码指纹、SDK 要求、参数、停止规则和验收证据见[真机基线文档](docs/aubo_teleop_baseline_2026-11-12.md)。

## MuJoCo 仿真基线

当前固定版本是 `sim-world-rg-2026-10-12`，默认使用 AUBO i5 与 RG 模型。

```bash
cd /home/lcw/VR_teleoperation
bash scripts/run_mujoco_teleop.sh --backend kinematic --rotation world --gripper rg
```

参数和操作步骤见[仿真基线文档](docs/simulation_baseline.md)与[仿真使用说明](docs/mujoco_teleop.md)。上游适配代码保存在 `third_party/vr-teleop-kit/`。

## 环境

项目运行时位于 `/data/VR_teleoperation_runtime`，Python 入口统一使用 `scripts/teleop_env.sh`。环境依赖和验证命令见[环境说明](environment/README.md)。AUBO SDK 通过 `robot_SDK` 软链接接入 `/data/robot_SDK`。

## 目录

- `teleop/` 保存真机与仿真运行时代码
- `assets/aubo_i5/` 保存 MuJoCo 模型和场景
- `tests/` 保存两条基线的回归测试
- `docs/` 保存基线、验收证据和设备手册
