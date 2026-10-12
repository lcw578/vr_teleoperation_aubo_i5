# 仿真入口

初步固定基线：`sim-world-rg-2026-10-12`，操作者已验收 `kinematic + world + rg`。
完整链路、参数与文件指纹见 [仿真遥操基线](docs/simulation_baseline.md)。
推荐入口：`bash scripts/run_mujoco_teleop.sh --backend kinematic --rotation world --gripper rg`。
tool_local保留比较项，未获同等操作者验收。
默认 `--backend kinematic`，直接显示作者 IK 输出，不执行动力学与新增加速度整形。
`--backend physics` 仅保留作实验对照，其暂态命令振荡尚未修复。
完整步骤见 [README.md](README.md) 和 [docs/mujoco_teleop.md](docs/mujoco_teleop.md)。

旧 ROS Route B 保留为历史回退，authoritative_session.sh仍进入旧链，不是当前基线入口。
路线 A 历史内容在 [docs/archive_route_a/](docs/archive_route_a/)。
