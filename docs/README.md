# 基线文档

仓库当前维护两条正式遥操基线。

- [真机六轴基线](aubo_teleop_baseline_2026-11-12.md)
- [MuJoCo 仿真基线](simulation_baseline.md)
- [MuJoCo 使用说明](mujoco_teleop.md)

`baselines/` 保存可复核的基线快照和哈希清单。`project/` 保存基线摘要与头显准备清单。`reference/` 保存 AUBO-i5 与 CB4 用户手册。

真机和仿真共用 `teleop/` 运行时代码。启动脚本位于 `scripts/`，测试位于 `tests/`。
