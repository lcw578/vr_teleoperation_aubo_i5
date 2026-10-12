# 当前遥操作入口

初步固定基线 `sim-world-rg-2026-10-12`，定义见 [完整基线](docs/simulation_baseline.md)。
操作者已验收world＋RG新姿态；tool_local仅作比较项。
独立 MuJoCo 入口：`bash scripts/run_mujoco_teleop.sh`，默认作者工作区旋转；
`--rotation tool_local` 选择工具局部旋转。共用作者的腕部枢轴与解耦 IK。
默认运动学预览，直接写关节位置并 `mj_forward`，绕过新增加速度整形与位置执行器。
动力学需显式 `--backend physics`，只作实验对照。
HMD 不参与映射，Trigger 独立两态，记录默认关闭。

运行说明与验证边界见 [docs/mujoco_teleop.md](docs/mujoco_teleop.md)。
旧 ROS Route B 保留回退；[docs/route_b_validation.md](docs/route_b_validation.md)
描述的是旧链，不能当作新入口的验证。
