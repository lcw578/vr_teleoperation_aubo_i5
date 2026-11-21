# 正式遥操基线仓库收敛设计

## 目标

仓库只保留当前已验收的真机六轴遥操基线和 MuJoCo 仿真基线，以及两条基线可运行、可验证、可恢复所需的源代码、模型、配置、测试和文档。旧 ROS Route B、探索性实验、重复入口、历史中间产物和本地构建产物从当前工作树移除。

## 基线边界

### 真机六轴基线

正式入口是 `scripts/aubo_quest_follow.sh`，对应 `teleop.aubo.quest_follow` 和其 Native 控制链。保留其 Python 依赖闭包、Native 源码、AUBO 模型和参数、运行环境配置、正式基线测试、启动说明和验收证据。当前基线参数来自 `aubo-c2-refined90-2026-11-12`。

### MuJoCo 仿真基线

正式入口是 `scripts/run_mujoco_teleop.sh`，对应 `teleop/mujoco_session.py`。保留其 MuJoCo 场景、网格、相机内参、作者固定提交的 `third_party/vr-teleop-kit`、仿真参数、正式仿真测试和使用说明。

## 清理策略

1. 清理前创建本地恢复分支，指向当前提交，并保留既有正式基线标签。
2. 按两个正式入口计算依赖闭包，先迁移仍被 MuJoCo 使用的 `gripper_profile.py` 和 `robot_spec.yaml` 到项目正式目录，再解除 `teleop/mujoco_session.py` 对 `ros2_ws` 的运行时依赖。
3. 移除旧 ROS Route B 源码、旧 `ros2_ws`、历史 probe 和单次实验入口、未纳入正式基线的测试和脚本、旧构建输出、运行日志、缓存和编辑器生成物。
4. 保留正式基线文档、固定标签对应的验收证据和必要参考配置。历史文档只在明确服务于两条基线时保留。
5. 不删除 `/data` 下 SDK、运行时、构建和录制数据，也不迁移用户既有软链接目标。

## 目录目标

```text
README.md
teleop/
  aubo/                 # 真机六轴基线
  mujoco_session.py     # MuJoCo 基线
scripts/
  aubo_quest_follow.sh
  run_mujoco_teleop.sh
  teleop_env.sh
assets/aubo_i5/         # 两条基线实际使用的模型、网格和场景
third_party/vr-teleop-kit/
docs/
  project/              # 项目基线和操作清单
  baselines/            # 机器可核对的正式基线材料
  reference/            # 纳入仓库的必要手册和参考资料
  aubo_teleop_baseline_2026-11-12.md
  simulation_baseline.md
  mujoco_teleop.md
  README.md
environment/
tests/                  # 两条正式基线的离线回归和入口检查
```

## 验证要求

- `teleop/mujoco_session.py` 不再读取 `ros2_ws` 路径。
- MuJoCo 入口可以在独立运行时目录下导入并完成无硬件 smoke check。
- 真机六轴正式模块可以通过离线测试和 Native 构建检查。
- 清理后仓库搜索不到正式入口对已删除路径的运行时引用。
- Git 工作树只包含基线源代码、测试、配置和文档，构建产物、缓存、日志和本地数据不在仓库中。
