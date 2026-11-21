# 真机六轴遥操入口

当前正式真机基线是 [aubo-c2-refined90-2026-11-12](aubo_teleop_baseline_2026-11-12.md)。本文件只保留启动入口和基线边界，历史试验记录已清理。

## 启动

先在项目根目录准备匹配的 AUBO SDK 和 `/data/VR_teleoperation_runtime` 构建环境，再运行

```bash
bash scripts/build_aubo_follow.sh
bash scripts/aubo_quest_follow.sh
```

默认命令只检查启动计划。真实设备启动使用[真机基线文档](aubo_teleop_baseline_2026-11-12.md#复现与启动)中的完整参数。

## 验证

```bash
source scripts/teleop_env.sh
teleop_env
"$TELEOP_PYTHON" -m unittest tests.test_aubo_six_axis tests.test_aubo_follow tests.test_aubo_refined_stream
```

完整参数、Native 与 SDK 指纹、停止规则和已确认的验收边界见[真机基线文档](aubo_teleop_baseline_2026-11-12.md)。
