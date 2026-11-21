# AUBO i5 MuJoCo 模型

本目录保存正式 MuJoCo 仿真基线使用的 AUBO i5、RG 和 AG95 场景、网格与 include 文件。运行时直接加载 MJCF 场景，不依赖 ROS 工作区。

## 场景入口

- `scene_ros2_demo.xml` 是默认无夹爪演示场景
- `scene_ros2_rg_cameras.xml` 是 RG 与相机仿真场景
- `scene_ros2_rg.xml` 是 RG 场景
- `scene_ros2.xml` 是基础 AUBO 场景

仿真启动命令见 [仿真基线文档](../../docs/simulation_baseline.md)。模型快照、默认姿态和哈希清单也由该文档维护。

## 模型来源

模型来自 AUBO 官方 ROS2 驱动仓库的 AUBO i5 描述。场景保留官方运动学和惯性参数，并按 MuJoCo 资源布局修正网格路径。`scene_ros2_demo.xml` 中的执行器、接触排除和初始姿态属于当前仿真基线配置。
