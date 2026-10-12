# 数据采集准备

独立 MuJoCo 入口的全量原始记录默认关闭，以 `--record` 显式开启；
图像另需 `--record-images`。字段、状态／命令应用时间、相机快照与输入时间域
见 [docs/mujoco_teleop.md](docs/mujoco_teleop.md)。尚未实现 LeRobot Dataset Builder。
默认运动学模式标记 `state_source=kinematic_preview`，只记录显示关节角与命令；
`q_measured`、物理速度和执行器力反馈为空，不能当作真实跟随或抓取任务数据。

旧 ROS Route B 的 rosbag 服务方式见 [docs/ros_route_b.md](docs/ros_route_b.md)。
历史路线 A 计划保存在 [归档](docs/archive_route_a/DATA_PIPELINE_PLAN_before_route_b.md)。

后续 Dataset Builder 必须明确 action 表达、采样率、图像匹配窗口、源图像年龄、
重复帧和缺失帧策略；不能把下一帧实测关节角默认当作 action，也不能以写盘时间
伪造同步。独立相机恢复完整状态快照；动态抓取场景须在动力学或真机另行验收。
