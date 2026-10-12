# aubo_i5_teleop

路线 B 的 ROS2 包：Quest 适配、混合位姿映射、实测重锚、位置跟踪、RG／AG95 夹爪和全量录制。

`launch/route_b_control.launch.py` 启动控制及旁路节点；`scripts/run_session.sh` 在仿真就绪并完成独占归位后调用它。默认 RG。MoveIt 仅作为共用规划/显示设施保留，PoseTracking 执行链已归档。

完整运行说明及限制见工作区根目录 README.md。
