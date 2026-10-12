> 此文档保留旧 ROS Route B 的使用方式与历史验证边界。新独立入口见 [mujoco_teleop.md](mujoco_teleop.md)。命令路径仍相对项目根目录。

# AUBO i5：路线 B 仿真遥操作基线

当前正式控制链只有路线 B，默认自研 RG，AG95 可选。项目仍处于仿真验证阶段；AUBO 实际控制柜版本和 RG 真机通信协议待确认。本次更新未声称已通过头显或真机验收。

```text
Quest WebXR → relay → quest_adapter → clutch_mapper → target_pose
                                            ↓ control（实测锚点／离合心跳）
                                        lara_style_tracker
                                            ↓ 关节位置命令
                                ros2_control → MuJoCo → joint_states
Trigger → gripper_fsm → 独立夹爪控制器       ↓
                           相机、实测 FK、全量 rosbag Logger
```

- 平移保持基座／工作区方向：前推 → −Y，右移 → −X，上抬 → +Z。
- 旋转采用手柄局部轴 → 工具局部轴。不同初始朝向下，“拧手柄”对应绕当前夹爪伸出方向拧动。
- Quest 输入为 `xyzw`，计算内部为 `wxyz`，ROS 消息为 `xyzw`。不再使用经验 `yxz` 换轴。
- WebXR grip frame 的前向为 −Z；工具接近方向为 +Z。局部轴变换采用 `Rx(π)`，保持右手坐标系；绕 −Z 的正旋转对应工具 +Z 的正旋转。其余轴对应和手感仍需头显验收，不能把离线通过等同于人体操作验证。
- RG 工具帧取固定 `rg_base`，位置偏移 `(0,0,−0.1048)` 与 URDF 的 TCP 一致，工具姿态偏移 `Rx(π)`。不使用转动齿轮定义工具姿态，因此夹爪开合不改变工具姿态。相机安装及六轴归位角度保留。
- Grip 每次接合使用新鲜实际关节状态及 FK。tracker 使用同一个关节锚点，旧会话目标不再接续；释放会显式取消旧目标。
- Trigger 与 Grip 独立，按一次在 OPEN／CLOSED 间切换；连续关节命令只是两态之间的斜坡，不是额外动作维度。

## 构建及运行

使用现有依赖及 `/home/lcw/tomato_robot/.venv/bin/python`（ROS Humble、MuJoCo、numpy、yaml、websockets、OpenCV）。固定外部依赖见 `deps.repos` 和 `DEPENDENCIES.md`。

```bash
source /opt/ros/humble/setup.bash
cd /home/lcw/VR_teleoperation/ros2_ws
colcon build --packages-select aubo_i5_teleop --cmake-clean-cache
source install/setup.bash
cd /home/lcw/VR_teleoperation

# 另一个终端启动 relay，头显进入现有 HTTPS 页面
bash ros2_ws/src/aubo_i5_teleop/scripts/run_relay.sh

# 前台运行完整 B＋RG 会话，Ctrl+C 关闭本会话拥有的进程
bash ros2_ws/src/aubo_i5_teleop/scripts/authoritative_session.sh

# 键盘输入；仍走 B，夹爪默认 RG
INPUT=mock_vr bash ros2_ws/src/aubo_i5_teleop/scripts/run_session.sh

# 保留的 AG95 可选配置
GRIPPER=ag95 bash ros2_ws/src/aubo_i5_teleop/scripts/authoritative_session.sh
```

`vr_session.sh`、`run_demo.sh`、`run_teleop.sh` 使用同一个 supervisor。`run_teleop.sh` 选择键盘；`--vr` / `--mock` 可显式选择输入。`restart_vr_chain.sh` 也重启完整会话，需先退出原 supervisor。工作区文件锁防止重复启动；不再用全局 `pkill` 清理其他 ROS 项目。

`CAMERAS=false`、`RECORDING=false`、`MOVEIT=false` 可关闭旁路模块。Logger 默认关闭；只有设置 `RECORDING=true` 才启动记录节点，启动后仍需显式调用 `/recording/start` 才会创建 rosbag。`TELEOP_PYTHON` 可指定解释器，`SCENE` 可指定与所选夹爪一致的场景；所有 FK 消费者使用同一 `TELEOP_MODEL`。RG 默认 `scene_ros2_rg_cameras.xml`：内置腕部相机姿态保留，外部相机沿用旧场景位置，无新增动态物体。

`go_ready` 在 tracker 启动前独占机械臂命令接口运行。完整 supervisor 只启动 MuJoCo，不连接真机。控制节点的 `use_sim_time` 由 launch 统一传入，默认仿真 true；独立运行节点时可通过 ROS 参数设为 false，不依赖 `/clock`。

## 录制

Logger 默认等待服务操作，未分配 Quest 录制按键：

```bash
ros2 service call /recording/start std_srvs/srv/Trigger '{}'
ros2 service call /recording/pause std_srvs/srv/Trigger '{}'  # 暂停／恢复
ros2 service call /recording/stop std_srvs/srv/Trigger '{}'
```

默认袋目录为项目 `data/rosbags/`。每次会话包含 rosbag、`teleop_manifest.json` 和 `timing.jsonl`。关闭会话会写入结束事件并关闭袋。独立 Logger 的 `--mock-buttons` 保留原键盘 P／Esc 操作；默认不监听它们，`--autostart` 可自动开始。

记录实际关节状态、实际发送的关节位置命令和其 stamped 副本、实际 FK、末端目标、离合锚点／状态、Quest 原始时间元数据、输入位姿及按键、夹爪命令／测量、三路压缩图像／相机参数、时钟和会话事件。

- 保留消息原始 source stamp，不改写图像时间来伪造同步。
- rosbag 时间取回调接收时的 ROS 时间；每条消息另记录接收 monotonic 时间、写入时 ROS／wall 时间及话题序号。
- 无 header 的原始控制命令 source stamp 为 null；stamped 副本记录控制器同次输出的 ROS 时间。
- `command_state` 是夹爪目标二态；`joint_states` 中的夹爪位置才是仿真实测反馈，不能将二态命令称为实际闭合状态。
- 不提前确定 π0.5／LeRobot action 表达，不把下一帧实测状态自动当作 action。
- 源消息异步，最终采样率、匹配时间窗口、图像年龄及重复帧检查需要在 Dataset Builder 中明确。

旁路渲染器用关节快照重建图像。RG 当前场景没有动态物体；AG95 旧 demo 的自由方块状态未随 `/joint_states` 同步，因此该旁路图像不能用于动态抓取示范训练。后续若采动态仿真物体，需要从物理实例输出完整物体状态或直接渲染物理实例，不能伪装成已同步。

## 参数与保护边界

`config/robot_spec.yaml` 为统一型号参数来源：仓库 AUBO-i5／CB4 手册 V4.5.11，2021-05-19，正文 p29、附录 E pVI。六轴 ±175°，前三轴 150°/s，后三轴 180°/s。工作区关节界限保持 ±2.94 rad，这是项目余量，不是厂家规格；RG ready 的腕3=2.94 保留，正向余量为零。未设置没有依据的厂家加速度规格。

官方 `aubo_description` 的几何及惯性以版本 `47fa5e0` 为来源，项目维护限位覆写副本 `urdf/aubo_i5_model.urdf`，不修改固定版本的外部依赖。可运行 `tools/check_robot_spec.py` 校验 URDF、MoveIt 和正式 MJCF／执行器的一致性。历史速度实验模型不属于正式 B 基线。

tracker 保留原阻尼 IK、姿态偏置及奇异点策略；它是项目自写控制器，不是原版 LARA，也未在此宣称真机验证。原始 IK 校正按每周期应用，dt 用于关节增量限速。已有阈值是项目控制参数，不是厂家推荐值。

新增保护参数：反馈和离合心跳超时均 0.2s；允许跟随误差按 `VMAX × following_lag_seconds` 计算，仿真初值 0.2s，持续超过 `following_error_duration=0.2s` 时锁止推进。即前三轴约0.524rad、后三轴约0.628rad，仅是宽松的仿真异常保护，需真机重新验证。异常后必须松开再接合；新会话才能重置。反馈失效时不发布新的位置命令。独立 tracker 必须收到 `/teleop/control`，不能仅发 `/target_pose` 使其运动。

释放会取消旧目标，并在反馈新鲜时采用当前测量作为保持命令；这不等于零制动距离、零冲击或安全认证的停止。当前 B **没有在线自碰撞／环境碰撞命令检查**，MoveIt 显示场景不代表在拦截 B 命令。真机部署仍需另行验证硬件停止行为、控制柜限制、模型/TCP、RG 通信和碰撞边界。

## 验证与历史

```bash
source /opt/ros/humble/setup.bash
source /home/lcw/VR_teleoperation/ros2_ws/install/setup.bash
PY=/home/lcw/tomato_robot/.venv/bin/python
PKG=/home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop
"$PY" "$PKG/scripts/test_clutch_mapper.py"
"$PY" "$PKG/tests/test_route_b.py"
"$PY" "$PKG/tools/check_robot_spec.py"
"$PY" "$PKG/tools/verify_tool_frames.py"
MUJOCO_GL=egl "$PY" "$PKG/scripts/multi_camera_renderer.py" --selftest
```

离线测试覆盖局部轴对应、组合旋转、半球连续性、滑移限幅、实际重锚、旧目标拒收、断流、跟随误差及日志时间。头显验证需在多个起始朝向下重复 roll/pitch/yaw，并观察纯旋转时 TCP 位置和相机行为。`rotation_check.py` 已改为比较局部轴；它只订阅，但需要操作者接合并实际操作仿真。

离屏自测只证明渲染成功。保留的 RG ready 姿态下，三路画面主要为背景／地面，TCP 位于视场之外；相机安装与归位姿态按已确认约束保留，任务视场仍需验收。

本次环境中 ROS DDS 无法枚举网络接口，自动审批服务返回503导致隔离本机联调未执行；完整 ROS 图、头显体验和录制吞吐尚待验收。详细记录见 `docs/route_b_validation.md`。

路线 A 曾在该项目的执行器／接口组合中出现极限环及速度超限，故退出正式链；这不是所有 MoveIt Servo 系统都会失败的结论。历史文档、代码及测试移至 `docs/archive_route_a/`，不构建、不安装、不在正式启动中调用。原始文档中的历史“已验证”表述不可视为当前工具系基线的验收。
