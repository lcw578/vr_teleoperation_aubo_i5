# 依赖与来源清单（DEPENDENCIES.md）

> 目的：说清**每一个外部包从哪来、哪个版本、我们是否改过**，以及**我们自己写的每一件东西依据什么**。
> 2026-09-23 建立。仓库里的东西分三类，本文按类别列全。

---

## 一、构建依赖（本仓库不入库，用 `deps.repos` 声明）

| 包 | 来源 | 固定版本 | 我们是否改过 |
|---|---|---|---|
| `aubo_ros2_driver`（含 `aubo_description` 子模块、`aubo_moveit_config`、`aubo_msgs`） | https://github.com/AuboRobot/aubo_ros2_driver | `8568407`（2026-03-13）；子模块 `aubo_description` = `47fa5e02` | **未改** |
| `mujoco_ros2_control`（含 `mujoco_ros2_control_msgs`、`_plugins`、`mujoco_3d_lidar`） | https://github.com/ros-controls/mujoco_ros2_control | `58f465b`（2026-09-15，无 tag 可引） | **未改** |
| `dh_ag95_description` | 见下（**已入库**，不在 `deps.repos` 里） | 上游 `ian-chuang/dh_gripper_ros` @ `9a97210` | **改过：构建系统移植** |

**`dh_ag95_description` 的特殊性**：上游那个包是 2018 年的 **catkin / ROS 1** 包，`colcon` 编不了。
我们把它移植成 `ament_cmake`——**只改了构建系统，`urdf/` 与 `meshes/` 与上游逐字节一致**（已用 `diff -rq` 验证）。
因为"移植"这一步 `vcs` 拉不回来，所以这个包作为**我们的产物**入库；它的 `package.xml` 里也写明了这一点。

### 拉取与构建（已验证可用）

```bash
# 1) 拉依赖（在 ros2_ws/src 下）
cd ros2_ws/src && vcs import < ../../deps.repos
git -C aubo_ros2_driver submodule update --init aubo_description   # ← 不能省：vcs 不处理子模块

# 2) 构建
cd ros2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-up-to aubo_i5_teleop \
             --packages-skip ros_joints_plan \
             --cmake-args -DFETCHCONTENT_UPDATES_DISCONNECTED=ON
```

两个必须的开关，原因见第四节"上游已知问题"：厂家 demo 包 `ros_joints_plan` 缺系统依赖编不了（我们不需要它）；
`mujoco_ros2_control` 构建时要从网上拉 `lodepng`，离线开关让它在本地已有的源码上继续。

---

## 二、参考克隆（不是构建依赖，仅供分析比对；在 `_official/` 与 `_refs/`，已 gitignore）

| 目录 | 来源 | 版本 | 用途 |
|---|---|---|---|
| `_official/aubo_ros2_driver` | AuboRobot/aubo_ros2_driver | `8568407` | 溯源审计：确认厂家全栈**没有**遥操、没有 MoveIt Servo |
| `_official/aubo_description` | AuboRobot/aubo_description | `47fa5e0` | 同上（子模块内容） |
| `_official/aubo_robot` | AuboRobot/aubo_robot | `32210c0` | ROS 1 栈，含 `aubo_driver` 的 `teach_controller`（厂家自带笛卡尔点动） |
| `_official/aubo_ros_driver` | AuboRobot/aubo_ros_driver | `295007e` | 同上 |
| `_official/aubo_driver_guide` | AuboRobot/aubo_driver_guide | `219eb27` | SDK 文档 |
| **`_refs/LARA_AUBOi5_AG95`** | ian-chuang/LARA_AUBOi5_AG95 | `0403d14` | **`teleop.srdf` 里 40 条"臂↔夹爪"跨组排除的来源**（那份 SRDF 由 MoveIt Setup Assistant 生成） |
| **`_refs/dh_gripper_ros`** | ian-chuang/dh_gripper_ros | `9a97210` | **AG95 的 45 条内部排除来源**（`dh_ag95_moveit_config/config/dh_ag95_gripper.srdf`）+ 描述包上游 |
| **`_refs/oculus_reader`** | ian-chuang/oculus_reader（RAIL Berkeley 的 fork） | `aae45ba` | **VR 手柄映射层**（`src/pose_teleop.py` 的相对参考帧做法）；APK 走 git-lfs |
| `_refs/Auboi5_Scan_Simulator` / `Manipulator-Mujoco` / `mujoco-Aubo-RL-PathPlanning` | 各第三方 | `2d457b8` / `f2d7d77` / `fe5d722` | 早期找模型时调研过的候选 |

---

## 三、我们自己写的东西，以及各自的依据

**① 原样使用官方/厂家文件（未改）**
- `aubo_description/urdf/aubo_i5.urdf`——**直接 include 厂家的 `.urdf`**。这样做是有意的：官方 `xacro.sh` 生成的 `.urdf.xacro` 会**删掉 `<property>` 标签**，而厂家的 `equa_inertia`、`protect_max_torque` 就在那里面。直接读 `.urdf` 反而保留了厂家数据。

**② 从官方/第三方"照抄或结构复制"（改了名字/字段，逐条标注）**
- `config/teleop.srdf` 的 **97 条**排除 =
  **45 条**（夹爪内部）取自官方 `dh_ag95_gripper.srdf`，
  **12 条**（臂内部）取自厂家 `aubo_i5.srdf`，
  **40 条**（臂↔夹爪跨组）取自 LARA 的 `lara_base_scene.srdf`，按名字映射（`wrist_3_link→wrist3_Link` 等）。
- `config/servo_pose_tracking.yaml`——底本是官方 `panda_simulated_config_pose_tracking.yaml`，改动 **8 处**，文件内逐条标注了原因。
- `urdf/aubo_i5_teleop.urdf.xacro` 里的 `ee_link`——逐字复制厂家 `inc/aubo_macro.xacro` 的定义，以与厂家 SRDF 保持一致。
- `config/controllers.yaml`——沿用厂家 `aubo_controllers.yaml`，只改 `update_rate` 并删掉 JTC（Servo 要用位置流，两者抢同一命令接口）。
- `src/pose_tracking_node.cpp`——骨架照抄官方 `moveit_servo` 的 `pose_tracking_demo.cpp`；算法 100% 在 MoveIt 的库里（`libpose_tracking.so` + `libmoveit_servo_lib.so`）。我们只做三件事：删掉官方的 demo 目标发布、等待仿真时钟就绪、修了官方 demo 早退不 join 线程的问题。
- `launch/*.py`——结构参照官方 launch（参数如何装进 `moveit_servo` 命名空间）。

**③ 我们实测或推导出来的值（厂家不提供，标在文件里）**
- **MuJoCo 执行器增益** `kp=25000`（臂）/`2500`（腕）：厂家只给 URDF，不给 MuJoCo 执行器参数。文件内注明。
- **`gravcomp="1"`（13 个刚体）**：补上真机控制器本来就有的重力补偿；`scene_aubo_i5.xml` 内写明了原因与实测效果。
- **PID 增益** `k_p=20 / k_i=0 / angular=20`：官方 panda 是 1.5/0/0（只跟随 58%），LARA 是 500/1/0.1——相差三个数量级，只能实测。扫描数据记在 `pose_tracking_settings.yaml` 里。
- **奇异阈值** `50/200`：抄 panda 的 17/30 会挡掉我们 62% 的可行位姿（8650 个位姿实测的条件数分布），依据记在 `servo_pose_tracking.yaml` 里。
- **`assets/aubo_i5/scene_aubo_i5.xml`**：由厂家 URDF 经 `mj_saveLastXML` 转换，再手工修正网格路径、关节限位、厂家 armature/torque、gravcomp。
- **`assets/aubo_i5/scene_ros2.xml`**：场景层（地板/灯光/天空盒）、9 对源自官方 SRDF 的 `<contact><exclude>`、7 个位置执行器。
- 各种诊断/验收脚本（`scripts/pose_target_test.py` 等）：自己写的工具，不是控制逻辑。

---

## 四、上游已知问题（不是我们引入的）

1. **厂家 demo 包 `ros_joints_plan` 编不了**：它的 CMake 链接 `tl::expected`，而系统没装提供它的包。我们不需要它 → 构建时 `--packages-skip`。
2. **`mujoco_ros2_control` 构建时要联网拉 `lodepng`**（CMake `FetchContent`）。网络不稳时会 `gnutls_handshake() failed`。源码已在本地构建目录时，用 `-DFETCHCONTENT_UPDATES_DISCONNECTED=ON` 跳过更新。
3. **`mujoco_3d_lidar` 想把插件装进 `/opt/ros/humble/...`**（需要 root）。只是警告，包本身构建成功；插件对我们无用。
4. **厂家 `xacro.sh` 会删除 `<property>` 标签** → 官方 ROS 2 xacro 路径看不到 `equa_inertia`/`protect_max_torque`。我们因此直接 include `.urdf`。
5. **厂家 `aubo_moveit.launch.py` 有 robot-name 不匹配**（URDF 里是 `aubo_i5`、SRDF 里是 `aubo_i5_robot`），会加载失败。我们不用它的 launch。
6. **厂家全栈没有遥操实现，也没有 MoveIt Servo**——`joystick_control.launch` 是 MoveIt Setup Assistant 的模板文件，不是他们的遥操代码。
7. 历史遗留：`build/mujoco_ros2_control_plugins` 里曾残留一个指向旧 ZCode 挂载点（`/tmp/.mount_ZCode-*/libEGL.so`）的 CMake 缓存路径，会导致链接失败；删掉该包的 `build/`+`install/` 重建即可（已处理）。

---

## 五、当前状态

阶段 3（MoveIt Servo pose tracking）验收 **12/12 通过**：三个平移 6/6、三个旋转 6/6。
复跑方式：`ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py` → `stage2b_moveit.launch.py` →
`stage3_pose_tracking.launch.py`，然后
`python3 scripts/pose_target_test.py --verify-translations`（或 `--verify-rotations`）。

未完成：接 VR 手柄（`_refs/oculus_reader` 的 ADB 链路；其 `pose_teleop.py` 的离合器按键语义是反的，建议改成"按住才动"）。
