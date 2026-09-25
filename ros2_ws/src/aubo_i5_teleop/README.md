# aubo_i5_teleop

Aubo i5 + AG95 夹爪的 ROS 2 遥操 bringup 层。

> **状态（2026-09-25）**：阶段 1/2/3 全部完成并标定（P=30 基线、三张表、断流安全）。
> 项目级文档见仓库根目录 **[README.md](../../../README.md)**（架构与用法）与
> **[BASELINE.md](../../../BASELINE.md)**（正式基线数字）。本文件只保留包内文件的
> 设计说明与 launch 理由。

## 为什么不直接用官方 `aubo_moveit.launch.py`

`aubo_ros2_driver` 自带的 launch 有两处问题，所以本包自己写：

1. 它的默认路径要 include `urdf/<aubo_type>.urdf.xacro`，但**全包 0 个这种文件**（`find` 验证过）。
   另一条路 `aubo_macro.xacro` 又是 ROS 1 + Gazebo Classic 的老路径
   （`libgazebo_ros_control.so`、ROS 1 的 transmission）。
2. 它额外发了一个 `world → base_link` 的静态 TF，而 **URDF 里已有 `world_joint`**，会冲突。

另外它还定义了 `joint_state_publisher_gui` 却没加进启动列表，导致 move_group 空等 10 秒。

## 文件说明

| 文件 | 作用 |
|---|---|
| `urdf/aubo_i5_teleop.urdf.xacro` | 官方 URDF + `ee_link` + AG95 夹爪 + 相机预留帧 + ros2_control(MuJoCo) |
| `config/teleop.srdf` | 臂 + 夹爪合并的 MoveIt 语义描述（60 条碰撞排除） |
| `config/joint_limits.yaml` | 含 ±3.04 位置限位（官方那份只有速度） |
| `config/controllers.yaml` | JTC，200 Hz |
| `config/teleop.rviz` | 调过视角的 RViz 配置（官方的焦点在基座 z=0.138，看不到夹爪） |
| `launch/stage1_moveit_rviz.launch.py` | 阶段 1：只起 MoveIt + RViz |
| `launch/stage2a_mujoco.launch.py` | 阶段 2a：MuJoCo + ros2_control + JTC |
| `launch/stage2b_moveit.launch.py` | 阶段 2b：叠加 move_group + RViz |
| `scripts/plan_execute_test.py` | 阶段 2b 的规划+执行测试 |

## AG95 夹爪（代用件）

⚠️ **真机上的夹爪是自制的，AG95 只是占位。** 它能验证链路，**验证不了任务**——
夹不住番茄、"手感"如何、以及惯性差异（AG95 质量 1.566 kg，自制件可能差不少）。
这些必须写进数据集元数据（`gripper: AG95-placeholder`），否则将来会有人把这份数据
当成"用真夹爪采的"。

来源：`ian-chuang/dh_gripper_ros`（**MIT**）。原包是 catkin（ROS 1, 2018），colcon 编不了，
已在 `ros2_ws/src/dh_ag95_description` 做最小 ament 移植——**只换构建系统，运动学与几何未改动**。

### 挂载变换

```xml
<xacro:arg name="gripper_mount_xyz" default="0 0 0"/>
<xacro:arg name="gripper_mount_rpy" default="0 0 0"/>   <!-- 恒等 -->
```

**自制夹爪到位后只改这两个参数。**

`rpy = 0 0 0`（恒等）的依据是实测：零位下 `wrist2_Link → wrist3_Link` 方向为
`[+0.141 -0.921 -0.363]`，而 `ee_link` 的 Z 轴与该方向点积 **+1.000**，即
**ee_link 的 +Z 就是工具轴向**。AG95 在自身坐标系里沿 +Z 伸出，所以恒等旋转即可对齐。

> ⚠️ **一个踩过的坑**：最初用"离基座最远"当启发式，得出 ee_link 的 X 轴朝外，
> 于是按 `rpy = 0 +90° 0` 装配。**那是错的**——那一姿态下臂是弯的，
> "远离基座"并不等于工具轴向。结果 MoveIt 的碰撞检查报出夹爪与 wrist2/wrist3
> 干涉最深 **25.4 mm**。**教训：工具轴向要用相邻连杆的连线来定，不能用离基座远近。**

### 碰撞排除（这是让它能工作的关键）

夹爪的相邻连杆网格在关节处本就重叠。**若不排除，MoveIt 会认为机器人永远处于自碰撞
状态，Servo 直接拒绝运动。**

`teleop.srdf` 的 60 条排除 = 臂 12 条（官方原样）+ 夹爪 45 条（AG95 官方原样）+ 新增 3 条：

| 新增 | 原因 |
|---|---|
| `ee_link` ↔ `ag95_base_link` | 挂接处，父子里程碑 |
| `ag95_base_link` ↔ `wrist3_Link` | 安装板与手腕法兰面**恰好贴面**，实测 `depth=0.0000`（不是干涉）；相对位姿固定，检查无意义 |
| `ag95_body` ↔ `ee_link` | 同上，贴面且相对位姿固定 |

修正后的实测：`/check_state_validity` 返回 **`valid = True`、接触数 0**。

### 与官方臂 SRDF 的唯一实质改动

`manipulator` 组的 chain 上端从 `ee_link` 改成 **`gripper_tip_link`**。
这样 MoveIt 的 IK 目标和 Servo 的 EE 帧都落在**夹爪尖端**（真正的交互点），而不是手腕。
链只经过固定关节，所以 `manipulator` 仍是 6 自由度，夹爪的手指关节不在其中。

**这个设计抄自 `LARA_AUBOi5_AG95`**（它用 `arm` 组 + `teleop_link` 做同一件事）。

## 相机预留帧

```xml
<xacro:arg name="camera_mount_xyz" default="0.05 0 0.02"/>   <!-- 占位值 -->
<xacro:arg name="camera_mount_rpy" default="0 0 0"/>
```

`camera_mount_link` 挂在 `ag95_body` 上（腕部法兰通常已被夹爪占用，且这样视角更贴近
夹爪工作区）。**位姿现在是占位值**，D405 到位后填实测值即可。

## 验证方式

**碰撞检查用 MoveIt 的服务做权威判断，不要靠 RViz 的颜色猜：**

```bash
# 需先起 stage1 或 stage2b
python3 - <<'EOF'
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
rclpy.init(); n = Node('cc')
cli = n.create_client(GetStateValidity, '/check_state_validity')
cli.wait_for_service(timeout_sec=15)
req = GetStateValidity.Request(); req.group_name='manipulator'; req.robot_state=RobotState()
fut = cli.call_async(req); rclpy.spin_until_future_complete(n, fut, timeout_sec=25)
r = fut.result()
print('valid =', r.valid, ' contacts =', len(r.contacts))
for c in r.contacts: print('  ', c.contact_body_1, '<->', c.contact_body_2, 'depth=%.4f' % c.depth)
EOF
```

## MJCF 侧的夹爪（已完成）

夹爪同时存在于 **URDF 和 MJCF**——这是必须的，因为 **Servo 的碰撞检查用 URDF 的 planning
scene，而物理在 MuJoCo**，两边不一致就会互相打架。

MJCF 侧的几何**不是手写的**，而是用 MuJoCo 自己的 URDF 解析器从 `dh_ag95_description`
导出（`mujoco.mj_saveLastXML`），保证 frame 与 URDF 严格一致。已逐 body 核对：

```
body                   位置Δ(m)     姿态Δ
left_finger            0.00000    0.00000
right_finger           0.00000    0.00000
left_inner_knuckle     0.00000    0.00000
left_outer_knuckle     0.00000    0.00000
right_inner_knuckle    0.00000    0.00000
right_outer_knuckle    0.00000    0.00000
```

**手写的只有物理耦合那部分**（在 `assets/aubo_i5/scene_ros2.xml`），因为 URDF 表达不了
四连杆闭环（解析时会断链，实测 MuJoCo 只解析出 6 个自由度而不是 8）：

```xml
<tendon><fixed name="ag95_split">
  <joint joint="left_outer_knuckle_joint" coef="0.5"/>
  <joint joint="right_outer_knuckle_joint" coef="0.5"/>
</fixed></tendon>
<equality>
  <connect anchor="-0.020673 0 0.007524" body1="ag95_left_finger" body2="ag95_left_inner_knuckle" .../>
  <connect anchor="-0.020673 0 0.007524" body1="ag95_right_finger" body2="ag95_right_inner_knuckle" .../>
  <joint joint1="left_outer_knuckle_joint" joint2="right_outer_knuckle_joint" polycoef="0 1 0 0 0" .../>
</equality>
```

手法与锚点数值照抄 `Manipulator-Mujoco` 的 `ag95.xml`——**可以直接抄，因为两边手指链的
frame 逐字相同**（`left_outer_knuckle pos=0.036673 -0.00875 0.098336 quat=0.924908 0 -0.380191 0`
等已核对）。实测闭环成立：`left_outer - right_outer = 2e-5`，`left_inner - left_outer = 0`。

### ⚠️ 挂载四元数必须是 `0.707107 0 0 0.707107`（绕 Z +90°）

对应 URDF 里 `gripper_base_joint` 的 `rpy="0 0 pi/2"`——因为 `ee_link` 相对 `wrist3_Link`
就是 `rpy="0 0 pi/2"`，而 `gripper_base_joint` 是恒等，所以 MJCF 侧要补上这 90°。

**这里踩过一个坑，值得记下来。** 曾经用"经验拟合"（Kabsch）反解挂载旋转，得到一个
`(-0.707107 0.707107 0 0)` 并在当时"验证通过"（位置和姿态偏差都是 0），但那**是错的**——
装出来的夹爪相对工具轴是**垂直**的（点积 +0.056，应≈0.87）。

错因在**比对方法**：URDF 那边用的是世界坐标，MJCF 那边转成了基座本地坐标，
**两边不在同一坐标系**，于是拟合用一个错误的旋转把它凑上了。

**正确的验收方法**（现在是这么做的）：把**两边都表示在同一个连杆的本地坐标系里**再比，
并且加一条**物理判据**——`wrist3 → 指尖` 与 `wrist2 → wrist3` 的点积应 ≈ +0.87（即沿工具轴伸出）。
修正后：

| 检查 | 结果 |
|---|---|
| 6 个夹爪 body 相对 `wrist3_Link` 的位置Δ / 姿态Δ | 0.00000 / 0.00000 |
| 工具轴点积 | +0.874（与 URDF 一致） |

**教训：几何一致性检查必须两边同坐标系，且要有独立于拟合的物理判据；纯位置拟合会被对称性骗过。**

### ⚠️ 一个必须遵守的约束（踩过坑）

**tendon 执行器的 `name` 必须写成某个真实关节名**，否则插件启动时报
`Tendon actuator '...' has no matching joint` 并 **FATAL 退出**，硬件初始化失败。

官方文档原话："The tendon name matches the controllable joint in the ros2_control
configuration. The drivers expose control and state for that single joint, while the
simulation enforces the mimic constraint internally."

所以这里把 `left_outer_knuckle_joint` 当作对外暴露的那一个（它的角即开合角），
其余手指关节由仿真内部联动、不暴露。**因此 `/joint_states` 里只有这一个夹爪关节，
其余联动关节看不到——这是设计如此，不是 bug。**

### 实测

| 项 | 值 |
|---|---|
| 模型 | nq=12 nv=12 nu=7 nbody=14 neq=3 ntendon=1，质量 23.107 kg（臂 21.988 + 夹爪 1.119） |
| 开合行程 | ctrl 0 → 152.6 mm，0.465 → 110.0 mm，0.93 → 60.7 mm（平滑单调） |
| 加夹爪后臂的跟踪 | 0.144°（加之前 0.124°，未退化） |
| `gripper_controller` 跟踪 | 命令 0.0/0.465/0.93 → 实际 -0.0001/0.4649/0.9300 |
| 全程 `ncon` | 0 |

夹爪控制器在 `config/controllers.yaml` 里是**独立的一个**（`forward_command_controller`），
不放进 JTC——MoveIt 的 `manipulator` 组是 6 自由度，夹爪是另一个组，混在一条轨迹里 MoveIt 不会规划它。

## 已知未做

- 关节限位 ±3.04 待 Aubo 数据手册核实。
- 夹爪的 `kp=200/kv=5` 是起步值（AG95 原模型用 kp=10/kv=0.1 很软），需按手感调。
- 夹爪的**视觉网格**用的是 `meshes_ag95/visual/`，但 MJCF 里视觉/碰撞是分开的两个 geom——
  如果发现渲染和碰撞不一致，检查这两条路径。
