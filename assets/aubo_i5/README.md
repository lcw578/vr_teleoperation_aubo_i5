# Aubo i5 模型（遥操工程用）

> 就位于 2026-09-22，供 ROS 2 + MoveIt Servo 方案使用。

## 两个文件，分工明确

| 文件 | 用途 | 是否可直接用于物理 |
|---|---|---|
| `scene_aubo_i5.xml` | **忠实版**：官方 URDF 转换产物，只改了 mesh 路径 | ❌ 无执行器 |
| `scene_ros2.xml` | **ROS 2 用版**：`<include>` 上面那份，加执行器/排除/起始位姿 | ✅ |

`scene_ros2.xml` 是给 `mujoco_ros2_control` 用的入口（`<param name="mujoco_model">`）。它不改动 `scene_aubo_i5.xml` 里的任何运动学与惯性参数——**动力学模型仍然来自官方 URDF**。

## 来源

| 项 | 值 |
|---|---|
| 原始模型 | Aubo 官方 ROS2 驱动仓库 `https://github.com/AuboRobot/aubo_ros2_driver.git` |
| 上游 commit | `8568407` |
| 源 URDF | `third_party/aubo_ros2_driver/aubo_description/urdf/aubo_i5_37.urdf` |
| 转换脚本 | `/home/lcw/tomato_robot/scripts/aubo_urdf_to_mjcf.py`（9 月 6 日跑通） |

本目录不是新下载的模型，而是从本机 `tomato_robot` 工程里取出来的既有产物（那份调试方案说它在"另一台机器上"，是写错了）。

## 对原始产物做的修改

### 1. mesh 路径（`scene_aubo_i5.xml`）

7 条路径从 `../meshes/...` 改为 `meshes/...`。

原因：`mj_saveLastXML` 把 MJCF 另存到了与原 URDF 不同的目录，却原样保留相对路径，导致 `../meshes/` 解析到不存在的目录，**原产物在它自己的位置上根本加载不了**。改成以自身为基准后整个文件夹可任意搬移。

网格用 `cp -rL` 复制（**必须解引用**：源目录里 `aubo_i5_37` 是指向 `aubo_i5` 的符号链接，普通 `cp -r` 只会搬一个空链接）。

### 2. `scene_ros2.xml` 里新增的四项

**`<option integrator="implicitfast"/>`** —— 位置执行器较硬，官方 demo 用这个积分器。

**`<contact><exclude>` 共 9 条** —— 来源是 `aubo_moveit_config/config/aubo_i5.srdf` 的 12 条 `disable_collisions`（其中 3 条涉及本模型没有的 `ee_link`，略去）。

这一条是实测发现的：**不加排除时，基座和 `shoulder_Link` 持续互相顶，重叠 6.5 mm，产生 4 个接触点**，把虚假接触力注进物理。URDF 的碰撞网格本来就会在相邻连杆的关节处重叠，ROS/MoveIt 靠 SRDF 排除，而 MuJoCo 不知道。加上排除后两个位姿下 `ncon` 都是 0。**这样 MuJoCo 的自碰撞判断与 MoveIt Servo 用的 SRDF 一致，两边不会打架。**

注意：本模型的基座是挂在 `world` 上的 geom（转换产物，不是独立的 `base_link` body），所以基座那两条写成 `world`。

**`<actuator>` 6 个 position 执行器** —— 形态抄自官方 demo
`mujoco_ros2_control_demos/demo_resources/robot/test_robot.xml`：

```xml
<position joint="joint1" name="joint1" kp="25000" dampratio="1.0" ctrlrange="..."/>
```

`kp` 按关节力矩上限分档：关节 1–3（URDF 的 `actuatorfrcrange` 为 ±133 N·m）用 `kp=3000`，关节 4–6（±13.5 N·m）用 `kp=300`。依据是官方 demo 的 kp/力矩上限 ≈ 25 这个比值。`dampratio="1.0"` 让 MuJoCo 按关节惯量自动算阻尼，不用手调 `kv`。

实测跟踪（目标 `[0.5, -0.8, 1.2, 0.3, 0.6, -0.4]`）：最大误差 **1.04°**，是重力引起的稳态误差。**这些 kp 是起步值，需要在阶段 2/3 里按跟踪效果调。**

**`<keyframe name="ready">`** —— 起始位姿，对应插件的 `initial_keyframe` 参数。刻意让肘部弯曲（`qpos="0 -0.4 0.8 0 0.4 0"`），因为**完全伸直的臂处于奇异位形，Servo 从那里起步会很糟**。`qpos` 与 `ctrl` 一致，启动时执行器不会立刻把臂拉走。

## 自检

```bash
cd assets/aubo_i5
/home/lcw/tomato_robot/.venv/bin/python -c "
import mujoco, numpy as np
m = mujoco.MjModel.from_xml_path('scene_ros2.xml')
d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
print('nq=%d nv=%d nu=%d nmesh=%d mass=%.3f kg' % (m.nq,m.nv,m.nu,m.nmesh,m.body_mass.sum()))
print('ncon =', d.ncon, '(应为 0)')
"
```

实测：`nq=6 nv=6 nu=6 nmesh=7`，总质量 `21.988 kg`，`ncon=0`。

> 系统 python（`/usr/bin/python3`，3.10）里的 mujoco 缺 `typing_extensions`，会直接报错。请用 `tomato_robot/.venv`（Python 3.10 + MuJoCo 3.12.0，与 `ros-humble-mujoco-vendor` 提供的 3.12.0 同版本）。

## 关节限位与阻尼（已按确认调整，⚠️ 限位待核实）

### 关节限位：±360° → ±3.04 rad（≈174.2°）

官方 URDF 把六个关节**全写成 ±360°**（`±6.283193`），对真机 i5 不可信——照这个值，机械臂在物理上可以自穿，也没有任何限位护栏。

两个第三方 i5 模型都用 **±3.04 rad**：`ian-chuang/Manipulator-Mujoco` 和 `zyz0721/Auboi5_Scan_Simulator`。经用户同意先按 ±174° 设上。

> ⚠️ **这个值尚未经 Aubo 官方数据手册核实**，来源只是那两个第三方模型。查证后应回填确认值。

**必须三处同步**，否则会出现"MoveIt 按 ±360° 规划、物理按 ±174° 执行"这类不一致：

| 位置 | 文件 | 改法 |
|---|---|---|
| MuJoCo 物理 | `scene_aubo_i5.xml` 的 `<joint range>` | 已改为 `-3.04 3.04` |
| ros2_control | `aubo_i5_teleop.urdf.xacro` 的 `command_interface` `min`/`max` | 已改为 `±3.04` |
| MoveIt | `aubo_i5_teleop/config/joint_limits.yaml` | 已加 `has_position_limits` + `min/max_position` |

> 注意最后一项：MoveIt 的限位来自 `joint_limits.yaml`，而官方那份**只写了速度、没有位置限位**，所以必须用本工程自己那份，否则 MoveIt 会退回 URDF 的 ±360°。

### 关节阻尼与 armature：0 → 0.1

原来六个关节的 `damping=0`、`armature=0`。**硬的位置执行器配零阻尼容易震**，官方 demo 的关节也带 `damping` 和 `frictionloss`。现按 `Auboi5_Scan_Simulator` 的取值设为 `damping="0.1" armature="0.1"`。

这两个是**调参起点**，不是标定值，手感调优阶段可能需要改。

> 附注：官方 URDF 的每个关节其实带 `<property inertia="2.0" damping="0" motor_constant="8.72" ratio="121"/>`——**有转子惯量和减速比**，理论上可以算出真实的 armature（反射惯量 = J×N²）。但 `inertia="2.0"` 的单位不明（按 SI 算得 29282 kg·m²，明显不对），**单位不查清就不能用**，所以先用经验值。

### 执行器增益：实测选定 kp = 25000 / 2500

**这是本工程里唯一"靠估"的参数，而且第一版估错了。**

| 变体 | 稳态误差 | 正弦跟踪滞后 | 超调 |
|---|---|---|---|
| ~~我最初 kp=3000/300（按 demo 的 kp/力矩比推）~~ | ~~1.037°~~ | ~~2.869°~~ | 54% |
| 参考模型 `Auboi5_Scan_Simulator` kp=4000 kv=300 | 0.776° | 2.526° | 56% |
| kp=6250/625 | 0.496° | 1.806° | 59% |
| **✅ kp=25000/2500（当前值）** | **0.124°** | **0.784°** | 77% |
| kp=30000/3000 + armature 0.3 | 0.103° | 0.704° | **134%（震荡）** |

结论：kp 越大跟踪越好；但把 armature 从 0.1 加到 0.3 会从"超调"恶化成"震荡"，所以 armature 保持 0.1。

那 77% 的超调**是力矩上限（`actuatorfrcrange` ±133 / ±13.5 N·m）饱和造成的，不是发散**——真实伺服驱动器也这样。而且阶跃响应不代表遥操场景（Servo 发的是连续小增量），所以以"正弦跟踪滞后"为准。

> ⚠️ **Aubo 官方不发布 MuJoCo 模型，只发 URDF。URDF 里没有"执行器增益"这个概念**——它属于 MuJoCo 层。所以不存在"厂家封装好的执行器"，任何人在 MuJoCo 里跑 Aubo i5 都必须自己定这组值。这是行业现状，不是绕路。
>
> 这组 kp 是**实测调出来的经验值，没有官方来源**，手感调优阶段应重新审视。

### 场景层：地面 / 网格 / 天空盒 / 灯光

原先这份只有机器人本体（URDF 转换产物里没有场景），所以在 MuJoCo 里是一片黑、机械臂悬空、没有空间参照。现按官方 demo 的 `scene.xml` 补上：棋盘网格地面、天空盒渐变、方向光、初始视角。

地面放在 **z=0（基座平面）**，`contype=1/conaffinity=0`——**机械臂会与地面碰撞**，工作空间的下边界因此是真实的，符合"从第一天就按真机标准限定工作空间"。

### 视觉网格：改用厂家的 DAE 转换产物

原先用**碰撞 STL 充当视觉网格**（MuJoCo 不支持 3DS/DAE，这是原始转换的妥协），所以外观是个"玩具"——碰撞网格每连杆只有约 1000 个面，且是简化的圆柱/方块。

现已改用**厂家的高精度视觉网格**。厂家的 `aubo_i5/visual/link0..6.DAE` 是真正的视觉模型（比碰撞网格大 10–18 倍，link3 是 325 KB vs 18 KB），**没有贴图文件，颜色嵌在 DAE 的 `<diffuse>` 里**。

转换流程（可复现）：

```
aubo_i5/visual/linkN.DAE
  → assimp export → linkN.obj + linkN.mtl
  → 按 usemtl 拆分（MuJoCo 一个 geom 只能一个材质）
  → 顶点旋转 +90°(X)   ← 见下
  → meshes_visual/linkN_k.obj
```

**⚠️ 转换中发现的关键差异：厂家的 DAE 视觉网格和它自己的碰撞 STL 朝向不一致。**

```
视觉 OBJ:  X[-0.0590, 0.4145]  Y[0.0420, 0.1623]  Z[-0.0590, 0.0590]
碰撞 STL:  X[-0.0590, 0.4145]  Y[-0.0589, 0.0589]  Z[0.0420, 0.1624]
                  ↑ 完全相同        Y↔Z 互换
```

即 DAE 是 Y-up、STL 是 Z-up 的约定差异。**不做旋转就会装反。** 已在 7 个连杆上逐一验证该规律一致（X/Z 完全吻合，Y 只在单侧差 12.5 mm，是碰撞网格多了视觉网格没做的安装凸台，非坐标错位）。

厂家的三种涂装色（从 MTL 提取）：

| 材质 | RGB | 用途 |
|---|---|---|
| Aubo 橙 | (232, 131, 0) | 连杆主体 |
| 灰 | (105, 105, 105) | 关节罩 |
| 深灰 | (79, 79, 79) | 少量细节 |

结果：`nmesh=20`（视觉）+ 7（碰撞），`nmat=4`，约 1 万个面。**物理完全未变**（质量 21.988 kg、稳态误差 0.124°、`ncon=0`）。

同时把碰撞 geom 移到 `group="3"`（只影响显示，不影响物理）——它们原来与视觉网格完全重叠、在互相 z-fighting。

## 自检

```bash
cd assets/aubo_i5
/home/lcw/tomato_robot/.venv/bin/python -c "
import mujoco, numpy as np
m = mujoco.MjModel.from_xml_path('scene_ros2.xml')
d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
print('nq=%d nv=%d nu=%d nmesh=%d mass=%.3f kg' % (m.nq,m.nv,m.nu,m.nmesh,m.body_mass.sum()))
print('限位(deg) =', [(round(float(np.degrees(m.jnt_range[i][0])),1), round(float(np.degrees(m.jnt_range[i][1])),1)) for i in range(m.njnt)])
print('阻尼 =', [round(float(m.dof_damping[i]),3) for i in range(m.nv)])
print('ncon =', d.ncon, '(应为 0)')
"
```

实测：`nq=6 nv=6 nu=6 nmesh=7`，总质量 `21.988 kg`，限位 `±174.2°`，阻尼 `0.1`，`ncon=0`，位置跟踪误差 `1.04°`（重力稳态误差）。

> 系统 python（`/usr/bin/python3`，3.10）里的 mujoco 缺 `typing_extensions`，会直接报错。请用 `tomato_robot/.venv`（Python 3.10 + MuJoCo 3.12.0，与 `ros-humble-mujoco-vendor` 提供的 3.12.0 同版本）。

## ROS 2 实测记录（阶段 2a / 2b 已通过）

这一层在 `VR_teleoperation/ros2_ws` 里跑通，实测结论：

| 阶段 | 验证内容 | 结果 |
|---|---|---|
| 2a | `ros2_control` + MuJoCo + JTC，200 Hz | 执行器按名字全部映射成功、`initial_keyframe` 生效、仿真时间生效 |
| 2a | 直接给 JTC 发 `FollowJointTrajectory` | `error_code=0` / `Goal successfully reached!` / 六关节到位 |
| 2b | MoveIt 规划 + 执行 | `error_code=1 (SUCCESS)`，19 点轨迹，1.71 s，六关节到位 |

MuJoCo 侧同时确认 `Status: Running`、`Actual Speed 100.0%`、**`Contacts: 0`**——后者顺带证明写进 MJCF 的 9 条 SRDF 对齐排除在实时仿真里确实生效。

## 下一步

阶段 3（MoveIt Servo）要接的东西：
- `move_group` 必须在跑——Servo 的自碰撞检查依赖它的 planning scene（阶段 2b 的 launch 已经起了）
- 写 Servo 的配置 yaml（`aubo_moveit_config` 里没有，要自己写）：`move_group_name: manipulator`、`ee_frame_name`、`robot_link_command_frame`、`command_in_type`、`scale`、`publish_period`、`singularity` 阈值、`collision_check`
- 起 `servo_node`，先用命令行手发 `TwistStamped` 验证，再接 Quest

**Phase 2 遗留的待核实项**：关节限位 ±3.04 rad 需要回填 Aubo 官方数据手册的确证值。
