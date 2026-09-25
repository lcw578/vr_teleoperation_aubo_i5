# Aubo i5 模型（遥操工程用）

> 就位于 2026-09-22，供 ROS 2 + MoveIt Servo 方案使用。

## 两个文件，分工明确

| 文件 | 用途 | 是否可直接用于物理 |
|---|---|---|
| `scene_aubo_i5.xml` | **忠实版**：官方 URDF 转换产物，只改了 mesh 路径 | ❌ 无执行器 |
| `scene_ros2.xml` | **ROS 2 用版**：`<include>` 上面那份，加执行器/排除/起始位姿 | ✅ |
| `scene_ros2_velocity.xml` | 同上，但执行器换成 `<velocity>`（**当前默认**，见 `actuators_velocity.xml`） | ✅ |
| ~~`scene_aubo_i5_armB.xml`~~ | ⚠️ **实验变体，不是模型**：把 armature 机械替换成厂家 `inertia` 字段，仅用于 armature A/B 对照实验 | 实验用 |
| ~~`scene_ros2_velocity_armB.xml`~~ | ⚠️ 同上（指向 armB 的场景层） | 实验用 |

> `armB` 两个文件只服务于 [armature A/B 对照实验](#关节阻尼与-armature)（结论：维持 `equa_inertia`）。
> **不要拿它们当模型基准**，也不要让任何 launch 默认指向它们。要复现实验：
> `python3 scripts/armature_ab_offline.py --fine`。

`scene_ros2.xml` 是给 `mujoco_ros2_control` 用的入口（`<param name="mujoco_model">`）。它不改动 `scene_aubo_i5.xml` 里的任何运动学与惯性参数——**动力学模型仍然来自官方 URDF**。

## 模型正确性验证（独立于生成器）

生成器（`aubo_urdf_to_mjcf.py`）本身不构成验证——它和自己的输出同源。
`ros2_ws/src/aubo_i5_teleop/scripts/verify_model_vs_urdf.py` 做**独立对照**：让 MuJoCo
重新解析厂家 URDF，再与我们的 MJCF 在若干随机位形下比运动学与质量矩阵。实测（2026-09-24）：

| 检查 | 偏差 |
|---|---|
| body 世界位置 / 姿态 | ~1e-16 / ~1e-15（机器精度）→ 运动学完全一致 |
| 六连杆质量和 | 21.987890 vs 厂家 21.987854 kg（我们文件 6 位有效数字的舍入） |
| 质量矩阵 M(q) | 绝对 ~1e-6，相对 ~4e-7 → 惯量（含质心与主轴）一致 |

> `base_link` 的 1.53902 kg 未进入动力学：两边都把它当静基座（厂家 `world_joint` 是固定关节，
> MuJoCo 把固定关节的子连杆并入父体）。这是正确的，不是缺项。

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

### 关节阻尼与 armature

**阻尼 `damping=0.1`：非厂家值**（厂家 `<property damping="0">` 是"电机侧无被动弹簧/阻尼"）。取 0.1 是参照第三方 `Auboi5_Scan_Simulator` 的值，理由是**硬的位置执行器配零阻尼容易震**，官方 demo 的关节也带 `damping`。这是调参起点，不是标定值。

**armature `1.5/1.5/1.2/0.05/0.05/0.01`：厂家值，逐关节等于 URDF `<property equa_inertia="...">`。**（本 README 早先写的"armature=0.1（第三方经验值）"是**旧状态，已作废**。）

> 附注：厂家每个关节的 `<property>` 同时给了**两个**惯量字段与电机参数：
> ```
> <property inertia="2.027236783" damping="0" stiffness="0" offset="0"
>           motor_constant="8.72" ratio="121" protect_max_torque="80.0" equa_inertia="1.5" .../>
> ```
> - `equa_inertia`：1.5/1.5/1.2/0.05/0.05/0.01，**逐关节不同** → 我们用这个当 armature
> - `inertia`：2.027236783（关节 1-3）、0.219280696（关节 4-6），**按电机分组相同**
> - `motor_constant` / `ratio`：8.72 & 121（关节 1-3）、7.092 & 101（关节 4-6）
>
> **`inertia` 与 `equa_inertia` 哪个才是 armature（关节侧折算惯量），无法从厂家资料判定**：
> 厂家 `xacro.sh` / `ros1_xacro.sh` 都用 `sed '/<property/d'` 主动删掉这些字段，**全栈无人消费**；
> 五个官方仓库、驱动手册、SDK API 文档、网络检索都查不到字段定义。
> "`inertia` × ratio² 当反射惯量"也走不通（关节 1-3 得 29282 kg·m²，明显不对，说明它不是电机侧 SI 值）。
>
> **A/B 对照实验（2026-09-24，`scripts/armature_ab_offline.py`）**：把 armature 换成 `inertia` 后，
> 被控对象内环时间常数 τ=(M_crb+armature)/(kv+damping) 变为

| 关节 | A: equa_inertia | B: inertia | τ_B/τ_A |
|---|---|---|---|
| shoulder | 9.58 ms | 10.77 ms | 1.12× |
| upperArm | 8.75 ms | 9.09 ms | 1.04× |
| foreArm | 5.43 ms | 6.99 ms | 1.29× |
| wrist1 | 3.21 ms | 9.17 ms | 2.86× |
| wrist2 | 3.10 ms | 8.64 ms | 2.78× |
| wrist3 | 1.19 ms | **20.05 ms** | **16.85×** |

> **闭环验收对之不敏感**：A 与 B 跑同一套 `pose_target_test.py`，平移都是 6/6（最大误差 1.4~1.7 mm），
> 旋转都是 6/6（各方向完成度差异 <0.1%）。原因是外环（τ≈74 ms）比两个内环都慢得多，
> 1 ms 与 20 ms 的内环在外环看来"都足够快"。**所以本工程的跟踪/增益结论不依赖这个选择。**
> 结论：**维持 `equa_inertia`**（逐关节字段，更像关节级等效惯量），并把上述不确定性记录在此。
> 若将来要做力矩/电流级的工作，必须先从厂家拿到字段定义再定。

### 执行器增益：实测选定 kp = 25000 / 2500

**这是本工程里唯一"靠估"的参数，而且第一版估错了。**

| 变体 | 稳态误差 | 正弦跟踪滞后 | 超调 |
|---|---|---|---|
| ~~我最初 kp=3000/300（按 demo 的 kp/力矩比推）~~ | ~~1.037°~~ | ~~2.869°~~ | 54% |
| 参考模型 `Auboi5_Scan_Simulator` kp=4000 kv=300 | 0.776° | 2.526° | 56% |
| kp=6250/625 | 0.496° | 1.806° | 59% |
| **✅ kp=25000/2500（当前值）** | **0.124°** | **0.784°** | 77% |
| kp=30000/3000 + armature 0.3 | 0.103° | 0.704° | **134%（震荡）** |

结论：kp 越大跟踪越好（上表是**实测**扫描，不是推导）。kp=25000 与官方 `mujoco_ros2_control` demo（`test_robot.urdf`：`kp="25000"` 配 `effort="1000"`）同值；腕部取 1/10。

那 77% 的超调**是力矩上限饱和造成的，不是发散**——真实伺服驱动器也这样。注意两个厂家力矩数：URDF `<limit effort>` 是 **133 / 13.5 N·m**，而 `<property protect_max_torque>` 是 **80/80/60/16/16/10 N·m**；**本模型 `actuatorfrcrange` 执行的是后者**（保护阈值，更保守）。所以"饱和"发生在 80/60/16/10，不是 133/13.5。

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
