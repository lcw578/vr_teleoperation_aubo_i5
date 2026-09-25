# VR 遥操作项目 · 正式基线（2026-09-25）

> 本文档是**当前唯一权威基线**。所有数字都标注了来源（哪次测量、哪条通道、如何核验）。
> 之前规格类文档（如《VR遥操调试方案.md》）已废弃，不要再引用。
> 状态标记：✅ 已验证并复现 ｜ ⚠️ 已测但有限定条件 ｜ ❌ 已尝试并否决 ｜ 🔲 未开工。

---

## 0. 一句话现状

控制链（MuJoCo 仿真内，位置模式）已达可接 VR 的质量水平：静止 8/8 可复现、跟随幅值比 ~1.00（1 Hz）、端到端起始延迟 29–30 ms、断流行为安全；**VR 侧（头显输入链路、映射层、视觉反馈）尚未开工**，头显未到货。

---

## 1. 系统架构与数据链路

```
操作者（未来=VR；现在=台架 follow_bench.py，125 Hz）
   │  PoseStamped @ /target_pose（world 帧，绝对位姿）
   ▼
MoveIt Servo PoseTracking（libpose_tracking.so）
   │  纯 P 控制（x/y/z 与 angular 均为 30，I=D=0）→ 笛卡尔速度
   │  容差 0.01 m / 0.1 rad；TARGET_POSE_TIMEOUT = 1.0 s（断流即停）
   ▼
MoveIt Servo（libmoveit_servo.so）
   │  雅可比伪逆 → 关节增量 Δ；publish_period = 5 ms（200 Hz）
   │  Butterworth 平滑 coeff=1.0（≈透明）
   │  奇异保护：50 降速 / 200 急停（离开阈值 ×2.0）；碰撞降速/急停；关节到界
   ▼
/servo_position_stream   ←─ 【锚定流：cmd = 实测 + 单周期 Δ】
   ▼
servo_interface.py（200 Hz，自研，唯一在链路上的自写控制件）
   │  把锚定流**积分成绝对位置命令** q_cmd（等价真机 servoj 的速度前馈步骤）
   │  保护①：单周期 |Δ| ≤ v_max·T（厂商手册 150°/s=2.618、180°/s=3.142 rad/s）
   │  保护②：抗积分饱和 |q_cmd − q_meas| ≤ lag_max = 0.15 rad（= 最大憋压深度）
   │  保护③：输入流断 >0.5 s（reinit_gap）→ 重新对齐到 实测+Δ
   ▼
/forward_command_controller_position/commands
   ▼
forward_command_controller → mujoco_ros2_control 插件（物理步长 1 ms，implicitfast）
   ▼
MuJoCo <position> 执行器 kp=25000/2500, dampratio=1.0（6 臂关节）+ AG95 肌腱执行器
   ▼
/joint_states（197 Hz，时间戳准时，比仿真时钟迟 −0.001 s）
   ▼
【测量通道】/joint_states + MuJoCo FK（⚠️ 不用 TF——TF2 Python listener 积压 0.3–0.5 s）
```

**为什么必须有伺服接口层**：Servo 的位置输出锚在"实测+Δ"上，任何 `error = cmd − 实测` 的下游
PID 的误差恒等于 Δ，对臂的真实位置是盲的（静止无保持力矩、运动中积分失控）。接口层把流还原成
绝对轨迹，这是结构问题，调参修不了。同一结论下两条被否决的路：❌ 拉高 kp 强制增益=1（破坏启动）；
❌ mujoco_ros2_control 内置 PID（error≡Δ）。

**被否决的替代框架**：❌ Manipulator-Mujoco（125 行手写控制器、无 ROS）；❌ Gazebo Classic
（厂商 aubo_gazebo 是空壳）。保留：❌ aubo_hardware_interface（真机专用、拒绝我们的 URDF）。

---

## 2. 模型基线（assets/aubo_i5/）

| 项 | 值 | 来源/验证 |
|---|---|---|
| 场景文件 | `scene_ros2.xml`（位置，正式）/ `scene_ros2_velocity.xml`（速度，仅对照） | 两场景除执行器外严格一致 |
| 与厂商 URDF 一致性 | M(q) 差 ~4e-7 | `verify_model_vs_urdf.py`，逐项审计 |
| armature | `equa_inertia`（厂商字段） | A/B 实验：16.85× plant 变化但闭环不敏感 → 保留 |
| 地板 | z = −1.2 m（真机基座在升降平台上） | 09-24 定 |
| 基座支撑 | chassis 1.30×0.84×0.49 m + platform_column 0.25×0.25 m（顶 z=−0.005，5 mm 间隙） | 尺寸出自 Xu et al., J. Field Robotics 2026 Table 4；ncon=0 已验证 |
| 物理步长 | 1 ms，integrator=implicitfast | |
| 关节限位 | ±3.04 rad（MJCF=URDF=launch 三处一致） | |
| 力矩限幅 | 80/80/60/16/16/10 N·m（厂商） | |
| 夹爪 | AG95，肌腱驱动（left_outer_knuckle），ctrl=0 为张开 | 语义未实测确认 🔲 |
| ready 关键帧 | qpos/qvel/ctrl **写满** 12/12/7；= go_ready READY = 启动 home（三处一致） | 两版 MuJoCo 都能编译 |
| home 末端 | (−0.0008, −0.7691, 0.1522)，距基座 784 mm，正前方作业区内 | |
| 夹持点 | MJCF ag95_base + 偏置 [−0.0405,−0.0143,0.1492]；与 URDF gripper_tip_link **现场对齐**：位置差 0.000000 m、姿态差 1e-16（139 样本） | `verify_fk_vs_tf.py` |
| MuJoCo 版本 | 仿真链 **3.12.0**（mujoco_vendor）；⚠️ 系统 python3 是 3.3.5 且缺依赖——**离线分析必须用 venv** | `ldd` 实测插件链接 |

---

## 3. 控制参数基线（全部在 config/，改前先读注释）

| 参数 | 值 | 依据 |
|---|---|---|
| x/y/z_proportional_gain | **30.0** | P 扫描 20/30/40（见 §5）；40 被阶跃探针否决 |
| angular_proportional_gain | **30.0**（随线向同步） | 旋转正弦验证：rx/ry 与线向同水平 |
| 积分/微分增益 | 0 | 纯 P；积分在锚定流上有害 |
| windup_limit | 0.05 | |
| publish_period（Servo） | 0.005 s | |
| butterworth_filter_coeff | 1.0（≈透明） | |
| lag_max（接口层） | **0.15 rad** | 健康滞后实测 ≤0.02；0.3 太松（领跑 0.36 rad 曾致折叠自碰撞） |
| v_max（接口层） | 2.618 / 3.142 rad/s（关节 1–3 / 4–6） | 厂商手册 V4.5.11 |
| 位置/姿态容差 | 0.01 m / 0.1 rad | LARA 先例（非官方 demo 的 0.001/0.01） |
| TARGET_POSE_TIMEOUT | 1.0 s | 断流停机的保险（已实测） |
| 奇异阈值 | 50 降速 / 200 急停 | **我们自己标定的**；与离线 cond 指标一致（减速≈96、停≈379）；VR 阶段可重调 |
| joint_limits.yaml 速度 | **2.618 / 3.142**（关节 1–3 / 4–6，厂商手册）；被 stage2b 与 stage3 同时加载。加速度为死配置（has_acceleration_limits: false，手册无此规格） | 2026-09-25 修正 |

**参数修改规则**（踩过的坑）：三处默认必须一起翻（stage2a 控制模式 / mujoco_model、stage3
output_mode）；gains 只在构造时读取，**改了必须重启 stage3**；go_ready 必须在 stage3 停止时跑
（否则与接口层在同一命令话题上打架）；`ros2 launch` 必须在 source 过 ROS 的 shell 里跑。

---

## 4. 测量仪器基线（口径决定结论，先读这个再信任何数字）

1. **末端位姿 = /joint_states + MuJoCo FK**，禁止用 TF2 Buffer（197 Hz×13 变换下积压
   0.3–0.5 s 且抖 ±0.14 s；/tf 话题本身只迟 0.003 s——伪像来自 listener 不来自话题）。
2. **台架必须用 `/home/lcw/tomato_robot/.venv/bin/python`**（rclpy + mujoco 3.12.0 都有；
   系统 python3 的 mujoco 缺 typing_extensions 连 import 都过不了）。
3. QoS：depth=1 + BEST_EFFORT（200 Hz 下默认 QoS 会积压成 0.2–0.5 s 旧数据）。
4. 相位/延迟的时基：数据帧时间戳（仿真时刻），不混用墙钟；跨时基的量必须做陈旧度标定
   （负载下时钟读数可旧 ~6–28 ms，D1 曾因此虚高 28 ms，靠 D2' 到达口径仲裁发现）。
5. 每轮测量后必须跑离线核验：`check_traj_contacts.py`（真实接触/cond/|cmd−q| 反算）。
6. 四元数一律 (x,y,z,w)；MuJoCo 原生是 [w,x,y,z]，曾因此把近恒等旋转读成 180°。
7. /joint_states 的消息内关节顺序 ≠ 我们 GROUP 的顺序（shoulder→foreArm→wrist1→upperArm…），
   按名字索引，不要按位置。

---

## 5. 性能基线（2026-09-24/25 实测，FK 通道，RTF=1.000）

### 5.1 P 扫描（为何是 30）

| P | 正弦跟随延迟 | 起始延迟 D2/D2' | survey | 20 mm 阶跃 | 判定 |
|---|---|---|---|---|---|
| 20 | 113 ms | 32–36 ms | 8/8 | 0% 超调 | 旧基线 |
| **30** | **83 ms** | **29–30 ms** | **8/8** | **0% 超调** | **✅ 采用** |
| 40 | 67 ms | 26–33 ms | 8/8（看不出！） | **638–722% 超调、2.25–2.47 rad/s、断流后冻结在离目标 11–28 mm** | ❌ 否决 |

教训：**survey/sine 看不出稳定性问题，阶跃探针才能**——VR 发的是离散目标且流会抖，阶跃探针
（`step_probe.py`，发 0.4 s 后切断让 Servo 走完）是安全验收门。P=30 距失稳边界余量 2 倍，
真机前不再上调。

### 5.2 三张表（P=30）

**① 停稳安静度（survey，8 位姿）**：✅ 8/8 静止，三次运行逐位一致。
末端 p2p 0.0000 m、命令 p2p 0.00000、终态误差 0.0000；用关节角 FK 独立复核 p2p ≤1.3 mm。
P06–P08（cond 65–128）整定过程出现 `奇异降速→离开奇异降速`，无急停无碰撞。

**② 跟随带宽（sine，A=15 mm，0.1/0.25/0.5/1.0 Hz，P01–P03）**：
- 线向 z：幅值比 **0.86–1.01**；滞后 = 恒定 **83 ms** 纯延迟（4.1°@0.1Hz → 31°@1Hz，秒数不变）。
- 旋转 rx/ry（10°）：幅值比 **0.96–1.00**，滞后 0.078–0.087 s——与线向同水平。
- 旋转 rz（世界系偏航）：⚠️ P02@1Hz 0.85、P03@1Hz 0.58——**运动学属性非调参缺陷**：
  绕世界 z 每 rad/s 的关节成本是 rx/ry 的 3–8 倍（P01 3.1 / P02 5.1 / P03 8.4），
  88–95% 压在 wrist1+wrist3（球形腕配平）。状态码无碰撞/到界。操作者会感觉"水平扫动重且迟"。
  （rz@P03 那行数据作废：当轮穿越差 66 mm 未到位。）
- VR 映射层设计输入：旋转建议映射到**工具轴参考**而非世界轴，可避开 rz 的腕型惩罚。

**③ 端到端延迟（latency，±5 mm × 10 reps × 3 位姿）**：
- D2 检测口径 = D2' 到达口径 = **29–30 ms**（两个独立墙钟估计一致 → 端到端真值）。
- D1 单时基口径 35–40 ms（多出起点帧年龄+状态戳约定 ~8 ms）。
- D3 命令侧（Servo 反应）**9 ms**；其余 ~20 ms 在 forward controller + MuJoCo 动力学到首动。

### 5.3 延迟分解（为什么是 83 ms、还能不能降）

| 段 | 实测 | 构成 |
|---|---|---|
| 命令路径（target→接口层输出） | ~48 ms | ≈ 33 ms（1/P，P=30 的闭环滞后）+ ~15–20 ms 管线（台架 8 + Servo 5 + 接口层 5 ms） |
| 被控对象侧（命令→实际末端） | ~36 ms | forward controller + 动力学 + joint_states 传输 + 检测（与阶跃延迟吻合） |

已到 ~83 ms；再降的唯一大杠杆是升 P（40 起阶跃失稳，否决）。管线最多再挤 ~8 ms，不值得。
**VR 链路还要叠加头显→中继→映射的 20–60 ms（未测）**，总回路预计 110–150 ms。

### 5.4 穿越与安全行为

- **穿越**：home→P02（0.51 m）74 s、home→P08（cond 127）80 s，到位误差 0.0000/0.0001 m，
  全程无警告、零接触、|cmd−q| 峰值 0.004–0.006 rad。✅
- **断流安全**（`stream_loss_test.py`）✅：硬切目标流后，Servo 按设计**走完最后一个收到的目标**
  （≤1.07 s），然后冻结——3 s 保持漂移 **0.0000 mm**；恢复发布**零跳变**（|cmd−q|=0.0000）。
  推论：VR 映射层必须只发小的、可独立成立的安全增量（reach limit 的职责）。
- **台架退让**：卡住检测分层（急停 2/4 → 0.3 s；碰撞降速 3、关节到界 5 → 1.2 s），
  退回移动起点 + 1.5 s 冷却 + 抬升重试。
- **历史教训存档**：幽灵地板（规划场景 TOP_Z 必须与 MJCF 地板联动，现 −1.23）；
  孤儿进程污染（清理模式覆盖 RSP/add_scene_floor/servo_interface/pose_tracking）；
  foreArm 2.94 rad 折叠构型（位姿生成加 |q|≤1.6 约束的原因）。

---

## 6. 位姿集基线（config/bench_poses.yaml）

- 8 个位姿，围绕 home 局部采样（σ=0.28 rad/关节，种子 42 可复现），任务区：
  基座前方 700–944 mm、y∈[−0.9,−0.7]、|z|≤0.501 m、工具轴近水平、|q|≤1.6 rad。
- 条件数分层选出：11.9 / 19.8 / 30.1 / 31.4 / 41.0 / 65.3 / 127.4 / 128.5。
- **路径检查是笛卡尔的**（位置 lerp + 姿态 slerp + 阻尼最小二乘 IK 连续跟踪，模拟 Servo
  局部解的分支连续性）：淘汰 4 个候选（2 个路径真碰撞、1 个 cond 97950、1 个 cond 228），
  幸存者 path_cond_max = 17.3–128.5 真实落盘。
- 反向校验：从表内 q 重算 FK，位置差 ~1e-7 m、cond 相对差 ≤3e-5。
- ⚠️ 已知局限：笛卡尔直线路径仍只是 Servo 实际走法的**模型**（它还受平滑/限幅影响）；
  且 Servo 局部控制器的穿越存在偶发分支漂移（rz 轮 P02→P03 差 66 mm 静默未达）。

---

## 7. 工具链一览（scripts/）

| 脚本 | 用途 |
|---|---|
| `follow_bench.py` | 三张表台架（survey/sine/latency；FK 通道；旋转轴支持） |
| `run_bench.sh` | 一键 bringup / stop / 跑表；台架固定 venv 解释器 |
| `servo_interface.py` | 伺服接口层（在链路上；`--selftest` 离线自检 T1–T4） |
| `gen_bench_poses.py` | 位姿集生成（笛卡尔路径检查 + 反向校验） |
| `step_probe.py` | 20 mm 阶跃超调/整定探针（VR 场景安全验收门） |
| `stream_loss_test.py` | 断流安全测试 |
| `verify_traversal.py` | 单条穿越故障复现 |
| `check_traj_contacts.py` | 离线核验：接触/cond/钳位反算（每轮必跑） |
| `verify_fk_vs_tf.py` | FK↔TF 现场对齐（换通道前置） |
| `simtime_skew_probe.py` / `tf_lag_probe.py` | 时基偏差 / TF 纯延迟标定 |
| `check_sine_fk.py` / `check_sine_target.py` | sine 数据双通道复核（TF 伪像定位用） |
| `verify_model_vs_urdf.py` / `armature_ab_offline.py` | 模型审计 / armature A/B |
| `pid_gain_calib.py` / `position_actuator_kp_calib.py` / `config/mujoco_pid.yaml` | 被否决实验的存档（不在链路上） |
| `go_ready.py` | 归位（stage3 停止时才能跑） |
| `add_scene_floor.py` | 规划场景地板盒（TOP_Z 与 MJCF 联动） |

一键流程：`bash scripts/run_bench.sh`（三张表）/ `bringup`（只起栈）/ `stop`（收摊）。

---

## 8. 开放项与已知限制（按优先级）

1. 🔲 **VR 输入链路验证**——vr-teleop-kit relay（FastAPI+WebRTC）+ Quest 3 浏览器从未在本机跑过；oculus_reader（2023 APK）只做兜底。**最大未知数。**
2. 🔲 **映射层**——ClutchPoseMapper（clutch + rot/pos reach limit 0.6 rad / 0.25 m）适配到 /target_pose；没有它没有 VR 遥操，没有 reach limit 就没有操作者保险。
3. 🔲 **视觉反馈**——相机未集成（camera_mount_link 占位）。仿真可先用 viewer / WebRTC 喂渲染画面；**真机前必须有相机方案**。
4. ✅ ~~sine/latency 静默失败~~（2026-09-25 修：go_to_pose 返回偏差 >5 mm 即打 ⚠️，标明数据基准）。
5. ✅ ~~规划场景缺 chassis/platform_column~~（2026-09-25 修：add_scene_floor.py 发布三个物体
   floor/chassis/platform_column，尺寸与 MJCF 逐字一致，现场验证在场景中且不影响现有位姿——
   P01/P02 survey 静止无警告。注意：moveit_msgs 的物体位姿存在**顶层 pose**，
   primitive_poses 是形状相对位姿——查询场景时两个都要看）。
6. ✅ ~~joint_limits 超厂商~~（2026-09-25 修：速度 2.618/3.142 对齐手册；加速度是死配置并已注明）。
7. ✅ ~~夹爪端到端~~（2026-09-25 验证：/gripper_controller/commands → position 肌腱执行器
   kp=200，ctrl **0=张开、0.93=闭合**，0.9→knuckle 0.9000、回 0→0.0000，链路无打架。
   抓真实物体的握力/滑移验证留给 VR 阶段）。
7a. ✅ 排查中顺带发现并修复：① run_bench.sh 起栈前自动 colcon build（install 曾落后于源）；
   ② stage3 launch 的 docstring 仍写着加载 panda demo 配置——实际从未加载（panda 的
   17/30 奇异阈值与 panda_hand 帧都是那文件的），已改为描述真实合并的两个 yaml。
8. ⚠️ rz 偏航体验（运动学，非缺陷）；大范围换构型属于规划器职责（move_group/OMPL 在、执行链故意断开）。
9. 🔲 **sim-to-real**：真机前硬关卡——P=30 余量、断流行为、lag_max、幅值比全部要重测。
10. ✅ git 远端（github.com/lcw578/vr_teleoperation_aubo_i5，SSH；改完代码记得 push）。

---

## 9. 版本与可复现性

- 仓库：`github.com/lcw578/vr_teleoperation_aubo_i5`（master）。外部依赖不入库，
  由 `deps.repos` + `DEPENDENCIES.md` 固定版本；`_refs/`、`_official/` 仅本地参考。
- 本基线对应的提交：见 `git log`（BASELINE.md 提交为准）。
- 复现路径：`bash scripts/run_bench.sh bringup` → 等 `forward_command_controller_position
  active` → `stream_loss_test.py` / `step_probe.py` / 三张表 → `check_traj_contacts.py` 核验。
- 环境要求：ROS 2 Humble + mujoco_vendor 3.12.0 + venv 解释器；改 P 需重启 stage3。

*文档作者注：本文中所有"实测"数字均可由 §7 工具复现；凡标注 ⚠️ 的数字带限定条件，标注 🔲 的尚不存在。*
