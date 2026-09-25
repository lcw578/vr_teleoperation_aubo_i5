# VR 遥操作 · Aubo i5 + AG95（仿真先行）

Aubo i5 六轴机械臂 + AG95 两指夹爪的 VR 遥操作系统，**仿真先行**：在 MuJoCo 里把控制链
做到可测量、可复现、可验证，再接真机与头显。

- 当前基线（所有标定数字）：**[BASELINE.md](BASELINE.md)** ← 唯一权威参考
- 依赖与来源清单：[DEPENDENCIES.md](DEPENDENCIES.md) + [deps.repos](deps.repos)
- 当前状态：控制链 ✅ 已标定完成；VR 输入层 🔲 未开工（头显未到货）

> ⚠️ 根目录的《VR遥操调试方案.md》**已废弃**（用户确认会误导设计），不要引用。

---

## 1. 一分钟看懂整条 Pipeline

```
【输入层】
  台架 follow_bench.py（现在，125 Hz）        ← VR 映射层（将来）同样发这里
      │  PoseStamped @ /target_pose（world 帧，绝对位姿；Servo 层输入流超时 0.1 s
      │  与目标超时 1.0 s ⇒ 映射层发布必须 >10 Hz 且断流≤1 s 是安全的）
      ▼
【Servo 层】moveit_servo::PoseTracking（C++，官方库）
      │  纯 P 外环（x/y/z 与 angular 均为 30，I=D=0）→ 笛卡尔速度
      │  容差 0.01 m / 0.1 rad；5 ms 周期
      ▼
  MoveIt Servo（官方库）
      │  雅可比伪逆 → 关节增量 Δ（200 Hz）
      │  奇异 50 降速/200 急停（离开 ×2.0）；碰撞降速/急停；关节到界；Butterworth≈透明
      │  ⚠️ 输出是【锚定流】：cmd = 实测 + 单周期 Δ —— 不是绝对轨迹
      ▼
  /servo_position_stream
      ▼
【伺服接口层】scripts/servo_interface.py（自研，唯一在链路上的自写控制件，200 Hz）
      │  把锚定流积分成绝对位置命令 q_cmd（等价真机 servoj 的速度前馈步骤）
      │  保护① |Δ|≤v_max·T（厂商 2.618/3.142 rad/s）
      │  保护② |q_cmd−q_meas|≤0.15 rad（抗积分饱和 = 最大憋压深度）
      │  保护③ 输入断流 >0.5 s → 重新对齐到 实测+Δ
      ▼
  /forward_command_controller_position/commands
      ▼
【被控对象】forward_command_controller → mujoco_ros2_control 插件（1 kHz 物理步长）
      │  MuJoCo <position> 执行器 kp=25000/2500, dampratio=1.0（6 臂关节）
      │  AG95 夹爪 = 肌腱执行器 kp=200（ctrl 0=张开，0.93=闭合）
      │  每个刚体 gravcomp=1.0（模拟真机控制柜的重力补偿）
      ▼
  /joint_states（197 Hz，时间戳与仿真时钟差 −0.001 s）
      ▼
【测量通道】/joint_states + MuJoCo FK —— ⚠️ 禁止走 TF
      （TF2 Python listener 在 197 Hz×13 变换下积压 0.3–0.5 s，伪像来源；
        /tf 话题本身只迟 0.003 s，/joint_states 迟 −0.001 s，都 punctual）

【规划层】move_group（stage2b）+ OMPL/Pilz —— 大范围换构型用；
          当前执行链故意不经它（JTC 未接），只做规划场景持有者。
```

**为什么必须有伺服接口层**（结构性理由，调参修不了）：Servo 的位置输出锚在"实测+Δ"上，
下游任何 `error = cmd − 实测` 的 PID 误差恒等于 Δ，对臂的真实位置是盲的——静止时无保持
力矩、运动时积分失控。接口层把流还原成绝对轨迹。同一条结论否决了两条替代路：拉高执行器
kp 强制增益=1（启动即失败）；mujoco_ros2_control 内置 PID（error≡Δ，同一盲区）。

---

## 2. 关键设计决策（以及被否决的路线）

| 决策 | 结论 | 一句话理由 |
|---|---|---|
| 位置 vs 速度模式 | **位置模式** | 该臂不支持速度模式（用户裁定 + 驱动源码验证：厂商驱动的 velocity 接口无人读） |
| 仿真器 | **MuJoCo**（mujoco_ros2_control 桥接 ROS 2） | 厂商 aubo_gazebo 是空壳；Gazebo Classic 死路 |
| 控制栈 | **MoveIt Servo pose tracking**（官方库，自写 C++ 入口装配） | LARA 同类先例；自写 IK/twist 流已否决 |
| 大范围换构型 | **规划器（move_group/OMPL）**，VR 内靠小步+clutch | Servo 是局部控制器，会折臂进自碰撞（实测） |
| 重力 | 每个 body `gravcomp=1.0` | 真机控制柜本来就做重力补偿，不是作弊 |
| armature | 厂商 `equa_inertia` 字段 | A/B 实验：16.85× plant 变化但闭环不敏感 |
| 依赖管理 | deps.repos 固定版本，外部代码不入库 | clone 可复现；`dh_ag95_description` 是我们的移植产物，入库 |
| VR 层 | **拿 vr-teleop-kit 的 relay + ClutchPoseMapper**，保留本控制层 | 用户已拍板；WebXR 免 APK；reach limit 是防折臂的关键 |

被否决的完整记录：内置 PID（`config/mujoco_pid.yaml` 与 `scripts/pid_gain_calib.py` 存档）、
kp 拉高标定（`position_actuator_kp_calib.py` 存档）、Manipulator-Mujoco 整包替换、
Gazebo Classic、aubo_hardware_interface（真机专用、拒绝我们的 URDF）、dora-rs。

---

## 3. 仓库结构

```
VR_teleoperation/
├── README.md                ← 本文件
├── BASELINE.md              ← 正式基线：全部标定数字、参数、仪器口径、开放项
├── EXECUTION_PLAN.md        ← 已执行完毕的六步计划（存档）
├── DEPENDENCIES.md          ← 每个外部包的来源/版本/是否改过 + 自写内容依据
├── deps.repos               ← vcstool 依赖清单（固定 commit）
├── "AUBO-i5 & CB4 user manual_V4.5.11.pdf"  ← 厂商手册（v_max 等常数的出处）
├── assets/aubo_i5/          ← MJCF 模型
│   ├── scene_aubo_i5.xml        机器人本体（SRDF 同源排除、gravcomp、armature）
│   ├── scene_ros2.xml           正式场景（地板-1.2 + 底盘 + 立柱 + 关键帧 12/12/7）
│   ├── scene_ros2_velocity.xml  速度模式对照场景（实验用）
│   ├── scene_*_armB.xml         冻结的 armature A/B 实验变体
│   └── actuators_position.xml   6 臂 <position> + 1 夹爪肌腱执行器
└── ros2_ws/                 ← colcon 工作空间
    └── src/
        ├── aubo_i5_teleop/      【我们自己的包】bringup + Servo 装配 + 台架 + 工具
        ├── dh_ag95_description/  AG95 夹爪（上游 catkin→ament 移植，几何逐字节未改，入库）
        ├── aubo_ros2_driver/     厂商驱动+描述（deps 固定版本，未改）
        └── mujoco_ros2_control/  MuJoCo↔ros2_control 桥（deps 固定版本，未改）
```

`aubo_i5_teleop` 内部：

```
├── launch/  stage1_moveit_rviz / stage2a_mujoco / stage2b_moveit / stage3_pose_tracking（stage3_servo 为旧 twist 流，留档）
├── config/  controllers.yaml（3 控制器）、servo_pose_tracking.yaml（Servo 主体参数）、
│            pose_tracking_settings.yaml（PID 增益，P=30 定标依据写在注释里）、
│            servo.yaml（twist 流时代参数，stage3_servo 用）、joint_limits.yaml（±3.04 + 厂商速度限位）、
│            teleop.srdf（98 对排除）、bench_poses.yaml（8 测试位姿）、mujoco_pid.yaml（否决存档）
├── src/     pose_tracking_node.cpp（Servo 装配：PSM + PoseTracking，官方 demo 骨架+4 处有理由的修改）
├── urdf/    aubo_i5_teleop.urdf.xacro（官方 URDF + ee_link + AG95 + ros2_control(MuJoCo)）
└── scripts/ 见 §6 工具清单
```

---

## 4. 环境要求（不满足会以奇怪的方式失败）

| 项 | 要求 | 坑 |
|---|---|---|
| ROS 2 | Humble | |
| MuJoCo（仿真） | **3.12.0**（mujoco_vendor） | 插件 `ldd` 实测链 3.12.0；MJCF 的关键帧 12 qpos 在 3.3.5 下编译不过 |
| MuJoCo（离线分析） | **必须与仿真同版本**：用 `/home/lcw/tomato_robot/.venv/bin/python`（rclpy + mujoco 3.12.0 都有） | 系统 python3 的 mujoco 3.3.5 缺 typing_extensions，import 都过不了 |
| 构建 | `colcon build --packages-select aubo_i5_teleop --cmake-args -DCMAKE_BUILD_TYPE=Release` | 本包 install 是普通拷贝——**run_bench.sh 起栈前会自动 build**，手改源文件后不 build 就跑是历史踩过的坑 |

---

## 5. 快速开始

```bash
# 起栈（自动编译 → stage2a 仿真 → stage2b MoveIt → 归位 → stage3 Servo；然后保持运行）
bash ros2_ws/src/aubo_i5_teleop/scripts/run_bench.sh bringup

# 跑三张表（安静度 / 跟随带宽 / 端到端延迟；也可单独指定，如 `survey`）
bash ros2_ws/src/aubo_i5_teleop/scripts/run_bench.sh            # 全部
bash ros2_ws/src/aubo_i5_teleop/scripts/run_bench.sh survey     # 只跑一张

# 收摊
bash ros2_ws/src/aubo_i5_teleop/scripts/run_bench.sh stop
```

单工具用法（栈在跑时）：

```bash
cd ros2_ws/src/aubo_i5_teleop
PY=/home/lcw/tomato_robot/.venv/bin/python
$PY scripts/step_probe.py --out /tmp/step.csv          # 20 mm 阶跃超调/整定（VR 场景安全验收门）
$PY scripts/stream_loss_test.py                        # 断流安全测试
$PY scripts/verify_traversal.py --pose P02             # 单条穿越复现
/home/lcw/tomato_robot/.venv/bin/python scripts/check_traj_contacts.py /tmp/*.csv   # 离线核验
```

---

## 6. 工具清单（scripts/）

| 脚本 | 用途 |
|---|---|
| `follow_bench.py` | 台架三模式：`--survey`（多点停稳安静度）/ `--sine`（跟随带宽，支持线向 x/y/z 与旋转 rx/ry/rz）/ `--latency`（端到端延迟，D1/D2/D2'/D3 四口径） |
| `run_bench.sh` | 一键 bringup/stop/跑表 + 起栈自动编译 + 孤儿进程清理 |
| `servo_interface.py` | 伺服接口层（在链路上）；`--selftest` 离线自检 T1–T4 |
| `gen_bench_poses.py` | 位姿集生成：任务区采样 + 笛卡尔路径 IK 检查 + 反向校验（种子 42 可复现） |
| `step_probe.py` | ±20 mm 阶跃超调/整定探针（发 0.4 s 后切断让 Servo 走完，暴露完整响应） |
| `stream_loss_test.py` | 断流安全：硬切目标流 → 走完最后目标 → 冻结 → 恢复零跳变 |
| `verify_traversal.py` | 单条穿越的故障复现工具 |
| `check_traj_contacts.py` | 离线核验：从 CSV 用 MuJoCo 3.12.0 反算真实接触/cond/\|cmd−q\|（每轮必跑） |
| `verify_fk_vs_tf.py` | FK↔TF 现场对齐验证（换测量通道的前置） |
| `simtime_skew_probe.py` / `tf_lag_probe.py` | 时基偏差 / TF 纯延迟标定 |
| `check_sine_fk.py` / `check_sine_target.py` | sine 数据双通道复核（TF 伪像定位） |
| `verify_model_vs_urdf.py` | MJCF↔厂商 URDF 逐项比对（M(q) 差 ~4e-7） |
| `armature_ab_offline.py` | armature A/B 离线实验 |
| `pid_gain_calib.py` / `position_actuator_kp_calib.py` | 两条被否决标定路线的存档（不在链路上） |
| `go_ready.py` | 归位（⚠️ 必须在 stage3 停止时跑，否则与接口层在命令话题上打架） |
| `add_scene_floor.py` | 把地板+底盘+立柱发布进规划场景（尺寸与 MJCF 逐字联动） |

---

## 7. 各环节细节（改任何东西前先读对应小节）

### 7.1 输入层约定
- 话题 `/target_pose`，`geometry_msgs/PoseStamped`，frame 必须是 `world`，**绝对位姿**
  （不是增量）；台架以 125 Hz 发布。
- go_to_pose 若超时/重试耗尽会**返回当前位置并打 ⚠️**（>5 mm 偏差）——sine/latency 的
  数字以该偏差基准测得。

### 7.2 Servo 层（config/servo_pose_tracking.yaml + pose_tracking_settings.yaml）
- 两份 yaml 合并进 `moveit_servo` 命名空间（stage3 launch 字典合并，后者被前者覆盖——
  见 launch 文件头说明；**不加载官方 panda demo 配置**，其中 17/30 阈值与 panda_hand 帧都是 Panda 的）。
- 改 PID 增益**必须重启 stage3**（构造时读取，不支持运行时改）。
- 奇异 50/200 与离线 cond 指标一致（减速≈96、停≈379）；VR 阶段可重调。

### 7.3 伺服接口层（servo_interface.py）
- 两条保护 + 断流重对齐见 §1 图内注释；`v_max` 与 `lag_max` 必须与 launch 参数一致
  （`lag_max` 默认与 stage3 launch 已同步为 0.15）。
- 历史教训：0.3 太松（领跑 0.36 rad 后换目标猛收→折臂自碰撞）。

### 7.4 被控对象（assets/aubo_i5/）
- `±3.04 rad` 关节限位**三处一致**：joint_limits.yaml / MJCF joint range / URDF 命令接口。
- 关节速度限位 2.618/3.142（厂商手册）；`joint_limits.yaml` 同时被 stage2b 与 stage3 加载。
- 夹爪：`/gripper_controller/commands` 发 `[0.0]`=张开、`[0.93]`=闭合（单命令，肌腱分左右）。
- 改 MJCF 场景几何（地板/底盘/立柱）时**必须同步改 `add_scene_floor.py` 的 OBJECTS 表**，
  否则重演"幽灵地板/物理挡住但 Servo 看不见"。

### 7.5 测量（仪器口径，违反任何一条数字就不可信）
1. 末端位姿 = `/joint_states` + MuJoCo FK（venv 解释器）；TF 通道禁用（积压 0.3–0.5 s）。
2. QoS：depth=1 + BEST_EFFORT（200 Hz 下默认 QoS 积压 0.2–0.5 s 旧数据）。
3. 相位/延迟用数据帧时间戳（仿真时刻），不混用墙钟；跨时基必须标定陈旧度。
4. 四元数一律 (x,y,z,w)；MuJoCo 原生 [w,x,y,z]（曾因此把近恒等旋转读成 180°）。
5. `/joint_states` 消息内关节顺序 ≠ 惯用顺序（shoulder→foreArm→wrist1→upperArm…），
   **按名字索引**。
6. 每轮测量跑 `check_traj_contacts.py` 离线核验（接触/cond/钳位）。
7. 位姿表数字的口径：夹持点 = MJCF `ag95_base` + 偏置 [−0.0405,−0.0143,0.1492]
   （与 URDF `gripper_tip_link` 现场对齐 0.000000 m / 1e-16）。

### 7.6 三处默认值规则（历史上翻车过）
`stage2a` 的 `arm_control_mode`/`mujoco_model`、`stage3` 的 `output_mode` 必须一致
（位置模式：position + scene_ros2.xml + position）；`run_bench.sh` 已显式传参，不依赖默认。

---

## 8. 安全机制汇总

| 机制 | 行为 | 状态 |
|---|---|---|
| 目标流断流（VR 崩溃/网络） | Servo 走完**最后一个收到的目标**（≤1.07 s）→ 冻结；3 s 漂移 0.0000 mm；恢复零跳变 | ✅ 实测 |
| 接口层抗饱和 | 命令领跑 ≤0.15 rad；钳位触发 = 以 kp·lag_max 力矩憋压（最大憋压深度） | ✅ 在链路 |
| 接口层单周期限幅 | \|Δ\|≤v_max·T（厂商速度上限） | ✅ 在链路 |
| Servo 奇异/碰撞/到界 | 50 降速 → 200 急停；碰撞降速→急停；关节到界 | ✅ 在链路 |
| 台架退让 | 状态 2/4（0.3 s）/3、5（1.2 s）→ 退回移动起点 → 冷却 1.5 s → 抬升重试 | ✅ 在链路 |
| 规划场景几何 | 地板/底盘/立柱进 Servo 碰撞检查 | ✅ 实测 |
| P=30 稳定余量 | 阶跃 0% 超调；P=40 阶跃 638–722% 超调已否决（survey/sine 看不出，**阶跃探针才是安全验收门**） | ✅ 实测 |

---

## 9. 位姿集与任务区

- 任务区（基座坐标系）：正前方 **700–944 mm**、y∈[−0.9,−0.7] m、|z|≤0.501 m、工具轴近水平。
  来源：Xu et al., J. Field Robotics 2026（番茄串采摘样机）。
- 8 个测试位姿围绕 home 局部采样（σ=0.28 rad，种子 42），条件数分层
  11.9–128.5，**笛卡尔路径**检查（2 个路径真碰撞、1 个 cond 97950、1 个 cond 228 被淘汰），
  `path_cond_max` 17.3–128.5 落盘；从表内 q 反向校验至 ~1e-7 m。
- home（ready）：`[-1.089254, -0.802598, 1.308255, 0.588418, 0.324607, -0.704605]`，
  末端 (−0.0008, −0.7691, 0.1522)，距基座 784 mm，三处一致（关键帧/go_ready/生成器）。

---

## 10. VR 接入路线（头显到货后）

1. **relay**：vr-teleop-kit 的 FastAPI+WebSocket 中继 + WebRTC 视频轨，Quest 浏览器 WebXR
   （免 APK/logcat；oculus_reader 的 2023 APK 只做兜底）。从未在本机验证——第一步。
2. **映射层**：✅ 已完成（2026-09-25，头显未到先用 Mock 验证）。
   `mock_vr_node.py`（键盘 WASD+QE 平移 / UO-IK-JL 自系旋转 / 空格离合 /
   Shift 缩放 / Z 夹爪；或 `--script` 回放）→ `clutch_mapper_node.py`
   （离合 + 接合轴对齐 + 滑移 reach limit 0.6 rad / 0.25 m + 0.3 s 断流自动脱离）
   → `/target_pose`。验收：`scripts/vr_mock_acceptance.py` 11 项判据连续两次全过
   （工具自系旋转 79° 漂移 0.5 mm、释放冻结 0.00 mm、切档零跳变、夹爪并行）。
   硬约束保留：发布 >10 Hz、只发小的可独立成立的安全增量。
3. **视觉反馈**：仿真期可用 viewer / relay 的 WebRTC 喂渲染画面；**真机前必须有相机方案**
   （URDF 已留 `camera_mount_link`，值为占位）。
4. 端到端叠加延迟预估：控制链 83 ms + VR 传输 20–60 ms ≈ 110–150 ms（待实测）。

---

## 11. 已知限制与开放项

见 [BASELINE.md §8](BASELINE.md)。要点：VR 三件事未开工；rz 偏航体验（运动学属性）；
大范围换构型属规划器职责（OMPL 在位、执行链故意断开）；**真机前所有标定数字必须重测**
（sim-to-real 关卡）。

## 12. 参考资料

- AUBO-i5 & CB4 用户手册 V4.5.11（本仓库根目录 PDF）——速度上限 150/180°/s、臂展、基座 Ø172 mm
- Xu et al., J. Field Robotics 2026——任务区几何（番茄串采摘样机）
- [LARA_AUBOi5_AG95](https://github.com/ian-chuang/LARA_AUBOi5_AG95)——Aubo i5+AG95+Servo 遥操先例（容差取值出处）
- [vr-teleop-kit](https://github.com/Dream-Machines-Robotics/vr-teleop-kit)（本地 `_refs/`）——WebXR relay + ClutchPoseMapper
- MoveIt 2 Servo `pose_tracking_demo`（Humble）——C++ 入口的官方骨架
- [Manipulator-Mujoco](https://github.com/ian-chuang/Manipulator-Mujoco)（本地 `_refs/`）——AG95 MJCF 手法出处；整包替换已否决
