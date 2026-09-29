# VR 遥操仿真 · 正式基线（2026-09-28）

> 本文档是**VR 遥操控制链的当前正式基线**，与 [BASELINE.md](BASELINE.md)（控制链标定基线）
> 互补。所有数字均标注来源与验证方式。状态标记：✅ 已验证 ｜ ⚠️ 有限定条件 ｜ 🔲 未完成。

---

## 0. 一句话现状

VR 遥操链（Quest 3 → relay → 适配器 → 映射器 → 积分 tracker → MuJoCo）**全线打通**（方位约定 B：操作者在臂后方 +y 侧，2026-09-29 定案）：
合成端到端验证通过（Grip 接合 → 跟随 -y 48.9mm → 冻结 0.00mm），注入阶跃 -78.5mm
稳定零摆动，夹爪/断流安全全部实测。2026-09-28 头显首测报"前后/左右反向"，
全链审查定位：**方位约定从未定义（根因）+ RViz 相机在臂背面（视角镜像）+
yaw 补偿跨接合累积（bug）**。2026-09-28 定案约定 A 后，2026-09-29 用户澄清实际站在臂后方 → **升级为约定 B**（水平 180°）并同步全链
（机位 / yaw / 注释 / 方向自检脚本），**待戴头显复测跟手感**。

---

## 1. 正式链路（路线 B）

```
Quest 3 手柄（WebXR ~90 Hz，浏览器本地采集）
   ▼  xr_frame JSON（位姿+10按钮+viewer+t_client）
relay（vr-teleop-kit，Apache-2.0；LAN HTTPS 8443 / USB adb reverse）
   ▼  WebSocket 广播
quest_adapter_node.py（收一发一，~90Hz 透传）
   · R_CALIB = Rz(180°)·Rx(90°)（方位约定 B 解出，2026-09-29）
   ▼  /quest/pose + /quest/joy + /quest/head_yaw
clutch_mapper_node.py --input quest（100 Hz）
   · Grip 按住=接合（buttons[1]），松开=脱离
   · ClutchPoseMapper（上游移植，17 项测试）：滑移 reach limit
     rot 0.6 rad / pos 0.25 m
   · yaw 修正默认关（--yaw-comp 可开——头显内无画面时扭头看屏会误触发）
   · 缩放 1:2（quest 默认 scale=0.5）/ 1:10 微调（SCALE_FINE）
   · 断流 0.3 s 自动脱离
   ▼  /target_pose（脱离时不发）
lara_style_tracker.py（100 Hz，积分型关节位置命令）
   · cmd += J⁺·e·gain（阻尼最小二乘，自适应 cond 缩放 50/200）
   · 反极点 park：旋转误差 >2.2 rad 只停腕部旋转、位置照常解
   · q_rest Tikhonov 偏置 μ=0.02（破肘部翻转歧义）
   · 关节 clip ±2.94 + 每周期 |Δ|≤v_max·dt
   · 目标断流 0.5 s 冻结
   ▼  /forward_command_controller_position/commands
forward_command_controller → mujoco_ros2_control（1 kHz）
   ▼  /joint_states（197 Hz）
```

**已退役（不在链上，代码存档）**：MoveIt Servo pose tracking + servo_interface
（路线 A：锚定流+接口层组合存在结构性极限环，qd 峰值 4.84-5.77 超厂商限值；
路线 B 对照：阶跃 -78.5mm 零摆动、正弦 1.00、qd 1.84）。

---

## 2. 方位约定与坐标系语义（2026-09-29 定案，约定 B"操作者在臂后方"）

**方位约定（用户 2026-09-29 明确："在机械臂的后面操作，夹爪在最远离我的位置"）**：

| 项 | 定义 |
|---|---|
| world ≡ 臂基座系 | 臂焊死原点；+z 上；**臂工作区方向 = 世界 -y**（夹爪工作区；ready 末端待定，见 §7） |
| 操作者站位 | **臂后方 = 工作区对侧（世界 +y 侧）**，面向 -y（越过臂看工作区） |
| 手柄语义（约定 B） | **前推 = 末端远离操作者（世界 -y = 深入工作区）**；右推 = -x（操作者右 = 屏幕右）；上抬 = +z |
| Quest 帧 | WebXR local-floor（+x右/+y上/-z前）；**进入页面时身体面朝 -y 方向**（= 面向臂后方越过臂；重进页面 = 重定向） |
| 观察端 | RViz 相机在**操作者身后（+y 侧）**（teleop.rviz Yaw=+π/2）：屏幕里=-y、屏幕右=-x，与手柄语义同构；操作者天然看到夹爪背面，旋转跷跷板观感消失 |

**R_CALIB = Rz(180°)·Rx(90°)**（quest_adapter_node.py）＝约定 B 的直接数学解：
前(-z)→-y ✓、右(+x)→-x ✓、上(+y)→+z ✓、左→+x ✓、下→-z ✓（det=+1）。
⚠️ **它不存在普适值**——取决于操作者站位与 Quest 重定向（上游 bi_quest_teleop.py
L87-91 原文："Derived empirically …; override if your robot faces the operator
differently"）。换站位/重定向后跑 `direction_check.py` + `rotation_check.py` 复核。

| 通道 | 参考系 | 说明 |
|---|---|---|
| **平移** | 臂基座系 | 手柄位移经 R_CALIB 旋转，按缩放加到接合锚点；head-yaw 补偿**默认关**（头显内无画面、扭头看外接屏会误触发——视频回传上线后再 `--yaw-comp` 打开） |
| **旋转** | 臂基座系（**世界轴**） | 手柄绕哪根世界轴转 θ → 末端目标绕**同一根世界轴**转 θ×缩放，**与接合姿态无关**（上游原版语义；2026-09-28 撤销"接合时工具系再表达"——它使轴对应随接合姿态漂移，头显欧拉辨识实锤手柄 pitch→末端 roll 轴交叉）。验证：rotation_check.py 三轴 dot=+1.000 |

**处置表（direction_check 三向有 ✗ 时）**：
- 前、右**同时**反（差 180°）→ Quest 重定向反了：重新进入页面，身体面朝 -y 方向（臂后方）。
- 单独一个轴反 → 纯旋转不可能做到（det=-1），说明链上有二次变换——查谁又在动 /quest/pose。
- 差 ~90° → 重定向朝向与站位差 90°，同样重进页面对齐后复测。

---

## 3. 键位（Quest 3 手柄，WebXR xr-standard 实锤）

| 按钮（xr-standard 索引） | 物理位置 | 功能 |
|---|---|---|
| **1** = squeeze/grip（模拟量） | 中指侧键 | **离合**：按住=接管，松开=急停冻结 |
| **0** = trigger（模拟量） | 食指 | 夹爪开/闭（两状态，v>0.7 阈值上升沿） |
| 3 = A（右）/X（左） | 上右钮 | 预留：缩放切换 |
| 4 = B（右）/Y（左） | 上左钮 | 预留：E-stop / rest-ramp |
| 2/5/6 | 摇杆按压/thumbrest/menu | 未映射 |

来源：client.js L1466 注释 + WebXR xr-standard profile + 用户按键指纹实测。

---

## 4. 已验证的性能数字

| 项 | 数值 | 验证方式 |
|---|---|---|
| 注入阶跃 z-8cm | **1.5 s 到位 -78.5 mm，零摆动**（两遍逐位一致） | inj_stab.py |
| 合成端到端（Grip→跟随→冻结） | -y 48.9mm 方向正确、冻结漂移 0.00mm | e2e_synth_test.py |
| 正弦跟随（tracker 首测） | 幅值比 1.00、qd 峰值 1.84 rad/s | sine 手动注入 |
| 上游映射器数学 | 17 项测试全过（工具轴语义差 0.0000°） | test_clutch_mapper.py |
| 夹爪 | OPEN 0.0 / CLOSED 0.9，两状态+斜率 | 用户实测 ✓ |
| 断流安全 | 头显息屏=下游自动冻结（stale 双保险：adapter 0.3s / tracker 0.5s） | stream_loss 实测语义 |
| 方向自检（约定 B 三向） | 🔲 待头显复测（2026-09-28 的 3/3 是约定 A 下测的，换 B 后需重测） | direction_check.py |
| 旋转自检（世界轴三轴） | 离线数值验证轴对齐 dot=+1.000（ready/朝下两种接合姿态）；🔥 待头显复测 | rotation_check.py |
| 映射器回归（20 项） | 世界轴语义全过，含换接合姿态不变性（TA2） | test_clutch_mapper.py |

---

## 5. 稳定性经验（全部踩过的坑，防御已固化）

| 坑 | 防御 |
|---|---|
| 对上游的"改进"（R_align 工具系再表达省腕部行程）使轴对应随接合姿态漂移 → 手柄 pitch 出末端 roll 的轴交叉（2026-09-28 头显欧拉辨识实锤） | 撤销偏离、回归上游世界轴语义（clutch_pose_mapper）；旋转自检 rotation_check.py；**新规矩：对上游任何偏离必须写明动机+真头显验证+可一键切回** |
| RViz 相机在臂背面（+y 侧）→ 屏幕方向与手柄语义整体镜像，表现为"前后左右全反" | 机位定案 -y 侧（Yaw=-π/2），与操作者站位同构（2026-09-28） |
| 扭头看外接屏 = head-yaw 误补偿；旧代码 R_trans 跨接合累乘，映射被永久转走 | yaw 补偿默认关（--yaw-comp 可开）；代码改绝对式覆写（2026-09-28） |
| R_CALIB 注释"验证表"与矩阵矛盾（det=-1 镜像）、"唯一正确"声明误导排查 | 注释重写：写明依站位约定而定 + direction_check 复核（2026-09-28） |
| MuJoCo viewer 关窗 = segfault 杀仿真 | stage2a 固化 headless:=true（无窗口） |
| viewer 空格键 = 暂停仿真 | 操作时焦点勿留 viewer（headless 后自然规避） |
| 多实例节点互发覆盖 | 所有启动脚本幂等（kill 名单 + 启动），禁止手动散起 |
| pkill 自匹配杀掉自己的 shell | 杀名单用 `[x]` 括号转义 |
| 息屏 = 页面 JS 暂停 = ws 断 | 摘头显前先说一声；系统侧 stale 双保险（0.3s/0.5s） |
| 四元数 wxyz/xyzw 三次错位 | 从 MuJoCo API 取值必须先转 (x,y,z,w)；工具轴比对验证法 |
| /joint_states 配对按消息内顺序 | 解析时禁止按 GROUP 顺序 zip |
| ROS 图发现延迟（新节点收不到话题） | 节点全起后 sleep 8s 再验证；restart_vr_chain.sh 已内置 |

---

## 6. 启动/关闭（完整流程）

```bash
# ── 拉起（两条命令）──
bash ros2_ws/src/aubo_i5_teleop/scripts/run_relay.sh          # relay（头显页面入口）
bash ros2_ws/src/aubo_i5_teleop/scripts/authoritative_session.sh  # 仿真栈+归位+四节点

# ── 头显端 ──
# 浏览器 https://<PC-IP>:8443/ → Enter VR → Calibrate wrist → Start Teleop
# （进入页面时身体面朝操作者前方 = 平时看显示器的方向，见 §2）

# ── 方向自检（首次会话/换站位后必做；全程不按 Grip）──
bash ros2_ws/src/aubo_i5_teleop/scripts/direction_check.py   # 三向 ✓ 再开始遥操

# ── 旋转自检（验证世界轴语义；臂会真实动，动作慢而小 ~20°）──
bash ros2_ws/src/aubo_i5_teleop/scripts/rotation_check.py    # 三轴 dot>0.8 ✓
# 换一个臂姿态再跑一遍——结果应不变（"与接合姿态无关"回归判据）

# ── 验证（一条命令）──
bash ros2_ws/src/aubo_i5_teleop/scripts/inj_stab.py           # 注入阶跃（应为 -78.5mm 稳定）

# ── 关闭 ──
bash ros2_ws/src/aubo_i5_teleop/scripts/run_bench.sh stop     # 仿真栈全清
# VR 节点：restart_vr_chain.sh 重跑即自动清
```

---

## 7. 开放项（按优先级）

1. 🔲 **真实头显佩戴复测**——方向自检（direction_check.py 三向）、跟手感、
   Trigger 手感、RViz 新机位（-y 侧）画面确认；代码全就位等实测
2. 🔲 A 键缩放切换接线（结构已留）
3. 🔲 Trigger 模拟量直驱夹爪（LARA 式，替代两状态——可选）
4. 🔲 头显内场景画面（视频回传）——上线后才能安全打开 `--yaw-comp`
5. 🔲 录制管线（/joint_states + /quest/pose + 图像 → lerobot 数据集）
6. 🔲 真机接入（lara_tracker 的积分命令语义 = 真机控制柜标准输入，直连）

---

*本基线对应提交：3cbc05c 之后。所有"实测"数字可由 §6 工具复现。*
