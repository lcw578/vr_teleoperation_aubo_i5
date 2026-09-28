# VR 遥操仿真 · 正式基线（2026-09-28）

> 本文档是**VR 遥操控制链的当前正式基线**，与 [BASELINE.md](BASELINE.md)（控制链标定基线）
> 互补。所有数字均标注来源与验证方式。状态标记：✅ 已验证 ｜ ⚠️ 有限定条件 ｜ 🔲 未完成。

---

## 0. 一句话现状

VR 遥操链（Quest 3 → relay → 适配器 → 映射器 → 积分 tracker → MuJoCo）**全线打通**：
合成端到端验证通过（Grip 接合 → 跟随 -y 48.9mm → 冻结 0.00mm），注入阶跃 -78.5mm
稳定零摆动，夹爪/断流安全全部实测。**剩余唯一未验证项：真实头显佩戴下的手感
（方向复测 + 跟手度）**——所有代码修复已就位等这一次实测。

---

## 1. 正式链路（路线 B）

```
Quest 3 手柄（WebXR ~90 Hz，浏览器本地采集）
   ▼  xr_frame JSON（位姿+10按钮+viewer+t_client）
relay（vr-teleop-kit，Apache-2.0；LAN HTTPS 8443 / USB adb reverse）
   ▼  WebSocket 广播
quest_adapter_node.py（收一发一，100 Hz）
   · R_CALIB = Rz(-90°)（用户四方向实测反解，2026-09-28）
   · stale 保护：帧龄 >0.3 s 停发
   ▼  /quest/pose + /quest/joy + /quest/head_yaw
clutch_mapper_node.py --input quest（100 Hz）
   · Grip 按住=接合（buttons[1]），松开=脱离
   · ClutchPoseMapper（上游移植，17 项测试）：滑移 reach limit
     rot 0.6 rad / pos 0.25 m
   · yaw 修正：接合时锁头显基准偏航，Δyaw 补偿平移系
   · 缩放 1:3（SCALE_COARSE）/ 1:10 微调（SCALE_FINE）
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

## 2. 坐标系语义（定案）

| 通道 | 参考系 | 说明 |
|---|---|---|
| **平移** | 臂基座系 | 手柄在世界里的移动向量，经 R_CALIB（Rz(-90°)，用户四方向实测反解）旋转后映射 |
| **旋转** | 接合时工具系 | 转手腕=绕末端自身轴转（R_align 接合对齐，会话内固定，重离合重对齐） |
| **yaw 修正** | 接合时锁定基准 | Δyaw（头显当前−标定）补偿进平移系——操作者转身不影响方向 |

**R_CALIB 依据**：用户头显四方向实测（前推→下、上推→操作者左、左推→后），
三组正交观测解出唯一纯旋转 Rz(-90°)（det=+1、正交性验证通过）。
矩阵/四元数两条路径逐向量一致性验证通过。

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

---

## 5. 稳定性经验（全部踩过的坑，防御已固化）

| 坑 | 防御 |
|---|---|
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

# ── 验证（一条命令）──
bash ros2_ws/src/aubo_i5_teleop/scripts/inj_stab.py           # 注入阶跃（应为 -78.5mm 稳定）

# ── 关闭 ──
bash ros2_ws/src/aubo_i5_teleop/scripts/run_bench.sh stop     # 仿真栈全清
# VR 节点：restart_vr_chain.sh 重跑即自动清
```

---

## 7. 开放项（按优先级）

1. 🔲 **真实头显佩戴复测**——方向（R_CALIB 确认）、跟手度、Trigger 手感；代码全就位等实测
2. 🔲 A 键缩放切换接线（结构已留）
3. 🔲 Trigger 模拟量直驱夹爪（LARA 式，替代两状态——可选）
4. 🔲 录制管线（/joint_states + /quest/pose + 图像 → lerobot 数据集）
5. 🔲 真机接入（lara_tracker 的积分命令语义 = 真机控制柜标准输入，直连）

---

*本基线对应提交：3cbc05c 之后。所有"实测"数字可由 §6 工具复现。*
