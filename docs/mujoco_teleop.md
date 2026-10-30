# 独立 MuJoCo 遥操：作者代码移植与 AUBO 测试入口

推荐仿真入口为 `scripts/run_mujoco_teleop.sh`。不需要启动 ROS、RViz、
MoveIt 或外部 relay。旧 ROS Route B 保留为回退，原启动脚本不自动改指向。
默认采用作者式运动学预览，直接显示 IK 的关节位置，用 `mj_forward` 刷新，
不执行 `mj_step` 或新增加速度整形。作者原始参考为 `examples/pure_sim.py`
和 `tools/viewer_client.py`。world＋当前RG姿态已获操作者手感验收，初步固定为
[sim-world-rg-2026-10-12](simulation_baseline.md)。tool_local仍为比较项。

## 启动与比较

先在原来运行旧仿真的终端 Ctrl+C 退出会话，在运行旧 relay 的终端 Ctrl+C
退出 relay。新旧入口使用同一个工作区锁；端口占用会报错，不会全局杀进程。

```bash
cd /home/lcw/VR_teleoperation
# 作者的工作区旋转，默认 RG、显示 MuJoCo Viewer、三路相机、记录关闭
bash scripts/run_mujoco_teleop.sh --rotation world

# 退出上一个会话后，运行工具局部旋转；其余参数相同
bash scripts/run_mujoco_teleop.sh --rotation tool_local

# 上面两条均默认 kinematic，也可以显式写明
bash scripts/run_mujoco_teleop.sh --backend kinematic --rotation world
```

LAN：头显打开 `https://电脑当前热点或局域网IP:8443/`，沿用现有证书。
端口 8443 是本地遥操服务，与头显上用于联网的电脑代理端口不同。

USB 对照：

```bash
adb reverse tcp:8443 tcp:8443
bash scripts/run_mujoco_teleop.sh --usb --rotation world
# Quest 浏览器打开 http://localhost:8443/
```

页面选择器可在松开 Grip 后切换模式；后台状态会显示实际生效的模式。
相机通过作者原有 WebRTC 页面显示，开关沿用页面 Settings；默认不录图像。
HMD 朝向只影响观看，不影响机械臂控制映射。

按作者流程先点 Calibrate wrist → Start calibration，进入 XR 后握右 Grip
开始约 5 秒采样，保持人体腕部位置、绕多个方向转手柄。失败不覆盖之前结果。
这是腕部位置偏移校准，不是手柄局部轴到工具局部轴的方向校准。
可 Export / Import 保存偏移；换 IP 后浏览器 localStorage 是不同来源。

## 操作约定

| 操作 | 行为 |
|---|---|
| 右 Grip | 离合遥操；每次接合从当前显示状态 FK 捕获末端与腕部枢轴 |
| 手柄平移 | 固定 Quest 环境参考系 → 基座系，前 −Y、右 −X、上 +Z |
| world 旋转 | 作者的环境旋转增量，转换到基座系后左乘目标朝向 |
| tool_local 旋转 | 局部手柄增量转换到工具局部系，右乘目标朝向 |
| 右 Trigger 真按下沿 | OPEN/CLOSED 切换，松开 Grip 时仍有效 |
| 右 A | 作者的 precision 模式，保持时降低平移与旋转增益 |
| 右摇杆按下 | 作者原有的归位插值；运动学预览不经过额外加速度整形 |
| 短时追踪／输入超时 | 暂停并保持显示关节位置；恢复时重新捕获锚点，不补跑旧输入 |
| 连接／XR 空间重置 | 要求松开再按 Grip，建立新空间锚点 |
| 持续跟随误差异常 | 仅动力学实验模式：锁止，检查原因后重启 |

沿用作者默认倍率 1.5、precision 0.5、输入 EMA 0.8、目标超前限制
0.25 m / 0.6 rad，以及原解耦 DLS 参数。它们是本次比较的初始控制配置，
并非厂家参数或已经完成 AUBO 手感调优的结论。两个模式使用同一组参数。
作者的限位／可操作度 haptic 已保留。运动学预览不提供夹爪执行器力矩反馈，
页面的 Gripper force haptics 在此模式下不代表物理力。动力学模式使用仿真
执行器力矩，也不是接触力传感器或真机力反馈。

工具局部轴初始采用现有 Rx(pi) 对应：手柄 −Z → RG 工具 +Z 接近轴。
人体自然握姿是否对应这个长轴，需要你的点头、摆动、拧钥匙测试确认。

## 模型、物理与边界

沿用 RG 的 TCP、相机安装、惯量、位置执行器和重力补偿。
原生入口的 RG 起始／摇杆归位采用用户选定的“夹爪摆正、相机在上方”预览姿态。
在运行时模型中添加 FK site 并更新 ready key；不修改旧 ROS 配置或原 MJCF 文件。
AG95 通过 `--gripper ag95` 选择；外部相机与原腕部相机挂载定义保留。

作者的 mapper 生成围绕腕部参考点的末端位置＋朝向目标，再由其原 IK
将末端目标转换为腕部参考点的位置任务，前 3 轴解位置、后 3 轴解朝向。
没有另写“直接控制腕部位置”的替代算法，也没有强制前三轴绝对静止。
AUBO 非球形腕部与作者 rest bias 都会影响实际轨迹。

默认控制／预览更新 200 Hz，直接把作者 IK 的位置输出写入 `data.qpos`，
再 `mj_forward`；没有额外参考加速度整形、位置执行器跟随或物理积分。
保留原 IK 的关节范围、DLS／偏置、每步增量限制、枢轴目标和输入滤波。
RG 两指按已有 MJCF 齿轮齿条 equality 多项式显示；AG95 按描述包已有 1:1
mimic 显示，并核验四连杆闭环。夹爪 Trigger 二态仍可在松开 Grip 时操作。
FK 和相机各有自己的 MjData 副本；相机恢复完整状态快照。

速度上界沿用 `robot_spec.yaml` 的来源记录：前三轴 150°/s、后三轴 180°/s。
运行时可用 `--speed 0.5` 将 IK 每步增量上限降到其 50%；这是位置参考的
步长约束，不需要速度命令接口。作者归位插值另行执行。工作范围仍 ±2.94 rad。
作者 rest bias 在非归位姿态重新接合时可产生小幅位置调整，未将其另行改写。

动力学实验入口需显式选择：

```bash
bash scripts/run_mujoco_teleop.sh --backend physics --rotation world
```

该模式仍为 1 ms 物理步长／200 Hz 控制，写 `data.ctrl` 后 `mj_step`，
保留原新增参考整形。5 mm 小幅输入后保持的离线实验已在该整形中复现
关节命令往返振荡，尚未修复，不用于当前手感验收。`--acceleration` 只允许
用于 physics（默认 5 rad/s²），它是项目配置，不是厂家数值。
模型执行器参数未经真机辨识；真机位置伺服接入须单独验证。

RG ready 使用以下六轴角度（rad）：

```text
[-1.137355, -0.4, 1.1707963267948966, 1.5707959803847207, 1.137355, 0.0]
```

来源是现有 AUBO 模型上的 FK／朝向求解，经操作者看预览选定；不是厂家归位值。
上臂前倾、前臂水平，工具接近轴为基座 −Y，工具横轴为基座 +Z，相机在上方。
TCP 基座位置约 `[0.11438, -0.78620, 0.60029] m`；相比旧姿态升高约 16.6 cm、
沿 −Y 前伸约 16.0 cm、横向变化约 7.4 cm。相机安装保持原值。
腕 3 为零位，双向工作余量约 168.45°，没有新增 ±90° 限制。
两种模式和摇杆归位使用同一新起点。从该起点的 world／tool_local 双向
90° 拧转均通过离线输入检查，采样中模型接触数为零；不能推广为所有动作无碰撞。
该余量不是任意姿态下六维可达性的保证，新起始视角仍需头显验收。

运动学模式用于方向、旋转映射和 IK 连续性测试；物体保持初始／显示状态，
不模拟抓取、重力下落、碰撞后的动力学响应。接触统计只能作为几何信息。
没有新增在线避碰规划器，也不能当作真机防碰撞控制。作者阻尼和偏置在
已测伸直回撤场景可退出，不能推广为所有奇异位形均可完美跟踪六维目标。

归位姿态同时改变作者 IK 的前三轴 rest bias。此前将人工姿态
`[0,0,0,-0.5,0.7,0]` 的水平移动测试称为“伸直向内回撤”不准确：
该姿态的 wrist/j4 参考点约 `[0,-0.1215,0.906] m`，两段臂向上伸直，
原脚本要求基座 +Y 水平移动。其表现不能用于判定新偏置无法向内回撤。
修正为沿基座 −Z 向内移动：4 cm 手柄输入（默认 1.5 倍率）、运行 6 秒，
当前新 bias 的腕部参考点向下回撤约 6.4 cm，TCP 向下约 6.0 cm，
目标位置残差约 1.3 cm；旧 bias 同样可以回撤。原历史测试保留并明确重命名
为水平需求的物理模式表征，新运动学回归验证真正的纵向回撤。
未因此修改作者算法或偏置参数。该场景通过不保证任意奇异位形可退出或
六维精确跟踪，若实际操作出现卡住，应按可复现输入另行评估。

此前动力学模式中可复现的枢轴行为：保持前三轴 ready 角度，腕 3 从 1.8 rad 中间姿态开始，
手柄局部 −Z 转 0.2 rad（约 11.5°，1.5 倍率），约 3 秒仿真后 TCP 位移
30.55 mm，肩关节变化约 0.07454 rad（4.27°）；末端位置距生成目标约
0.083 mm，未触发故障。两种模式在这组初始轴恰好对应的动作中结果接近。
这证明执行了作者的枢轴弧线语义，不是“尖端原地／大臂绝对静止”的控制，
也不是精细接触任务的合格结论。测试在 `test_author_pivot_arc_observed_on_aubo`。

## 记录与时钟

默认 OFF，启动不会创建数据集或图像目录。

```bash
# 仅记录全量结构化信号，图像仍不落盘
bash scripts/run_mujoco_teleop.sh --record

# 明确开启图像；JPEG 序列会占空间
bash scripts/run_mujoco_teleop.sh --record --record-images

# 指定一个尚不存在的输出目录
bash scripts/run_mujoco_teleop.sh --record /data/VR_teleoperation_runtime/recordings/my-session
```

`manifest.json`：上游版本、模型、配置、夹爪、参数来源与时间域。
`signals.jsonl`：关节命令与显示状态、末端 FK、目标、离合／暂停、夹爪二态
命令与显示位置、原始 XR 帧与校准偏移、输入统计。运动学模式包含
`backend=kinematic`、`state_source=kinematic_preview`、`q_simulated`；
`q_measured`、`q_velocity`、`physics_qpos/qvel`、`gripper_measured` 和执行器力矩
为空，不能把显示状态标为动力学实测。physics 模式才填充这些物理仿真字段。
启用图像时另记录每张图的相机名、帧号及物理快照时间。

`sim_time` 在运动学模式是按控制周期累加的预览采样时钟，在 physics 模式
才是物理积分时钟；`command_apply_sim_time` 是该周期命令写入的时钟值。
PC monotonic 与 Quest t_client 保持各自的时间域，不伪造精确同步。
相对网络排队年龄由最佳观测时钟偏移估计，不等于单程绝对延迟。
这仍是原始日志，未写 LeRobot Dataset Builder；action 表达留待训练选择。

## 环境与验证

默认使用已存在的 `/data/VR_teleoperation_runtime/.venv/bin/python`，由本项目独立管理。
可用 `TELEOP_PYTHON=/path/to/python` 指定另一环境。
新环境在项目根目录执行 `python -m pip install -r requirements-mujoco.txt`。
运行不需要 LeRobot；其原接口保留为可选 facade，尚未做 LeRobot 训练集成验收。

```bash
/data/VR_teleoperation_runtime/.venv/bin/python tests/test_mujoco_teleop.py
/data/VR_teleoperation_runtime/.venv/bin/python tests/smoke_mujoco_transport.py
# 第二条创建并关闭自己的本机测试会话、使用临时记录目录，不需头显。
```

本轮运动学更新（2026-10-12）：26 项回归通过，新增真正的垂直伸直向内回撤、RG 起始／归位一致性
和工具轴双向 90° 拧转验证；同时验证默认运动学、无
`mj_step`、无新增加速度整形、两种模式直接显示 IK 位置、Grip／超时保持、
夹爪联动和 5 mm 小幅输入收敛。此前 19 项核心／动力学回归保留，其中原作者数值基准
包含 200 组工作区 mapper、100 组相同 AUBO 模型上的 IK 求解。浏览器原始
腕部拟合函数恢复已知三维偏移，并拒绝不足采样与单轴采样。旧 ROS Route B
14 项回归也通过，作为回退保持原状。

本轮默认运动学的约 10 秒 WS/WebRTC 测试：687 输入帧、1988 状态消息、
2006 控制记录；三路收到 640×480、640×480、1280×720 有效视频。
显示位置与命令差为 0（运动学显示一致性，不代表真机跟随误差）；
invalid／queued／out_of_order 均为 0，最大接收帧间隔约 29.9 ms。
临时记录目录自动删除。默认运动学的桌面 Viewer 与 LAN HTTPS/WSS
也完成启动／退出核验，Logger 默认 OFF。

此前动力学本机约 10 秒合成 Quest 输入＋真实 WS/WebRTC 测试：686 输入帧、2003 状态
消息、2017 控制记录；三路收到 640×480、640×480、1280×720 有效视频。
当时最大关节跟随误差约 0.0011 rad；输入 invalid、queued、out_of_order 均为 0，
最大接收帧间隔约 30.4 ms，无跟随误差故障。临时记录目录自动删除。
桌面 Viewer、LAN HTTPS/WSS、USB 模式的 HTTP/WS 均完成启动与退出核验。
这些是本机合成输入结果，不证明实际 Quest 无线链路的断流已经解决。

上游版本和逐文件修改范围见 `third_party/vr-teleop-kit/LOCAL_CHANGES.md`。
当前world及新姿态已获操作者手感验收。后续保留该核心作为真机适配对照；
tool_local比较、长时无线稳定性与真实Quest断流诊断不由这次验收自动覆盖。
