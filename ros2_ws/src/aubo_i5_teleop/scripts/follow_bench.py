#!/usr/bin/env python3
"""遥操跟随台架：① 工作空间多点的稳态安静度；② 连续流式目标的跟随带宽；③ 端到端延迟。

为什么需要它（前几轮验证留下的缺口）：
  之前所有验证都是"定点 + 一次 5 cm 阶跃"，而真实遥操是
    (a) 机械臂要**跑遍任务工作空间**（不同位姿的稳定性可能完全不同——惯量随位形变化）；
    (b) 人手以 ~125 Hz **连续流式**给目标，而不是一步到位的阶跃。
  阶跃+定点这两个条件都是"最容易稳"的工况，所以之前测不出问题。

三个模式：
  --survey   逐个去位姿表里的每个位姿，停稳后量：末端 3D 散布（该停住时到底停没停）、
             命令峰峰值、终态误差、Servo 状态码集合。
  --sine     在每个位姿上流式发正弦目标 A·sin(2πft)，频率扫 0.1/0.25/0.5/1.0 Hz，
             量**幅值比**与**相位滞后**——这就是操作者手感上的"跟得紧不紧"。
             发布速率 125 Hz（对齐 oculus_reader 的控制器读取率）。
  --latency  静置建噪声基线 → ±step 交替阶跃 N 次 → 量"发目标到末端动起来"的时差，
             分解为 D1 因果（仿真时刻）/ D2 观测（墙钟，扣 TF 传输）/ D3 命令侧反应。

位姿来源：`--poses <yaml>`（每项含 name/position/orientation[/condition_number]，world 帧）。
不给就用当前末端位姿当单一位姿。

⚠️ 本脚本的四条仪器约定（都踩过坑，不要改）：
  1. /joint_states 与命令话题都用 depth=1 BEST_EFFORT（默认 QoS 在 200 Hz 下会积压成
     0.2-0.5 s 的旧数据，读到的是排队中的旧样本）。
  2. **安静度用命令的"位置峰峰值"而不是速度 RMS**——位置模式下命令是位置量纲。
  3. 存活判据用**图检查**（话题有没有发布者/订户），不用"收到消息"：Servo 的 status 是
     变化才发的一次性消息，启动后才订阅会永远收不到；命令话题在静止时也一条都不发。
  4. 末端位姿**不用 TF**（2026-09-24 更正）：TF2 的 Python listener 在 197 Hz × 13 变换的
     /tf 流下积压 0.3–0.5 s 且抖动 ±0.14 s（/tf 话题本身只迟 0.003 s，/joint_states 迟
     −0.001 s，都是探针 scripts/simtime_skew_probe.py / verify_fk_vs_tf.py 实测），曾把
     sine 的相位和 latency 的 D1/D2 全部污染成伪像（幅值比 1.38、滞后 −0.48 s 这种
     物理上不可能的值）。现在末端 = **/joint_states + MuJoCo FK**（模型已与厂商 URDF
     对到机器精度、与 TF 现场对齐验证位置差 0.000000 m / 姿态差 1e-16）。
     代价：本脚本必须用带 mujoco 的解释器跑（见文件尾"运行环境"）。
     另外报实时因子 RTF（仿真时刻推进/墙钟推进）：若 ≠1，所有时间口径都要折算。

运行环境：必须用 /home/lcw/tomato_robot/.venv/bin/python 跑（rclpy + mujoco 3.12.0 都有，
且 mujoco 版本与 mujoco_ros2_control 链的 mujoco_vendor 一致）。系统 python3 的 mujoco
3.3.5 缺 typing_extensions 连 import 都过不了。

用法（先起 stage2a/2b/3；Servo 未启动时先归位）：
    ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py
    ros2 launch aubo_i5_teleop stage2b_moveit.launch.py
    python3 scripts/go_ready.py --mode position
    ros2 launch aubo_i5_teleop stage3_pose_tracking.launch.py
    python3 scripts/follow_bench.py --poses config/bench_poses.yaml --survey
    python3 scripts/follow_bench.py --poses config/bench_poses.yaml --sine
    python3 scripts/follow_bench.py --poses config/bench_poses.yaml --latency
"""

import argparse
import csv
import math
import statistics
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int8

# FK 通道（仪器约定 4）：MuJoCo 只在必要时 import，失败时给出可执行的指引而不是堆栈
try:
    import numpy as np
    import mujoco
except ModuleNotFoundError as _e:      # pragma: no cover
    raise SystemExit(
        "本台架需要 mujoco + numpy（FK 测量通道）：%s\n"
        "用 /home/lcw/tomato_robot/.venv/bin/python 跑（见文件头'运行环境'）" % _e)

# 位置模式：Servo 发位置给 forward_command_controller_position
CMD_TOPIC = "/forward_command_controller_position/commands"
STATUS_TOPIC = "/servo_pose_tracking/status"
TARGET_TOPIC = "/target_pose"
WORLD = "world"
EE_FRAME = "gripper_tip_link"
AXIS_IDX = {"x": 0, "y": 1, "z": 2}
# 旋转激励轴（世界系单位轴）。2026-09-25 补：三张表原本只激励线向 z，
# 而 angular_proportional_gain 与线向同步从 20 提到 30，角向跟踪从未建表。
ROT_AXES = {"rx": (1.0, 0.0, 0.0), "ry": (0.0, 1.0, 0.0), "rz": (0.0, 0.0, 1.0)}
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
STATUS_MEANING = {0: "无警告", 1: "奇异降速", 2: "奇异急停", 3: "碰撞降速",
                  4: "碰撞急停", 5: "关节到界", 6: "离开奇异降速", -1: "无效"}

# ── FK 模型（仪器约定 4）：与 gen_bench_poses.py / check_traj_contacts.py 同一套已验证资产 ──
MJCF_MODEL = "/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml"
# MJCF ag95_base → 夹持点（gripper_tip_link）。2026-09-24 现场对齐验证（verify_fk_vs_tf.py，
# 栈运行中，139 样本）：与 TF 的 world→gripper_tip_link 位置差中位 0.000000 m、
# 姿态差（1−|四元数点积|）中位 0.00e+00 —— 两个 frame 完全重合，可直接互换。
TIP_OFF = (-0.0405, -0.0143, 0.1492)

_mj_model = mujoco.MjModel.from_xml_path(MJCF_MODEL)
_mj_data = mujoco.MjData(_mj_model)
_mj_qadr = [_mj_model.jnt_qposadr[mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in GROUP]
_mj_dofs = [_mj_model.jnt_dofadr[mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in GROUP]
_mj_bid = mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")

# ee() 返回的旋转对象：带 .x/.y/.z/.w 属性，与 TF 的 Quaternion 同约定（x,y,z,w）
from collections import namedtuple
_Quat = namedtuple("_Quat", "x y z w")


def fast_qos():
    """depth=1 + BEST_EFFORT：只取最新样本，不积压（见文件头的仪器约定）。"""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


class Bench(Node):
    def __init__(self, csv_path: str | None = None):
        super().__init__("follow_bench")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.cmd = None
        self.cmd_t = None
        self.last_tgt = None      # 最近一次发出的目标位置（写进 CSV，用于精确切段）
        self.cmd_log = []
        self.q = {}
        self.status = None
        self.status_since = 0.0
        self.status_hist = {}
        self.status_log = []
        self.js_stamp = {}      # 仿真时刻 -> 到达时刻（rtf() 用）
        # FK 通道的最新状态：_on_js 按 GROUP 顺序填 _qvec，六个都见过才算就绪
        self._qvec = [float("nan")] * len(GROUP)
        self._js_t = None
        qos = fast_qos()
        self.create_subscription(Float64MultiArray, CMD_TOPIC, self._on_cmd, qos)
        self.create_subscription(Int8, STATUS_TOPIC, self._on_status, qos)
        self.create_subscription(JointState, "/joint_states", self._on_js, qos)
        self.pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 10)
        self.n_pub = 0
        self.pub_times = []
        self.csv = None
        self.writer = None
        if csv_path:
            self.csv = open(csv_path, "w", newline="")
            self.writer = csv.writer(self.csv)
            self.writer.writerow(["sim_t", "wall_t", "ee_x", "ee_y", "ee_z"]
                                 + ["cmd%d" % i for i in range(6)] + ["status"]
                                 + ["q%d" % i for i in range(6)]
                                 + ["qd%d" % i for i in range(6)]
                                 + ["tgt_x", "tgt_y", "tgt_z"])

    # ---------- 回调 ----------
    def _on_cmd(self, msg):
        self.cmd = list(msg.data)
        self.cmd_t = time.time()
        self.cmd_log.append((self.cmd_t, list(msg.data)))

    def _on_status(self, msg):
        new = int(msg.data)
        if new != self.status:
            self.status_since = time.time()
        self.status = new
        self.status_hist[self.status] = self.status_hist.get(self.status, 0) + 1
        self.status_log.append((time.time(), new))

    def _on_js(self, msg):
        now = time.time()
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.js_stamp[round(st, 4)] = now
        self._js_t = st
        for i, name in enumerate(msg.name):
            if name in GROUP:
                self.q[name] = (msg.position[i], msg.velocity[i])
                self._qvec[GROUP.index(name)] = msg.position[i]

    # ---------- 基础工具 ----------
    def spin(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.001)

    def sim_now(self):
        """仿真时刻（秒）。用于把正弦的相位算在**数据产生时刻**上。"""
        return self.get_clock().now().nanoseconds * 1e-9

    def ee(self, log=False):
        """返回 (translation, rotation, sim_stamp)；六个关节没齐就返回 None。

        ⚠️ 2026-09-24 起这是 **FK 通道**（/joint_states + MuJoCo FK），不再走 TF：
        TF2 的 Python listener 在这个栈上积压 0.3–0.5 s（仪器约定 4）。
        sim_stamp 取 /joint_states 的时间戳（探针实测比仿真时钟只迟 −0.001 s），
        所以相位/延迟的时基是准的。rotation 返回带 .x/.y/.z/.w 属性的对象，
        与原 TF 的 Quaternion 同约定（现场对齐验证姿态差 1e-16）。
        """
        q = self._qvec
        if any(not math.isfinite(x) for x in q):
            return None
        for a, v in zip(_mj_qadr, q):
            _mj_data.qpos[a] = v
        mujoco.mj_forward(_mj_model, _mj_data)
        R = _mj_data.xmat[_mj_bid].reshape(3, 3).copy()
        p = _mj_data.xpos[_mj_bid] + R @ np.array(TIP_OFF)
        qw = np.zeros(4)
        mujoco.mju_mat2Quat(qw, np.ascontiguousarray(R).flatten())
        rot = _Quat(x=qw[1], y=qw[2], z=qw[3], w=qw[0])
        st = self._js_t
        if log and self.writer:
            self._log_row(st if st is not None else time.time(), tuple(p))
        return (tuple(float(x) for x in p), rot, st)

    def _log_row(self, sim_t, ee_pos):
        cmd = self.cmd or [float("nan")] * 6
        q = [self.q.get(j, (float("nan"),))[0] for j in GROUP]
        qd = [self.q.get(j, (float("nan"), float("nan")))[1] for j in GROUP]
        # ⚠️ 记录**当前目标**（2026-09-24 加）：没有它就只能靠猜"哪一段是正弦段"
        #    （曾用滚动标准差切窗，结果把 go_to_pose 的爬行段当成了振荡段，
        #    得出"幅值 133 mm / 目标 15 mm"这种假结论）。有了目标列，
        #    任何段都能精确对齐，也能反算真实的跟随误差。
        self.writer.writerow([sim_t, time.time(), ee_pos[0], ee_pos[1], ee_pos[2]]
                             + list(cmd) + [self.status if self.status is not None else -99]
                             + q + qd
                             + list(self.last_tgt if self.last_tgt else (float("nan"),) * 3))

    def publish_target(self, tgt):
        tgt.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(tgt)
        self.last_tgt = (tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z)
        self.n_pub += 1
        self.pub_times.append(time.time())

    def rtf(self):
        """实时因子：仿真时刻推进 / 墙钟推进（用 /joint_states 的时间戳与到达时刻配对）。"""
        pairs = sorted((st, w) for st, w in self.js_stamp.items())
        if len(pairs) < 10:
            return None
        (s0, w0), (s1, w1) = pairs[0], pairs[-1]
        return (s1 - s0) / max(1e-9, (w1 - w0))


def make_target(base_pos, base_rot, offset):
    tgt = PoseStamped()
    tgt.header.frame_id = WORLD
    tgt.pose.position.x = base_pos[0] + offset[0]
    tgt.pose.position.y = base_pos[1] + offset[1]
    tgt.pose.position.z = base_pos[2] + offset[2]
    # base_rot 可能是 geometry_msgs Quaternion，也可能是 ee() 的 _Quat（FK 通道）——
    # 统一重建成真正的 Quaternion 消息，别直接赋值
    tgt.pose.orientation = Quaternion(x=base_rot.x, y=base_rot.y, z=base_rot.z, w=base_rot.w)
    return tgt


def make_target_from_pose(pose):
    """位姿表里的一项 -> PoseStamped（world 帧）。"""
    tgt = PoseStamped()
    tgt.header.frame_id = WORLD
    p, o = pose["position"], pose["orientation"]
    tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z = float(p[0]), float(p[1]), float(p[2])
    tgt.pose.orientation.x = float(o[0])
    tgt.pose.orientation.y = float(o[1])
    tgt.pose.orientation.z = float(o[2])
    tgt.pose.orientation.w = float(o[3])
    return tgt


def measure_window(node, tgt, seconds, rate_hz=125.0, log=True):
    """持续发目标 tgt 共 seconds 秒，返回窗口内 (末端样本, 命令样本)。"""
    ee_samples, cmds = [], []
    t0 = time.time()
    period = 1.0 / rate_hz
    nxt = t0
    while True:
        if time.time() - t0 >= seconds:
            break
        node.publish_target(tgt)
        rclpy.spin_once(node, timeout_sec=0.0)
        e = node.ee(log=log)
        if e:
            ee_samples.append((e[2], e[0]))          # (仿真时刻, 位置)
        if node.cmd and all(math.isfinite(x) for x in node.cmd):
            cmds.append(list(node.cmd))              # 起点未收命令时是 NaN，别混进统计
        nxt += period
        dt = nxt - time.time()
        if dt > 0:
            time.sleep(dt)
    return ee_samples, cmds


def _slerp(qa, qb, t):
    """四元数球面插值（qa/qb 为 (x,y,z,w)）。"""
    import math as _m
    dot = sum(a * b for a, b in zip(qa, qb))
    if dot < 0:
        qb = tuple(-b for b in qb)
        dot = -dot
    if dot > 0.9995:
        out = [a + t * (b - a) for a, b in zip(qa, qb)]
    else:
        th = _m.acos(max(-1.0, min(1.0, dot)))
        s = _m.sin(th)
        w1, w2 = _m.sin((1 - t) * th) / s, _m.sin(t * th) / s
        out = [w1 * a + w2 * b for a, b in zip(qa, qb)]
    n = _m.sqrt(sum(x * x for x in out)) or 1.0
    return tuple(x / n for x in out)


def _ee_q(e):
    return (e[1].x, e[1].y, e[1].z, e[1].w)


def _step_toward(node, from_p, from_q, to_p, to_q, step_m, dwell, log):
    """从当前位姿朝 (to_p, to_q) 走一步（笛卡尔直线 + 姿态 slerp）。

    ⚠️ 2026-09-24 改为"**走到位再走下一步**"：持续发这个中间目标，直到末端进入
    via 点的容差内（或 dwell 秒上限到）。原来的实现固定发 0.3 s 就发下一步，
    臂跟不上时（实测爬行只有 5–7 mm/s，每步要 ~0.75 s）命令会一步步领跑，
    |cmd−q| 积到 0.36 rad；换目标的瞬间臂猛收，直接折进自碰撞。
    每步收拢滞后到 1.5 mm 才继续，从源头掐掉领跑。
    """
    d = math.dist(from_p, to_p)
    alpha = min(1.0, step_m / d) if d > 1e-9 else 1.0
    # 姿态先给单位四元数占位（马上覆盖成 slerp 结果）
    mid = make_target(tuple(from_p), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                      tuple(alpha * (to_p[i] - from_p[i]) for i in range(3)))
    q = _slerp(from_q, to_q, alpha)
    mid.pose.orientation.x, mid.pose.orientation.y, mid.pose.orientation.z, mid.pose.orientation.w = q
    via_p = tuple(from_p[i] + alpha * (to_p[i] - from_p[i]) for i in range(3))
    tol = max(0.0015, 0.3 * step_m)
    t0 = time.time()
    while time.time() - t0 < dwell:
        node.publish_target(mid)
        rclpy.spin_once(node, timeout_sec=0.0)
        e = node.ee(log=log)
        if e and math.dist(e[0], via_p) < tol:
            break
        time.sleep(0.008)
    return via_p, q


def _stuck(node, hard_secs=0.3, soft_secs=1.2):
    """Servo 是否已经把机械臂按住不动了。

    ⚠️ 2026-09-24 修正：原来只认 2/4（奇异急停/碰撞急停），而实际卡死全程停在
    **3（碰撞降速）**上、一次都没跳到 4——退让逻辑整轮没触发过（grep 计数 0）。
    分两档：硬停（2 奇异急停 / 4 碰撞急停）0.3 s 就算；软降速（3 碰撞降速 /
    5 关节到界）持续 1.2 s 也算（给正常路过留余地，正常爬行段全程是 0）。
    """
    s = node.status
    if s in (2, 4):
        return (time.time() - node.status_since) > hard_secs
    if s in (3, 5):
        return (time.time() - node.status_since) > soft_secs
    return False


def go_to_pose(node, tgt, tol_m=0.002, timeout_s=45.0, log=True,
               step_m=0.005, lift_m=0.04, max_attempts=3):
    """走到目标位姿；**卡住就退让、换策略重试**。

    ⚠️ 2026-09-24 两个修正（依据 /tmp/bench_survey.csv 的 314 s 完整轨迹回放）：
    · **超时按距离缩放**：实测爬行速度只有 5–7 mm/s（原来是 45 s 固定超时），
      0.5 m 的移动需要 ~80–100 s——P02/P05 的"未到位、无警告"就是在半路被掐断的，
      而且掐断换目标的瞬间 |cmd−q| 正积到 0.36 rad，臂猛收折进自碰撞（两次暴走
      都发生在 45 s 超时触发后的目标切换）。现在按 3.5 mm/s 的保守下限折算超时。
    · **退让退到"本次移动的起点"**：原来退到 last_good（上一条 5 mm 中间点），
      等于只后退 5 mm，根本解不开自碰撞；退到起点再换策略（抬高 lift_m）重试。
      退让后加 1.5 s 冷却，避免状态 3 未及清除就立刻再次判卡、把重试次数烧光。

    （文档更正：早先写的"停在奇异位形 cond≈379"是错的——按真实轨迹回算，
      全程 cond ≤ 130、奇异急停 0 次；卡死是真自碰撞 AG95底座↔upperArm。）

    返回 (实际位置, 终态误差)；走不到就返回最后一次的位置与误差，由调用方判失败。
    """
    goal_p = (tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z)
    goal_q = (tgt.pose.orientation.x, tgt.pose.orientation.y,
              tgt.pose.orientation.z, tgt.pose.orientation.w)
    t0 = time.time()
    attempt = 0
    start_p = start_q = None
    timeout_eff = timeout_s
    resume_at = 0.0

    while time.time() - t0 < timeout_eff:
        e = node.ee()
        if e is None:
            node.spin(0.2)
            continue
        cur_p, cur_q = e[0], _ee_q(e)
        if start_p is None:
            start_p, start_q = cur_p, cur_q
            # 按实测爬行下限 3.5 mm/s 折算预算（另加 30 s 起步/收尾余量）
            timeout_eff = max(timeout_s, 30.0 + math.dist(start_p, goal_p) / 0.0035)
        if math.dist(cur_p, goal_p) < tol_m:
            measure_window(node, tgt, 0.6, log=log)
            e2 = node.ee()
            if e2 and math.dist(e2[0], goal_p) < tol_m * 2:
                return e2[0], math.dist(e2[0], goal_p)
            continue

        # ── 卡住 → 退回本次移动的起点，换策略 ──
        if time.time() >= resume_at and _stuck(node):
            attempt += 1
            if attempt > max_attempts or start_p is None:
                break
            node.get_logger().warn("检测到卡住（状态 %s），退回到移动起点重试（第 %d 次）"
                                   % (node.status, attempt))
            for _ in range(400):                      # 退回（分步，别跳变）
                e = node.ee()
                if e is None:
                    break
                if math.dist(e[0], start_p) < tol_m:
                    break
                _step_toward(node, e[0], _ee_q(e), start_p, start_q, step_m, 0.20, log)
            node.status_hist.clear()
            resume_at = time.time() + 1.5             # 给状态 3 消退留时间
            continue

        # ── 正常推进：第 1 次直接走；之后先抬到目标上方 lift_m 再走 ──
        via_p, via_q = goal_p, goal_q
        if attempt >= 1:
            via_p = (goal_p[0], goal_p[1], goal_p[2] + lift_m)
        d = math.dist(cur_p, via_p)
        if d < step_m:
            via_p, via_q = goal_p, goal_q
        _step_toward(node, cur_p, cur_q, via_p, via_q, step_m, 0.30, log)

    e = node.ee()
    if e is None:
        return None
    return e[0], math.dist(e[0], goal_p)


# ══════════════════════════ 模式①：停稳安静度 ══════════════════════════
def survey(node, poses, args):
    print("%-22s %10s %10s %11s %9s %8s  %s"
          % ("位姿", "末端p2p3D", "命令p2p", "终态误差", "cond", "判定", "status"))
    print("-" * 106)
    bad = []
    for ps in poses:
        tgt = make_target_from_pose(ps)
        got = go_to_pose(node, tgt, log=True)
        if got is None:
            print("%-22s ❌ 走不到该位姿" % ps["name"])
            bad.append(ps["name"])
            continue
        node.status_hist.clear()
        ees, cmds = measure_window(node, tgt, args.settle, log=True)
        tail = [(t, p) for (t, p) in ees if t >= (ees[-1][0] - 1.2)] if ees else []
        if len(tail) < 5:
            print("%-22s ❌ 末端样本不足" % ps["name"])
            continue
        pts = [p for _, p in tail]
        c = [sum(x[i] for x in pts) / len(pts) for i in range(3)]
        p2p3d = max(math.dist(p, c) for p in pts)                # 3D 散布，不是单轴
        # ⚠️ 过滤掉非有限值：起点还没收到命令时 _log_row 写的是 NaN，
        #    直接 max() 会让整列变成 nan（实测 |cmd−q| 峰值报 nan）。
        cmd_tail = [cv for cv in (cmds[-150:] if len(cmds) > 150 else cmds)
                    if all(math.isfinite(x) for x in cv)]
        cmd_p2p = max((max(cv[j] for cv in cmd_tail) - min(cv[j] for cv in cmd_tail))
                      for j in range(6)) if cmd_tail else float("nan")
        err = math.dist(pts[-1], (tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z))
        if err > 0.002:
            verdict = "未到位"
        elif p2p3d < 0.002 and cmd_p2p < 0.001:
            verdict = "静止"
        else:
            verdict = "**抖**"
        if verdict != "静止":
            bad.append(ps["name"])
        st = sorted(node.status_hist.keys()) if node.status_hist else []
        print("%-22s %10.4f %10.5f %11.4f %9.1f %8s  %s"
              % (ps["name"], p2p3d, cmd_p2p, err,
                 ps.get("condition_number", float("nan")), verdict,
                 [STATUS_MEANING.get(k, k) for k in st]))
    print()
    if bad:
        print("⚠️  需要复查的位姿：%s" % bad)
    else:
        print("✅ 所有位姿停稳后都安静（末端 3D 散布 <2 mm、命令峰峰 <1 mrad、终态误差 <2 mm）")
    return 0


# ══════════════════════════ 模式②：跟随带宽 ══════════════════════════
def _fit(ts, ys, f):
    """相关法拟合基频分量，返回 (幅值, 相位)。"""
    w = 2 * math.pi * f
    sc = sum(y * math.sin(w * t) for t, y in zip(ts, ys))
    cc = sum(y * math.cos(w * t) for t, y in zip(ts, ys))
    return 2.0 / len(ts) * math.hypot(sc, cc), math.atan2(cc, sc)


def _quat_mul(a, b):
    """(x,y,z,w) 约定四元数乘法（与 geometry_msgs / 本文件的 FK 输出同约定）。

    ⚠️ 不用 mujoco.mju_* 的等价函数：它的约定是 [w,x,y,z]，2026-09-24 在
    gen_bench_poses 上已经因此把近恒等旋转读成 180° 一次。这里全程 (x,y,z,w)。
    """
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _quat_axis_angle(axis, ang):
    """绕单位轴转 ang 弧度 -> (x,y,z,w)。"""
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    s = math.sin(ang / 2.0)
    return (axis[0] / n * s, axis[1] / n * s, axis[2] / n * s, math.cos(ang / 2.0))


def _quat_to_rotvec(q):
    """(x,y,z,w) -> 旋转向量（轴×角）。"""
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    x, y, z, w = (c / n for c in q)
    w = max(-1.0, min(1.0, w))
    ang = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return (0.0, 0.0, 0.0)
    return (x / s * ang, y / s * ang, z / s * ang)


def _angle_about_axis(q_ee, q_base, axis):
    """末端相对基准的旋转量里，**绕世界系 axis 的分量**（rad）。"""
    qrel = _quat_mul((q_ee.x, q_ee.y, q_ee.z, q_ee.w),
                     _quat_conj((q_base.x, q_base.y, q_base.z, q_base.w)))
    v = _quat_to_rotvec(qrel)
    return v[0] * axis[0] + v[1] * axis[1] + v[2] * axis[2]


def sine(node, poses, args):
    rot = args.axis in ROT_AXES
    if rot:
        axis = ROT_AXES[args.axis]
        amp = math.radians(args.amp_deg)
    else:
        i = AXIS_IDX[args.axis]
        amp = args.amp
    for ps in poses:
        tgt = make_target_from_pose(ps)
        got = go_to_pose(node, tgt, log=True)
        if got is None:
            print("%s：❌ 走不到该位姿，跳过" % ps["name"])
            continue
        base_p, base_r = got[0], tgt.pose.orientation
        if rot:
            print("── %s ──  基准末端 (%.4f, %.4f, %.4f)，绕**世界** %s 轴正弦 A=%.1f°"
                  % (ps["name"], base_p[0], base_p[1], base_p[2], args.axis, args.amp_deg))
        else:
            print("── %s ──  基准末端 (%.4f, %.4f, %.4f)，%s 轴正弦 A=%.1f mm"
                  % (ps["name"], base_p[0], base_p[1], base_p[2], args.axis, args.amp * 1000))
        for f in args.freqs:
            ee_samples, cmds = [], []
            # 相位用**仿真时刻**算（数据产生时刻），不用墙钟。
            # 历史注：TF 通道时代这里积压 0.288 s；2026-09-24 换 FK 通道后，
            # 末端位姿的时间戳就是 /joint_states 的时间戳（比仿真时钟迟 −0.001 s），
            # 相位口径天然是准的。
            t0 = time.time()
            t0_sim = node.sim_now()
            period = 1.0 / 125.0
            nxt = t0
            while True:
                el_wall = time.time() - t0
                if el_wall >= args.cycles / f:
                    break
                el = node.sim_now() - t0_sim          # 仿真时刻（秒）
                if rot:
                    # 目标姿态 = 绕世界轴的增量**前乘**基准姿态（世界系旋转，
                    # 等价于操作者绕固定世界轴摆手；工具的等价做法是后乘，
                    # 二者在小子样下近似、大角度不同，这里明确选世界系并记录）。
                    dq = _quat_axis_angle(axis, amp * math.sin(2 * math.pi * f * el))
                    o = _quat_mul(dq, (base_r.x, base_r.y, base_r.z, base_r.w))
                    mid = make_target(base_p, base_r, (0.0, 0.0, 0.0))
                    mid.pose.orientation.x, mid.pose.orientation.y = o[0], o[1]
                    mid.pose.orientation.z, mid.pose.orientation.w = o[2], o[3]
                    node.publish_target(mid)
                else:
                    off = [0.0, 0.0, 0.0]
                    off[i] = amp * math.sin(2 * math.pi * f * el)
                    node.publish_target(make_target(base_p, base_r, tuple(off)))
                rclpy.spin_once(node, timeout_sec=0.0)
                e = node.ee(log=True)
                if e:
                    # 用关节帧的仿真时间戳（FK 通道；见仪器约定 4）
                    ee_samples.append((e[2] - t0_sim,
                                       _angle_about_axis(e[1], base_r, axis) if rot
                                       else e[0][i]))
                if node.cmd and all(math.isfinite(x) for x in node.cmd):
                    cmds.append(max(abs(x) for x in node.cmd))   # 滤掉起点未收命令时的 NaN
                nxt += period
                dt = nxt - time.time()
                if dt > 0:
                    time.sleep(dt)
            # 丢弃第 1 个周期（去暂态）
            sel = [(t, y) for (t, y) in ee_samples if t >= 1.0 / f]
            if len(sel) < 20:
                print("  f=%.2f Hz  样本不足，跳过" % f)
                continue
            ts = [t for t, _ in sel]
            ys = [y for _, y in sel]
            # ⚠️ 参考基准必须加**该位姿的直流偏置**：ys 是末端的绝对坐标（z≈0.18 m），
            #    只写 A·sin 会把 EE 的位置当成误差（曾报"误差RMS 0.18 m"）。
            #    幅值比与相位用的是相关法拟合（会滤掉直流），所以那两列本来就没受影响。
            #    旋转轴不需要偏置：_angle_about_axis 是**相对基准**的量，基准处恒为 0。
            if rot:
                ref_ys = [amp * math.sin(2 * math.pi * f * t) for t in ts]
                unit, scale = "°", math.degrees
            else:
                ref_ys = [base_p[i] + amp * math.sin(2 * math.pi * f * t) for t in ts]
                unit, scale = "m", lambda v: v
            a_in, p_in = _fit(ts, ref_ys, f)
            a_out, p_out = _fit(ts, ys, f)
            dphi = p_in - p_out
            while dphi > math.pi:
                dphi -= 2 * math.pi
            while dphi < -math.pi:
                dphi += 2 * math.pi
            err_rms = math.sqrt(sum((y - r) ** 2 for y, r in zip(ys, ref_ys)) / len(ys))
            print("  f=%.2f Hz | 幅值比 %.2f | 滞后 %+6.1f° = %+6.3f s | 误差RMS %.4f %s | "
                  "命令峰 %.4f | 末端 %.0f Hz"
                  % (f, a_out / abs(a_in) if a_in else float("nan"), math.degrees(dphi),
                     dphi / (2 * math.pi * f), scale(err_rms), unit,
                     max(cmds) if cmds else float("nan"),
                     len(ys) / max(1e-9, ts[-1] - ts[0])))
        print()
    rtf = node.rtf()
    print("仪器说明：末端位姿 = /joint_states + MuJoCo FK（不经 TF，时间戳与仿真时钟差 ≈ −0.001 s）"
          " | 实时因子 RTF %s"
          % ("%.3f" % rtf if rtf is not None else "未能估计"))
    print("           相位滞后**不再含** TF 传输延迟（TF 通道已于 2026-09-24 弃用，见文件头约定 4）；"
          "RTF≠1 时时间口径要折算。")
    return 0


# ══════════════════════════ 模式③：端到端延迟 ══════════════════════════
def latency(node, poses, args):
    print("%-22s %10s %10s %10s %10s %9s  %s"
          % ("位姿", "D1因果", "D2观测", "D3命令侧", "噪声σ", "有效", "status"))
    print("-" * 100)
    for ps in poses:
        tgt = make_target_from_pose(ps)
        got = go_to_pose(node, tgt, log=True)
        if got is None:
            print("%-22s ❌ 走不到该位姿" % ps["name"])
            continue
        base_p, base_r = got[0], tgt.pose.orientation
        node.status_hist.clear()
        ees, _ = measure_window(node, tgt, 1.5, log=True)
        base = [p for _, p in ees]
        if len(base) < 20:
            print("%-22s ❌ 基线样本不足" % ps["name"])
            continue
        c = [sum(p[i] for p in base) / len(base) for i in range(3)]
        sigma = statistics.pstdev([math.dist(p, c) for p in base])
        thr = max(3 * sigma, 0.0002)
        d1s, d2s, d3s, d2ps = [], [], [], []
        for k in range(args.reps):
            measure_window(node, tgt, 0.6, log=True)          # 回到基准位姿
            e0 = node.ee()
            if e0 is None:
                continue
            off = [0.0, 0.0, 0.0]
            off[AXIS_IDX[args.axis]] = args.step * (1 if k % 2 == 0 else -1)
            step_tgt = make_target(base_p, base_r, tuple(off))
            wall0 = time.time()
            sim_pub_js = node._js_t        # 发布前最后收到的 js 时间戳（单一时基起点）
            n_cmd0 = len(node.cmd_log)
            cmd_ref = node.cmd_log[n_cmd0 - 1][1] if n_cmd0 else None
            node.publish_target(step_tgt)
            onset_sim = onset_wall = None
            t0 = time.time()
            while time.time() - t0 < 1.2:
                rclpy.spin_once(node, timeout_sec=0.0)
                node.publish_target(step_tgt)
                e = node.ee(log=True)
                if e and math.dist(e[0], e0[0]) > thr:
                    onset_sim, onset_wall = e[2], time.time()
                    break
                time.sleep(0.001)
            cmd_onset = None
            if cmd_ref is not None:
                for (t, cvec) in node.cmd_log[n_cmd0:]:
                    if math.dist(cvec, cmd_ref) > 1e-4:
                        cmd_onset = t
                        break
            if onset_sim is not None:
                # D1 用**单一时基**（两端都是 js 时间戳），不用发布时的时钟读数——
                # 2026-09-24 实测（D2' 仲裁列）：高频循环下时钟读数偏旧 ~28 ms，
                # 用它当起点会把 D1 整体抬高 28 ms（60 ms vs 真值 ~32 ms）。
                if sim_pub_js is not None:
                    d1s.append(onset_sim - sim_pub_js)
                d2s.append(onset_wall - wall0)
                # D2'（第三个口径）：运动帧的**到达墙钟** − 发布墙钟。js_stamp 记录了
                # 每个时间戳的到达时刻，所以这个口径既不用时钟读数（D1 的旧嫌疑），
                # 也不含检测循环的处理延迟（D2 的嫌疑）。
                w_arr = node.js_stamp.get(round(onset_sim, 4))
                if w_arr is not None:
                    d2ps.append(w_arr - wall0)
            if cmd_onset is not None:
                d3s.append(cmd_onset - wall0)
        st = sorted(node.status_hist.keys()) if node.status_hist else []
        def med(v):
            return "%.3f s" % statistics.median(v) if v else "n/a"
        print("%-22s %10s %10s %10s %10s %10.5f %7s  %s"
              % (ps["name"], med(d1s), med(d2s), med(d2ps),
                 med(d3s), sigma, "%d/%d" % (len(d1s), args.reps),
                 [STATUS_MEANING.get(k, k) for k in st]))
    print()
    print("D1 时间戳口径（运动帧 js 时间戳 − 发布前最后一个 js 时间戳，**单一时基**，"
          "比 D2 多出起点帧年龄与状态戳约定 ≈ +8 ms）")
    print("D2 检测口径（墙钟，含检测循环延迟）| D2' 到达口径（运动帧到达墙钟 − 发布墙钟，两嫌疑都不沾）")
    print("D2 与 D2' 实测一致（2026-09-24，差 <2 ms）→ **端到端真值 ≈ 32–36 ms**。")
    print("D3 命令侧（Servo 内部反应）。若 status 出现奇异/碰撞缩放，该次测的是缩放后的响应。")
    # 历史（2026-09-24，TF 通道时代）：D1/D2 曾报 −0.224/−0.480 s 的负值——TF2 Python
    #   listener 积压 0.3–0.5 s 把末端帧的时间戳回拨，D1 整体偏负；D2 又被扣了随负载变的
    #   transport_lag，双重污染。换成 FK 通道（末端 = /joint_states + MJCF FK）后两个
    #   时基都是准的（探针 verify_fk_vs_tf.py / simtime_skew_probe.py），D1/D2 才第一次
    #   成为有意义的测量。若再见到系统性负值，先查时基再谈控制。
    return 0


# ══════════════════════════ 入口 ══════════════════════════
def load_poses(path):
    if not path:
        return None
    data = yaml.safe_load(Path(path).read_text())
    poses = data["poses"] if isinstance(data, dict) else data
    for p in poses:
        assert "name" in p and "position" in p and "orientation" in p, p
    return poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--survey", action="store_true")
    ap.add_argument("--sine", action="store_true")
    ap.add_argument("--latency", action="store_true")
    ap.add_argument("--poses", default=None, help="位姿表 YAML；不给就用当前末端位姿")
    ap.add_argument("--limit", type=int, default=0,
                    help="只用位姿表的前 N 个（0=全部）。sine/latency 用 3 个代表位姿即可，否则 8 个位姿 × 4 频率会跑很久")
    ap.add_argument("--axis", default="z", choices=list(AXIS_IDX) + list(ROT_AXES),
                    help="sine 激励轴：x/y/z = 线向 [m]；rx/ry/rz = 绕**世界**轴的旋转 [°]")
    ap.add_argument("--amp", type=float, default=0.015, help="线向正弦幅值 [m]")
    ap.add_argument("--amp-deg", type=float, default=10.0,
                    help="旋转正弦幅值 [°]（--axis 为 rx/ry/rz 时生效）")
    ap.add_argument("--step", type=float, default=0.005, help="延迟模式阶跃 [m]")
    ap.add_argument("--reps", type=int, default=10, help="延迟模式重复次数")
    ap.add_argument("--cycles", type=float, default=4.0, help="每个频率几个周期")
    ap.add_argument("--freqs", type=float, nargs="+", default=[0.1, 0.25, 0.5, 1.0])
    ap.add_argument("--settle", type=float, default=2.5, help="survey：到位后再测多久 [s]")
    ap.add_argument("--csv", default="/tmp/follow_bench_trace.csv",
                    help="原始轨迹落盘（空字符串 = 不落盘）")
    args = ap.parse_args()
    if not (args.survey or args.sine or args.latency):
        print("选一个模式：--survey / --sine / --latency")
        return 1
    if args.amp > 0.03:
        print("❌ amp 太大（>3 cm），先用小幅度做线性测量")
        return 1
    if args.amp_deg > 25.0:
        print("❌ amp-deg 太大（>25°），先用小幅度做线性测量")
        return 1

    rclpy.init()
    node = Bench(args.csv or None)
    try:
        node.spin(2.0)
        # 存活判据：图检查（见文件头仪器约定 3）
        if node.count_publishers(CMD_TOPIC) == 0:
            print("❌ %s 上没有发布者 —— Servo/pose tracking 起了吗？" % CMD_TOPIC)
            return 1
        if node.count_subscribers(CMD_TOPIC) == 0:
            print("❌ %s 上没有订户 —— forward_command_controller_position 起了吗？" % CMD_TOPIC)
            return 1
        e = node.ee()
        if e is None:
            print("❌ 六个关节还没从 /joint_states 到齐 —— stage2a（controller_manager/joint_state_broadcaster）起了吗？")
            return 1
        # 功能预检：发 1 s"保持当前位姿"，要求真的收到命令
        n_before = len(node.cmd_log)
        measure_window(node, make_target(e[0], e[1], (0, 0, 0)), 1.0, log=False)
        if len(node.cmd_log) == n_before:
            print("❌ 发了保持目标但收不到任何命令 —— 链路没通")
            return 1
        print("预检通过：命令话题有发布者与订户，发目标后能收到命令\n")

        poses = load_poses(args.poses)
        if poses is None:
            e = node.ee()
            poses = [{"name": "当前位姿", "position": list(e[0]),
                      "orientation": [e[1].x, e[1].y, e[1].z, e[1].w]}]
            print("未给 --poses，用当前末端位姿作为单一位姿\n")
        else:
            if args.limit and len(poses) > args.limit:
                poses = poses[:args.limit]
                print("位姿表：取前 %d 个（--limit）\n" % len(poses))
            else:
                print("位姿表：%d 个\n" % len(poses))

        rc = 0
        if args.survey:
            print("════════ 模式①：停稳安静度 ════════")
            rc = survey(node, poses, args) or rc
        if args.sine:
            print("════════ 模式②：跟随带宽 ════════")
            rc = sine(node, poses, args) or rc
        if args.latency:
            print("════════ 模式③：端到端延迟 ════════")
            rc = latency(node, poses, args) or rc
        if node.csv:
            node.csv.close()
            print("\n原始轨迹已落盘：%s" % args.csv)
        return rc
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
