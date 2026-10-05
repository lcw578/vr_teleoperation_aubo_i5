#!/home/lcw/tomato_robot/.venv/bin/python
# -*- coding: utf-8 -*-
# 便捷入口：`bash 本文件` 也能跑——sh 把下一行解析为 source ROS 后 exec 进 venv
# Python；Python 把它解析为无操作字符串。（direction_check.py 同款）
''''exec /bin/bash -c ". /opt/ros/humble/setup.bash 2>/dev/null || :; exec /home/lcw/tomato_robot/.venv/bin/python -- \"\$0\" \"\$@\"" "$0" "$@" # '''
"""手感基线探针（P0）：把"仿真手感不太行"翻译成数字的测量脚本。

出处与边界（2026-10-01）：
  本脚本为**自写测量仪器**（无上游），是 feel-smoothness 路线图 P0 的执行件。
  它只订阅不发布——与遥操链（clutch_mapper/tracker 均发 /target_pose）并行安全，
  也不碰 follow_bench 的互斥约束（那是发布侧的事）。
  所有阈值（分段门限、相关窗、FFT 参数）都是**缺省值**，不是标定结论。

测什么（对应已知的四个手感嫌疑）：
  1. 端到端滞后    自由操作段：手速 vs 臂端速度的互相关峰值滞后（手→目标→臂 分解）
  2. 静息抖动传导  静止段：目标/关节速度 RMS + 频谱峰（8-12Hz 手抖 / 25Hz 拍频）
  3. 小信号死区    微旋转/微平移段：手动 X° → 目标 X°×0.5 → 臂实际动多少（有效增益+死区）
  4. 接合跳变      全程自动：每次按下 Grip 后首条目标 vs 接合前末条目标的跳变量

用法（顺序）：
  1) 终端1：bash scripts/run_relay.sh            （若未在跑）
  2) 终端2：bash scripts/authoritative_session.sh（若未在跑）
  3) 头显进页面 → Start Teleop（进入页面身体面朝 -y，见 TELEOP_BASELINE §2）
  4) 终端3：bash scripts/feel_baseline_probe.py
  5) 按终端提示逐段操作（全程戴头显）；报告存 /tmp/feel_baseline_report.txt
  只读观察：任何一步都可以 Ctrl-C 退出，不影响遥操链。
"""
import json
import math
import sys
import threading
import time
from collections import deque

import numpy as np

try:
    import mujoco
except ModuleNotFoundError as _e:      # pragma: no cover
    raise SystemExit("需要 mujoco（FK）：%s\n用 /home/lcw/tomato_robot/.venv/bin/python 跑" % _e)

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy

MJCF = "/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml"
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
TIP_OFF = np.array([-0.0405, -0.0143, 0.1492])   # ag95_base → 夹持点（与 mapper 一致）

# ---- 可调缺省值（非标定结论）----
REST_SECONDS = 30.0        # 静息段时长
FREE_SECONDS = 60.0        # 自由操作段时长
REPS_PER_ROUND = 5         # 每轮微动作次数
ANG_SEG_START = 0.05       # rad/s：微旋转段起点门限
ANG_SEG_END = 0.02         # rad/s：微旋转段终点门限
POS_SEG_START = 0.010      # m/s
POS_SEG_END = 0.004        # m/s
SEG_QUIET = 0.6            # s：速度低于终点门限多久算段结束
CORR_GRID_HZ = 100         # 互相关重采样率
CORR_MAX_LAG = 0.40        # s：滞后扫描上限
VENDOR_VMAX = np.array([2.618, 2.618, 2.618, 3.142, 3.142, 3.142])
REPORT_PATH = "/tmp/feel_baseline_report.txt"
JSON_PATH = "/tmp/feel_baseline_report.json"

_mj_model = mujoco.MjModel.from_xml_path(MJCF)
_mj_data = mujoco.MjData(_mj_model)
_qadr = [_mj_model.jnt_qposadr[mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_JOINT, j)]
         for j in GROUP]
_bid = mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


# ---------- 四元数助手（内部统一 [w,x,y,z]） ----------

def xyzw_to_wxyz(q):
    return np.array([q[3], q[0], q[1], q[2]], float)


def q_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_rotvec(q):
    """[w,x,y,z] → 旋转向量（最短路径）。"""
    q = q / max(np.linalg.norm(q), 1e-12)
    if q[0] < 0:
        q = -q
    w = max(-1.0, min(1.0, q[0]))
    ang = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return np.zeros(3)
    return q[1:] / s * ang


def window_rot_deg(quats):
    """窗口内首尾四元数的世界系净旋转角（度）。"""
    if len(quats) < 2:
        return 0.0
    rel = q_mul(quats[-1], q_conj(quats[0]))
    return float(np.linalg.norm(q_rotvec(rel))) * 180.0 / math.pi


def resample(t, v, t_grid):
    """不规则时间序列 → 均匀网格（逐列线性插值）。"""
    t = np.asarray(t, float)
    out = np.empty((len(t_grid), v.shape[1]))
    for c in range(v.shape[1]):
        out[:, c] = np.interp(t_grid, t, v[:, c])
    return out


def top_spectrum_peaks(t, values, n_peaks=3):
    """均匀重采样后做 FFT，返回 top-n 谱峰 [(Hz, 幅值)]（剔除直流邻域）。
    values 支持 (N,) 标量序列或 (N,3) 位置类序列（后者取相对均值的模长=漂移量谱）。"""
    if len(t) < 32:
        return []
    v = np.asarray(values, float)
    if v.ndim == 2:
        v = np.linalg.norm(v - v.mean(axis=0), axis=1)
    else:
        v = v.ravel()
    if len(v) != len(t):
        return []
    t = np.asarray(t, float)
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        return []
    grid = np.arange(t[0], t[-1], dt)
    sig = resample(t, v.reshape(-1, 1), grid)[:, 0]
    sig = sig - sig.mean()
    spec = np.abs(np.fft.rfft(sig * np.hanning(len(sig))))
    freqs = np.fft.rfftfreq(len(sig), dt)
    mask = freqs > 1.0          # 剔除直流与极低频漂移
    if not mask.any():
        return []
    spec = np.where(mask, spec, 0.0)
    order = np.argsort(spec)[::-1]
    peaks, seen = [], set()
    for i in order:
        f = round(float(freqs[i]), 1)
        if f in seen or spec[i] <= 0:
            continue
        seen.add(f)
        peaks.append((f, round(float(spec[i] / len(sig) * 4), 5)))
        if len(peaks) >= n_peaks:
            break
    return peaks


class FeelProbe(Node):
    def __init__(self, input_prefix: str = "quest"):
        super().__init__("feel_baseline_probe")
        self.quest_t = deque()          # (wall_t, pos3, quat_wxyz)
        self.tgt_t = deque()            # (wall_t, pos3, quat_wxyz)
        self.ee_t = deque()             # (wall_t, pos3, quat_wxyz)
        self.qd_t = deque()             # (wall_t, vel6)
        self.grip_t = deque()           # (wall_t, grip_bool)
        self.report = []
        self._lock = threading.Lock()   # 回调线程写 / 主线程读的互斥（deque 迭代竞态实测崩过）
        qos = fast_qos()
        self.create_subscription(PoseStamped, "/%s/pose" % input_prefix, self._on_quest, qos)
        self.create_subscription(PoseStamped, "/target_pose", self._on_tgt, qos)
        self.create_subscription(JointState, "/joint_states", self._on_js, qos)
        self.create_subscription(Joy, "/%s/joy" % input_prefix, self._on_joy, qos)
        self.last_q = {}
        self._engaged_prev = False
        self._last_tgt_before = None    # 接合前最后一条目标（跳变基准）
        self.engage_jumps = []

    # ---------- 回调 ----------
    def _on_quest(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        rec = (time.time(),
               np.array([p.x, p.y, p.z]),
               xyzw_to_wxyz([o.x, o.y, o.z, o.w]))
        with self._lock:
            self.quest_t.append(rec)
            if len(self.quest_t) > 20000:
                self.quest_t.popleft()

    def _on_tgt(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        rec = (time.time(), np.array([p.x, p.y, p.z]),
               xyzw_to_wxyz([o.x, o.y, o.z, o.w]))
        with self._lock:
            self.tgt_t.append(rec)
            self._last_tgt_before = rec
            if len(self.tgt_t) > 20000:
                self.tgt_t.popleft()

    def _on_js(self, msg):
        for i, name in enumerate(msg.name):
            if name in GROUP:
                self.last_q[name] = msg.position[i]
        if len(self.last_q) != len(GROUP):
            return
        for a, j in zip(_qadr, GROUP):
            _mj_data.qpos[a] = self.last_q[j]
        mujoco.mj_forward(_mj_model, _mj_data)
        R = _mj_data.xmat[_bid].reshape(3, 3).copy()
        p = _mj_data.xpos[_bid] + R @ TIP_OFF
        qw = np.zeros(4)
        mujoco.mju_mat2Quat(qw, np.ascontiguousarray(R).flatten())
        # velocity 必须按关节名取——[:6] 在消息顺序含夹爪时会读错（2026-10-01 自查）
        v = None
        if msg.velocity and len(msg.velocity) == len(msg.name):
            idx = {n: i for i, n in enumerate(msg.name)}
            if all(j in idx for j in GROUP):
                v = np.abs(np.array([msg.velocity[idx[j]] for j in GROUP]))
        with self._lock:
            self.ee_t.append((time.time(), p, qw))
            if len(self.ee_t) > 20000:
                self.ee_t.popleft()
            if v is not None:
                self.qd_t.append((time.time(), v))
                if len(self.qd_t) > 20000:
                    self.qd_t.popleft()
        # 接合跳变（自动，全程）：Grip 上升沿后的首条目标 vs 接合前最后一条目标
        grip = bool(self.grip_t and self.grip_t[-1][1])
        if grip and not self._engaged_prev:
            self._pending_jump = True
            self._jump_pre = self._last_tgt_before
        self._engaged_prev = grip
        if getattr(self, "_pending_jump", False) and len(self.tgt_t) > 1:
            if self.tgt_t[-1][0] - self.tgt_t[-2][0] > 0.5:  # 这条是脱离后的第一条（新会话）
                first = self.tgt_t[-1]
                if self._jump_pre is not None:
                    dp = float(np.linalg.norm(first[1] - self._jump_pre[1]))
                    rel = q_mul(first[2], q_conj(self._jump_pre[2]))
                    da = float(np.linalg.norm(q_rotvec(rel))) * 180 / math.pi
                    self.engage_jumps.append((dp, da))
                    self._pending_jump = False

    def _on_joy(self, msg):
        if len(msg.buttons) >= 1:
            with self._lock:
                self.grip_t.append((time.time(), bool(msg.buttons[0])))
                if len(self.grip_t) > 5000:
                    self.grip_t.popleft()

    # ---------- 工具 ----------
    def _window(self, buf, t0, t1):
        with self._lock:
            items = [x for x in buf if t0 <= x[0] <= t1]
        if not items:
            return None
        t = np.array([x[0] for x in items])
        if len(items) >= 2 and isinstance(items[0][1], np.ndarray) and items[0][1].ndim == 1 and len(items[0][1]) == 3 and isinstance(items[0][2], np.ndarray) and len(items[0][2]) == 4:
            pos = np.vstack([x[1] for x in items])
            quat = np.vstack([x[2] for x in items])
            return t, pos, quat
        return t, None, None

    def _grip_held(self, t0, t1):
        """窗口内 Grip 全程按住才算有效（any() 语义会放过中途松开的 rep——实测教训）。"""
        with self._lock:
            gs = [g for (tt, g) in self.grip_t if t0 <= tt <= t1]
        return len(gs) > 0 and all(gs)

    def _wait_grip(self, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                held = bool(self.grip_t) and self.grip_t[-1][1]
            if held:
                return True
            time.sleep(0.1)
        return False

    def _wait_quiet(self, kind, timeout=12.0):
        """等臂端与目标都静止（上一动作的残余运动消退）——rep 窗口干净的前提。
        （缺失它时，窗口会把上次动作的残余运动算进本次"目标/臂"位移——2026-10-01 实测教训）"""
        end = time.time() + timeout
        thr = ANG_SEG_END if kind == "rot" else POS_SEG_END
        quiet_since = None
        while time.time() < end:
            time.sleep(0.1)
            now = time.time()
            ok = True
            for buf in (self.ee_t, self.tgt_t):
                ts, lin, ang = self._speeds(buf, now - 0.4, now)
                if len(ts):
                    s = ang.max() if kind == "rot" else lin.max()
                    if s >= thr:
                        ok = False
                        break
            if ok:
                if quiet_since is None:
                    quiet_since = now
                elif now - quiet_since >= 0.3:
                    return True
            else:
                quiet_since = None
        return False

    def _speeds(self, buf, t0, t1):
        """窗口内线速度/角速度序列。"""
        with self._lock:
            items = [x for x in buf if t0 <= x[0] <= t1]
        ts, lin, ang = [], [], []
        for a, b in zip(items, items[1:]):
            dt = b[0] - a[0]
            if dt <= 0:
                continue
            ts.append(b[0])
            lin.append(float(np.linalg.norm(b[1] - a[1])) / dt)
            ang.append(float(np.linalg.norm(q_rotvec(q_mul(b[2], q_conj(a[2]))))) / dt)
        return np.array(ts), np.array(lin), np.array(ang)

    def _wait_burst(self, kind, timeout=15.0):
        """等一次动作爆发：速度越过起始门限 → 持续 0.45s 低于终点门限 → 返回 (t0,t1)。
        （整段切分器在 Quest 抖动下会把多次动作黏成一段——实测教训——改为逐次等待。）"""
        a0 = ANG_SEG_START if kind == "rot" else POS_SEG_START
        a1 = ANG_SEG_END if kind == "rot" else POS_SEG_END
        t_mark = time.time()
        t_start = None
        end = time.time() + timeout
        while time.time() < end:
            time.sleep(0.05)
            ts, lin, ang = self._speeds(self.quest_t, t_mark, time.time())
            if len(ts) == 0:
                continue
            speed = ang if kind == "rot" else lin
            if t_start is None:
                if speed[-1] > a0:
                    t_start = ts[-1]
            else:
                recent = speed[ts >= time.time() - 0.45]
                if len(recent) > 0 and recent.max() < a1 and ts[-1] - t_start > 0.25:
                    return (t_start - 0.05, time.time())
        return (t_start - 0.05, time.time()) if t_start is not None else None

    def micro_phase(self, kind, title, action):
        unit = "deg" if kind == "rot" else "cm"
        self.get_logger().info("[阶段2] %s：共 %d 次，每次按提示做【一次】小的「%s」后保持静止。"
                               % (title, REPS_PER_ROUND, action))
        rows = []
        for k in range(REPS_PER_ROUND):
            input("  第 %d/%d 次 —— 按住 Grip，回车开始这一次…" % (k + 1, REPS_PER_ROUND))
            if not self._wait_grip():
                self.get_logger().warn("  未检测到 Grip，本轮记为无效")
                rows.append({"hand": 0.0, "tgt": 0.0, "arm": 0.0, "unit": unit, "grip": False})
                continue
            if not self._wait_quiet(kind):
                self.get_logger().warn("  臂/目标 12s 未静止（残余运动？），仍继续本次测量（结果带污染风险）")
            res = self._wait_burst(kind)
            if res is None:
                self.get_logger().warn("  未检测到动作（15s 超时），本轮记为无效")
                rows.append({"hand": 0.0, "tgt": 0.0, "arm": 0.0, "unit": unit, "grip": False})
                continue
            row = self._rep_row(res, kind)
            rows.append(row)
            self.get_logger().info("  第%d次: 手 %.2f %s → 目标 %.2f → 臂 %.2f %s%s"
                                   % (k + 1, row["hand"], unit, row["tgt"], row["arm"], unit,
                                      "" if row["grip"] else "（⚠️ 该窗口内 Grip 未按住，无效）"))
        valid = [r for r in rows if r["grip"] and r["hand"] > 0.05]
        out = {"phase": title, "unit": kind, "rows": rows}
        if valid:
            gains = [r["arm"] / r["hand"] for r in valid if r["hand"] > 0.3]
            if gains:
                out["eff_gain_median"] = float(np.median(gains))
                dead = [r for r in valid if r["arm"] < 0.1 and r["hand"] >= 1.0]
                out["deadband_hits"] = len(dead)
        self._print_block(title, out)
        self.report.append(out)

    def _rep_row(self, seg, kind):
        t0, t1 = seg
        qw = self._window(self.quest_t, t0, t1)
        tw = self._window(self.tgt_t, t0, t1)
        ew = self._window(self.ee_t, t0, t1)
        row = {"grip": self._grip_held(t0, t1)}
        if kind == "rot":
            row["hand"] = window_rot_deg(qw[2]) if qw else 0.0
            row["tgt"] = window_rot_deg(tw[2]) if tw and tw[2] is not None else 0.0
            row["arm"] = window_rot_deg(ew[2]) if ew and ew[2] is not None else 0.0
            row["unit"] = "deg"
        else:
            row["hand"] = float(np.linalg.norm(qw[1][-1] - qw[1][0])) * 100 if qw else 0.0
            row["tgt"] = float(np.linalg.norm(tw[1][-1] - tw[1][0])) * 100 if tw and tw[1] is not None else 0.0
            row["arm"] = float(np.linalg.norm(ew[1][-1] - ew[1][0])) * 100 if ew and ew[1] is not None else 0.0
            row["unit"] = "cm"
        return row

    # ---------- 各阶段 ----------
    def channel_check(self):
        self.get_logger().info("[阶段0] 通道健康检查 5s：请确认头显已进页面（不必操作）…")
        marks = {k: len(getattr(self, k)) for k in ("quest_t", "tgt_t", "ee_t", "qd_t")}
        time.sleep(5.0)
        rates = {}
        for k, base in marks.items():
            rates[k] = (len(getattr(self, k)) - base) / 5.0
        self.get_logger().info(
            "  /quest/pose %.1f Hz | /target_pose %.1f Hz | /joint_states(FK) %.1f Hz"
            % (rates["quest_t"], rates["tgt_t"], rates["ee_t"]))
        ok = rates["quest_t"] > 20 and rates["ee_t"] > 50
        if not ok:
            self.get_logger().warn(
                "  通道不健康：/quest/pose 低 = 未进页面或 relay 未跑；FK 低 = 仿真未跑。先拉起链路。")
        return ok

    def rest_phase(self):
        input("\n[阶段1] 静息抖动 %ds：请【按住 Grip】并把手保持完全静止，按回车开始…" % REST_SECONDS)
        if not self._wait_grip():
            self.get_logger().warn("  未检测到 Grip——静息段照跑，但结果按无效解读")
        t0 = time.time()   # Grip 确认后才开窗（窗口混入接合前动作=漂移虚高，实测教训）
        base = len(self.qd_t)
        time.sleep(REST_SECONDS)
        t1 = time.time()
        qw = self._window(self.quest_t, t0, t1)
        tw = self._window(self.tgt_t, t0, t1)
        qd = [(tt, v) for (tt, v) in self.qd_t if t0 <= tt <= t1]
        out = {"phase": "rest", "grip_held": self._grip_held(t0, t1)}
        if qw and qw[1] is not None:
            out["hand_drift_mm"] = float(np.std(qw[1], axis=0).max() * 1000)
        if tw and tw[1] is not None:
            out["tgt_rms_mm"] = float(np.std(tw[1], axis=0).mean() * 1000)
            peaks = top_spectrum_peaks([x[0] for x in self.tgt_t if t0 <= x[0] <= t1],
                                       np.array([x[1] for x in self.tgt_t if t0 <= x[0] <= t1]))
            out["tgt_spectrum_Hz_amp"] = peaks
        if qd:
            v = np.vstack([x[1] for x in qd])
            out["qd_rms"] = float(np.sqrt((v ** 2).mean()))
            out["qd_max"] = float(v.max())
            peaks = top_spectrum_peaks([x[0] for x in qd], v.max(axis=1))
            out["qd_spectrum_Hz_amp"] = peaks
        self._print_block("静息抖动", out)
        self.report.append(out)

    def free_phase(self):
        input("\n[阶段3] 自由操作 %ds：按住 Grip 随意操作（幅度可以大、速度可以快）。按回车开始…" % FREE_SECONDS)
        t0 = time.time()
        time.sleep(FREE_SECONDS)
        t1 = time.time()
        # 手速 vs 臂端速度 互相关滞后
        h_ts, h_lin, h_ang = self._speeds(self.quest_t, t0, t1)
        e_ts, e_lin, e_ang = self._speeds(self.ee_t, t0, t1)
        t_ts, t_lin, t_ang = self._speeds(self.tgt_t, t0, t1)
        out = {"phase": "free"}
        if len(h_ts) > 50 and len(e_ts) > 50:
            hand = (h_lin / 0.3 + h_ang)          # 位置 0.3m≈1rad 的量纲折算
            arm = (e_lin / 0.3 + e_ang)
            grid = np.arange(max(h_ts[0], e_ts[0]) + 0.1, min(h_ts[-1], e_ts[-1]) - 0.1,
                             1.0 / CORR_GRID_HZ)
            if len(grid) < 300:
                out["lag_note"] = "自由段有效重叠不足（目标流有断档?），滞后未算"
            else:
                H = resample(h_ts, hand.reshape(-1, 1), grid)[:, 0]
                A = resample(e_ts, arm.reshape(-1, 1), grid)[:, 0]
                H = H - H.mean()
                A = A - A.mean()
                best = (0.0, -2.0)
                for lag in range(0, int(CORR_MAX_LAG * CORR_GRID_HZ)):
                    c = float(np.corrcoef(H[:len(H) - lag], A[lag:])[0, 1])
                    if c > best[1]:
                        best = (lag / CORR_GRID_HZ, c)
                out["lag_hand_to_arm_s"] = round(best[0], 3)
                out["corr_peak"] = round(best[1], 3)
                if len(t_ts) > 50:
                    tgt = (t_lin / 0.3 + t_ang)
                    T = resample(t_ts, tgt.reshape(-1, 1), grid)[:, 0]
                    T = T - T.mean()
                    best_t = (0.0, -2.0)
                    for lag in range(0, int(CORR_MAX_LAG * CORR_GRID_HZ)):
                        c = float(np.corrcoef(H[:len(H) - lag], T[lag:])[0, 1])
                        if c > best_t[1]:
                            best_t = (lag / CORR_GRID_HZ, c)
                    out["lag_hand_to_target_s"] = round(best_t[0], 3)
                    out["corr_hand_to_target"] = round(best_t[1], 3)
            out["metric_note"] = ("滞后=速度通道互相关（管线量）；位置跟踪带宽以 follow_bench 正弦为准")
        qd = [(tt, v) for (tt, v) in self.qd_t if t0 <= tt <= t1]
        if qd:
            v = np.vstack([x[1] for x in qd])
            out["qd_p50"] = float(np.percentile(v, 50))
            out["qd_p95"] = float(np.percentile(v, 95))
            out["qd_max"] = float(v.max())
            out["qd_over_vendor_limit"] = bool((v > VENDOR_VMAX).any())
        self._print_block("自由操作", out)
        self.report.append(out)

    def summary(self):
        lines = ["=" * 60, "手感基线报告  %s" % time.strftime("%Y-%m-%d %H:%M:%S"), "=" * 60]
        for r in self.report:
            lines.append(json.dumps(r, ensure_ascii=False, default=str))
        if self.engage_jumps:
            dps = [d for d, _ in self.engage_jumps]
            das = [a for _, a in self.engage_jumps]
            lines.append("接合跳变（%d 次）：位移中位 %.1f mm，旋转中位 %.2f°"
                         % (len(dps), float(np.median(dps)) * 1000, float(np.median(das))))
        text = "\n".join(lines)
        with open(REPORT_PATH, "w") as f:
            f.write(text + "\n")
        with open(JSON_PATH, "w") as f:
            json.dump(self.report, f, ensure_ascii=False, default=str)
        self.get_logger().info("报告已存：%s（JSON: %s）" % (REPORT_PATH, JSON_PATH))

    def _print_block(self, title, out):
        self.get_logger().info("  ── %s ──" % title)
        for k, v in out.items():
            if k in ("phase", "rows", "unit"):
                continue
            self.get_logger().info("  %s = %s" % (k, v))


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", default="quest", choices=["quest", "mock_vr"],
                    help="输入前缀：quest=真头显；mock_vr=键盘 Mock（无头显干跑，"
                         "验证测量机器本身，数据不代表真手感）")
    args = ap.parse_args()
    rclpy.init()
    node = FeelProbe(input_prefix=args.input)
    import threading
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    time.sleep(1.0)
    try:
        if not node.channel_check():
            node.get_logger().warn("通道检查未过——本次结果无效，先拉起链路再跑。")
            return 1
        node.rest_phase()
        node.micro_phase("rot", "微旋转·水平转", "水平面内向左或向右平转（单程）")
        node.micro_phase("rot", "微旋转·俯仰", "向上或向下点头（单程）")
        node.micro_phase("rot", "微旋转·滚转", "向左或向右拧钥匙（单程）")
        node.micro_phase("pos", "微平移·前后", "向前或向后小幅推（单程）")
        node.micro_phase("pos", "微平移·左右", "向左或向右小幅平移（单程）")
        node.free_phase()
        node.summary()
        node.get_logger().info("完成。把 /tmp/feel_baseline_report.txt 交给分析。")
        return 0
    except KeyboardInterrupt:
        node.summary()
        node.get_logger().info("已中断，已保存部分报告。")
        return 130
    except Exception as e:
        node.get_logger().error("阶段异常：%r —— 已保存已完成部分的报告" % (e,))
        node.summary()
        return 2
    finally:
        # 先停 executor 再销毁节点——反序会销毁竞态 abort（冒烟实测 core dump）
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
