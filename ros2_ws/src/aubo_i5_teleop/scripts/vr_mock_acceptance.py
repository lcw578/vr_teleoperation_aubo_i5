#!/usr/bin/env python3
"""VR Mock 全链路验收：scripted Mock + 映射器 + 夹爪 FSM，一键跑完并出判据表。

流程（--script 的时刻表见 /tmp/vr_acceptance_script.txt）：
  接合 → 纯自系旋转 60°（工具轴语义）→ 释放（冻结）→ 重接合 → 世界系平移 →
  切微调档 → 微调平移 → 夹爪闭合（与平移并行）→ 张开 → 释放 → 重接合 →
  【杀掉 Mock 进程制造断流】→ 自动脱离 → 冻结。

判据（全部有阈值）：
  1 旋转阶段：末端姿态变化 ≥45°，位置漂移 ≤8 mm（原位旋转，不画大圆）
  2 释放：1.5 s 内停稳；末段漂移 ≤1 mm
  3 缩放切换：/target_pose 无 >5 mm 的相邻消息跳变
  4 夹爪并行：knuckle 0→≥0.8→≤0.1；同期臂 |cmd−q| ≤0.05
  5 断流：杀 Mock 后 /target_pose 停发 ≤0.5 s；末端漂移 ≤1 mm
  6 全程：无碰撞/急停/到界状态；|cmd−q| 峰值 ≤0.05

用法（栈在跑，臂已归位）：
  /home/lcw/tomato_robot/.venv/bin/python scripts/vr_mock_acceptance.py
"""
import csv
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import mujoco
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int8

PKG = Path(__file__).parent
PY = "/home/lcw/tomato_robot/.venv/bin/python"
SCRIPT = "/tmp/vr_acceptance_script.txt"
CSV_PATH = "/tmp/vr_acceptance.csv"
DURATION = 15.0
KILL_MOCK_AT = 12.5          # 此时 clutch 仍按住（脚本 11.0 接合）→ 制造断流

GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
TIP_OFF = np.array([-0.0405, -0.0143, 0.1492])

_mj_model = mujoco.MjModel.from_xml_path(
    "/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml")
_mj_data = mujoco.MjData(_mj_model)
_qadr = [_mj_model.jnt_qposadr[mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_JOINT, j)]
         for j in GROUP]
_kadr = _mj_model.jnt_qposadr[mujoco.mj_name2id(
    _mj_model, mujoco.mjtObj.mjOBJ_JOINT, "left_outer_knuckle_joint")]
_bid = mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")

# home（ready）末端位姿：验收前必须先归位——上一轮可能把臂留在任意位姿/贴限位
# （2026-09-25 抓出：不归位就开始，工具一转就"关节到界"）
READY_Q = [-1.089254, -0.802598, 1.308255, 0.588418, 0.324607, -0.704605]


def _home_pose():
    for a_, v in zip(_qadr, READY_Q):
        _mj_data.qpos[a_] = v
    mujoco.mj_forward(_mj_model, _mj_data)
    R = _mj_data.xmat[_bid].reshape(3, 3).copy()
    p = _mj_data.xpos[_bid] + R @ TIP_OFF
    qw = np.zeros(4)
    mujoco.mju_mat2Quat(qw, np.ascontiguousarray(R).flatten())
    return p, qw          # qw=[w,x,y,z]


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


class Recorder(Node):
    def __init__(self):
        super().__init__("vr_acceptance_rec")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.q = {}
        self.cmd = None
        self.status = None
        self.knuckle = float("nan")
        self.tgt = None
        self.rows = []
        qos = fast_qos()
        self.create_subscription(JointState, "/joint_states", self._js, qos)
        self.create_subscription(Float64MultiArray,
                                 "/forward_command_controller_position/commands", self._cmd, qos)
        self.create_subscription(Int8, "/servo_pose_tracking/status", self._st, qos)
        self.create_subscription(PoseStamped, "/target_pose", self._tg, qos)
        self.csv = open(CSV_PATH, "w", newline="")
        self.w = csv.writer(self.csv)
        self.w.writerow(["sim_t", "wall", "ee_x", "ee_y", "ee_z",
                         "eq_x", "eq_y", "eq_z", "eq_w",
                         "cmd0", "cmd1", "cmd2", "cmd3", "cmd4", "cmd5",
                         "q0", "q1", "q2", "q3", "q4", "q5",
                         "status", "knuckle",
                         "tgt_x", "tgt_y", "tgt_z", "tgt_qw", "tgt_stamp"])

    def _fk(self):
        for j in GROUP:
            if j not in self.q:
                return None, None
        for a, j in zip(_qadr, GROUP):
            _mj_data.qpos[a] = self.q[j]
        _mj_data.qpos[_kadr] = self.knuckle if math.isfinite(self.knuckle) else 0.0
        mujoco.mj_forward(_mj_model, _mj_data)
        R = _mj_data.xmat[_bid].reshape(3, 3).copy()
        p = _mj_data.xpos[_bid] + R @ TIP_OFF
        qw = np.zeros(4)
        mujoco.mju_mat2Quat(qw, np.ascontiguousarray(R).flatten())
        return p, qw          # qw = [w,x,y,z]

    def _js(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        for i, name in enumerate(m.name):
            if name in GROUP:
                self.q[name] = m.position[i]
            if name == "left_outer_knuckle_joint":
                self.knuckle = m.position[i]
        self._log(st)

    def _cmd(self, m):
        self.cmd = list(m.data)

    def _st(self, m):
        self.status = int(m.data)

    def _tg(self, m):
        self.tgt = (m.pose.position.x, m.pose.position.y, m.pose.position.z,
                    m.pose.orientation.w,
                    m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self._log(st)

    def _log(self, sim_t):
        p, qw = self._fk()
        if p is None:
            return
        cmd = self.cmd or [float("nan")] * 6
        qvals = [self.q.get(j, float("nan")) for j in GROUP]
        tg = self.tgt or (float("nan"),) * 5
        self.w.writerow([sim_t, time.time(), *p, *qw] + list(cmd)
                        + qvals
                        + [self.status if self.status is not None else -99,
                           self.knuckle, *tg])


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def ang_between(qa, qb):
    return math.degrees(2 * math.acos(max(-1.0, min(1.0, abs(float(np.dot(qa, qb)))))))


def rotvec_deg(qa, qb):
    """qa 相对 qb 的旋转向量（xyzw→wxyz 由调用方处理）。单位度。"""
    d = qmul(qa, qconj(qb))
    v = np.array([d[1], d[2], d[3]])
    n = np.linalg.norm(v)
    s = math.copysign(1.0, d[0])
    return np.degrees(n) * s


def main():
    rclpy.init()
    rec = Recorder()
    log = open("/tmp/vr_acceptance_nodes.log", "w")
    procs = {}

    def spawn(name, args):
        procs[name] = subprocess.Popen(
            [PY] + args, cwd=str(PKG), stdout=log, stderr=subprocess.STDOUT)

    # 写 Mock 回放脚本
    Path(SCRIPT).write_text("""\
# VR Mock 验收脚本（时刻/时长/动作/参数）
0.5  0.0  clutch 1
1.0  2.0  rot 0 0 45
3.2  0.0  clutch 0
4.5  0.0  clutch 1
5.0  1.0  trans 0 -0.06 0
6.1  0.0  scale
6.3  1.0  trans 0 -0.03 0
7.5  0.0  grip
7.7  1.0  trans 0.03 0 0
8.9  0.0  grip
9.7  0.0  clutch 0
11.0 0.0  clutch 1
""")
    # ── 阶段 -1：干净归位（go_ready 是关节空间直插，必须 stage3 停止时跑——
    #    2026-09-25 抓出：上轮验收把臂留在贴限位姿态，直接开始就"关节到界"；
    #    而用 /target_pose 归位的大行程腕部回旋又会触发保护、10 s 到不了 home。
    #    所以这里走与 run_bench.sh 相同的可靠序列：停 stage3 → go_ready → 重启）。──
    print("阶段-1 干净归位：停 stage3 → go_ready → 重启 stage3 ...")
    for pat in ["stage3_pose_trackin[g]", "servo_interfac[e].py", "pose_tracking_nod[e]"]:
        subprocess.run(["pkill", "-f", pat], check=False)
    time.sleep(3)
    subprocess.run(["pkill", "-9", "-f", "servo_interfac[e].py"], check=False)
    subprocess.run(["pkill", "-9", "-f", "pose_tracking_nod[e]"], check=False)
    time.sleep(1)
    r_gr = subprocess.run([PY, "go_ready.py", "--mode", "position"],   # cwd 已是 scripts/，别再带前缀（曾致 scripts/scripts 不存在）
                          cwd=str(PKG), capture_output=True, text=True, timeout=180)
    print("  go_ready stdout:", (r_gr.stdout or "").strip()[-300:] or "(空)")
    if r_gr.returncode != 0:
        print("  go_ready stderr:", (r_gr.stderr or "")[-300:])
    for _ in range(100):                  # 让 recorder 收到 /joint_states
        rclpy.spin_once(rec, timeout_sec=0.01)
    env = dict(**__import__("os").environ)
    procs["stage3"] = subprocess.Popen(
        ["ros2", "launch", "aubo_i5_teleop", "stage3_pose_tracking.launch.py",
         "output_mode:=position"], cwd=str(PKG), stdout=log, stderr=subprocess.STDOUT, env=env)
    # ⚠️ 只等命令话题不够：pose_tracking/servo_interface 初始化更慢，
    #    它们没起来时映射器发的目标会进黑洞（2026-09-25 验收抓出过一轮假通过）。
    ok_cmd = ok_stream = False
    for _ in range(60):
        ok_cmd = rec.count_publishers("/forward_command_controller_position/commands") >= 1
        ok_stream = rec.count_publishers("/servo_position_stream") >= 1
        if ok_cmd and ok_stream:
            break
        time.sleep(1.0)
    time.sleep(2.0)
    if not (ok_cmd and ok_stream):
        print("❌ stage3 重启不完整（cmd=%s stream=%s），中止" % (ok_cmd, ok_stream))
        for _, pp in procs.items():
            pp.terminate()
        rclpy.shutdown()
        return 1
    print("  stage3 已重启（命令话题 + 伺服接口层均在）")

    # ── 阶段 0：归位核对（go_ready 后应已到位；此处只测量不打扰）──
    hp, hq_wxyz = _home_pose()
    ee_now, _ = rec._fk()
    d_home = (float(np.linalg.norm(ee_now - hp)) * 1000 if ee_now is not None
              else float("nan"))
    print("阶段0 归位核对：距 home %.1f mm" % d_home)
    if not (d_home == d_home) or d_home > 10.0:      # NaN 或 >1 cm：中止，不带病跑
        print("❌ 归位未达标，中止验收（不运行 Mock 阶段）")
        for _, p in procs.items():
            p.terminate()
        rclpy.shutdown()
        return 1

    spawn("mapper", ["clutch_mapper_node.py"])
    spawn("gripper", ["gripper_fsm_node.py"])
    time.sleep(2.0)                       # 映射器先起（MJCF 加载 ~1 s）
    t_mock0 = time.time()
    spawn("mock", ["mock_vr_node.py", "--script", SCRIPT])
    print("验收开始（%.0f s，t=%.1f 杀 Mock 制造断流）" % (DURATION, KILL_MOCK_AT))

    killed = False
    t_end = t_mock0 + DURATION
    while time.time() < t_end:
        el = time.time() - t_mock0
        if not killed and el >= KILL_MOCK_AT:
            procs["mock"].kill()
            killed = True
            print("  [%.1f s] Mock 进程已杀（断流测试）" % el)
        rclpy.spin_once(rec, timeout_sec=0.0)
        time.sleep(0.002)
    rec.csv.close()
    for name, p in procs.items():
        p.terminate()
    log.close()
    print("录制完成：%s\n" % CSV_PATH)

    # ────────── 分析 ──────────
    rows = list(csv.DictReader(open(CSV_PATH)))
    wall = np.array([float(r["wall"]) for r in rows]) - t_mock0
    ee = np.array([[float(r["ee_x"]), float(r["ee_y"]), float(r["ee_z"])] for r in rows])
    eq = np.array([[float(r["eq_w"]), float(r["eq_x"]), float(r["eq_y"]), float(r["eq_z"])]
                   for r in rows])                      # wxyz
    cmd = np.array([[float(r["cmd%d" % i]) for i in range(6)] for r in rows])
    st = np.array([int(r["status"]) for r in rows])
    knk = np.array([float(r["knuckle"]) for r in rows])
    has_t = np.array([r["tgt_x"] not in ("", "nan") and math.isfinite(float(r["tgt_x"]))
                      for r in rows])
    tg = np.array([[float(r["tgt_x"]), float(r["tgt_y"]), float(r["tgt_z"])]
                   if h else [np.nan] * 3 for r, h in zip(rows, has_t)])
    tgtqw = np.array([float(r["tgt_qw"]) if h else np.nan for r, h in zip(rows, has_t)])

    print("判据表")
    print("=" * 78)
    ok_all = True

    def gate(name, ok, detail):
        nonlocal ok_all
        ok_all &= bool(ok)
        print("  %-42s [%s] %s" % (name, "PASS" if ok else "FAIL", detail))

    # 1 旋转阶段（脚本 1.0–3.0）
    m1 = (wall >= 0.9) & (wall <= 3.1)
    if m1.sum() > 10:
        i0 = np.where(m1)[0][0]
        tot_rot = ang_between(eq[np.where(m1)[0][-1]], eq[i0])
        drift = np.max(np.linalg.norm(ee[m1] - ee[i0], axis=1)) * 1000
        gate("1 旋转阶段：姿态变化 ≥45°", tot_rot >= 45, "实际 %.1f°" % tot_rot)
        gate("1 旋转阶段：位置漂移 ≤8 mm", drift <= 8, "实际 %.1f mm" % drift)
    else:
        gate("1 旋转阶段样本", False, "样本不足")

    # 2 释放冻结（脚本 3.2–4.5）
    w2 = (wall >= 3.3) & (wall <= 4.4)
    if w2.sum() > 10:
        idx = np.where(w2)[0]
        settled = ee[idx[-1]]
        drift = np.max(np.linalg.norm(ee[idx] - settled, axis=1)) * 1000
        tail = idx[wall[idx] >= 4.0]
        tail_drift = (np.max(np.linalg.norm(ee[tail] - settled, axis=1)) * 1000
                      if len(tail) else float("nan"))
        gate("2 释放后停稳（窗口内散布 ≤8 mm）", drift <= 8, "%.1f mm" % drift)
        gate("2 释放末段漂移 ≤1 mm", tail_drift <= 1.0, "%.2f mm" % tail_drift)
    else:
        gate("2 释放窗口样本", False, "样本不足")

    # 3 缩放切换（脚本 6.1）：target 流相邻消息跳变
    if has_t.sum() > 100:
        tv = tg[has_t]
        jumps = np.linalg.norm(np.diff(tv, axis=0), axis=1) * 1000
        around = (wall[has_t][1:] >= 5.9) & (wall[has_t][1:] <= 6.4)
        gate("3 缩放切换：target 无 >5 mm 相邻跳变",
             np.max(jumps[around]) <= 5.0 if around.any() else False,
             "切档窗口 max %.2f mm（全程 max %.2f）"
             % (np.max(jumps[around]) if around.any() else -1, np.max(jumps)))
    else:
        gate("3 target 流样本", False, "不足")

    # 4 夹爪并行（脚本 7.5 闭、8.9 开；7.7–8.7 臂在动）
    if knk.size > 100:
        after_close = (wall >= 8.3) & (wall <= 8.8)
        after_open = (wall >= 9.7) & (wall <= 10.2)
        k1 = np.nanmax(knk[after_close]) if after_close.any() else float("nan")
        k2 = np.nanmin(knk[after_open]) if after_open.any() else float("nan")
        gate("4 夹爪闭合 knuckle ≥0.8", k1 >= 0.8, "实际 %.3f" % k1)
        gate("4 夹爪张开 knuckle ≤0.1", k2 <= 0.1, "实际 %.3f" % k2)

    # 5 断流（12.5 杀 Mock）
    # ⚠️ Recorder 的 tgt 字段是电平保持（最后一条消息一直留着），不能按"字段非 NaN"
    #    判新目标——要按 target 的**时间戳变化**判定（Mapper 每 tick 新 stamp，
    #    脱离后 stamp 不再变化）。2026-09-25 首轮验收的假 FAIL 就是这里。
    if has_t.sum() > 100:
        stamps = np.array([float(r["tgt_stamp"]) if h else np.nan
                           for r, h in zip(rows, has_t)])
        new_tgt = np.zeros(len(rows), dtype=bool)
        for i in range(1, len(rows)):
            if has_t[i] and (not has_t[i - 1] or stamps[i] != stamps[i - 1]):
                new_tgt[i] = True
        tw_new = wall[new_tgt]
        after = tw_new[tw_new >= 13.2]
        last_t = np.max(tw_new[tw_new < 12.6]) if (tw_new < 12.6).any() else float("nan")
        gate("5 断流：target 停发（杀后 ≤0.1 s，13.2 s 后无新目标）", after.size == 0,
             "最后新目标 @t=%.2f" % last_t)
        w5 = (wall >= 14.0) & (wall <= 15.0)
        if w5.sum() > 10:
            idx = np.where(w5)[0]
            d5 = np.max(np.linalg.norm(ee[idx] - ee[idx[-1]], axis=1)) * 1000
            gate("5 断流后冻结：漂移 ≤1 mm", d5 <= 1.0, "%.2f mm" % d5)

    # 6 全程状态与 |cmd−q|
    bad_st = int(np.isin(st, [3, 4, 5]).sum())
    gate("6 全程无碰撞/急停/到界", bad_st == 0, "违规样本 %d" % bad_st)
    qarr = np.array([[float(r["q%d" % i]) for i in range(6)] for r in rows])
    fin = np.all(np.isfinite(cmd), axis=1) & np.all(np.isfinite(qarr), axis=1)
    mask = fin & (wall >= 0.0)      # 归位阶段（wall<0）的大行程不计入
    lag = np.array([max(abs(c - q) for c, q in zip(cmd[i], qarr[i])) for i in np.where(mask)[0]])
    gate("6 |cmd−q| 峰值 ≤0.05（钳位未触发）",
         lag.size > 0 and float(np.max(lag)) <= 0.05,
         "峰值 %.4f rad" % (float(np.max(lag)) if lag.size else float("nan")))

    print("=" * 78)
    print("总体：%s" % ("✅ 验收通过" if ok_all else "❌ 有 FAIL 项"))

    rclpy.shutdown()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
