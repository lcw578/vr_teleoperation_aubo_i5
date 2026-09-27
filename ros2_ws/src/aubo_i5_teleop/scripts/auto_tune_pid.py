#!/usr/bin/env python3
"""自主 PID 调参验证器——不需要头显、不需要人操控。

回答的问题："能否自主验证调参？"→ 能。
原理：手柄只是 /target_pose 的激励源之一。用**合成信号**（阶跃 + 正弦）直接发
/target_pose，用 /joint_states + FK 量响应，就能完整复现"操控时的激励→响应"
闭环——这正是经典伺服整定方法（Ziegler-Nichols 同款思路：固定激励、量响应、
改参数、重复）。人手只在最后做"手感验收"。

流程（一次运行完成）：
  归位 → 对当前参数发 8 cm 阶跃 + 0.5 Hz 正弦 → 量（超调%、2% 整定时间、
  关节速度峰值）→ 汇报。跨参数扫描用 --pid 参数多次调用（每档重启 stage3
  约 40 s，脚本不做进程管理，由调用方 sweep 脚本控制）。

判据（对照）：
  修复前基线：关节速度峰值 4.84 rad/s（超厂商限值 2.618/3.142）→ 震荡
  目标：峰值 <2.5 rad/s、阶跃超调 <10%、2% 整定时间 <1.5 s

用法：
  /home/lcw/tomato_robot/.venv/bin/python scripts/auto_tune_pid.py            # 单次测量
  /home/lcw/tomato_robot/.venv/bin/python scripts/auto_tune_pid.py --label P30I1D01
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

G = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
     "wrist1_joint", "wrist2_joint", "wrist3_joint"]
READY_Q = [-1.089254, -0.802598, 1.308255, 0.588418, 0.324607, -0.704605]
STEP_M = 0.08
SINE_HZ = 0.5
SINE_AMP_M = 0.04
VENDOR_VMAX = 3.142

# home 的末端位姿（FK 已验证）
HOME_P = np.array([-0.0008, -0.7691, 0.1522])
# home 姿态（xyzw）：MuJoCo FK 实测 [w,x,y,z]=[0.3794,0.7942,-0.169,-0.4436]
# ⚠️ 历史坑：第一版把 [w,x,y,z] 直接当 (x,y,z,w)（w/x 错位）→ 目标姿态偏 33°
# → IK 解到 wrist3 贴限位（2.94）→ Servo Halting 死锁。第二版又转错一次。
# 正确换算：xyzw = (qw[1], qw[2], qw[3], qw[0]) = (0.7942,-0.1690,-0.4436,0.3794)
# 已用工具轴逐位验证与 FK 一致（[-0.833,-0.453,-0.319]）。
HOME_Q = (0.7942, -0.1690, -0.4436, 0.3794)  # xyzw（已验证工具轴一致）


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


class Tuner(Node):
    def __init__(self):
        super().__init__("auto_tune")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.q = {}
        self.pub = self.create_publisher(PoseStamped, "/target_pose", 10)
        self.create_subscription(JointState, "/joint_states", self._js, fast_qos())
        self.samples = []          # (sim_t, ee_pos, qd_max)
        self._t0 = time.time()

    def _js(self, m):
        for i, name in enumerate(m.name):
            if name in G:
                self.q[name] = (m.position[i], m.velocity[i])

    def fk(self):
        import mujoco
        m = self._mj_model
        d = self._mj_data
        qa = self._mj_qadr
        bid = self._mj_bid
        for a, j in zip(qa, G):
            if j not in self.q:
                return None, None
        for a, j in zip(qa, G):
            d.qpos[a] = self.q[j][0]
        mujoco.mj_forward(m, d)
        R = d.xmat[bid].reshape(3, 3).copy()
        p = d.xpos[bid] + R @ np.array([-0.0405, -0.0143, 0.1492])
        qw = np.zeros(4)
        mujoco.mju_mat2Quat(qw, np.ascontiguousarray(R).flatten())
        qd = max(abs(self.q[j][1]) for j in G)
        return (p, (qw[1], qw[2], qw[3], qw[0])), qd

    @staticmethod
    def _mk_pose(p, q):
        msg = PoseStamped()
        msg.header.frame_id = "world"
def send_pose(node_pub, node_clock, p, q_xyzw):
    msg = PoseStamped()
    msg.header.frame_id = "world"
    st = node_clock.now()
    if st.nanoseconds == 0:
        raise RuntimeError("仿真时钟为 0——/clock 没起来，目标 stamp 会全部无效")
    msg.header.stamp = st.to_msg()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q_xyzw
    node_pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="current")
    args = ap.parse_args()

    import mujoco
    m = mujoco.MjModel.from_xml_path("/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml")
    d = mujoco.MjData(m)
    G_ = G
    qa = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in G_]
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")

    def fk(q):
        for a, v in zip(qa, q):
            d.qpos[a] = v
        mujoco.mj_forward(m, d)
        R = d.xmat[bid].reshape(3, 3).copy()
        p = d.xpos[bid] + R @ np.array([-0.0405, -0.0143, 0.1492])
        qw = np.zeros(4)
        mujoco.mju_mat2Quat(qw, np.ascontiguousarray(R).flatten())
        return p, (qw[1], qw[2], qw[3], qw[0])

    rclpy.init()
    node = rclpy.node.Node("auto_tune")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    # 等 /clock 第一帧（use_sim_time 下 node 时钟在收到 /clock 前恒为 0）
    t0 = time.time()
    while node.get_clock().now().nanoseconds == 0 and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.get_clock().now().nanoseconds == 0:
        print("❌ 10 s 未收到 /clock——先跑 authoritative_session.sh")
        sys.exit(1)
    pub = node.create_publisher(PoseStamped, "/target_pose", 10)
    q = {}
    node.create_subscription(JointState, "/joint_states", lambda m: q.update(
        {n: (m.position[i], m.velocity[i]) for i, n in enumerate(m.name) if n in G_}), fast_qos())

    def spin(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(node, timeout_sec=0.01)

    def fk_now():
        if len(q) < 6:
            return None, None
        qv = [q[j][0] for j in G_]
        qd = max(q[j][1] for j in G_)
        return fk(qv), qd

    def hold(p, quat, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            send_pose(pub, node.get_clock(), p, quat)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.004)

    # ── 0. 归位：**先 go_ready（关节空间，绕过 Servo）**——臂若在贴限位/奇异，
    # Servo 会 Halting 拒绝一切笛卡尔目标，"发 home 目标"永远到不了（死锁）。
    # 2026-09-27 实测死锁三轮的教训。go_ready 要求 stage3 已起（本脚本前置已满足）：
    # 它直发 commands，servo_interface 不在时无冲突；在时其 200Hz 冻结流会覆盖——
    # 所以正确顺序是：杀 servo_interface → go_ready → 让 stage3 的 launch 不重启、
    # 手动重起 servo_interface。为简化：本脚本要求调用方先跑 authoritative_session.sh
    # （它保证 Servo 干净），这里只做"检查臂是否在 home，不在则提示用救援流程"。
    print("归位（发 home 目标 12 s；若臂贴限位请先跑 authoritative_session.sh）…")
    hold(HOME_P, HOME_Q, 12.0)
    (p_now, _), _ = fk_now()
    if p_now is None or np.linalg.norm(p_now - HOME_P) > 0.01:
        print("⚠️ 归位未达标（差 %.1f mm）——继续（数据仅作相对比较）" %
              (np.linalg.norm(p_now - HOME_P) * 1000 if p_now is not None else -1))
    else:
        print("归位 OK（%.2f mm）" % (np.linalg.norm(p_now - HOME_P) * 1000))

    # ── 1. 8 cm 阶跃（z 向下 4 cm + 回）──
    print("阶跃 %.0f mm（z）…" % (STEP_M * 1000))
    target_p = HOME_P + np.array([0.0, 0.0, -STEP_M])
    step_rows = []
    qd_peak = 0.0
    t0 = time.time()
    while time.time() - t0 < 8.0:      # 3.5 s 不够 Servo 走完 8 cm（实测 ~25 mm/s 终端速度）
        send_pose(pub, node.get_clock(), target_p, HOME_Q)
        rclpy.spin_once(node, timeout_sec=0.005)
        (pn, _), qd = fk_now()
        if pn is not None:
            step_rows.append((time.time() - t0, float(np.linalg.norm(pn - target_p)), qd))
            qd_peak = max(qd_peak, qd)
        time.sleep(0.004)
    # 量化：超调（越过目标的最深 excursion）与 2% 整定时间
    settled = np.median([r[1] for r in step_rows if r[0] > 6.5])
    over = max((settled - r[1]) for r in step_rows) if step_rows else 0
    # 修正：r[1] 是"到目标的距离"，超调 = 最小距离 < 0 之后重新拉大——
    # 简化：超调% = (settled_d - min_d)/STEP_M（min_d 接近 0 时超调即 settled_d 的回落）
    min_d = min(r[1] for r in step_rows)
    over_pct = max(0.0, (settled - min_d)) / STEP_M * 100.0 if settled > min_d else 0.0
    settle_t = None
    band = 0.02 * STEP_M
    for tt, dd, _ in step_rows:
        if all(abs(r[1] - settled) <= band for r in step_rows if r[0] >= tt):
            settle_t = tt
            break
    print("  阶跃：稳态差 %.1f mm | 整定(2%%) %s | 关节速度峰值 %.2f rad/s"
          % (settled * 1000, ("%.2f s" % settle_t) if settle_t else "未整定", qd_peak))

    # ── 2. 回 home ──
    hold(HOME_P, HOME_Q, 8.0)

    # ── 3. 0.5 Hz 正弦（x 向，幅值 4 cm）──
    print("正弦 %.1f Hz A=%.0f mm…" % (SINE_HZ, SINE_AMP_M * 1000))
    rows = []
    t0 = time.time()
    t0_sim = node.get_clock().now().nanoseconds * 1e-9
    while time.time() - t0 < (4.0 / SINE_HZ):
        el = node.get_clock().now().nanoseconds * 1e-9 - t0_sim
        off = SINE_AMP_M * math.sin(2 * math.pi * SINE_HZ * el)
        send_pose(pub, node.get_clock(), HOME_P + np.array([off, 0, 0]), HOME_Q)
        rclpy.spin_once(node, timeout_sec=0.005)
        (pn, _), qd = fk_now()
        if pn is not None:
            rows.append((el, pn[0] - HOME_P[0], qd))
        time.sleep(0.004)
    ts = [r[0] - 1.0 / SINE_HZ for r in rows if r[0] >= 1.0 / SINE_HZ]
    ys = [r[1] for r in rows if r[0] >= 1.0 / SINE_HZ]
    vs = [r[2] for r in rows if r[0] >= 1.0 / SINE_HZ]
    if len(ys) > 50:
        w = 2 * math.pi * SINE_HZ
        sc = sum(y * math.sin(w * t) for t, y in zip(ts, ys))
        cc = sum(y * math.cos(w * t) for t, y in zip(ts, ys))
        a_out = 2.0 / len(ts) * math.hypot(sc, cc)
        print("  正弦：幅值比 %.2f（目标 %.0f mm→实测 %.1f mm）| 关节速度峰值 %.2f rad/s"
              % (a_out / SINE_AMP_M, SINE_AMP_M * 1000, a_out * 1000, max(vs)))

    # ── 汇总 ──
    print()
    print("══ [%s] 汇总 ══" % args.label)
    print("  阶跃：整定 %s，稳态差 %.1f mm" % (("%.2f s" % settle_t) if settle_t else "未整定", settled * 1000))
    print("  正弦：幅值比 %.2f" % (a_out / SINE_AMP_M if len(ys) > 50 else float("nan")))
    print("  关节速度峰值 %.2f rad/s（厂商限 2.618/3.142；修复前震荡基线 4.84）" % max(qd_peak, max(vs) if vs else 0))
    ok = (settle_t is not None and settle_t < 1.5 and max(qd_peak, max(vs) if vs else 0) < 2.5)
    print("  判定：%s" % ("✅ 通过（收敛且不超速）" if ok else "❌ 未达标——需要继续调参"))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
