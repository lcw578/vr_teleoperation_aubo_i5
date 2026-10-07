#!/home/lcw/tomato_robot/.venv/bin/python
# -*- coding: utf-8 -*-
# 便捷入口：`bash 本文件` 也能跑——sh 把下一行解析为 source ROS 后 exec 进 venv
# Python；Python 把它解析为无操作字符串。（与 direction_check.py 同款）
''''exec /bin/bash -c ". /opt/ros/humble/setup.bash 2>/dev/null || :; exec /home/lcw/tomato_robot/.venv/bin/python -- \"\$0\" \"\$@\"" "$0" "$@" # '''
"""旋转自检：端到端验证"手柄旋转 → 末端旋转"的世界轴语义（2026-09-28 定案）。

与 direction_check.py 的区别：那个验平移（不按 Grip、纯观察）；本脚本验**旋转**，
必须真实接合（臂会动）。判据来自 2026-09-28 的轴交叉缺陷：
  旧语义（工具系再表达）下手柄 pitch 出末端 roll、随接合姿态漂移；
  新语义（世界轴）下：手柄绕哪根世界轴转 θ → 末端绕同一根世界轴转 θ×缩放。
脚本对手柄与末端各算"世界系旋转增量"（四元数），比对旋转轴（dot→1）与角度比
（→缩放档位）。**与接合姿态无关**是本语义的核心性质——换一个臂姿态重跑一遍，
结果应不变（这是回归测试的关键步骤）。

用法（戴头显、页面 Start Teleop 已开、遥操链在跑）：
  bash scripts/rotation_check.py
  每步提示：按住 Grip 接合 → 静止 1 s → 慢转 ~20° → 停住 2 s → 松开 Grip。
全程只订阅不发布；Ctrl-C 退出。
"""
import math
import sys
import time

import numpy as np
import rclpy
import mujoco
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy

# ---- 夹爪档（TELEOP_GRIPPER=ag95|rg）：EE 体名/抓取点/场景文件随夹爪切换 ----
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from gripper_profile import resolve as _resolve_gripper  # noqa: E402
GRIP = _resolve_gripper()
MJCF_MODEL = GRIP["mjcf"]
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
TIP_OFF = np.array(GRIP["tip_offset"])

GESTURE_START_DEG = 8.0     # 手柄增量超过此角度视为动作开始
HAND_SETTLE_DEG = 1.0       # 手柄静止判定窗内变化
HAND_SETTLE_S = 0.5
EE_SETTLE_DEG = 0.5         # 末端静止判定（等跟踪收敛）
EE_SETTLE_S = 0.5
STAGE_TIMEOUT = 40.0
AXIS_DOT_PASS = 0.80        # 旋转轴平行判据
RATIO_LO, RATIO_HI = 0.30, 0.75   # 角度比（quest 默认缩放 0.5）

STAGES = [
    ("【1/3】按住 Grip → 静止 1 s → 手柄绕竖直轴慢慢转 ~20°（像摇头）→ 停住 2 s → 松开",
     "yaw"),
    ("【2/3】按住 Grip → 静止 1 s → 手柄绕水平左右轴慢慢俯仰 ~20°（像点头）→ 停住 2 s → 松开",
     "pitch"),
    ("【3/3】按住 Grip → 静止 1 s → 手柄绕自身前轴慢慢滚转 ~20°（像拧钥匙）→ 停住 2 s → 松开",
     "roll"),
]


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


def quat_wxyz_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def quat_wxyz_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_wxyz_rotvec(q):
    """[w,x,y,z] → 旋转向量（轴×角，最短路径）。"""
    q = q / max(np.linalg.norm(q), 1e-12)
    if q[0] < 0:
        q = -q
    v = q[1:4]
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.zeros(3)
    angle = 2.0 * math.atan2(n, q[0])
    return v / n * angle


_mj_model = mujoco.MjModel.from_xml_path(MJCF_MODEL)
_mj_data = mujoco.MjData(_mj_model)
_mj_qadr = [_mj_model.jnt_qposadr[mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in GROUP]
_mj_bid = mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_BODY, GRIP["ee_body"])


class RotCheck(Node):
    def __init__(self):
        super().__init__("rotation_check")
        self.hand_q = None        # [w,x,y,z]，臂基座系（adapter 已变换）
        self.hand_t = 0.0
        self.grip = 0
        self.joy_t = 0.0
        self.ee_q = None          # 末端 [w,x,y,z]（/joint_states + FK）
        self.ee_t = 0.0
        self.create_subscription(PoseStamped, "/quest/pose", self._pose, fast_qos())
        self.create_subscription(Joy, "/quest/joy", self._joy, fast_qos())
        self.create_subscription(JointState, "/joint_states", self._js, fast_qos())

    def _pose(self, msg):
        o = msg.pose.orientation
        self.hand_q = np.array([o.w, o.x, o.y, o.z])
        self.hand_t = time.time()

    def _joy(self, msg):
        if len(msg.buttons) >= 1:
            self.grip = int(msg.buttons[0])
            self.joy_t = time.time()

    def _js(self, msg):
        q = {}
        for i, name in enumerate(msg.name):
            if name in GROUP:
                q[name] = msg.position[i]
        if len(q) == len(GROUP):
            for a, j in zip(_mj_qadr, GROUP):
                _mj_data.qpos[a] = q[j]
            mujoco.mj_forward(_mj_model, _mj_data)
            qw = np.zeros(4)
            R = _mj_data.xmat[_mj_bid].reshape(3, 3)
            mujoco.mju_mat2Quat(qw, np.ascontiguousarray(R).flatten())
            self.ee_q = qw
            self.ee_t = time.time()


def fresh(node, which, max_age=0.5):
    t = node.hand_t if which == "hand" else node.ee_t
    v = node.hand_q if which == "hand" else node.ee_q
    return v is not None and time.time() - t < max_age


def wait_grip(node, want, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.grip == want and time.time() - node.joy_t < 0.5:
            return True
    return False


def settled(delta_deg_now, delta_deg_hist, eps_deg, win_s):
    """delta_deg_hist: list of (t, deg)。近 win_s 内变化 < eps_deg 即静止。"""
    now = time.time()
    recent = [d for t, d in delta_deg_hist if now - t <= win_s]
    return recent and (max(recent) - min(recent)) < eps_deg


def run_stage(node, hint):
    print("\n%s" % hint)
    # ── 等接合 ──
    t0 = time.time()
    while not (node.grip == 1 and fresh(node, "hand") and fresh(node, "ee")):
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.time() - t0 > STAGE_TIMEOUT:
            print("   ❌ 等待接合超时（Grip 是否按住？数据是否新鲜？）")
            return None
    q_h0 = node.hand_q.copy()
    q_e0 = node.ee_q.copy()
    time.sleep(1.0)                       # 静止 1 s（锚点稳定）
    rclpy.spin_once(node, timeout_sec=0.0)
    q_h0 = node.hand_q.copy()
    q_e0 = node.ee_q.copy()
    # ── 等动作：手柄世界增量 ──
    t0 = time.time()
    hist = []
    while time.time() - t0 < STAGE_TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.02)
        if not fresh(node, "hand"):
            continue
        d = quat_wxyz_rotvec(quat_wxyz_mul(node.hand_q, quat_wxyz_conj(q_h0)))
        deg = float(np.degrees(np.linalg.norm(d)))
        hist.append((time.time(), deg))
        if deg > GESTURE_START_DEG and settled(deg, hist, HAND_SETTLE_DEG, HAND_SETTLE_S):
            break
    else:
        print("   ❌ 未检测到手柄旋转")
        return None
    d_h = d.copy()
    # ── 等末端收敛（手保持不动，等跟踪到位）──
    t0 = time.time()
    hist_e = []
    d_e = None
    while time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.02)
        if not fresh(node, "ee") or not fresh(node, "hand"):
            continue
        d = quat_wxyz_rotvec(quat_wxyz_mul(node.ee_q, quat_wxyz_conj(q_e0)))
        deg = float(np.degrees(np.linalg.norm(d)))
        hist_e.append((time.time(), deg))
        d_e = d.copy()
        if settled(deg, hist_e, EE_SETTLE_DEG, EE_SETTLE_S):
            break
    if d_e is None:
        print("   ❌ 末端数据中断")
        return None
    # ── 等松开 ──
    wait_grip(node, 0, 15.0)
    return d_h, d_e


def main():
    rclpy.init()
    node = RotCheck()
    print("=" * 68)
    print("旋转自检（世界轴语义：手柄绕哪根世界轴转 → 末端绕同一根轴转×缩放）")
    print("注意：臂会真实跟随！动作要慢要小（~20°）。与接合姿态无关——")
    print("换一个臂姿态再跑一遍，结果应完全一致（本语义的回归判据）。")
    print("=" * 68)
    t0 = time.time()
    while not (fresh(node, "hand") and fresh(node, "ee")):
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - t0 > 15:
            print("❌ 没有手柄/关节数据——quest_adapter、遥操链是否在跑？")
            return 1
    print("数据正常。三步都会真实驱动机械臂。\n")

    results = []
    for hint, name in STAGES:
        out = run_stage(node, hint)
        if out is None:
            results.append((name, False, "数据中断/超时"))
            continue
        d_h, d_e = out
        a_h = d_h / max(np.linalg.norm(d_h), 1e-9)
        a_e = d_e / max(np.linalg.norm(d_e), 1e-9)
        # 诊断：手柄实际旋转轴偏离世界竖直轴多少（判"摇头却点头"是手势轴斜还是 bug）
        tilt_h = math.degrees(math.acos(min(1.0, abs(float(a_h[2])))))
        tilt_e = math.degrees(math.acos(min(1.0, abs(float(a_e[2])))))
        print("   [诊断] 手柄旋转轴偏离竖直 %.0f°（轴=%s）| 末端旋转轴偏离竖直 %.0f°"
              % (tilt_h, np.round(a_h, 2).tolist(), tilt_e))
        dot = float(np.dot(a_h, a_e))
        ang_h = float(np.degrees(np.linalg.norm(d_h)))
        ang_e = float(np.degrees(np.linalg.norm(d_e)))
        ratio = ang_e / max(ang_h, 1e-9)
        ok_axis = dot > AXIS_DOT_PASS
        ok_ratio = RATIO_LO <= ratio <= RATIO_HI
        ok = ok_axis and ok_ratio
        why = []
        why.append("轴 dot=%+.2f %s" % (dot, "✓" if ok_axis else "✗ 轴不平行=交叉!"))
        if not ok_ratio and 0.05 <= ratio <= 0.15:
            why.append("角比=%.2f ✗ —— ≈0.1：你可能误触 A 键进了 1:10 微调档，"
                       "按一次 A 回常规档再重跑本步" % ratio)
        else:
            why.append("角比=%.2f %s(期望≈0.5×缩放)" % (ratio, "✓" if ok_ratio else "✗"))
        results.append((name, ok, "；".join(why)))
        print("   手柄转 %.1f° → 末端转 %.1f° | %s" % (ang_h, ang_e, "；".join(why)))

    print("\n" + "=" * 68)
    n_ok = sum(1 for _, ok, _ in results if ok)
    for name, ok, why in results:
        print("  %s %s —— %s" % ("✓" if ok else "✗", name, why))
    print("%d/3 通过" % n_ok)
    if n_ok == 3:
        print("旋转映射符合世界轴语义。建议：把臂挪到另一个姿态（重新接合）再跑")
        print("一遍——结果应不变，即'与接合姿态无关'成立。")
    else:
        print("✗ 存在轴不平行或比例异常——把本输出原样发给排查。")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if n_ok == 3 else 1


if __name__ == "__main__":
    sys.exit(main())
