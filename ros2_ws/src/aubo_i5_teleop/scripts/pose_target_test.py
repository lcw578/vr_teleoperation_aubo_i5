#!/usr/bin/env python3
"""阶段 3（pose tracking）：仿真里**代替 VR 手柄**发目标位姿，并测量跟随。

为什么先要这个脚本：手柄还没到，但"机械臂能不能跟随目标位姿"这件事现在就能验证。
将来把手柄接上时，只需把"目标位姿从哪来"从本脚本换成 oculus_reader 的映射层
（_refs/oculus_reader/src/pose_teleop.py 的那套相对参考帧做法），下游完全不变。

它做的事，与 pose_teleop.py 的对应关系：
    pose_teleop.py: 查 world->teleop_link 得到末端位姿 -> 叠加手柄位移 -> 发 PoseStamped
    本脚本:         查 world->gripper_tip_link 得到末端位姿 -> 叠加一个匀速位移/旋转 -> 发 PoseStamped
即本脚本就是"手柄恒速移动"这一最简单情形的替身。

测量方式：只用 TF（world -> gripper_tip_link），不用 MJCF 正运动学——
测的必须是控制器/伺服看到的那条链路，才不会被另一套模型的口径差异污染。

三种用法：
  单轴跟随（看完成度与稳态滞后）：
      python3 scripts/pose_target_test.py --axis z --speed 0.02 --duration 4
  阶段 3 验收 · 三个平移方向逐一验证（方案文档 §B4 的要求）：
      python3 scripts/pose_target_test.py --verify-translations
  阶段 3 验收 · 三个旋转轴逐一验证：
      python3 scripts/pose_target_test.py --verify-rotations

判据（与早期 servo_twist_test.py 一致，便于前后对比）：
  平移：沿命令方向的位移 > 0.5 mm，且大于垂直分量的 2 倍；
  旋转：绕命令轴的转角 > 0.005 rad，且大于垂直分量的 2 倍。
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

TARGET_TOPIC = "/target_pose"
EE_FRAME = "gripper_tip_link"
WORLD = "world"
AXES = ["x", "y", "z"]


# ---------- 四元数小工具（避免引入 tf_transformations 依赖）----------
def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_axis_angle(axis_idx, angle):
    half = angle / 2.0
    s = math.sin(half)
    q = [0.0, 0.0, 0.0, math.cos(half)]
    q[axis_idx] = s
    return tuple(q)


def q_to_R(q):
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def q_inv(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_rel(a, b):
    """a 相对 b 的旋转（同一参照系下），返回旋转向量。"""
    return R_to_rotvec(q_to_R(q_mul(a, q_inv(b))))


def R_to_rotvec(R):
    """旋转矩阵 -> 旋转向量（轴角），用于比较旋转方向。"""
    c = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-9:
        return (0.0, 0.0, 0.0)
    axis = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]]
    n = math.sqrt(sum(v * v for v in axis))
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return tuple(v / n * ang for v in axis)


class PoseTargetTest(Node):
    def __init__(self, rate_hz):
        super().__init__("pose_target_test")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 1)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.period = 1.0 / rate_hz

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def ee_pose(self):
        try:
            tf = self.buffer.lookup_transform(WORLD, EE_FRAME, Time())
        except Exception:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def publish_target(self, position, quat):
        m = PoseStamped()
        m.header.frame_id = WORLD
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = position
        m.pose.orientation.x, m.pose.orientation.y = quat[0], quat[1]
        m.pose.orientation.z, m.pose.orientation.w = quat[2], quat[3]
        self.pub.publish(m)


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def run_case(node, axis_idx, is_rot, value, duration, hold, verbose=False):
    """发一段匀速目标（平移或旋转），返回 (命令位移/转角, 实际位移/转角, vector)。"""
    node.spin(1.2)
    start = node.ee_pose()
    if start is None:
        return None
    p0, q0 = start
    target_p = list(p0)
    errors = []

    t0 = time.time()
    n = 0
    while True:
        t = time.time() - t0
        if t >= duration:
            break
        if is_rot:
            # 注意顺序：q_aa ⊗ q0 表示绕**世界系**给定轴旋转（左乘）。
            # 若写成 q0 ⊗ q_aa 则是绕末端机体系轴旋转——那样在大滞后时世界系下的转轴会偏，
            # 判据会把"轴偏"和"没转够"混在一起（实测 ±y 就是这样误判的）。
            q_t = q_mul(q_axis_angle(axis_idx, value * t), q0)
        else:
            target_p[axis_idx] = p0[axis_idx] + value * t
            q_t = q0
        node.publish_target(target_p, q_t)
        cur = node.ee_pose()
        if cur is not None:
            errors.append(dist(target_p, cur[0]))
            if verbose and n % 12 == 0:
                print("      t=%.1fs 目标 %+.4f 实际 %+.4f 误差 %.4f" %
                      (t, target_p[axis_idx], cur[0][axis_idx], errors[-1]))
        n += 1
        node.spin(node.period)

    cmd_p, cmd_q = list(target_p), q_t
    t1 = time.time()
    while time.time() - t1 < hold:
        node.publish_target(cmd_p, cmd_q)
        node.spin(node.period)

    end = node.ee_pose()
    if end is None:
        return None
    p1, q1 = end
    if is_rot:
        # 两侧都用"相对起点的旋转"比较，避免受四元数左右乘（世界系/机体系）的影响
        v = q_rel(q1, q0)
        cmd_v = q_rel(cmd_q, q0)
    else:
        v = tuple(a - b for a, b in zip(p1, p0))
        cmd_v = tuple(c - b for c, b in zip(cmd_p, p0))
    return cmd_v, v, errors


def verdict(cmd_v, v, is_rot):
    n = math.sqrt(sum(c * c for c in cmd_v))
    if n < 1e-12:
        return False, 0.0, 0.0
    u = [c / n for c in cmd_v]
    along = sum(x * y for x, y in zip(v, u))
    perp = math.sqrt(max(0.0, sum(x * x for x in v) - along * along))
    thr = 0.005 if is_rot else 5e-4
    return (abs(along) > thr and abs(along) > 2 * perp), along, perp


def step_response(node, axis_idx, size, duration, sample_dt=0.005):
    """给目标位姿一个阶跃，记录末端实际响应，用来表征外环动态。

    返回 (t, pos) 序列（pos 为该轴相对起点的位移）。用于提取：
    延迟、上升时间、超时量、稳态误差，以及等效时间常数 τ ≈ 上升时间/2.2。
    理论预期：P-only 外环的 τ ≈ 1/k_p（k_p=20 → 50 ms），这是"跟得多紧"的直接度量。
    """
    node.spin(1.2)
    start = node.ee_pose()
    if start is None:
        return None
    p0, q0 = start
    target = list(p0)
    target[axis_idx] = p0[axis_idx] + size

    ts, xs = [], []
    t0 = time.time()
    while time.time() - t0 < duration:
        node.publish_target(target, q0)
        cur = node.ee_pose()
        if cur is not None:
            ts.append(time.time() - t0)
            xs.append(cur[0][axis_idx] - p0[axis_idx])
        node.spin(sample_dt)
    return ts, xs


def analyze_step(ts, xs, size):
    if not ts:
        return None
    a = abs(size)
    def first_above(frac):
        thr = frac * a
        for t, x in zip(ts, xs):
            if (size > 0 and x >= thr) or (size < 0 and x <= -thr):
                return t
        return None
    t10, t90 = first_above(0.1), first_above(0.9)
    peak = max(xs) if size > 0 else min(xs)
    overshoot = (peak / size - 1.0) * 100.0
    final = xs[-1]
    return {
        "delay_10": t10, "t90": t90,
        "rise": (t90 - t10) if (t10 is not None and t90 is not None) else None,
        "tau_est": ((t90 - t10) / 2.2) if (t10 is not None and t90 is not None) else None,
        "overshoot_pct": overshoot, "final_err": abs(final - size),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", default="z", choices=AXES)
    ap.add_argument("--speed", type=float, default=0.02, help="平移速度 [m/s]")
    ap.add_argument("--rot-speed", type=float, default=0.15, help="旋转角速度 [rad/s]")
    ap.add_argument("--duration", type=float, default=4.0)
    ap.add_argument("--hold", type=float, default=1.0)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--verify-translations", action="store_true")
    ap.add_argument("--verify-rotations", action="store_true")
    ap.add_argument("--step", type=float, default=None,
                    help="阶跃响应模式：给目标位姿一个该大小的阶跃（米），记录实际响应对应动态")
    ap.add_argument("--step-duration", type=float, default=1.5)
    args = ap.parse_args()

    rclpy.init()
    node = PoseTargetTest(args.rate)
    try:
        node.spin(2.0)
        if node.ee_pose() is None:
            print("  ❌ 取不到 TF %s -> %s —— stage2a（robot_state_publisher）起了吗？" % (WORLD, EE_FRAME))
            return 1

        results = []
        if args.step is not None:
            r = step_response(node, AXES.index(args.axis), args.step, args.step_duration)
            if r is None:
                print("  ❌ 取不到位姿")
                return 1
            ts, xs = r
            a = analyze_step(ts, xs, args.step)
            print("  === %s 轴 阶跃响应：目标跳变 %+.4f m，采样 %d 点（%.1f Hz 量级）==="
                  % (args.axis, args.step, len(ts), len(ts) / max(1e-9, ts[-1])))
            print("  延迟(到 10%%) = %s s" % ("%.3f" % a["delay_10"] if a["delay_10"] is not None else "未达到"))
            if a["rise"] is not None:
                print("  上升时间(10→90%%) = %.3f s" % a["rise"])
                print("  等效时间常数 τ ≈ 上升/2.2 = %.3f s   ← 与理论 1/k_p 对比" % a["tau_est"])
            print("  超调量 = %+.1f%%" % a["overshoot_pct"])
            print("  稳态误差 = %.4f m（%.1f%% of 阶跃）" % (a["final_err"], 100 * a["final_err"] / abs(args.step)))
            # 打印一条粗略的响应轨迹，便于人工判读是否振荡
            step_n = max(1, len(ts) // 12)
            print("  轨迹(每 %d 点取一点): %s" % (step_n,
                  " ".join("%.3f" % x for x in xs[::step_n][:14])))
            return 0

        if args.verify_translations or args.verify_rotations:
            is_rot = args.verify_rotations
            val = args.rot_speed if is_rot else args.speed
            kind = "旋转" if is_rot else "平移"
            unit = "rad" if is_rot else "m"
            print("  === 三个%s方向逐一验证（每方向 %.1f s，%.3f %s/s）===" %
                  ("旋转轴" if is_rot else "平移", args.duration, val, unit + ("/s" if not is_rot else "")))
            for ax in AXES:
                for sign in (+1, -1):
                    label = "%s%s" % ("+" if sign > 0 else "-", ax)
                    r = run_case(node, AXES.index(ax), is_rot, sign * val, args.duration, args.hold)
                    if r is None:
                        print("  %-4s ❌ 取不到位姿" % label)
                        results.append(False)
                        continue
                    cmd_v, v, errs = r
                    ok, along, perp = verdict(cmd_v, v, is_rot)
                    print("  %-4s 指令 %+.4f %s | 实际沿命令方向 %+.4f %s | 垂直分量 %.4f | 最大误差 %.4f m  %s"
                          % (label, cmd_v[0] + cmd_v[1] + cmd_v[2], unit, along, unit, perp,
                             max(errs) if errs else float("nan"), "✅" if ok else "❌"))
                    results.append(ok)
            passed = sum(1 for x in results if x)
            print()
            print("  结果: %d/%d 通过  %s" % (passed, len(results),
                  "✅ 全部方向正确" if passed == len(results) else "❌ 有方向不对"))
            return 0 if passed == len(results) else 1

        # 单轴跟随
        r = run_case(node, AXES.index(args.axis), False, args.speed, args.duration, args.hold, args.verbose)
        if r is None:
            print("  ❌ 取不到位姿")
            return 1
        cmd_v, v, errs = r
        print("  指令位移 = %s" % [round(x, 4) for x in cmd_v])
        print("  实际位移 = %s" % [round(x, 4) for x in v])
        print("  沿 %s 方向 指令 %+.4f m / 实际 %+.4f m（完成度 %.0f%%）" %
              (args.axis, cmd_v[AXES.index(args.axis)], v[AXES.index(args.axis)],
               100.0 * v[AXES.index(args.axis)] / cmd_v[AXES.index(args.axis)]
               if abs(cmd_v[AXES.index(args.axis)]) > 1e-9 else float("nan")))
        if errs:
            tail = errs[len(errs) * 2 // 3:]
            print("  跟随误差 全程均值 %.4f m | 最大 %.4f m | 末段均值 %.4f m"
                  % (sum(errs) / len(errs), max(errs), sum(tail) / len(tail)))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
