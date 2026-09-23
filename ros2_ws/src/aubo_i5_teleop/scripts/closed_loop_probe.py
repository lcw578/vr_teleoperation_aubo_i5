#!/usr/bin/env python3
"""闭环（pose tracking）阶跃的**完整分解**：命令侧 / 执行侧 / Servo 状态码。

背景：闭环阶跃实测 τ≈0.15 s（10→90% 上升 0.35 s），而两侧都已单独排掉：
  * 外环 P（20 vs 100）→ τ 0.212 → 0.169，几乎不动；
  * 滤波系数（30 vs 1.5，按源码是 2/(c+2) 的**增益**变化：1/16 vs 0.57）→ 0.169 → 0.158；
  * **被控对象自身**（scripts/plant_velocity_step.py，直接给速度控制器发阶跃）
    → τ ≈ 0.010-0.015 s，比值 1.000，无超调 —— 被控对象很快。
所以剩下的嫌疑只在 Servo 的"笛卡尔→关节"这一段，而这一段会**同时**受三样东西影响：
  ① 平滑滤波器的暂态；② 奇异/碰撞的速度缩放；③ 关节速度/位置限幅（enforce_limits.cpp）。
只看末端轨迹分不出这三者。本脚本把**它们各自的输出**都记下来：
  - /forward_command_controller_velocity/commands  ← Servo 实际发出的关节速度命令
  - /joint_states（depth=1 BEST_EFFORT，只取最新样本）← 实际执行
  - /servo_pose_tracking/status（std_msgs/Int8）   ← 0 无警告 / 1、6 奇异缩放 / 3 碰撞缩放 / 5 关节到界
  - 关节速度上限从 /robot_description 里**读出来**（不手抄），用来判断"命令是否正好停在限幅上"

用法（Servo 停止时先归位）：
    ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py     # 终端 1
    ros2 launch aubo_i5_teleop stage2b_moveit.launch.py     # 终端 2
    python3 scripts/go_ready.py --mode velocity             # 终端 3（此时还没有 Servo）
    ros2 launch aubo_i5_teleop stage3_pose_tracking.launch.py   # 终端 4
    python3 scripts/closed_loop_probe.py --step-z 0.05      # 终端 5
"""

import argparse
import re
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int8
from std_msgs.msg import String
import tf2_ros

CMD_TOPIC = "/forward_command_controller_velocity/commands"
STATUS_TOPIC = "/servo_pose_tracking/status"
TARGET_TOPIC = "/target_pose"
WORLD = "world"
EE_FRAME = "gripper_tip_link"
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]

STATUS_MEANING = {0: "无警告", 1: "接近奇异-减速", 2: "奇异-急停", 3: "接近碰撞-减速",
                  4: "碰撞-急停", 5: "关节到界-停", 6: "离开奇异-减速"}


def fast_qos():
    # 仪器纪律：depth=1 + BEST_EFFORT，只取**最新**样本。
    # 默认 RELIABLE+depth 几十会在 200 Hz 下积压成 0.2-0.5 s 的旧数据（踩过）。
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


class Probe(Node):
    def __init__(self):
        super().__init__("closed_loop_probe")
        self.cmd = None
        self.cmd_t = None
        self.q = {}
        self.q_t = None
        self.status = None
        self.status_hist = {}
        self.status_log = []
        self.cmd_log = []
        self.js_log = []
        self.limits = {}
        self.n_js = 0

        qos = fast_qos()
        self.create_subscription(JointState, "/joint_states", self._on_js, qos)
        self.create_subscription(Float64MultiArray, CMD_TOPIC, self._on_cmd, qos)
        self.create_subscription(Int8, STATUS_TOPIC, self._on_status, qos)
        desc_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/robot_description", self._on_desc, desc_qos)
        self.pub = self.create_publisher(PoseStamped, TARGET_TOPIC, 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def _on_desc(self, msg):
        # 从 URDF 里读关节速度上限（<limit ... velocity="x"/>）
        for m in re.finditer(r'<joint name="([^"]+)"[^>]*>(.*?)</joint>', msg.data, re.S):
            name, body = m.group(1), m.group(2)
            lm = re.search(r'<limit[^>]*velocity="([-0-9.eE]+)"', body)
            if lm and name in GROUP:
                self.limits[name] = abs(float(lm.group(1)))

    def _on_js(self, msg):
        self.n_js += 1
        for i, name in enumerate(msg.name):
            if name in GROUP:
                self.q[name] = (msg.position[i], msg.velocity[i])
        self.q_t = time.time()
        self.js_log.append((self.q_t, [self.q.get(k, (float("nan"), float("nan")))[1] for k in GROUP]))

    def _on_cmd(self, msg):
        self.cmd = list(msg.data)
        self.cmd_t = time.time()
        self.cmd_log.append((self.cmd_t, list(msg.data)))

    def _on_status(self, msg):
        self.status = int(msg.data)
        self.status_hist[self.status] = self.status_hist.get(self.status, 0) + 1
        self.status_log.append((time.time(), int(msg.data)))

    def ee(self):
        """返回 (z, quaternion)。**姿态必须一起用**：只发位置、姿态留 identity 的话，
        目标里会隐含一个大的旋转，角速度分量会污染 z 阶跃的测量。"""
        try:
            tf = self.buffer.lookup_transform(WORLD, EE_FRAME, Time())
        except Exception:
            return None
        t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        tr = tf.transform.translation
        return (t, (tr.x, tr.y, tr.z), tf.transform.rotation)

    def spin(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.002)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step-z", type=float, default=0.05, help="+z 阶跃幅值 [m]")
    ap.add_argument("--dur", type=float, default=3.0, help="发目标位姿的时长 [s]")
    ap.add_argument("--settle", type=float, default=1.0, help="阶跃前保持现位的时长 [s]")
    ap.add_argument("--hold", type=float, default=1.5, help="阶跃后继续发目标的时长 [s]（用于看稳态）")
    ap.add_argument("--label", default="", help="写进 RESULT 行的标签，便于扫描对比")
    args = ap.parse_args()

    rclpy.init()
    node = Probe()
    try:
        # ⚠️ 必须用仿真时钟给目标位姿打时间戳：PoseTracking::haveRecentTargetPose 用
        #    (node_->now() - target_pose_.header.stamp) 判断新鲜度，而 targetPoseCallback
        #    只有在 frame_id != planning_frame 时才把 stamp 改成 now()。
        #    我们的 frame_id 就是 planning_frame（world），所以**发送方必须自己打戳**，
        #    否则 stamp=0 → 目标永远"不新鲜" → 每次 moveToPose 立刻 abort
        #    （实测报错：The target pose was not updated recently. Aborting.）
        node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        node.spin(2.0)
        if not node.q:
            print("❌ 收不到 /joint_states —— stage2a 起了吗？")
            return 1
        ee0 = None
        t_wait = time.time()
        while ee0 is None and time.time() - t_wait < 5.0:
            node.spin(0.2)
            ee0 = node.ee()
        if ee0 is None:
            print("❌ TF 里拿不到 %s → %s 的变换" % (WORLD, EE_FRAME))
            return 1
        p0 = ee0[1]      # (x, y, z) —— 必须用**完整**位置做基准
        z0 = p0[2]
        q0 = ee0[2]
        print("起始末端 = (%.4f, %.4f, %.4f)，只对 z 加 +%.3f m → %.4f"
              % (p0[0], p0[1], z0, args.step_z, z0 + args.step_z))
        print("起始姿态 = (%.4f, %.4f, %.4f, %.4f)  ← 一起作为目标发出去，保证是纯平移"
              % (q0.x, q0.y, q0.z, q0.w))
        print("关节速度上限（从 /robot_description 读出）: %s" %
              {k: round(v, 3) for k, v in node.limits.items()})
        if node.cmd_t is None:
            print("⚠️  还没收到 %s —— 闭环没在跑？" % CMD_TOPIC)

        tgt = PoseStamped()
        tgt.header.frame_id = WORLD
        tgt.pose.position.x = p0[0]
        tgt.pose.position.y = p0[1]
        tgt.pose.position.z = p0[2]
        tgt.pose.orientation = q0

        ee_trace = []
        t0 = time.time()
        # 阶段 0：保持当前位姿（让 Servo 有目标、不要走 timeout 分支）
        ee_now = node.ee()
        while time.time() - t0 < args.settle:
            node.spin(0.02)
            ee_now = node.ee() or ee_now
            tgt.pose.position.x = ee_now[1][0]
            tgt.pose.position.y = ee_now[1][1]
            tgt.pose.position.z = ee_now[1][2]
            tgt.pose.orientation = ee_now[2]
            tgt.header.stamp = node.get_clock().now().to_msg()
            node.pub.publish(tgt)
            ee_trace.append((time.time(), ee_now[1][2]))
        t_step = time.time()
        print("── 阶跃时刻 t=%.3f（目标 z=%.4f）──" % (t_step - t0, z0 + args.step_z))
        # 阶段 1：阶跃到 z0+step（持续发，target_pose 需要持续更新）
        while time.time() - t_step < args.dur:
            node.spin(0.02)
            tgt.pose.position.x = p0[0]
            tgt.pose.position.y = p0[1]
            tgt.pose.position.z = z0 + args.step_z
            tgt.header.stamp = node.get_clock().now().to_msg()
            node.pub.publish(tgt)
            e = node.ee()
            if e:
                ee_trace.append((time.time(), e[1][2]))
        # 阶段 2：保持新目标 1 s（Servo 不至于 timeout 停住）
        t_hold = time.time()
        while time.time() - t_hold < args.hold:
            node.spin(0.02)
            tgt.header.stamp = node.get_clock().now().to_msg()
            node.pub.publish(tgt)
            e = node.ee()
            if e:
                ee_trace.append((time.time(), e[1][2]))

        # ─────────── 分析 ───────────
        print()
        print("══ 1. 末端 z（TF，含 TF 自身的传输延迟；只看上升形状）══")
        step_vals = [z for (t, z) in ee_trace if t >= t_step]
        if step_vals:
            z_final = sum(step_vals[-40:]) / len(step_vals[-40:])
            print("   稳态 z = %.4f（起点 %.4f，位移 %.4f m，目标 %.4f）"
                  % (z_final, z0, z_final - z0, args.step_z))
            marks = {}
            for frac, lab in ((0.1, "10%"), (0.632, "63%"), (0.9, "90%")):
                hit = next((t for (t, z) in ee_trace if t >= t_step and z >= z0 + frac * args.step_z), None)
                marks[lab] = None if hit is None else hit - t_step
            print("   到 10%% = %s  到 63%% = %s  到 90%% = %s" % tuple(
                "未达" if marks[k] is None else "%.3f s" % marks[k] for k in ("10%", "63%", "90%")))
            if marks["10%"] is not None and marks["90%"] is not None:
                rise = marks["90%"] - marks["10%"]
                print("   上升(10→90%%) = %.3f s   τ≈上升/2.2 = %.3f s" % (rise, rise / 2.2))

        print()
        print("══ 2. Servo 发出的关节速度命令（命令侧，depth=1 无积压）══")
        step_cmds = [(t, c) for (t, c) in node.cmd_log if t >= t_step]
        if step_cmds:
            peak = [0.0] * len(GROUP)
            for _, c in step_cmds:
                for i in range(min(len(c), len(GROUP))):
                    peak[i] = max(peak[i], abs(c[i]))
            print("   各关节 |命令| 峰值 (rad/s) 与上限:")
            for i, j in enumerate(GROUP):
                lim = node.limits.get(j, float("nan"))
                pinned = "  ← 正好压在上限！" if lim == lim and abs(peak[i] - lim) < 1e-3 else ""
                print("     %-16s 峰值 %.4f   上限 %.4f%s" % (j, peak[i], lim, pinned))
            print("   命令范数（6 关节合成）随时间:")
            for (t, c) in step_cmds:
                if int((t - t_step) * 1000) % 100 < 8:
                    norm = sum(x * x for x in c) ** 0.5
                    print("     +%.3f s  |qdot_cmd| = %.4f   args=%s"
                          % (t - t_step, norm, [round(x, 3) for x in c[:3]]))
            t_last = step_cmds[-1][0]
            tail = [sum(x * x for x in c) ** 0.5 for (t, c) in step_cmds if t >= t_last - 0.5]
            print("   末尾 0.5 s 平均 |qdot_cmd| = %.4f rad/s" % (sum(tail) / len(tail)))
        else:
            print("   ❌ 阶跃期间没有收到命令 —— Servo 没在发命令？")

        print()
        print("══ 3. 执行侧：实测 |qdot| 与命令的比（应≈1，被控对象已单独测过 τ≈0.01 s）══")
        step_js = [(t, v) for (t, v) in node.js_log if t >= t_step]
        if step_js and step_cmds:
            # 用时间最近邻配对
            import bisect
            ct = [t for (t, _) in node.cmd_log]
            ratios = []
            for (t, v) in step_js:
                i = bisect.bisect_left(ct, t)
                if i >= len(ct):
                    continue
                c = node.cmd_log[i][1]
                for k in range(min(len(v), len(c))):
                    if abs(c[k]) > 0.02:
                        ratios.append(v[k] / c[k])
            if ratios:
                ratios.sort()
                print("   实测/命令 的关节速度比值: 中位数 %.3f  10-90 分位 [%.3f, %.3f]  n=%d"
                      % (ratios[len(ratios) // 2], ratios[int(0.1 * len(ratios))],
                         ratios[int(0.9 * len(ratios))], len(ratios)))

        print()
        print("══ 4. Servo 状态码（0 无警告 / 1 奇异减速 / 3 碰撞减速 / 5 关节到界）══")
        print("   直方图: %s" % {k: (v, STATUS_MEANING.get(k, "?")) for k, v in sorted(node.status_hist.items())})
        changes = []
        last = None
        for (t, s) in node.status_log:
            if s != last:
                changes.append((t - t_step, s))
                last = s
        print("   变化序列（相对阶跃时刻）: %s" % [(round(dt, 3), s) for dt, s in changes[:12]])

        print()
        print("══ 5. 稳态（最后 1.5 s）：**这是遥操能不能用的判据** —— 人不动时机械臂不能抖 ══")
        t_s = t_hold + max(0.0, args.hold - 1.5)   # 保持阶段的最后 1.5 s
        tail_ee = [z for (t, z) in ee_trace if t >= t_s]
        tail_cmd = [(t, c) for (t, c) in node.cmd_log if t >= t_s]
        tail_status = [s for (t, s) in node.status_log if t >= t_s]
        p2p = (max(tail_ee) - min(tail_ee)) if tail_ee else float("nan")
        if tail_cmd:
            norms = [sum(x * x for x in c) ** 0.5 for _, c in tail_cmd]
            busy = sum(1 for n in norms if n > 0.5) / len(norms)
            rms = (sum(n * n for n in norms) / len(norms)) ** 0.5
        else:
            norms, busy, rms = [], float("nan"), float("nan")
        err = (tail_ee[-1] - (z0 + args.step_z)) if tail_ee else float("nan")
        print("   末端 z 峰峰值 = %.4f m（%s）" % (p2p, "抖动" if p2p > 0.002 else "静止"))
        print("   |qdot_cmd| RMS = %.3f rad/s，>0.5 rad/s 的样本占 %.0f%%" % (rms, 100 * busy))
        print("   稳态 z 误差 = %+.4f m ；status 集合 = %s" % (err, sorted(set(tail_status))))
        print()
        print("RESULT %s p2p_ee=%.5f rms_cmd=%.3f busy=%.2f err=%.5f rise=%s"
              % (args.label or "?", p2p, rms, busy, err,
                 ("%.3f" % (marks["90%"] - marks["10%"])) if marks.get("10%") is not None
                 and marks.get("90%") is not None else "nan"))

        with open("/tmp/closed_loop_trace.csv", "w") as f:
            f.write("t_rel,kind,value\n")
            for (t, c) in node.cmd_log:
                f.write("%.6f,cmd_norm,%.6f\n" % (t - t_step, sum(x * x for x in c) ** 0.5))
            for (t, v) in node.js_log:
                f.write("%.6f,jsdot_norm,%.6f\n" % (t - t_step, sum(x * x for x in v) ** 0.5))
            for (t, z) in ee_trace:
                f.write("%.6f,ee_z,%.6f\n" % (t - t_step, z))
            for (t, s) in node.status_log:
                f.write("%.6f,status,%d\n" % (t - t_step, s))
        print("\n原始轨迹已存 /tmp/closed_loop_trace.csv")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
