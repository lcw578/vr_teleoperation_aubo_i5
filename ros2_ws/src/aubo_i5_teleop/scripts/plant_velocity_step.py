#!/usr/bin/env python3
"""测**被控对象自身**（MuJoCo + mujoco_ros2_control + 速度控制器）的速度阶跃响应。

为什么需要这个工具：
  闭环（pose tracking）的阶跃 τ 实测 ≈0.15 s，且对两个东西都不敏感——
  外环 P 从 20 到 100（τ 0.212→0.169）、滤波系数从 30 到 1.5（τ 0.169→0.158）。
  一个"外环增益改变 9 倍而 τ 几乎不动"的系统，瓶颈通常不在外环，而在**内环有一个
  速率/滞后限制**。Servo 那条链路已经单独量过了（开环 twist 路径），所以这里量剩下的
  那一半：**只往速度控制器发阶跃，不经 Servo、不经 MoveIt、不经 PID**。

  如果这个阶跃本身就慢（τ≈0.15 s），那么闭环 τ 就是被控对象的速度环滞后，外环再怎么
  调都动不了它；如果它很快（几十 ms），那瓶颈就还在 Servo 侧。

用法：
    # 先起 stage2a（速度模式是当前默认），Servo 不要起
    ros2 launch aubo_i5_teleop stage2a_mujoco.launch.py
    python3 scripts/go_ready.py --mode velocity
    python3 scripts/plant_velocity_step.py --joint upperArm_joint --amp 0.2

注意：
  * 必须在 Servo 停止时跑（Servo 会同时往同一个命令话题发"保持位姿"的命令）。
  * 只对**一个**关节发阶跃，其余关节发 0；结束后发 0 停住。
  * amp×dur = 关节转过的角度，默认 0.2 rad/s×1.0 s = 0.2 rad ≈ 11°，安全。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

CMD_TOPIC = "/forward_command_controller_velocity/commands"
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]


class PlantStep(Node):
    def __init__(self):
        super().__init__("plant_velocity_step")
        self.js = {}
        self.pub = self.create_publisher(Float64MultiArray, CMD_TOPIC, 10)
        # ⚠️ QoS 是**仪器**的一部分，踩过一次坑：默认 RELIABLE + depth 50 在 200 Hz 下
        #    等于 0.25 s 的积压；订户跟不上时读到的是**排队中的旧样本**（实测滞后 0.315 s，
        #    而位置通道因为积分正确看不出来）。必须用 depth=1 + BEST_EFFORT 只取最新样本。
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, "/joint_states", self._on_js, qos)
        self.n_js = 0
        self.js_times = []
        self.js_series = []
        # (t, cmd, q, qdot) 按发布顺序记录
        self.trace = []

    def _on_js(self, msg):
        self.n_js += 1
        self.js_times.append(time.time())
        for i, name in enumerate(msg.name):
            if name in GROUP:
                self.js[name] = (msg.position[i], msg.velocity[i], time.time())
                if hasattr(self, "joint") and name == self.joint:
                    self.js_series.append((time.time(), msg.position[i], msg.velocity[i]))

    def spin(self, seconds):
        """转指定时长（rclpy 的 Node 没有 spin(秒)，这里自己实现）。"""
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.002)

    def hold(self, seconds, cmd):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.pub.publish(Float64MultiArray(data=list(cmd)))
            self.sample(cmd)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / 200.0)  # 控制器 update_rate = 200 Hz

    def sample(self, cmd):
        q = self.js.get(self.joint, (float("nan"), float("nan"), 0.0))
        self.trace.append((time.time(), cmd[self.idx], q[0], q[1], q[2]))


def analyze(series, amp, t_step, t_end_step, label):
    """series = [(t, q, qdot)]，只用**消息到达时**的样本。
    给出两条独立通道的阶跃响应：/joint_states 自带的 velocity，以及位置的数值微分。"""
    if len(series) < 10:
        print("  ❌ 样本太少（%d 个），无法分析" % len(series))
        return
    ts = [s[0] for s in series]
    qs = [s[1] for s in series]
    vs = [s[2] for s in series]

    # 数值微分：中心差分 + 5 点滑动平均（200 Hz 下 5 点 = 25 ms 平滑）
    dv = []
    for i in range(len(series)):
        if i == 0 or i == len(series) - 1:
            dv.append(float("nan"))
            continue
        dv.append((qs[i + 1] - qs[i - 1]) / (ts[i + 1] - ts[i - 1]))
    w = 5
    dv_s = []
    for i in range(len(dv)):
        lo, hi = max(0, i - w // 2), min(len(dv), i + w // 2 + 1)
        seg = [x for x in dv[lo:hi] if x == x]
        dv_s.append(sum(seg) / len(seg) if seg else float("nan"))

    print("  ── %s ──" % label)
    for name, sig in (("joint_states.velocity", vs), ("d(position)/dt", dv_s)):
        marks = {}
        for frac, lab in ((0.1, "10%"), (0.632, "63%"), (0.9, "90%")):
            hit = None
            for t, v in zip(ts, sig):
                if t < t_step or v != v:
                    continue
                if (amp >= 0 and v >= frac * amp) or (amp < 0 and v <= frac * amp):
                    hit = t
                    break
            marks[lab] = None if hit is None else hit - t_step
        tail = [v for t, v in zip(ts, sig) if t_end_step - 0.3 <= t <= t_end_step and v == v]
        steady = sum(tail) / len(tail) if tail else float("nan")
        step_vals = [v for t, v in zip(ts, sig) if t_step <= t <= t_end_step and v == v]
        peak = (max(step_vals) if amp > 0 else min(step_vals)) if step_vals else float("nan")
        rise = (marks["90%"] - marks["10%"]) if (marks["10%"] and marks["90%"]) else None
        print("    %-22s 10%%=%-9s 63%%=%-9s τ(上升/2.2)=%-9s 稳态=%.4f(比值 %.3f) 超调=%+.1f%%"
              % (name, fmt(marks["10%"]), fmt(marks["63%"]),
                 ("%.3f s" % (rise / 2.2)) if rise is not None else "未达到",
                 steady, steady / amp if amp else float("nan"),
                 100.0 * (peak - amp) / abs(amp) if amp else 0.0))


def fmt(x):
    return "未达到" if x is None else "%.3f s" % x


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="upperArm_joint", choices=GROUP)
    ap.add_argument("--amp", type=float, default=0.2, help="阶跃幅值 [rad/s]")
    ap.add_argument("--pre", type=float, default=0.6, help="阶跃前发 0 的时长 [s]")
    ap.add_argument("--dur", type=float, default=1.0, help="阶跃持续 [s]")
    ap.add_argument("--post", type=float, default=1.0, help="阶跃后发 0 的时长 [s]")
    args = ap.parse_args()
    if abs(args.amp) > 3.0:
        print("❌ amp 太大（MuJoCo 执行器 ctrlrange 是 ±3.15 rad/s）")
        return 1

    rclpy.init()
    node = PlantStep()
    node.joint = args.joint
    node.idx = GROUP.index(args.joint)
    try:
        node.spin(1.0)
        if not node.js:
            print("❌ 收不到 /joint_states —— stage2a 起了吗？")
            return 1

        q0 = [node.js.get(k, (float("nan"),))[0] for k in GROUP]
        print("起始关节 = %s" % [round(x, 4) for x in q0])
        print("关节 = %s（索引 %d）  话题 = %s" % (args.joint, node.idx, CMD_TOPIC))

        zero = [0.0] * len(GROUP)
        node.hold(args.pre, zero)
        t_step = time.time()
        step = list(zero)
        step[node.idx] = args.amp
        node.hold(args.dur, step)
        t_end_step = time.time()
        node.hold(args.post, zero)
        node.spin(0.5)

        q1 = [node.js.get(k, (float("nan"),))[0] for k in GROUP]
        print("结束关节 = %s" % [round(x, 4) for x in q1])
        moved = q1[node.idx] - q0[node.idx]
        print("该关节净转过 %.4f rad（期望约 %+.4f）" % (moved, args.amp * args.dur))
        others = max(abs(node.js.get(k, (0.0,))[1]) for k in GROUP if k != args.joint)
        print("其余关节在结束时 |速度| 最大 = %.4f rad/s（应≈0）" % others)

        print("── 阶跃响应（被控对象自身，无 Servo / 无 PID）──")
        rate = node.n_js / max(1e-9, (t_end_step + args.post - t_step + args.pre))
        print("  /joint_states 收到 %d 条，平均 %.1f Hz" % (node.n_js, rate))
        analyze(node.js_series, args.amp, t_step, t_end_step,
                "命令: 单关节阶跃 %+.3f rad/s，持续 %.2f s" % (args.amp, t_end_step - t_step))
        if node.js_series:
            with open("/tmp/plant_step_trace.csv", "w") as f:
                f.write("t_rel,cmd,q,qdot\n")
                for (t, c, q, v, tj) in node.trace:
                    f.write("%.6f,%.6f,%.6f,%.6f\n" % (t - t_step, c, q, v))
            print("  原始轨迹已存 /tmp/plant_step_trace.csv")

        print("── 前 0.30 s 采样（t, 命令, 位置, 实测速度）──")
        for (t, c, q, v, tj) in node.trace:
            if t_step <= t <= t_step + 0.30 and int((t - t_step) * 1000) % 20 < 6:
                print("   +%.3f s   cmd %+.3f   q %+.5f   qdot %+.4f"
                      % (t - t_step, c, q, v))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
