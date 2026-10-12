#!/usr/bin/env python3
"""震荡取证记录器：挂 60 s，同时录 /target_pose、/quest/pose、/joint_states、Servo 状态。
   输出逐事件分析：目标单帧跳变 TOP10、手柄速度 vs 臂速度、状态码时间线。

用法：python3 scripts/oscillation_forensics.py [秒数=60]
"""
import csv
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
CSV = "/tmp/osc_foren.csv"


class Rec(Node):
    def __init__(self):
        super().__init__("osc_forensics")
        self.q = {}
        self.tgt = None
        self.quest = None
        self.status = None
        self.rows = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, "/joint_states", self._js, qos)
        self.create_subscription(PoseStamped, "/target_pose", self._tg, qos)
        self.create_subscription(PoseStamped, "/quest/pose", self._qst, qos)
        self.create_subscription(Int8, "/servo_pose_tracking/status", self._st, qos)
        self.csv = open(CSV, "w", newline="")
        self.w = csv.writer(self.csv)
        self.w.writerow(["t", "src", "x", "y", "z", "extra"])

    def _js(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        for i, name in enumerate(m.name):
            if name in ("shoulder_joint", "upperArm_joint", "foreArm_joint",
                        "wrist1_joint", "wrist2_joint", "wrist3_joint"):
                self.q[name] = (m.position[i], m.velocity[i])
        qd = max((abs(self.q[j][1]) for j in self.q), default=0)
        self.w.writerow([st, "qd", 0, 0, 0, qd])

    def _tg(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.position
        self.w.writerow([st, "tgt", p.x, p.y, p.z,
                         self.status if self.status is not None else -99])
        self.tgt = (p.x, p.y, p.z)

    def _qst(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.position
        self.w.writerow([st, "quest", p.x, p.y, p.z, 0])

    def _st(self, m):
        self.status = int(m.data)


def main():
    rclpy.init()
    rec = Rec()
    t0 = time.time()
    print("记录 %d s——去头显复现震荡（按 Grip 大幅快速移动）…" % DUR)
    while time.time() - t0 < DUR:
        rclpy.spin_once(rec, timeout_sec=0.02)
    rec.csv.close()
    print("落盘 %s，分析中…" % CSV)

    rows = list(csv.DictReader(open(CSV)))
    tgt = [(float(r["t"]), np.array([float(r["x"]), float(r["y"]), float(r["z"])]), int(r["extra"]))
           for r in rows if r["src"] == "tgt"]
    qst = [(float(r["t"]), np.array([float(r["x"]), float(r["y"]), float(r["z"])]))
           for r in rows if r["src"] == "quest"]
    qd = [(float(r["t"]), float(r["extra"])) for r in rows if r["src"] == "qd"]
    print("样本：tgt %d、quest %d、qd %d" % (len(tgt), len(qst), len(qd)))

    if len(tgt) > 2:
        jumps = [(tgt[i][0], float(np.linalg.norm(tgt[i][1] - tgt[i - 1][1])),
                  tgt[i][2]) for i in range(1, len(tgt))]
        jumps.sort(key=lambda x: -x[1])
        print("\n目标单帧跳变 TOP8（正常 <10 mm；>25 mm = 撞 reach limit 弹跳的实锤）：")
        for t, d, st in jumps[:8]:
            print("   t=%.2f  Δ=%.1f mm  status=%d" % (t, d * 1000, st))
        big = [j for j in jumps if j[1] > 0.025]
        print("→ >25 mm 跳变共 %d 次（占 %.1f%%）" % (len(big), 100 * len(big) / len(jumps)))
        sts = {}
        for _, _, st in tgt:
            sts[st] = sts.get(st, 0) + 1
        print("目标帧内的 Servo 状态分布：", dict(sorted(sts.items())))
    if len(qd) > 10:
        v = [v for _, v in qd]
        v.sort()
        print("\n关节速度：p50 %.3f  p95 %.3f  max %.3f rad/s"
              % (v[len(v) // 2], v[int(len(v) * .95)], v[-1]))
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
