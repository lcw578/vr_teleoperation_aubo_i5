#!/usr/bin/env python3
"""诊断：/tf 与 /joint_states 之间到底有没有传输滞后，以及台架自己的开销贡献了多少。

判据（与 follow_bench.py 的 transport_lag() 同一算法，但这里**只订阅、不发布**）：
  对**同一仿真时刻**的消息，比较 /tf 与 /joint_states 的**到达时刻**之差。
  · 若这里报 ~0，而台架报 0.288 s，说明那 0.288 s 是台架自己（125 Hz 发布 + 同步 TF 查询、
    单线程）造成的排队，不是 TF 链路的问题。
  · 若这里也报几百毫秒，那才是 TF 链路本身滞后。

用法（stage2a 起来后）：
    python3 tf_lag_probe.py [--publish-rate 0]
"""
import argparse
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
import tf2_ros


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


class Probe(Node):
    def __init__(self, publish_rate):
        super().__init__("tf_lag_probe")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.js_arrival = {}
        self.tf_arrival = {}
        self.publish_rate = publish_rate
        self.n_pub = 0
        # 可选：模仿台架同时高频发布目标（看"自己发布"是否就是滞后的来源）
        from std_msgs.msg import Float64MultiArray
        self.pub = self.create_publisher(Float64MultiArray, "/tf_lag_probe_sink", 10)

        q = fast_qos()
        self.create_subscription(JointState, "/joint_states", self._on_js, q)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def _on_js(self, msg):
        st = round(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 4)
        self.js_arrival[st] = time.time()

    def sample_tf(self):
        try:
            tf = self.buffer.lookup_transform("world", "gripper_tip_link", rclpy.time.Time())
        except TransformException:
            return
        st = round(tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9, 4)
        self.tf_arrival[st] = time.time()

    def lag(self):
        diffs = []
        for st, t_tf in self.tf_arrival.items():
            t_js = self.js_arrival.get(st)
            if t_js is not None:
                d = t_tf - t_js
                if -0.5 < d < 0.5:
                    diffs.append(d)
        return (len(diffs), statistics.median(diffs) if len(diffs) >= 5 else None,
                min(diffs) if diffs else None, max(diffs) if diffs else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--publish-rate", type=float, default=0.0,
                    help="0=只订阅（干净测量）；>0=模仿台架同时以该频率发布，看自己的开销贡献多少")
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.publish_rate)
    try:
        node.spin(2.0) if hasattr(node, "spin") else None
        t0 = time.time()
        period = 1.0 / args.publish_rate if args.publish_rate else 0.0
        nxt = time.time()
        while time.time() - t0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.sample_tf()
            if args.publish_rate:
                from std_msgs.msg import Float64MultiArray
                node.pub.publish(Float64MultiArray(data=[0.0] * 6))
                node.n_pub += 1
                nxt += period
                dt = nxt - time.time()
                if dt > 0:
                    time.sleep(dt)
            else:
                time.sleep(0.001)
        n, med, lo, hi = node.lag()
        print("发布频率 %s Hz | 匹配样本 %d" % (args.publish_rate or "仅订阅", n))
        if med is None:
            print("  ❌ 匹配样本不足（/joint_states 与 /tf 的仿真时刻对不上？）")
            return 1
        print("  /tf 到达 − /joint_states 到达（同一仿真时刻）：中位 %.3f s | 最小 %.3f | 最大 %.3f"
              % (med, lo, hi))
        print("  → %s" % ("这就是台架 transport_lag() 报的那个数" if med > 0.05
                          else "TF 链路本身几乎无滞后"))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
