"""量两个时基的真实偏差：TF/关节帧的时间戳 vs 节点当前仿真时钟。

为什么需要（2026-09-24）：latency 表的 D1 定义是
    D1 = (检测到运动的 TF 帧的仿真时间戳) − (发布目标时读到的仿真时钟)
实测 D1 = −0.224 s、D2 = −0.116 s，**负延迟物理上不可能**。两者不是同一个时基：
前者是"数据产生的仿真时刻"（由 mujoco_ros2_control 按仿真步打），后者是"节点读
/clock 拿到的当前仿真时刻"。若 jont_states/TF 的时间戳系统性滞后于仿真时钟，
D1/D2 就会整体偏负——这是仪器缺陷，不是被控对象的性质。

本探针在**静止**状态下采样 (ee 的仿真时间戳, 节点仿真时钟)，报告差值的中位数与分位。

用法：python3 scripts/simtime_skew_probe.py [秒数]
"""
import statistics
import sys
import time

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from sensor_msgs.msg import JointState

WORLD = "world"
EE_FRAME = "gripper_tip_link"


class Probe(Node):
    def __init__(self):
        super().__init__("simtime_skew_probe")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.js_stamp = None
        self.create_subscription(JointState, "/joint_states", self._on_js, qos)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def _on_js(self, msg):
        self.js_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    rclpy.init()
    node = Probe()
    tf_skew, js_skew = [], []
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.0)
        now = node.get_clock().now().nanoseconds * 1e-9
        if now == 0:
            continue
        try:
            tf = node.buffer.lookup_transform(WORLD, EE_FRAME, Time())
            st = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            tf_skew.append(now - st)
        except TransformException:
            pass
        if node.js_stamp is not None:
            js_skew.append(now - node.js_stamp)
        time.sleep(0.004)

    def rep(name, v):
        if len(v) < 10:
            print("%s：样本不足（%d）" % (name, len(v)))
            return
        v = sorted(v)
        print("%s：中位 %+.4f s，5%% %+.4f，95%% %+.4f（样本 %d）"
              % (name, statistics.median(v), v[int(len(v) * .05)], v[int(len(v) * .95)], len(v)))
    print("差值 = 节点仿真时钟 − 数据帧的时间戳；**正值表示数据帧的时间戳落后于时钟**")
    rep("TF  帧", tf_skew)
    rep("关节帧", js_skew)
    print("\n结论用法：D1/D2 为负时，把这个偏差减掉才是有意义的延迟（或改用同一帧内的")
    print("          数据自己估运动起始，不要混用发布瞬间的时钟读数）。")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
