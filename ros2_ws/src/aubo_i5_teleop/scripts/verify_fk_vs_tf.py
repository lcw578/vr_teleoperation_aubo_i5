"""FK（/joint_states + MuJoCo）与 TF（tf2 Buffer）的现场对齐验证。

这是把台架测量通道从 TF 换成 FK 之前的**必要前置**：
  · 位置必须对上（否则 FK 的模型/偏置有错）；
  · 姿态也必须对上（go_to_pose 的步进目标要对两个通道的四元数做 slerp，
    若 MJCF 的 ag95_base 系与 URDF 的 gripper_tip_link 系之间有固定旋转，
    FK 的姿态就不能直接用）。
要求栈在跑。用 venv python（mujoco 3.12.0，与 mujoco_ros2_control 同版本）。

用法：/home/lcw/tomato_robot/.venv/bin/python scripts/verify_fk_vs_tf.py [秒数]
"""
import math
import sys
import time

import numpy as np
import mujoco
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState

SC = "/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml"
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
OFF = np.array([-0.0405, -0.0143, 0.1492])   # MJCF ag95_base -> 夹持点（与 gen_bench_poses 一致）

m = mujoco.MjModel.from_xml_path(SC)
d = mujoco.MjData(m)
qadr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in GROUP]
dofs = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in GROUP]
bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")


def fk(q):
    for a, v in zip(qadr, q):
        d.qpos[a] = v
    mujoco.mj_forward(m, d)
    R = d.xmat[bid].reshape(3, 3).copy()
    p = d.xpos[bid] + R @ OFF
    qw = np.zeros(4)
    mujoco.mju_mat2Quat(qw, np.ascontiguousarray(R).flatten())
    return p, np.array([qw[1], qw[2], qw[3], qw[0]])   # 转 (x,y,z,w) 与 TF 同约定


class Probe(Node):
    def __init__(self):
        super().__init__("fk_vs_tf_probe")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.q = {}
        self.js_t = None
        self.create_subscription(JointState, "/joint_states", self.on_js, qos)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def on_js(self, msg):
        self.js_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for i, name in enumerate(msg.name):
            if name in GROUP:
                self.q[name] = msg.position[i]


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    rclpy.init()
    node = Probe()
    dpos, dots, tf_lag = [], [], []
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.0)
        if len(node.q) < 6:
            continue
        q = [node.q[j] for j in GROUP]
        p_fk, q_fk = fk(q)
        try:
            tf = node.buffer.lookup_transform("world", "gripper_tip_link", Time())
        except Exception:
            continue
        t = tf.transform.translation
        r = tf.transform.rotation
        q_tf = np.array([r.x, r.y, r.z, r.w])
        p_tf = np.array([t.x, t.y, t.z])
        dpos.append(float(np.linalg.norm(p_fk - p_tf)))
        dot = abs(float(np.dot(q_fk / np.linalg.norm(q_fk), q_tf / np.linalg.norm(q_tf))))
        dots.append(min(1.0, dot))
        now = node.get_clock().now().nanoseconds * 1e-9
        if now:
            tf_lag.append(now - tf.header.stamp.sec - tf.header.stamp.nanosec * 1e-9)
        time.sleep(0.05)

    def rep(name, v, fmt):
        if len(v) < 5:
            print(f"{name}：样本不足（{len(v)}）")
            return
        v = sorted(v)
        print(f"{name}：中位 {fmt.format(v[len(v) // 2])}，最大 {fmt.format(v[-1])}（样本 {len(v)}）")
    print("FK（/joint_states + MJCF FK） vs TF（world→gripper_tip_link），静止状态下：")
    rep("位置差", dpos, "{:.6f} m")
    rep("姿态差（1−dot）", [1 - x for x in dots], "{:.2e}")
    rep("TF 帧滞后（时钟−stamp）", tf_lag, "{:+.3f} s")
    if dpos and sorted(dpos)[len(dpos) // 2] < 0.002:
        print("→ 位置：FK 与 TF 一致（< 2 mm），可以换通道。")
    else:
        print("→ ⚠️ 位置差超过 2 mm：FK 的模型/偏置与 URDF 链不一致，先别换！")
    if dots and min(dots) > 0.9999:
        print("→ 姿态：一致（|dot| > 0.9999），FK 的姿态可直接用于 slerp 目标。")
    else:
        print("→ ⚠️ 姿态不一致：两个 frame 之间有固定旋转，FK 姿态不能直接当步进目标！")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
