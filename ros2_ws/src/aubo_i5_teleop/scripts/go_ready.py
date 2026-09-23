#!/usr/bin/env python3
"""把机械臂驱动到 MJCF 的 ready 位姿（0 -0.4 0.8 0 0.4 0）。

为什么需要这个工具：
  1. **仿真启动时机械臂并不在 ready**。`scene_ros2.xml` 里虽然定义了
     `<key name="ready" qpos="0 -0.4 0.8 0 0.4 0">`，但 mujoco_ros2_control 加载模型时
     不应用 keyframe（实测起始关节与 ready 不符），所以每次都要显式驱动过去。
  2. **做对比测量前必须归位**。尤其现在 `gravcomp` 让机械臂几乎不承重，任何一次运动
     结束后它都不会自己回到某个位姿；不归位的话，两轮实验的起始位姿不同，数据不可比。
     （这一点踩过坑：曾因此得出过"自碰撞与场景碰撞都无关"的错误结论。）
  3. **必须在 Servo 停止时调用**。否则 Servo 也在往同一个命令话题发布（它的"保持当前
     位姿"命令），两边会互相覆盖，机械臂可能停在半路。所以用法是：
        先停 Servo → 跑本脚本 → 再启动 Servo。

原理：往 /forward_command_controller_position/commands 连续发 4 秒
Float64MultiArray（顺序即 controllers.yaml 里 `joints` 的顺序），然后读回关节状态。
ready 的数值来自 MJCF 的 keyframe，属于"我们已有的产物"，不是这里另定的值。

用法：
    python3 scripts/go_ready.py                # 归位并打印结果
    python3 scripts/go_ready.py --ready 0 -0.6 0.9 0 0.3 0    # 用别的目标位姿
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import tf2_ros

CMD_TOPIC = "/forward_command_controller_position/commands"
CMD_TOPIC_VEL = "/forward_command_controller_velocity/commands"
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
# 与 MJCF scene_ros2.xml 的 <key name="ready"> 一致
READY = [0.0, -0.4, 0.8, 0.0, 0.4, 0.0]
EE = "gripper_tip_link"
WORLD = "world"
# 速度模式归位用的比例系数与速度上限。上限取厂家 joint_limits.yaml 的 max_velocity。
# ⚠️ 这是**测试工具**里的比例律（把位置误差换成速度命令），不是控制链路的一部分——
#    和位置模式下"直接发目标位置"是同一性质：只为把机械臂摆到已知位姿。
VEL_KP = 1.5          # [1/s]：v = VEL_KP * (q_target - q)
VEL_LIMIT = 3.0       # [rad/s]，低于厂家上限 3.15/3.2，留余量


class GoReady(Node):
    def __init__(self, duration, mode):
        super().__init__("go_ready")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.duration = duration
        self.mode = mode
        topic = CMD_TOPIC_VEL if mode == "velocity" else CMD_TOPIC
        self.pub = self.create_publisher(Float64MultiArray, topic, 1)
        self.topic = topic
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.js = {}
        self.create_subscription(JointState, "/joint_states", self._js, 10)

    def _js(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def ee(self):
        try:
            t = self.buffer.lookup_transform(WORLD, EE, Time()).transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ready", type=float, nargs=6, default=READY,
                    help="目标关节角（默认= MJCF 的 ready 关键帧）")
    ap.add_argument("--duration", type=float, default=4.0, help="持续发送的时长 [s]")
    ap.add_argument("--mode", default="position", choices=["position", "velocity"],
                    help="position=直接发目标位置（默认）；velocity=按位置误差比例下发速度")
    args = ap.parse_args()

    rclpy.init()
    node = GoReady(args.duration, args.mode)
    try:
        node.spin(1.5)
        if not node.js:
            print("❌ 收不到 /joint_states —— stage2a 起了吗？")
            return 1
        q0 = [node.js.get(k, float("nan")) for k in GROUP]
        print("模式: %s  话题: %s" % (args.mode, node.topic))
        print("发送前: 关节 = %s" % [round(x, 4) for x in q0])

        msg = Float64MultiArray()
        msg.data = list(args.ready)
        t0 = time.time()
        while time.time() - t0 < args.duration:
            if args.mode == "velocity":
                # v = k*(q_target - q)，逐关节限幅。位置误差收敛后速度自然趋 0。
                data = []
                for k, tgt in zip(GROUP, args.ready):
                    e = tgt - node.js.get(k, tgt)
                    data.append(max(-VEL_LIMIT, min(VEL_LIMIT, VEL_KP * e)))
                msg.data = data
            node.pub.publish(msg)
            node.spin(0.05)
        node.spin(1.5)

        q1 = [node.js.get(k, float("nan")) for k in GROUP]
        err = [a - b for a, b in zip(q1, args.ready)]
        print("发送后: 关节 = %s" % [round(x, 4) for x in q1])
        print("目标   :        %s" % [round(x, 4) for x in args.ready])
        print("到位误差(rad) = %s   最大 %.5f"
              % ([round(x, 5) for x in err], max(abs(x) for x in err)))
        p = node.ee()
        if p:
            print("末端位置 = (%.4f, %.4f, %.4f)" % p)
        # 注意：重力补偿不足时，这里的"到位误差"就是静态下垂量（= (1-gravcomp)*τg/kp 量级）
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
