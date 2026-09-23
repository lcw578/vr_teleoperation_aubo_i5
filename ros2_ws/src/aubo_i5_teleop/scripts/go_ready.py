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
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
# 与 MJCF scene_ros2.xml 的 <key name="ready"> 一致
READY = [0.0, -0.4, 0.8, 0.0, 0.4, 0.0]
EE = "gripper_tip_link"
WORLD = "world"


class GoReady(Node):
    def __init__(self, duration):
        super().__init__("go_ready")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.duration = duration
        self.pub = self.create_publisher(Float64MultiArray, CMD_TOPIC, 1)
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
    args = ap.parse_args()

    rclpy.init()
    node = GoReady(args.duration)
    try:
        node.spin(1.5)
        if not node.js:
            print("❌ 收不到 /joint_states —— stage2a 起了吗？")
            return 1
        q0 = [node.js.get(k, float("nan")) for k in GROUP]
        print("发送前: 关节 = %s" % [round(x, 4) for x in q0])

        msg = Float64MultiArray()
        msg.data = list(args.ready)
        t0 = time.time()
        while time.time() - t0 < args.duration:
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
