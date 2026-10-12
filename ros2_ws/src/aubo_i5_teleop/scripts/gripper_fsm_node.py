#!/usr/bin/env python3
"""Two-state gripper command: Trigger press toggles OPEN/CLOSED independently
of arm Grip. Values are simulation joint coordinates from gripper_profile.
Measured joint feedback and desired binary state are logged separately.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
import json

CMD_TOPIC = "/gripper_controller/commands"
# ---- 夹爪档（TELEOP_GRIPPER=ag95|rg）：EE 体名/抓取点/场景文件随夹爪切换 ----
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from gripper_profile import resolve as _resolve_gripper  # noqa: E402
GRIP = _resolve_gripper()
OPEN_V = GRIP["open"]
CLOSED_V = GRIP["close"]
RATE = 1.5              # ctrl 单位/秒（0→0.9 约 0.6 s）
RATE_HZ = 50.0


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


class GripperFSM(Node):
    def __init__(self):
        super().__init__("gripper_fsm")
        self.state = "OPEN"           # 对外的离散状态（OPEN/CLOSED）
        self.target_v = OPEN_V
        self.cmd_v = OPEN_V
        self._prev_btn = 0
        self._toggles = 0
        self.declare_parameter("input_prefix", "quest")
        self.measured = None
        self.measured_stamp = None
        self.create_subscription(JointState, "/joint_states", self._on_feedback, fast_qos())
        self.pub_stamped = self.create_publisher(JointState, "/teleop/gripper_command", 10)
        self.pub_status = self.create_publisher(String, "/teleop/gripper_status", 10)
        for t in ["/" + str(self.get_parameter("input_prefix").value) + "/joy"]:
            self.create_subscription(Joy, t, self._on_joy, fast_qos())
        self.pub = self.create_publisher(Float64MultiArray, CMD_TOPIC, 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info("夹爪 FSM 就绪（档=%s）：OPEN(%.3f)↔CLOSED(%.3f)，斜率 %.1f/s，"
                               "buttons[1] 上升沿切换"
                               % (GRIP["name"], OPEN_V, CLOSED_V, RATE))

    def _on_feedback(self, msg):
        if GRIP["grasp_joint"] in msg.name:
            index = msg.name.index(GRIP["grasp_joint"])
            if index < len(msg.position):
                self.measured = msg.position[index]
                self.measured_stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

    def _on_joy(self, msg):
        if len(msg.buttons) < 2 or len(msg.axes) < 1:
            return
        # ⚠️ Trigger 用**模拟量 v 的阈值上升沿**判定（上游 GRIP_BUTTON/TRIGGER 同源
        #    SDK 数据）：p/t 字段含"触碰"（手指搭上就算 true），曾导致"碰一下就动"。
        #    v>0.7 的上升沿才是真按压。Grip 与 Trigger 独立，脱离时也允许切换夹爪。
        v = msg.axes[0]
        btn = 1 if v > 0.7 else 0
        if self._prev_btn == 0 and btn == 1:      # 模拟量阈值上升沿
            self.state = "CLOSED" if self.state == "OPEN" else "OPEN"
            self.target_v = CLOSED_V if self.state == "CLOSED" else OPEN_V
            self._toggles += 1
            self.get_logger().info("夹爪 → %s（第 %d 次切换）" % (self.state, self._toggles))
        self._prev_btn = btn

    def _tick(self):
        # 命令值以固定斜率滑向目标态
        step = RATE / RATE_HZ
        if self.cmd_v < self.target_v:
            self.cmd_v = min(self.target_v, self.cmd_v + step)
        elif self.cmd_v > self.target_v:
            self.cmd_v = max(self.target_v, self.cmd_v - step)
        now = self.get_clock().now()
        stamped = JointState()
        stamped.header.stamp = now.to_msg()
        stamped.name = [GRIP["grasp_joint"]]
        stamped.position = [self.cmd_v]
        self.pub_stamped.publish(stamped)
        self.pub.publish(Float64MultiArray(data=[self.cmd_v]))
        self.pub_status.publish(String(data=json.dumps({
            "stamp_ns": now.nanoseconds, "command_state": self.state,
            "command_position": self.cmd_v, "measured_position": self.measured,
            "measured_stamp_ns": self.measured_stamp,
            "moving_command": abs(self.cmd_v - self.target_v) > 1e-8})))


def main():
    rclpy.init()
    node = GripperFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
