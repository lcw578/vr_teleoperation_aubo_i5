#!/usr/bin/env python3
"""夹爪状态机节点：手柄 Trigger（未来）/ 键盘 Z（现在）→ 开/闭两状态 + 斜率执行。

设计（2026-09-25 已批）：
  · **两状态**（用户裁定）：OPEN(0.0) ↔ CLOSED(0.9)——0.93 是 ctrlrange 顶格，
    留 0.03 余量；不做剪刀，HarvestFlex 的 Detach/剪断阶段在仿真里没有对应物。
  · **开关量**（不做模拟量）：buttons[1] 的**上升沿**切换一次状态（真 VR 的
    Trigger 键同布局）；VLA 需要的离散状态就是 OPEN/CLOSED（+ moving 标志）。
  · **斜率执行**：命令值以 `rate`（默认 1.5 /s，即 0→0.9 约 0.6 s）滑向目标态，
    避免闭合瞬间冲击（将来抓真实物体时接触近静态）。
  · **并行**：夹爪与臂是独立控制器/关节/执行器——本节点只订阅按键、只发
    /gripper_controller/commands，与 /target_pose 链路零耦合（验收时会显式测）。
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

JOY_TOPIC = "/mock_vr/joy"
CMD_TOPIC = "/gripper_controller/commands"
OPEN_V = 0.0
CLOSED_V = 0.9
RATE = 1.5              # ctrl 单位/秒（0→0.9 约 0.6 s）
RATE_HZ = 50.0


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


class GripperFSM(Node):
    def __init__(self):
        super().__init__("gripper_fsm")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.state = "OPEN"           # 对外的离散状态（OPEN/CLOSED）
        self.target_v = OPEN_V
        self.cmd_v = OPEN_V
        self._prev_btn = 0
        self._toggles = 0
        self.create_subscription(Joy, JOY_TOPIC, self._on_joy, fast_qos())
        self.pub = self.create_publisher(Float64MultiArray, CMD_TOPIC, 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info("夹爪 FSM 就绪：OPEN(0.0)↔CLOSED(0.9)，斜率 %.1f/s，"
                               "buttons[1] 上升沿切换" % RATE)

    def _on_joy(self, msg):
        if len(msg.buttons) < 2:
            return
        btn = msg.buttons[1]
        if self._prev_btn == 0 and btn == 1:      # 上升沿
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
        self.pub.publish(Float64MultiArray(data=[self.cmd_v]))


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
