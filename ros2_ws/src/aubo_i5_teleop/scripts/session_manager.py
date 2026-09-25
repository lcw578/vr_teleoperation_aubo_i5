#!/usr/bin/env python3
"""会话管理节点（DATA_PIPELINE_PLAN.md 阶段 C）：Joy 按键事件 → 录制状态机。

设计（HarvestFlex 同构）：
  · 数据流 = 每会话**一条连续袋**（/data/rosbags/session_<时间戳>/），按键事件
    （暂停/恢复/结束）作为 /session_events 标记写进袋里，episode 切片在转换
    阶段按标记做——操作失误可重新切片，录制过程无需人工干预文件。
  · 暂停 = 停止写入（数据不进袋），恢复 = 继续写入。离合锚点语义天然实现
    "恢复时目标插值"，无需额外处理（HarvestFlex 特意实现的那一环，我们免费）。
  · 录制用 rosbag2_py.SequentialWriter 手动写（Humble 的 Recorder 类停止语义
    不明确，手写 write 的控制权完整且是官方文档模式）；消息以 CDR 序列化
    落盘，时间戳 = 接收时刻（各消息自身的 header stamp 保留在序列化数据内）。

按键（/mock_vr/joy，来自键盘 mock 或未来 Quest3 手柄——同构契约）：
  buttons[3] 上升沿 = 暂停/恢复切换（RECORDING↔PAUSED；IDLE 时=开新袋）
  buttons[4] 上升沿 = 结束当前录制段（关袋回 IDLE；新段再按 [3] 开）

录制话题（阶段 C 定稿清单）：
  /joint_states, /camera/*/image_rect_color/compressed ×3, /target_pose,
  /gripper_controller/commands, /mock_vr/joy, /servo_pose_tracking/status,
  /session_events（自身事件流）。TF 不录（LeRobotDataset 不需要）。
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message

BAGS_DIR = "/data/rosbags"

# (话题, 类型, 订阅 QoS)；QoS：传感器流用 best_effort 深度 10（不阻塞发布方），
# 指令/事件流用 reliable 深度 10（一条不丢）
TOPICS = [
    ("/joint_states", "sensor_msgs/msg/JointState", "reliable"),
    ("/camera/d405_wrist/image_rect_color/compressed", "sensor_msgs/msg/CompressedImage", "sensor"),
    ("/camera/d435_right/image_rect_color/compressed", "sensor_msgs/msg/CompressedImage", "sensor"),
    ("/camera/d435_left/image_rect_color/compressed", "sensor_msgs/msg/CompressedImage", "sensor"),
    ("/target_pose", "geometry_msgs/msg/PoseStamped", "reliable"),
    ("/gripper_controller/commands", "std_msgs/msg/Float64MultiArray", "reliable"),
    ("/mock_vr/joy", "sensor_msgs/msg/Joy", "sensor"),
    ("/servo_pose_tracking/status", "std_msgs/msg/Int8", "sensor"),
    ("/session_events", "std_msgs/msg/String", "reliable"),
]

BTN_PAUSE, BTN_STOP = 3, 4   # joy.buttons 索引（mock_vr_node.py 契约）


class SessionManager(Node):
    def __init__(self, bags_dir: str):
        super().__init__("session_manager")
        import rosbag2_py

        self.bags_dir = bags_dir
        self.state = "IDLE"
        self.writer = None
        self.bag_uri = None
        self._msg_count = 0
        self._t0 = None
        self._prev_pause = 0
        self._prev_stop = 0

        from std_msgs.msg import String
        self.pub_state = self.create_publisher(String, "/session_state", 1)
        self.pub_events = self.create_publisher(String, "/session_events", 1)

        for topic, typ, qos in TOPICS:
            # /mock_vr/joy 特殊：既是录制数据流，也是状态机的触发源
            cb = self._on_joy if topic == "/mock_vr/joy" else self._on_msg
            self.create_subscription(self._type_cls(typ), topic,
                                     lambda msg, t=topic, c=cb: c(t, msg),
                                     qos_profile_sensor_data if qos == "sensor" else 10)

        self._publish_state()
        self.get_logger().info("IDLE：按 [3]（键盘 P）开始新录制段；话题清单 %d 项" % len(TOPICS))

    # ---------- 工具 ----------
    @staticmethod
    def _type_cls(typ):
        pkg, name = typ.split("/msg/")
        import importlib
        mod = importlib.import_module(f"{pkg}.msg")
        return getattr(mod, name)

    def _publish_state(self):
        from std_msgs.msg import String
        m = String()
        m.data = self.state
        self.pub_state.publish(m)

    def _emit_event(self, event: str, write_to_bag: bool):
        """状态迁移事件：发布到 /session_events；RECORDING 中同时写进袋子
        （转换阶段按这些标记切 episode）。"""
        from std_msgs.msg import String
        payload = json.dumps({"event": event,
                              "wall": time.time(),
                              "state": self.state})
        m = String()
        m.data = payload
        self.pub_events.publish(m)
        if write_to_bag and self.writer is not None:
            self._write("/session_events", m)
        self.get_logger().info("事件: %s" % payload)

    # ---------- 录制生命周期 ----------
    def _open_bag(self):
        import rosbag2_py
        uri = f"{self.bags_dir}/session_{time.strftime('%Y%m%d_%H%M%S')}"
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
                         rosbag2_py.ConverterOptions("cdr", "cdr"))
        for topic, typ, _ in TOPICS:
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=typ, serialization_format="cdr"))
        self.bag_uri = uri
        self._msg_count = 0
        self._t0 = time.time()

    def _write(self, topic: str, msg):
        self.writer.write(topic, serialize_message(msg), time.time_ns())
        self._msg_count += 1

    def _on_msg(self, topic: str, msg):
        if self.state != "RECORDING" or self.writer is None:
            return
        try:
            self._write(topic, msg)
        except Exception as exc:  # 序列化失败不能拖垮状态机
            self.get_logger().error(f"写袋失败 {topic}: {exc}")

    # ---------- Joy 事件 → 状态机 ----------
    def _on_joy(self, topic: str, msg):
        pause = len(msg.buttons) > BTN_PAUSE and msg.buttons[BTN_PAUSE] == 1
        stop = len(msg.buttons) > BTN_STOP and msg.buttons[BTN_STOP] == 1

        if pause and not self._prev_pause:          # 上升沿
            if self.state == "IDLE":
                self._open_bag()
                self.state = "RECORDING"
                self._emit_event("recording_start", write_to_bag=False)
            elif self.state == "RECORDING":
                self._emit_event("pause", write_to_bag=True)   # 标记先进袋
                self.state = "PAUSED"
            elif self.state == "PAUSED":
                self.state = "RECORDING"
                self._emit_event("resume", write_to_bag=True)
            self._publish_state()

        if stop and not self._prev_stop:            # 上升沿
            if self.state in ("RECORDING", "PAUSED"):
                if self.state == "PAUSED":
                    self.state = "RECORDING"        # 恢复写入以便结束标记落袋
                self._emit_event("session_end", write_to_bag=True)
                self.writer = None                  # SequentialWriter 无显式 close，解引用即收尾
                dur = time.time() - self._t0
                self.get_logger().info(
                    f"录制段结束：{self.bag_uri}（{self._msg_count} 条消息 / {dur:.0f}s）→ IDLE")
                self.bag_uri = None
                self.state = "IDLE"
                self._publish_state()

        self._prev_pause = pause
        self._prev_stop = stop
        # Joy 本身也是录制数据流（离合/按键事件是转换阶段切 episode 的依据之一）
        if self.state == "RECORDING" and self.writer is not None:
            try:
                self._write(topic, msg)
            except Exception as exc:
                self.get_logger().error(f"写袋失败 {topic}: {exc}")


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bags-dir", default=BAGS_DIR)
    args = ap.parse_args()

    # ⚠️ 2026-09-26：忘了这一行 → 节点启动即抛 NotInitializedException 死亡，
    #    且错误被重定向进日志文件，终端一片安静（用户按 P 毫无反应）。
    rclpy.init()
    node = SessionManager(args.bags_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
