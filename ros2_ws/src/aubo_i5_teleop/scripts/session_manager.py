#!/usr/bin/env python3
"""Route B rich rosbag logger; source timestamps are never overwritten.

Services /recording/start, /recording/pause (toggle), /recording/stop control
sessions. No Quest recording buttons are assigned. --mock-buttons preserves P
and Esc for the keyboard input only. timing.jsonl indexes each recorded message
by topic/sequence with source, reception, storage and monotonic timestamps.
This raw stream is asynchronous; dataset resampling is a separate operation.
"""
import argparse
import json
import time
from collections import Counter
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from std_srvs.srv import Trigger
from gripper_profile import resolve
from robot_parameters import SPEC

TOPICS = [
    ("/clock", "rosgraph_msgs/msg/Clock"),
    ("/joint_states", "sensor_msgs/msg/JointState"),
    ("/forward_command_controller_position/commands", "std_msgs/msg/Float64MultiArray"),
    ("/teleop/joint_command", "sensor_msgs/msg/JointState"),
    ("/teleop/ee_measured", "geometry_msgs/msg/PoseStamped"),
    ("/target_pose", "geometry_msgs/msg/PoseStamped"),
    ("/teleop/control", "sensor_msgs/msg/JointState"),
    ("/teleop/tracker_status", "std_msgs/msg/String"),
    ("/gripper_controller/commands", "std_msgs/msg/Float64MultiArray"),
    ("/teleop/gripper_command", "sensor_msgs/msg/JointState"),
    ("/teleop/gripper_status", "std_msgs/msg/String"),
    ("/quest/pose", "geometry_msgs/msg/PoseStamped"),
    ("/quest/joy", "sensor_msgs/msg/Joy"),
    ("/quest/frame_timing", "std_msgs/msg/String"),
    ("/mock_vr/pose", "geometry_msgs/msg/PoseStamped"),
    ("/mock_vr/joy", "sensor_msgs/msg/Joy"),
    ("/session_events", "std_msgs/msg/String"),
    ("/session_state", "std_msgs/msg/String"),
]
for camera in ("d405_wrist", "d435_right", "d435_left"):
    TOPICS += [(f"/camera/{camera}/image_rect_color/compressed", "sensor_msgs/msg/CompressedImage"),
               (f"/camera/{camera}/camera_info", "sensor_msgs/msg/CameraInfo")]


class SessionManager(Node):
    def __init__(self, bags_dir, mock_buttons=False):
        super().__init__("session_manager")
        self.bags_dir = Path(bags_dir)
        self.state = "IDLE"
        self.writer = None
        self.timing = None
        self.bag_uri = None
        self.sequences = Counter()
        self.mock_buttons = mock_buttons
        self.prev_buttons = (False, False)
        self.pub_state = self.create_publisher(String, "/session_state", 10)
        self.pub_events = self.create_publisher(String, "/session_events", 10)
        for topic, typ in TOPICS:
            if topic in ("/session_events", "/session_state"):
                continue  # Own transitions are recorded directly, exactly once.
            self.create_subscription(self._type_cls(typ), topic,
                                     lambda msg, t=topic: self._on_msg(t, msg),
                                     qos_profile_sensor_data)
        for operation in ("start", "pause", "stop"):
            self.create_service(Trigger, f"/recording/{operation}",
                                lambda req, resp, op=operation: self._service(op, resp))
        self._publish_state()

    @staticmethod
    def _type_cls(typ):
        import importlib
        pkg, name = typ.split("/msg/")
        return getattr(importlib.import_module(f"{pkg}.msg"), name)

    def _publish_state(self):
        msg = String(data=self.state)
        self.pub_state.publish(msg)
        if self.writer:
            self._write("/session_state", msg)

    def _event(self, event):
        msg = String(data=json.dumps({"event": event, "state": self.state,
                                     "ros_ns": self.get_clock().now().nanoseconds,
                                     "wall_ns": time.time_ns()}))
        self.pub_events.publish(msg)
        if self.writer:
            self._write("/session_events", msg)

    def start(self):
        if self.state != "IDLE":
            return False, "session already open"
        if self.get_parameter("use_sim_time").value and self.get_clock().now().nanoseconds == 0:
            return False, "waiting for /clock"
        import rosbag2_py
        self.bags_dir.mkdir(parents=True, exist_ok=True)
        self.bag_uri = self.bags_dir / ("session_" + time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1000000000:09d}")
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(self.bag_uri), storage_id="sqlite3"),
                         rosbag2_py.ConverterOptions("cdr", "cdr"))
        for topic, typ in TOPICS:
            self.writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typ, serialization_format="cdr"))
        self.timing = (self.bag_uri / "timing.jsonl").open("w", buffering=1)
        self.sequences.clear()
        manifest = {"schema_version": 1, "gripper": resolve(), "robot_spec": SPEC,
                    "use_sim_time": self.get_parameter("use_sim_time").value,
                    "mapping": {"translation": "base", "rotation": "tool"},
                    "timestamps": "bag timestamp is callback receive ROS time; source header preserved",
                    "gripper_feedback": "joint_states simulated joint measurement; OPEN/CLOSED is command state",
                    "topics": TOPICS}
        (self.bag_uri / "teleop_manifest.json").write_text(json.dumps(manifest, indent=2))
        self.state = "RECORDING"
        self._event("recording_start")
        self._publish_state()
        return True, str(self.bag_uri)

    def pause(self):
        if self.state == "IDLE":
            return False, "no session"
        self.state = "PAUSED" if self.state == "RECORDING" else "RECORDING"
        self._event("pause" if self.state == "PAUSED" else "resume")
        self._publish_state()
        return True, self.state

    def stop(self):
        if not self.writer:
            return False, "no session"
        self._event("session_end")
        self.state = "IDLE"
        self._publish_state()
        self.writer = None  # Humble SequentialWriter closes on destruction.
        self.timing.close()
        self.timing = None
        return True, str(self.bag_uri)

    def _service(self, operation, response):
        try:
            response.success, response.message = getattr(self, operation)()
        except Exception as exc:
            response.success, response.message = False, str(exc)
            self.get_logger().error(response.message)
        return response

    def _write(self, topic, msg, receive_ros_ns=None, receive_monotonic_ns=None):
        receive_ros_ns = self.get_clock().now().nanoseconds if receive_ros_ns is None else receive_ros_ns
        receive_monotonic_ns = time.monotonic_ns() if receive_monotonic_ns is None else receive_monotonic_ns
        storage_ros_ns = self.get_clock().now().nanoseconds
        header = getattr(msg, "header", None)
        source_ns = header.stamp.sec * 1000000000 + header.stamp.nanosec if header else None
        self.writer.write(topic, serialize_message(msg), receive_ros_ns)
        self.sequences[topic] += 1
        self.timing.write(json.dumps({"topic": topic, "sequence": self.sequences[topic],
                                     "source_ros_ns": source_ns, "receive_ros_ns": receive_ros_ns,
                                     "storage_ros_ns": storage_ros_ns,
                                     "receive_monotonic_ns": receive_monotonic_ns,
                                     "write_wall_ns": time.time_ns()}) + "\n")

    def _on_msg(self, topic, msg):
        received = self.get_clock().now().nanoseconds
        mono = time.monotonic_ns()
        if self.mock_buttons and topic == "/mock_vr/joy":
            pause = len(msg.buttons) > 3 and bool(msg.buttons[3])
            stop = len(msg.buttons) > 4 and bool(msg.buttons[4])
            if pause and not self.prev_buttons[0]:
                self.start() if self.state == "IDLE" else self.pause()
            if stop and not self.prev_buttons[1]:
                self.stop()
            self.prev_buttons = (pause, stop)
        if self.state == "RECORDING" and self.writer:
            self._write(topic, msg, received, mono)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bags-dir", default=str(next(parent for parent in Path(__file__).resolve().parents
                        if (parent / "assets/aubo_i5").is_dir()) / "data/rosbags"))
    ap.add_argument("--mock-buttons", action="store_true")
    ap.add_argument("--autostart", action="store_true")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = SessionManager(args.bags_dir, args.mock_buttons)
    if args.autostart:
        def start_when_clock_ready():
            if node.state == "IDLE":
                ok, _ = node.start()
                if ok:
                    node.start_timer.cancel()
        node.start_timer = node.create_timer(0.1, start_when_clock_ready)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
