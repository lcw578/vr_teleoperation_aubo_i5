#!/usr/bin/env python3
"""阶段 3 验证：给 MoveIt Servo 持续发 TwistStamped，确认机械臂**按正确方向**跟随。

两个设计要点：
  1. 必须"持续发"：servo.yaml 里 incoming_command_timeout = 0.1 s，
     只发一条的话 0.1 秒后 Servo 就自动停止，看不出效果。
  2. 必须调 /servo_node/start_servo：Servo 节点启动后处于停止态。
     实际服务名是 start_servo / stop_servo / pause_servo / unpause_servo / reset_servo_status
     （不是二进制度字符串里看到的 ~/start —— 以 ros2 service list 为准）。

判定标准不是"关节动了"，而是**末端实际位移方向与命令方向一致**——
方案文档 §B4 要求三个平移 + 三个旋转逐一验证。所以本脚本用 MJCF 做正运动学，
把关节角换算成末端位移来判方向。

用法（需先跑 stage2a / stage2b / stage3_servo）：
    python3 scripts/servo_twist_test.py                       # 单次：+Z 0.03 m/s，3 秒
    python3 scripts/servo_twist_test.py --axis x --value 0.05
    python3 scripts/servo_twist_test.py --verify-translations  # 逐一验证 6 个平移方向
    python3 scripts/servo_twist_test.py --verify-rotations     # 逐一验证 6 个旋转方向
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

SERVO_START = "/servo_node/start_servo"
SERVO_STATUS = "/servo_node/status"
TWIST_TOPIC = "/servo_node/delta_twist_cmds"

DEFAULT_MJCF = "/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml"

MANIP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]

AXES = ["x", "y", "z"]


class FkModel:
    """用 MJCF 做正运动学：关节角 -> 末端（左指尖）位姿。"""

    def __init__(self, path):
        import mujoco
        self.mujoco = mujoco
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        self.tip = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "ag95_left_finger")
        self.adr = {}
        for i in range(self.m.njnt):
            if self.m.jnt_type[i] == 3:
                self.adr[self.m.joint(i).name] = self.m.jnt_qposadr[i]

    def tip_pose(self, joints):
        for nm, q in joints.items():
            if nm in self.adr:
                self.d.qpos[self.adr[nm]] = q
        self.mujoco.mj_forward(self.m, self.d)
        return self.d.xpos[self.tip].copy(), self.d.xmat[self.tip].reshape(3, 3).copy()


class ServoTwistTest(Node):
    def __init__(self, frame, rate_hz):
        super().__init__("servo_twist_test")
        self.frame = frame
        self.pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 1)
        self.joints = {}
        self.status = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        try:
            from moveit_msgs.msg import ServoStatus
            self.create_subscription(ServoStatus, SERVO_STATUS, self._on_status, 10)
        except Exception:
            pass
        self.period = 1.0 / rate_hz

    def _on_js(self, m):
        for i, n in enumerate(m.name):
            self.joints[n] = m.position[i]

    def _on_status(self, m):
        self.status = m

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def start_servo(self):
        cli = self.create_client(Trigger, SERVO_START)
        if not cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(f"{SERVO_START} 不可用")
            return False
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        r = fut.result()
        ok = r is not None and r.success
        self.get_logger().info(f"调用 {SERVO_START}: success={ok}")
        return ok

    def send_for(self, twist, seconds):
        msg = TwistStamped()
        msg.header.frame_id = self.frame
        (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z) = twist[:3]
        (msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z) = twist[3:]
        n = int(seconds / self.period)
        for _ in range(n):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            self.spin(self.period)

    def manips(self):
        return {k: self.joints.get(k, float("nan")) for k in MANIP}


def rotvec(R):
    """旋转矩阵 -> 旋转向量（轴角），用于比较旋转方向。"""
    from numpy.linalg import norm
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    ang = np.arccos(c)
    if ang < 1e-9:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis / (2.0 * np.sin(ang)) * ang


def run_case(node, fk, twist, label, seconds, settle, results):
    node.spin(settle)
    p0, R0 = fk.tip_pose(node.manips())
    node.send_for(twist, seconds)
    node.spin(0.6)
    p1, R1 = fk.tip_pose(node.manips())

    cmd = np.array(twist[:3])
    dp = p1 - p0
    if np.linalg.norm(cmd) > 1e-9:
        n = cmd / np.linalg.norm(cmd)
        along = float(dp @ n)
        perp = float(np.linalg.norm(dp - along * n))
        ok = along > 5e-4 and along > 2 * perp
        print("  %-9s Δp=[%+.4f %+.4f %+.4f]  沿命令方向 %+.4f m  垂直分量 %.4f m  %s"
              % (label, *dp, along, perp, "✅" if ok else "❌"))
        results.append((label, ok, along))
    else:
        w = rotvec(R1.T @ R0)
        a = np.array(twist[3:])
        n = a / np.linalg.norm(a)
        along = float(w @ n)
        perp = float(np.linalg.norm(w - along * n))
        ok = abs(along) > 0.005 and abs(along) > 2 * perp
        print("  %-9s Δrot=[%+.4f %+.4f %+.4f]  沿命令轴 %+.4f rad  垂直 %.4f rad  %s"
              % (label, *w, along, perp, "✅" if ok else "❌"))
        results.append((label, ok, along))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", default="z", choices=AXES)
    ap.add_argument("--value", type=float, default=0.03)
    ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--frame", default="world")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--mjcf", default=DEFAULT_MJCF)
    ap.add_argument("--no-start", action="store_true")
    ap.add_argument("--verify-translations", action="store_true")
    ap.add_argument("--verify-rotations", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(args.mjcf):
        print("找不到 MJCF:", args.mjcf); return 1

    rclpy.init()
    node = ServoTwistTest(args.frame, args.rate)
    try:
        node.spin(2.0)
        if not node.joints:
            print("  ❌ 收不到 /joint_states —— stage2a 起了吗？"); return 1
        fk = FkModel(args.mjcf)

        if not args.no_start:
            node.start_servo()
            node.spin(1.0)

        results = []
        if args.verify_translations:
            print("  === 三个平移方向逐一验证（每个方向 %.1f s，%.3f m/s）===" % (args.duration, args.value))
            for ax in AXES:
                for sign in (+1, -1):
                    tw = [0.0] * 6
                    tw[AXES.index(ax)] = sign * args.value
                    run_case(node, fk, tw, "%s%s" % ("+" if sign > 0 else "-", ax),
                             args.duration, 1.2, results)
        elif args.verify_rotations:
            print("  === 三个旋转轴逐一验证（每个方向 %.1f s，%.3f rad/s）===" % (args.duration, args.value))
            for ax in AXES:
                for sign in (+1, -1):
                    tw = [0.0] * 6
                    tw[3 + AXES.index(ax)] = sign * args.value
                    run_case(node, fk, tw, "%s%s" % ("+" if sign > 0 else "-", ax),
                             args.duration, 1.2, results)
        else:
            tw = [0.0] * 6
            tw[AXES.index(args.axis)] = args.value
            run_case(node, fk, tw, "+" + args.axis, args.duration, 1.0, results)

        print()
        if results:
            passed = sum(1 for _, ok, _ in results if ok)
            print("  结果: %d/%d 通过" % (passed, len(results)))
            print("  " + ("✅ 全部方向正确" if passed == len(results) else "❌ 有方向不对，需查坐标系约定"))
        return 0 if all(ok for _, ok, _ in results) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
