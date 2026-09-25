#!/usr/bin/env python3
"""ClutchPoseMapper 的 ROS 包装：Mock/真手柄位姿 → /target_pose。

这是遥操作"大脑"的运行形态。数据流：
  /mock_vr/pose (PoseStamped, quest_world) ─┐
  /mock_vr/joy  (Joy, buttons[0]=离合)      ├→ 本节点(100 Hz) → /target_pose (world)
  /joint_states (FK 供 reach limit)        ─┘

状态机（2026-09-25 已批）：
  脱离（默认）  不发 /target_pose——Servo 走完最后目标后冻结（实测断流安全行为）。
  接合          手柄离合键**上升沿**且 pose/FK 都新鲜时 engage：
                锚点 = 手柄当前位姿 + 末端**实测**位姿（/joint_states + MuJoCo FK，
                punctual 通道，禁 TF）；若臂尚在运动中（|qd| 大）打警告——锚在实测上
                会有小幅回拉（量级=跟踪滞后）。
  运行中        每 tick 调一次 ClutchPoseMapper.target()（其内部状态演进的前提），
                把目标发到 /target_pose。
  脱离触发      ① 离合键松开（下降沿）；② **输入断流**：/mock_vr/pose 超过 0.3 s
                没有新消息（Mock 节点崩溃保护，用户已批）；③ 位姿含 NaN 视为异常丢弃。

纪律：
  · 与 follow_bench.py **互斥**——两者都发 /target_pose，绝不能同时跑。
  · 必须用 venv 解释器（rclpy + mujoco 3.12.0，见 BASELINE.md §4）。
  · use_sim_time=True，与其余栈一致。
"""
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Int8

sys.path.insert(0, str(Path(__file__).parent))
from clutch_pose_mapper import (ClutchPoseMapper, quat_wxyz_to_xyzw,  # noqa: E402
                                quat_xyzw_to_wxyz)

try:
    import mujoco
except ModuleNotFoundError as _e:      # pragma: no cover
    raise SystemExit("需要 mujoco（FK + 映射器）：%s\n"
                     "用 /home/lcw/tomato_robot/.venv/bin/python 跑" % _e)

MJCF_MODEL = "/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml"
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
TIP_OFF = np.array([-0.0405, -0.0143, 0.1492])   # ag95_base → 夹持点（已验证与 URDF 一致）

POSE_TOPIC = "/mock_vr/pose"
JOY_TOPIC = "/mock_vr/joy"
TARGET_TOPIC = "/target_pose"
RATE_HZ = 100.0
POSE_STALE_DISENGAGE = 0.3     # 输入断流自动脱离（秒）
FRESH_ENGAGE = 0.2             # 接合要求的 pose/FK 新鲜度（秒）
SCALE_FINE = 0.2               # 微调档（1:5）
QD_WARN = 0.5                  # 接合时关节速度警告阈值 (rad/s)

STATUS_MEANING = {0: "无警告", 1: "奇异降速", 2: "奇异急停", 3: "碰撞降速",
                  4: "碰撞急停", 5: "关节到界", 6: "离开奇异降速", -1: "无效"}


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


_mj_model = mujoco.MjModel.from_xml_path(MJCF_MODEL)
_mj_data = mujoco.MjData(_mj_model)
_mj_qadr = [_mj_model.jnt_qposadr[mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in GROUP]
_mj_bid = mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")


class ClutchMapperNode(Node):
    def __init__(self):
        super().__init__("clutch_mapper")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.mapper = ClutchPoseMapper(rot_reach_limit=0.6, pos_reach_limit=0.25)
        self.scale = 1.0                     # 1.0（1:1）↔ SCALE_FINE（1:5 微调）
        self._engaged = False
        self._last_tgt_p = None            # 最后发布的基准（重接合锚点用）
        self._last_tgt_q_wxyz = None
        self._last_tgt_time = 0.0
        self._ctrl_p = None
        self._ctrl_q_wxyz = None             # 手柄姿态，mujoco 约定
        self._pose_arrival = None            # wall time
        self._joy_arrival = None
        self._joy_buttons = [0, 0, 0]
        self._prev_clutch = 0
        self._prev_scale_btn = 0
        self._q = {}                         # 关节名 → (pos, vel)
        self._ee_p = None
        self._ee_q_wxyz = None
        self._fk_arrival = None
        self._status = None
        self._n_target = 0
        self._dbg = 0
        qos = fast_qos()
        self.create_subscription(PoseStamped, POSE_TOPIC, self._on_pose, qos)
        self.create_subscription(Joy, JOY_TOPIC, self._on_joy, qos)
        self.create_subscription(JointState, "/joint_states", self._on_js, qos)
        self.create_subscription(Int8, "/servo_pose_tracking/status", self._on_status, qos)
        self.pub_target = self.create_publisher(PoseStamped, TARGET_TOPIC, 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(
            "clutch_mapper 就绪：脱离中。离合=buttons[0] 上升沿接合；微调 scale=%.2f；"
            "断流 %.1f s 自动脱离" % (SCALE_FINE, POSE_STALE_DISENGAGE))

    # ---------- 回调 ----------
    def _on_pose(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, o.x, o.y, o.z, o.w)):
            self._drop = getattr(self, "_drop", 0) + 1
            return
        self._ctrl_p = np.array([p.x, p.y, p.z])
        self._ctrl_q_wxyz = quat_xyzw_to_wxyz([o.x, o.y, o.z, o.w])
        self._pose_arrival = time.time()

    def _on_joy(self, msg):
        if len(msg.buttons) >= 3:
            self._joy_buttons = list(msg.buttons[:3])
            self._joy_arrival = time.time()

    def _on_js(self, msg):
        now = time.time()
        for i, name in enumerate(msg.name):
            if name in GROUP:
                self._q[name] = (msg.position[i], msg.velocity[i])
        if len(self._q) == len(GROUP):
            q = [self._q[j][0] for j in GROUP]
            for a, v in zip(_mj_qadr, q):
                _mj_data.qpos[a] = v
            mujoco.mj_forward(_mj_model, _mj_data)
            R = _mj_data.xmat[_mj_bid].reshape(3, 3).copy()
            p = _mj_data.xpos[_mj_bid] + R @ TIP_OFF
            qw = np.zeros(4)
            mujoco.mju_mat2Quat(qw, np.ascontiguousarray(R).flatten())
            self._ee_p = p
            self._ee_q_wxyz = qw
            self._fk_arrival = now

    def _on_status(self, msg):
        self._status = int(msg.data)

    # ---------- 状态切换 ----------
    def _engage(self):
        self.mapper.scale = self.scale
        self.mapper.scale_rotation = self.scale
        # 锚点选择：2.5 s 内发过目标 → 锚到**最后基准**（臂可能还在走完它，
        # 锚实测会让目标后跳、快速点离合时表现为来回摆动——2026-09-25 用户实测）；
        # 否则锚到实测末端（FK）。
        anchor_p, anchor_q = self._ee_p, self._ee_q_wxyz
        src = "实测"
        if (self._last_tgt_p is not None
                and time.time() - self._last_tgt_time < 2.5):
            anchor_p, anchor_q = self._last_tgt_p, self._last_tgt_q_wxyz
            src = "最后基准"
        self.mapper.engage(self._ctrl_p, self._ctrl_q_wxyz, anchor_p, anchor_q)
        self._engaged = True
        qd = max(abs(self._q[j][1]) for j in GROUP)
        warn = "（⚠️ 臂运动中接合）" if qd > QD_WARN else ""
        self.get_logger().info("接合：锚点末端 (%.3f, %.3f, %.3f) [%s]%s"
                               % (*anchor_p, src, warn))

    def _disengage(self, reason):
        self.mapper.disengage()
        self._engaged = False
        self.get_logger().warn("脱离（%s）——停发目标，臂走完最后目标后冻结" % reason)

    # ---------- 主循环 ----------
    def _tick(self):
        self._dbg += 1
        if self._dbg % 200 == 1:
            self.get_logger().info("DBG tick#%d: engaged=%s joy_btn=%s joy_age=%s pose_age=%s fk_age=%s"
                                   % (self._dbg, self._engaged, self._joy_buttons,
                                      (time.time() - self._joy_arrival) if self._joy_arrival else None,
                                      (time.time() - self._pose_arrival) if self._pose_arrival else None,
                                      (time.time() - self._fk_arrival) if self._fk_arrival else None))
        now = time.time()
        clutch = self._joy_buttons[0] if (self._joy_arrival and
                                          now - self._joy_arrival < FRESH_ENGAGE) else 0
        # 缩放切换（上升沿，脱离/接合皆可）
        if self._prev_scale_btn == 0 and self._joy_buttons[2] == 1:
            self.scale = SCALE_FINE if self.scale == 1.0 else 1.0
            self.mapper.scale = self.scale
            self.mapper.scale_rotation = self.scale
            self.get_logger().info("缩放 → %s" % ("1:1" if self.scale == 1.0 else "1:5 微调"))
        self._prev_scale_btn = self._joy_buttons[2]

        pose_fresh = (self._pose_arrival and now - self._pose_arrival < FRESH_ENGAGE)
        fk_fresh = (self._fk_arrival and now - self._fk_arrival < FRESH_ENGAGE)

        if not self._engaged:
            if clutch and not self._prev_clutch:
                if not pose_fresh:
                    self.get_logger().warn("接合被拒：手柄位姿不新鲜")
                elif not fk_fresh:
                    self.get_logger().warn("接合被拒：FK 不新鲜（/joint_states 没到？）")
                else:
                    self._engage()
            self._prev_clutch = clutch
            return

        # 已接合
        if not clutch:
            self._disengage("离合松开")
            self._prev_clutch = clutch
            return
        if not pose_fresh or now - self._pose_arrival > POSE_STALE_DISENGAGE:
            self._disengage("输入断流 >%.1f s" % POSE_STALE_DISENGAGE)
            return
        if not fk_fresh:
            self.get_logger().warn("FK 不新鲜，本 tick 跳过（不脱离，等 /joint_states 恢复）")
            return

        out = self.mapper.target(self._ctrl_p, self._ctrl_q_wxyz,
                                 self._ee_p, self._ee_q_wxyz)
        if out is None:
            return
        tp, tq = out
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = tp
        qx, qy, qz, qw = quat_wxyz_to_xyzw(tq)
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.pub_target.publish(msg)
        self._last_tgt_p = np.array(tp, float)
        self._last_tgt_q_wxyz = np.array(tq, float)
        self._last_tgt_time = time.time()
        self._n_target += 1
        if self._n_target % (int(RATE_HZ) * 10) == 0:
            st = STATUS_MEANING.get(self._status, self._status)
            self.get_logger().info("运行中：目标 (%.3f, %.3f, %.3f) scale=%s Servo=%s"
                                   % (*tp, self.scale, st))


def main():
    rclpy.init()
    node = ClutchMapperNode()
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
