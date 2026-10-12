#!/usr/bin/env python3
"""Route B damped differential IK with integrated position commands.

This is a project controller, not the ROS1 LARA implementation. The existing
IK/posture correction and singularity gains are retained. dt bounds each joint
increment; the raw correction is not a velocity law multiplied by dt.
The controller requires a fresh mapper control heartbeat and full arm feedback.
Release/fault discards old targets; re-engagement starts from measured joints.
No online collision checking or real hardware validation is claimed.
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import json
from teleop_geometry import tool_pose
from robot_parameters import POSITION_LIMIT, VELOCITY_LIMITS, WORKING_LIMIT

try:
    import mujoco
except ModuleNotFoundError as _e:      # pragma: no cover
    raise SystemExit("需要 mujoco（FK/Jacobian）：%s\n用 venv 解释器跑" % _e)

# ---- 夹爪档（TELEOP_GRIPPER=ag95|rg）：EE 体名/抓取点/场景文件随夹爪切换 ----
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from gripper_profile import resolve as _resolve_gripper  # noqa: E402
GRIP = _resolve_gripper()
MJCF = GRIP["mjcf"]
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
CMD_TOPIC = "/forward_command_controller_position/commands"
TARGET_TOPIC = "/target_pose"

# ready=Q1 水平前伸（2026-09-29 IK 反解二次定案，与 MJCF keyframe/go_ready 同源）：
Q_REST = np.array(GRIP["q_rest"])   # ready 位姿随夹爪档（见 gripper_profile.py）
MU_REST = 0.02              # Existing additive posture bias, not a null-space term.
ROT_ERR_HOLD = 2.2          # 反极点 park 门限（rad，>126° 停腕；上游 rot_err_hold=2.2）
JOINT_LIMIT = POSITION_LIMIT
MARGIN = POSITION_LIMIT - WORKING_LIMIT  # retain the approved existing working bound
VMAX = np.array(VELOCITY_LIMITS)
COND_SOFT = 50.0
COND_HARD = 200.0
TARGET_STALE = 0.5          # 秒；超过则冻结
RATE_HZ = 100.0
POS_TOL = 0.002             # 到达容差（m）——到停（避免终端抖动）
ANG_TOL = 0.05              # rad


_mj_model = mujoco.MjModel.from_xml_path(MJCF)
_mj_data = mujoco.MjData(_mj_model)
_qadr = [_mj_model.jnt_qposadr[mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_JOINT, j)]
         for j in GROUP]
_dofs = [_mj_model.jnt_dofadr[mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_JOINT, j)]
         for j in GROUP]
_bid = mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_BODY, GRIP["ee_body"])


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


def quat_xyzw_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_rotvec(R):
    """旋转矩阵 → 旋转向量（世界系，最短路径）。"""
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R).flatten())
    w, x, y, z = q
    ang = 2.0 * math.acos(max(-1.0, min(1.0, w)))
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return np.zeros(3)
    return np.array([x, y, z]) / s * ang


class LaraTracker(Node):
    def __init__(self):
        super().__init__("lara_tracker")
        self.declare_parameter("feedback_timeout", 0.2)
        self.declare_parameter("control_timeout", 0.2)
        self.declare_parameter("following_lag_seconds", 0.2)
        self.declare_parameter("following_error_duration", 0.2)
        self.feedback_timeout = float(self.get_parameter("feedback_timeout").value)
        self.control_timeout = float(self.get_parameter("control_timeout").value)
        self.following_limit = VMAX * float(self.get_parameter("following_lag_seconds").value)
        self.following_duration = float(self.get_parameter("following_error_duration").value)
        self.js_time = None
        self.control_time = None
        self.session = None
        self.engaged = False
        self.fault = None
        self.following_since = None
        self.pub_status = self.create_publisher(String, "/teleop/tracker_status", 10)
        self.q = {}                       # 关节名 → 位置
        self.js_fresh = False
        self.target = None                # (pos np3, R np3x3)
        self.target_time = 0.0
        self.cmd = None                   # 当前积分命令（6,）
        qos = fast_qos()
        self.create_subscription(JointState, "/joint_states", self._js, qos)
        self.create_subscription(PoseStamped, TARGET_TOPIC, self._tgt, qos)
        self.pub = self.create_publisher(Float64MultiArray, CMD_TOPIC, 10)
        self.create_subscription(JointState, "/teleop/control", self._control, 10)
        self.pub_stamped = self.create_publisher(JointState, "/teleop/joint_command", 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)
        self._n = 0
        self.get_logger().info("lara_tracker 就绪：积分型关节位置命令 → %s | "
                               "cond 缩放 %.0f/%.0f | 目标断流 %.1f s 冻结"
                               % (CMD_TOPIC, COND_SOFT, COND_HARD, TARGET_STALE))

    # ---------- 回调 ----------
    @staticmethod
    def _stamp_ns(stamp):
        return stamp.sec * 1000000000 + stamp.nanosec

    def _js(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(j in values and math.isfinite(values[j]) for j in GROUP):
            return
        self.q = {j: values[j] for j in GROUP}
        self.js_time = time.monotonic()
        self.js_fresh = True

    def _control(self, msg):
        session = self._stamp_ns(msg.header.stamp)
        if self.session is not None and session < self.session:
            return
        self.control_time = time.monotonic()
        if msg.header.frame_id != "engaged":
            self.engaged = False
            self.target = None
            if self._feedback_fresh():
                self.cmd = self._measured()
            return
        if session != self.session:
            self.target = None
            self.engaged = False
            self.session = session
            self.following_since = None
            if not self._feedback_fresh():
                self._fault("feedback_stale_at_engage")
                return
            values = dict(zip(msg.name, msg.position))
            if not all(j in values and math.isfinite(values[j]) for j in GROUP):
                self._fault("invalid_anchor")
                return
            # Exact joint snapshot used by mapper FK, rather than a later sample.
            self.cmd = np.array([values[j] for j in GROUP])
            self.fault = None
            self.engaged = True

    def _feedback_fresh(self):
        return self.js_time is not None and time.monotonic() - self.js_time < self.feedback_timeout

    def _measured(self):
        return np.array([self.q[j] for j in GROUP])

    def _fault(self, reason):
        if self.fault != reason:
            self.get_logger().warn("停止推进：%s；需要松开并重新接合" % reason)
        self.fault = reason
        self.engaged = False
        self.target = None
        if self._feedback_fresh():
            self.cmd = self._measured()

    def _tgt(self, msg):
        if not self.engaged or self.fault or self.session is None:
            return
        if self._stamp_ns(msg.header.stamp) < self.session or msg.header.frame_id != "world":
            return
        p = msg.pose.position
        o = msg.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, o.x, o.y, o.z, o.w)):
            return
        norm = np.linalg.norm([o.x, o.y, o.z, o.w])
        if norm < 1e-8:
            return
        self.target = (np.array([p.x, p.y, p.z]),
                       quat_xyzw_to_R(np.array([o.x, o.y, o.z, o.w]) / norm))
        self.target_time = time.monotonic()

    # ---------- 每周期 ----------
    def _tick(self):
        now = time.monotonic()
        if not self._feedback_fresh():
            self._fault("feedback_stale")
            self._publish_status()
            return  # Do not issue a new command based on stale measurements.
        if self.cmd is None:
            self.cmd = self._measured()
        if self.engaged and (self.control_time is None or now - self.control_time > self.control_timeout):
            self._fault("control_stale")
        target_live = self.target is not None and now - self.target_time < TARGET_STALE
        if self.engaged and self.target is not None and not target_live:
            self._fault("target_stale")
        if self.engaged and np.any(np.abs(self.cmd - self._measured()) > self.following_limit):
            if self.following_since is None:
                self.following_since = now
            elif now - self.following_since >= self.following_duration:
                self._fault("following_error")
        else:
            self.following_since = None
        if target_live and self.engaged and not self.fault:
            # FK + Jacobian（在 cmd 命令轨迹上做微分运动——不是实测 q）
            for a, v in zip(_qadr, self.cmd):
                _mj_data.qpos[a] = v
            mujoco.mj_forward(_mj_model, _mj_data)
            p_c, R_c = tool_pose(_mj_model, _mj_data, _bid, GRIP)
            p_t, R_t = self.target
            e = np.concatenate([p_t - p_c, R_to_rotvec(R_t @ R_c.T)])
            e_pos = float(np.linalg.norm(e[:3]))
            e_ang = float(np.linalg.norm(e[3:]))
            gain = 0.0
            # 超工作空间保护：位置误差 > 0.45 m 截断，防止"伸直锁死"
            POS_ERR_MAX = 0.45
            if e_pos > POS_ERR_MAX:
                e[:3] *= POS_ERR_MAX / e_pos
                e_pos = POS_ERR_MAX
            if e_pos < POS_TOL and e_ang < ANG_TOL:
                pass                                   # 已到容差内：只保持
            else:
                jp = np.zeros((3, _mj_model.nv))
                jr = np.zeros((3, _mj_model.nv))
                mujoco.mj_jac(_mj_model, _mj_data, jp, jr, p_c, _bid)
                J = np.vstack([jp, jr])[:, _dofs]
                sv = np.linalg.svd(J, compute_uv=False)
                cond = sv[0] / max(sv[-1], 1e-9)
                gain = 1.0
                if cond > COND_HARD:
                    gain = 0.0                         # 硬停
                elif cond > COND_SOFT:
                    gain = (COND_HARD - cond) / (COND_HARD - COND_SOFT)
                dt = 1.0 / RATE_HZ
                lam = 1e-4
                # 反极点 park（上游语义）：旋转误差 >126° 时置零旋转分量，
                # 位置任务照常解——上游的 park 只作用于腕部子问题
                if e_ang >= ROT_ERR_HOLD:
                    e[3:] = 0.0
                # q_rest Tikhonov 偏置（上游 mu=0.02）：破肘部翻转歧义
                e_rest = Q_REST - self.cmd
                dq = (J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), e)
                      + MU_REST * e_rest) * gain
                dq = np.clip(dq, -VMAX * dt, VMAX * dt)
                self.cmd = np.clip(self.cmd + dq,
                                   -(JOINT_LIMIT - MARGIN), JOINT_LIMIT - MARGIN)
        # Publish the exact outgoing command with the same ROS timestamp.
        stamped = JointState()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = "world"
        stamped.name = GROUP
        stamped.position = self.cmd.tolist()
        self.pub_stamped.publish(stamped)
        self.pub.publish(Float64MultiArray(data=self.cmd.tolist()))
        self._publish_status()
        self._n += 1
        if self._n % (int(RATE_HZ) * 10) == 0:
            self.get_logger().info("cmd=[%.2f %.2f %.2f %.2f %.2f %.2f] target_live=%s"
                                   % (*self.cmd, target_live))

    def _publish_status(self):
        self.pub_status.publish(String(data=json.dumps({
            "stamp_ns": self.get_clock().now().nanoseconds,
            "engaged": self.engaged, "fault": self.fault,
            "feedback_fresh": self._feedback_fresh(),
            "session": self.session,
            "following_error": (np.abs(self.cmd - self._measured()).tolist()
                if self.cmd is not None and self.q else None)})))


def main():
    rclpy.init()
    node = LaraTracker()
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
