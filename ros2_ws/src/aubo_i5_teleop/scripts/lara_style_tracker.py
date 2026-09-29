#!/usr/bin/env python3
"""lara_style_tracker：LARA 式积分型关节位置跟踪器（路线 B 的执行核心）。

设计出处（2026-09-27 对照审查后选型）：
  LARA（_refs/LARA_AUBOi5_AG95，已在真机 Aubo i5 验证）的控制语义 =
  "目标位姿 → 雅可比逆解 → **积分型关节位置命令**（cmd = cmd_prev + J⁺·e·dt）→
  ros_control position 接口"。我们的对照实验实锤了它对我们执行器的适配性：
  积分型命令流 10 s 走完 78.4/80 mm（98%）、平滑收敛、无极限环；而
  Servo 锚定流 + 接口层钳位在同一执行器上极限环（qd 4.84-5.77 rad/s）+ 静差。

替换关系（对 VLA 数据采集目标的适配）：
  替换掉：pose_tracking_node（PoseTracking PID）+ servo_interface（锚定流转换）
    ——它们只服务于"消费 Servo 锚定流"，是路线 A 的负担
  保留：quest_adapter、clutch_mapper（离合/reach limit/缩放）——输入侧全部
  保留：/forward_command_controller_position/commands（执行接口不变）
  真机：同一节点，commands 换发厂商驱动接口（积分型关节位置 = servoj 语义）

安全（自建，替代 Servo 内建保护）：
  · 条件数缩放：cond > 50 线性降速、> 200 停止推进（阈值已与离线指标对齐）
  · 关节限位：clip ±(3.04 − margin 0.1)
  · 速度限幅：每周期 |Δ| ≤ v_max·dt（厂商 2.618/3.142）
  · 目标断流：/target_pose 停 >0.5 s → 冻结 cmd（保持最后位置）
  · 输入新鲜度：/joint_states 缺失不推进

用法（authoritative_session.sh 的 stage3 之后，替代/停用 clutch_mapper+servo 链）：
  /home/lcw/tomato_robot/.venv/bin/python scripts/lara_style_tracker.py
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray

try:
    import mujoco
except ModuleNotFoundError as _e:      # pragma: no cover
    raise SystemExit("需要 mujoco（FK/Jacobian）：%s\n用 venv 解释器跑" % _e)

MJCF = "/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml"
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
TIP_OFF = np.array([-0.0405, -0.0143, 0.1492])
CMD_TOPIC = "/forward_command_controller_position/commands"
TARGET_TOPIC = "/target_pose"

# ready=P1（2026-09-29 IK 反解定案，与 MJCF keyframe/go_ready 同源）：
Q_REST = np.array([-1.135857, 0.052094, 1.773601, -1.987012, -1.197269, 0.228262])
MU_REST = 0.02              # Tikhonov 刚度（拉向 q_rest，破肘部翻转歧义；上游 mu=0.02）
ROT_ERR_HOLD = 2.2          # 反极点 park 门限（rad，>126° 停腕；上游 rot_err_hold=2.2）
JOINT_LIMIT = 3.04
MARGIN = 0.1
VMAX = np.array([2.618, 2.618, 2.618, 3.142, 3.142, 3.142])
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
_bid = mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")


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
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.q = {}                       # 关节名 → 位置
        self.js_fresh = False
        self.target = None                # (pos np3, R np3x3)
        self.target_time = 0.0
        self._was_stale = False           # 断流标记：恢复后首 tick 重锚（上游 needs_reanchor 同款）
        self._grip_engaged = False        # Grip 电平（从 /quest/joy 读，供重锚判断）
        self.cmd = None                   # 当前积分命令（6,）
        self.status = None
        qos = fast_qos()
        self.create_subscription(JointState, "/joint_states", self._js, qos)
        self.create_subscription(PoseStamped, TARGET_TOPIC, self._tgt, qos)
        self.pub = self.create_publisher(Float64MultiArray, CMD_TOPIC, 10)
        self.create_subscription(Joy, "/quest/joy", self._joy, fast_qos())
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)
        self._n = 0
        self.get_logger().info("lara_tracker 就绪：积分型关节位置命令 → %s | "
                               "cond 缩放 %.0f/%.0f | 目标断流 %.1f s 冻结"
                               % (CMD_TOPIC, COND_SOFT, COND_HARD, TARGET_STALE))

    # ---------- 回调 ----------
    def _js(self, msg):
        for i, name in enumerate(msg.name):
            if name in GROUP:
                self.q[name] = msg.position[i]
        self.js_fresh = len(self.q) == len(GROUP)

    def _joy(self, msg):
        if len(msg.buttons) >= 1:
            self._grip_engaged = bool(msg.buttons[0])

    def _tgt(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, o.x, o.y, o.z, o.w)):
            return
        self.target = (np.array([p.x, p.y, p.z]),
                       quat_xyzw_to_R((o.x, o.y, o.z, o.w)))
        self.target_time = time.time()

    # ---------- 每周期 ----------
    def _tick(self):
        now = time.time()
        if self.cmd is None:
            if self.js_fresh:
                self.cmd = np.array([self.q[j] for j in GROUP])   # 从当前关节起步
            return
        # 目标断流 → 冻结（不再推进；继续发当前 cmd 以保持位置）
        target_live = self.target is not None and (now - self.target_time) < TARGET_STALE
        # ── 断流重锚（上游 bi_quest_teleop needs_reanchor 同款）：断流发生后，
        # 若操作者仍握 Grip，恢复的首 tick 把 cmd 重置为实测关节——engage 增量
        # 从零开始，杜绝"断流积压→恢复后猛冲"（2026-09-26 卡死教训）。
        if not target_live and self._grip_engaged and not self._was_stale:
            self._was_stale = True
            self.get_logger().warn("目标断流——冻结；恢复且 Grip 仍按住时将重锚")
        if target_live and self._was_stale:
            self._was_stale = False
            if self._grip_engaged and self.js_fresh:
                # 断流恢复且 Grip 仍按住：cmd 重锚到实测关节（engage 增量从零开始）
                self.cmd = np.array([self.q[j] for j in GROUP])
                self.get_logger().info("断流恢复且 Grip 仍按住——cmd 重锚到实测关节")

        if target_live and self.js_fresh:
            # FK + Jacobian（在 cmd 命令轨迹上做微分运动——不是实测 q）
            for a, v in zip(_qadr, self.cmd):
                _mj_data.qpos[a] = v
            mujoco.mj_forward(_mj_model, _mj_data)
            R_c = _mj_data.xmat[_bid].reshape(3, 3).copy()
            p_c = _mj_data.xpos[_bid] + R_c @ TIP_OFF
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
        self.pub.publish(Float64MultiArray(data=self.cmd.tolist()))
        self._n += 1
        if self._n % (int(RATE_HZ) * 10) == 0:
            names = {0: "无警告", 1: "奇异降速", 3: "碰撞降速", 4: "碰撞急停", 5: "关节到界"}
            st = names.get(self.status, self.status) if self.status is not None else "未知"
            self.get_logger().info("cmd=[%.2f %.2f %.2f %.2f %.2f %.2f] target_live=%s"
                                   % (*self.cmd, target_live))


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
