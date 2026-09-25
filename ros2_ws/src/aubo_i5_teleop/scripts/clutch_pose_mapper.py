"""ClutchPoseMapper —— 离合相对位姿映射（VR 手柄 → 末端目标）。

出处与边界（重要）：
  移植自 _refs/vr-teleop-kit（Dream-Machines-Robotics/vr-teleop-kit）的
  src/vr_teleop_kit/core/pose_mapping.py（2026-09-25 移植时 11 项上游 sanity
  测试在本机 mujoco 3.12.0 全部通过）。类体数学**未改**，只做了两处已批准的
  适配（见下）。内部四元数约定 [w,x,y,z]（mujoco 原生）——ROS 边界的
  (x,y,z,w) 转换放在 clutch_mapper_node，配往返测试，**不要改这里的约定**
  （本项目曾因 mju_* 的 wxyz 约定栽过跟头，见 BASELINE.md §4）。

两处适配（相对上游的差异）：
  1. **平移/旋转双 R 分离**：上游用一个 R（quest→armbase）同时变换平移与旋转。
     我们的键盘 Mock 平移在世界系（W=向任务区），旋转在手柄自系——所以
     平移用 `R_trans`（默认恒等；真 VR 时可在接合时设成 yaw 修正），
     旋转用 `R_align`（接合时自动计算，见 2）。
  2. **接合时轴对齐 R_align = R_ee ⊗ R_ctrl⁻¹**：把手柄自身轴系的增量重新表达进
     工具（接合时）轴系——"按键 roll/pitch/yaw = 末端绕自己的轴转"，而不是绕
     世界轴画大圆（本臂腕型对世界系偏航的代价是 rx/ry 的 3–8 倍，实测）。
     语义是"接合时工具坐标系"在整个会话内固定，重离合即重新对齐。

上游原有机制原样保留：
  · 离合状态机：engage 捕获（手柄+末端）锚点 / disengage 后 target() 返回 None
    （调用方停发 /target_pose → Servo 走完最后目标 → 冻结，即实测的断流安全行为）。
  · 滑移 reach limit：目标被钳在末端**当前**位姿的 rot 0.6 rad / pos 0.25 m 内，
    超出部分被吸收（反向立即生效）——防"180° 误差处最短路翻转抖振"与"超程
    bang-bang"两个真机失败模式。
  · 逐 tick 增量累积 + 半球对齐（q 与 −q）；scale/scale_rotation 为逐 tick 增益
    （切档天然连续）；rotation_pivot 支撑绕支点旋转（当前未用）。

脱离状态纪律：target() 每 tick 恰好调一次（内部状态演进）；未接合时调用无副作用。
"""

from __future__ import annotations

import mujoco
import numpy as np

# 与仿真同版本的 mujoco；环境不对时给出可执行的指引而不是堆栈
try:
    import mujoco as _mj  # noqa: F401  （上面已 import，这里只做存在性确认）
except ModuleNotFoundError as _e:      # pragma: no cover
    raise SystemExit(
        "clutch_pose_mapper 需要 mujoco（见文件头）：%s\n"
        "用 /home/lcw/tomato_robot/.venv/bin/python 跑" % _e)


# ------- 四元数助手（[w,x,y,z] 约定，mujoco 原生）-------

def quat_mul(qa: np.ndarray, qb: np.ndarray) -> np.ndarray:
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(qa, float), np.asarray(qb, float))
    return out


def quat_conj(q: np.ndarray) -> np.ndarray:
    out = np.zeros(4)
    mujoco.mju_negQuat(out, np.asarray(q, float))
    return out


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    out = np.zeros(4)
    mujoco.mju_mat2Quat(out, np.ascontiguousarray(R, float).ravel())
    return out


def quat_pow(q: np.ndarray, k: float) -> np.ndarray:
    """四元数的标量幂：保轴、角度×k。k=1 恒等、k=0 单位、k=0.5 开方。
    用在 reach-limit 路径的逐 tick 增量上（速率增益语义）。"""
    w = float(q[0])
    v = np.asarray(q[1:], dtype=float)
    half_angle = float(np.arctan2(float(np.linalg.norm(v)), w))
    if half_angle < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = v / np.sin(half_angle)
    new_half = k * half_angle
    s = float(np.sin(new_half))
    return np.array([float(np.cos(new_half)), s * axis[0], s * axis[1], s * axis[2]])


def quat_to_rotvec(q: np.ndarray) -> np.ndarray:
    """[w,x,y,z] 四元数 → 旋转向量（轴×角，最短路径）。"""
    out = np.zeros(3)
    mujoco.mju_quat2Vel(out, np.asarray(q, float), 1.0)
    return out


def rotvec_to_quat(v: np.ndarray) -> np.ndarray:
    """旋转向量 → [w,x,y,z] 四元数。"""
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = np.asarray(v, float) / angle
    s = np.sin(angle / 2.0)
    return np.array([np.cos(angle / 2.0), s * axis[0], s * axis[1], s * axis[2]])


# ------- ROS 边界转换（唯一允许出现 (x,y,z,w) 的地方）-------

def quat_xyzw_to_wxyz(q):
    """geometry_msgs/TF 的 (x,y,z,w) → mujoco [w,x,y,z]。"""
    q = np.asarray(q, float)
    return np.array([q[3], q[0], q[1], q[2]])


def quat_wxyz_to_xyzw(q):
    """mujoco [w,x,y,z] → (x,y,z,w)。"""
    q = np.asarray(q, float)
    return np.array([q[1], q[2], q[3], q[0]])


# ------- 映射器 -------

class ClutchPoseMapper:
    """单手离合相对映射：手柄位姿 → 末端目标（arm base 系）。

    参数：
        R_trans: 平移增量的系变换（默认恒等 = Mock 世界系 ≡ 臂基座系）。
                 真 VR 接入时可在接合时设成 yaw 修正。
        scale / scale_rotation: 平移/旋转的逐 tick 增益（1.0 = 1:1）。
                 微调档把两者一起调小（如 0.2）——逐 tick 应用，切档零跳变。
        rotation_pivot: 可选，绕支点旋转（当前未用，保留上游能力）。
        rot_reach_limit / pos_reach_limit: 目标相对末端**当前**位姿的跑前上限，
                 超出被吸收（滑移离合）。需要调用方把末端当前位姿喂给 target()。
    """

    def __init__(self, R_trans=None, scale: float = 1.0, scale_rotation: float = 1.0,
                 rotation_pivot=None, rot_reach_limit: float | None = 0.6,
                 pos_reach_limit: float | None = 0.25):
        self.R_trans = np.eye(3) if R_trans is None else np.asarray(R_trans, float).copy()
        self.scale = float(scale)
        self.scale_rotation = float(scale_rotation)
        self.rotation_pivot = None if rotation_pivot is None else np.array(rotation_pivot, float)
        self.rot_reach_limit = rot_reach_limit
        self.pos_reach_limit = pos_reach_limit
        self._engaged = False
        self._ctrl_engage_pos = None
        self._ctrl_engage_quat = None
        self._ee_engage_pos = None
        self._ee_engage_quat = None
        # 旋转轴对齐（接合时算一次）：R_align = R_ee ⊗ R_ctrl⁻¹
        self._R_align_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._R_align_quat_conj = np.array([1.0, 0.0, 0.0, 0.0])
        # 增量累积状态（每次接合重置）
        self._ctrl_prev_quat = None
        self._ctrl_prev_pos = None
        self._d_quat_eff = np.array([1.0, 0.0, 0.0, 0.0])
        self._d_pos_eff = np.zeros(3)

    @property
    def engaged(self) -> bool:
        return self._engaged

    def set_R_trans(self, R) -> None:
        """替换平移系变换（如真 VR 的 yaw 修正）。旋转对齐不受影响。"""
        self.R_trans = np.asarray(R, dtype=float).copy()

    def engage(self, controller_pos, controller_quat, ee_pos, ee_quat,
               pivot=None) -> None:
        """接合：捕获锚点。在手柄离合键的**上升沿**调用。"""
        self._ctrl_engage_pos = np.array(controller_pos, float, copy=True)
        self._ctrl_engage_quat = np.array(controller_quat, float, copy=True)
        self._ee_engage_pos = np.array(ee_pos, float, copy=True)
        self._ee_engage_quat = np.array(ee_quat, float, copy=True)
        self.rotation_pivot = None if pivot is None else np.array(pivot, float, copy=True)
        # 轴对齐：手柄自系轴 → 工具（接合时）自系轴
        self._R_align_quat = quat_mul(self._ee_engage_quat,
                                      quat_conj(self._ctrl_engage_quat))
        self._R_align_quat /= np.linalg.norm(self._R_align_quat)
        self._R_align_quat_conj = quat_conj(self._R_align_quat)
        self._ctrl_prev_quat = self._ctrl_engage_quat.copy()
        self._ctrl_prev_pos = self._ctrl_engage_pos.copy()
        self._d_quat_eff = np.array([1.0, 0.0, 0.0, 0.0])
        self._d_pos_eff = np.zeros(3)
        self._engaged = True

    def disengage(self) -> None:
        self._engaged = False

    def target(self, controller_pos, controller_quat,
               ee_pos=None, ee_quat=None):
        """当前末端目标（arm base 系），未接合返回 None。

        ee_pos/ee_quat = 末端**当前**位姿（我们用 /joint_states + FK，punctual 通道）。
        传入即启用滑移 reach limit（生产路径）；不传为上游的绝对映射（测试用）。
        ⚠️ 每 tick 恰好调用一次；未接合时调用无副作用。
        """
        if not self._engaged:
            return None
        assert self._ctrl_engage_pos is not None
        assert self._ctrl_engage_quat is not None
        assert self._ee_engage_pos is not None
        assert self._ee_engage_quat is not None

        # ---- 平移：reach-limit 路径逐 tick 累积（向量可交换，scale 逐 tick 等价
        #      于绝对缩放，直到 reach limit 吸收）；否则绝对映射。 ----
        p_now = np.asarray(controller_pos, float)
        pos_limited = ee_pos is not None and bool(self.pos_reach_limit)
        if pos_limited:
            assert self._ctrl_prev_pos is not None
            self._d_pos_eff = self._d_pos_eff + self.R_trans @ (
                self.scale * (p_now - self._ctrl_prev_pos))
            d_pos_arm = self._d_pos_eff
        else:
            d_pos_arm = self.R_trans @ (self.scale * (p_now - self._ctrl_engage_pos))
        self._ctrl_prev_pos = p_now.copy()

        # ---- 旋转：增量路径逐 tick 累积（增量小、方向无歧义；半球对齐）----
        q_now = np.asarray(controller_quat, float)
        rot_limited = ee_quat is not None and bool(self.rot_reach_limit)
        if rot_limited:
            assert self._ctrl_prev_quat is not None
            if float(np.dot(q_now, self._ctrl_prev_quat)) < 0.0:
                q_now = -q_now  # 半球对齐：q 与 −q 是同一旋转
            inc = quat_mul(q_now, quat_conj(self._ctrl_prev_quat))
            self._ctrl_prev_quat = q_now.copy()
            if self.scale_rotation != 1.0:
                inc = quat_pow(inc, self.scale_rotation)
            # 轴对齐：手柄自系轴增量 → 工具（接合时）自系轴增量
            inc_arm = quat_mul(quat_mul(self._R_align_quat, inc),
                               self._R_align_quat_conj)
            d_quat_arm = quat_mul(inc_arm, self._d_quat_eff)
            d_quat_arm /= np.linalg.norm(d_quat_arm)
            self._d_quat_eff = d_quat_arm
        else:
            # 绝对路径（测试用）：总增量同样经 R_align 重表达，保持两路径一致
            if self._ctrl_prev_quat is not None:
                if float(np.dot(q_now, self._ctrl_prev_quat)) < 0.0:
                    q_now = -q_now
                self._ctrl_prev_quat = q_now.copy()
            d_quat_quest = quat_mul(q_now, quat_conj(self._ctrl_engage_quat))
            if self.scale_rotation != 1.0:
                d_quat_quest = quat_pow(d_quat_quest, self.scale_rotation)
            d_quat_arm = quat_mul(quat_mul(self._R_align_quat, d_quat_quest),
                                  self._R_align_quat_conj)

        target_quat = quat_mul(d_quat_arm, self._ee_engage_quat)

        if rot_limited:
            # 旋转 reach limit：钳到末端当前姿态的 rot_reach_limit 内，
            # 超出吸收进有效增量（滑移——反向不需要倒回超程量）
            e = quat_to_rotvec(quat_mul(target_quat,
                                        quat_conj(np.asarray(ee_quat, float))))
            e_norm = float(np.linalg.norm(e))
            if e_norm > self.rot_reach_limit:
                e *= self.rot_reach_limit / e_norm
                target_quat = quat_mul(rotvec_to_quat(e), np.asarray(ee_quat, float))
                self._d_quat_eff = quat_mul(target_quat, quat_conj(self._ee_engage_quat))
                d_quat_arm = self._d_quat_eff

        if self.rotation_pivot is not None:
            offset = self._ee_engage_pos - self.rotation_pivot
            rotated_offset = np.zeros(3)
            mujoco.mju_rotVecQuat(rotated_offset, offset, d_quat_arm)
            target_pos = self.rotation_pivot + rotated_offset + d_pos_arm
        else:
            target_pos = self._ee_engage_pos + d_pos_arm

        if pos_limited:
            ee_p = np.asarray(ee_pos, float)
            dp = target_pos - ee_p
            dp_norm = float(np.linalg.norm(dp))
            if dp_norm > self.pos_reach_limit:
                clamped = ee_p + dp * (self.pos_reach_limit / dp_norm)
                self._d_pos_eff = self._d_pos_eff + (clamped - target_pos)
                target_pos = clamped

        return target_pos, target_quat
