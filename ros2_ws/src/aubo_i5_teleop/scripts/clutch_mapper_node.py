#!/usr/bin/env python3
"""Route B mapper: base translation, local tool rotation and measured FK anchors.
/teleop/control carries a fixed joint snapshot/session stamp on engagement,
and explicit release plus a heartbeat. Targets older than a new session are
rejected by the tracker. Clock mode is selected by launch, not hardcoded here.
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
from std_msgs.msg import Float64MultiArray
from teleop_geometry import QUEST_LOCAL_TO_TOOL, tool_pose

sys.path.insert(0, str(Path(__file__).parent))
from clutch_pose_mapper import (ClutchPoseMapper, quat_wxyz_to_xyzw,  # noqa: E402
                                quat_xyzw_to_wxyz,
                                quat_mul, quat_conj, quat_to_rotvec,
                                rotvec_to_quat)

try:
    import mujoco
except ModuleNotFoundError as _e:      # pragma: no cover
    raise SystemExit("需要 mujoco（FK + 映射器）：%s\n"
                     "用 /home/lcw/tomato_robot/.venv/bin/python 跑" % _e)

# ---- 夹爪档（TELEOP_GRIPPER=ag95|rg）：EE 体名/抓取点/场景文件随夹爪切换 ----
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from gripper_profile import resolve as _resolve_gripper  # noqa: E402
GRIP = _resolve_gripper()
MJCF_MODEL = GRIP["mjcf"]
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]

POSE_TOPIC = "/mock_vr/pose"     # 默认 = 键盘 Mock；Quest 时由 --input quest 切到 /quest/*
JOY_TOPIC = "/mock_vr/joy"
TARGET_TOPIC = "/target_pose"
RATE_HZ = 100.0
POSE_STALE_DISENGAGE = 0.3     # 输入断流自动脱离（秒）
FRESH_ENGAGE = 0.2             # 接合要求的 pose/FK 新鲜度（秒）
SCALE_FINE = 0.1               # 微调档（1:10）
SCALE_COARSE = 0.33            # 常规档（1:3）：2026-09-27 首飞实测 1:2 下手部自然挥动就
                               # 把关节推到 3-4.8 rad/s（厂商限值 2.6/3.1）→ 极限环震荡
QD_WARN = 0.5                  # 接合时关节速度警告阈值 (rad/s)


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


_mj_model = mujoco.MjModel.from_xml_path(MJCF_MODEL)
_mj_data = mujoco.MjData(_mj_model)
_mj_qadr = [_mj_model.jnt_qposadr[mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in GROUP]
_mj_bid = mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_BODY, GRIP["ee_body"])


class ClutchMapperNode(Node):
    def __init__(self, input_prefix: str = "mock_vr", scale_coarse: float | None = None,
                 yaw_comp: bool = False, no_scale_toggle: bool = False,
                 ):
        super().__init__("clutch_mapper")
        self.mapper = ClutchPoseMapper(rot_reach_limit=0.6, pos_reach_limit=0.25,
                                      controller_to_tool=QUEST_LOCAL_TO_TOOL if input_prefix == "quest" else np.eye(3))
        # head-yaw 平移补偿默认关闭（2026-09-28 定案）：头显里没有场景画面、操作者
        # 扭头看外接屏就会触发 Δyaw——把"看一眼屏幕"误当"转身"，映射被永久转走。
        # 等头显内视频回传上线后再用 --yaw-comp 打开。
        self._yaw_comp = yaw_comp
        # 2026-09-29 修复切档 quirk：旧代码 SCALE_COARSE = scale_coarse 是【局部赋值】
        # （未改模块全局），而 _tick 的 A 键切档读全局 0.33——quest 默认 0.5 首按
        # 落到 0.33 且之后 0.33↔0.1 循环、回不到 0.5。现存实例属性参与切档。
        self._scale_coarse = scale_coarse if scale_coarse is not None else SCALE_COARSE
        self.scale = self._scale_coarse      # 常规档 ↔ SCALE_FINE（1:10 微调）
        # --no-scale-toggle（单档模式）：A 键误触会让整套测试/操作莫名变小 10 倍
        # （2026-09-29 实测踩中：左手柄 X 钮无效、右手柄易蹭到），调试期可锁定常规档
        self._scale_toggle = not no_scale_toggle
        self._control_stamp = None
        self._engaged = False
        self._smooth_p = None              # EMA 平滑后的发送位姿（抗手抖/抗极限环）
        self.SMOOTH_ALPHA = 0.35           # 100 Hz tick 下 ~8 Hz 截止（0.35@100Hz）
        self._ctrl_p = None
        self._ctrl_q_wxyz = None             # 手柄姿态，mujoco 约定
        self._pose_arrival = None            # wall time
        self._joy_arrival = None
        self._joy_buttons = [0, 0, 0]
        self._head_yaw = None            # 头显当前偏航（rad）
        self._yaw_calib = None           # 标定基准 yaw（首次接合时锁定）
        self._prev_clutch = 0
        self._prev_scale_btn = 0
        self._q = {}                         # 关节名 → (pos, vel)
        self._ee_p = None
        self._ee_q_wxyz = None
        self._fk_arrival = None
        self._n_target = 0
        self._dbg = 0
        qos = fast_qos()
        self._pose_topic = "/%s/pose" % input_prefix
        self._joy_topic = "/%s/joy" % input_prefix
        self.create_subscription(PoseStamped, self._pose_topic, self._on_pose, qos)
        self.create_subscription(Joy, self._joy_topic, self._on_joy, qos)
        self.create_subscription(Float64MultiArray, "/quest/head_yaw", self._on_yaw, qos)
        self.create_subscription(JointState, "/joint_states", self._on_js, qos)
        self.pub_control = self.create_publisher(JointState, "/teleop/control", 10)
        self.pub_measured = self.create_publisher(PoseStamped, "/teleop/ee_measured", 10)
        self.pub_target = self.create_publisher(PoseStamped, TARGET_TOPIC, 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(
            "clutch_mapper 就绪（输入 %s）：脱离中。离合=buttons[0] 上升沿接合；"
            "断流 %.1f s 自动脱离；常规档 scale=%.2f%s；head-yaw 补偿=%s"
            % (self._pose_topic, POSE_STALE_DISENGAGE, self._scale_coarse,
               "（A 可切 %.2f 微调）" % SCALE_FINE if self._scale_toggle else "（单档锁定）",
               "开" if self._yaw_comp else "关（默认）"))

    # ---------- 回调 ----------
    def _on_pose(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, o.x, o.y, o.z, o.w)):
            self._drop = getattr(self, "_drop", 0) + 1
            return
        if np.linalg.norm([o.x, o.y, o.z, o.w]) < 1e-8:
            return
        self._ctrl_p = np.array([p.x, p.y, p.z])
        self._ctrl_q_wxyz = quat_xyzw_to_wxyz([o.x, o.y, o.z, o.w])
        self._ctrl_q_wxyz /= np.linalg.norm(self._ctrl_q_wxyz)
        self._pose_arrival = time.monotonic()

    def _on_joy(self, msg):
        if len(msg.buttons) >= 3:
            self._joy_buttons = list(msg.buttons[:3])
            self._joy_arrival = time.monotonic()

    def _on_js(self, msg):
        now = time.monotonic()
        if not all(j in msg.name and msg.name.index(j) < len(msg.position)
                   and math.isfinite(msg.position[msg.name.index(j)]) for j in GROUP):
            return
        for i, name in enumerate(msg.name):
            if name in GROUP:
                if i < len(msg.position) and math.isfinite(msg.position[i]):
                    self._q[name] = (msg.position[i], msg.velocity[i] if i < len(msg.velocity) else 0.0)
        if len(self._q) == len(GROUP):
            q = [self._q[j][0] for j in GROUP]
            for a, v in zip(_mj_qadr, q):
                _mj_data.qpos[a] = v
            mujoco.mj_forward(_mj_model, _mj_data)
            p, R = tool_pose(_mj_model, _mj_data, _mj_bid, GRIP)
            qw = np.zeros(4)
            mujoco.mju_mat2Quat(qw, np.ascontiguousarray(R).flatten())
            self._ee_p = p
            self._ee_q_wxyz = qw
            self._fk_arrival = now
            measured = PoseStamped()
            measured.header.stamp = msg.header.stamp
            measured.header.frame_id = "world"
            measured.pose.position.x, measured.pose.position.y, measured.pose.position.z = p.tolist()
            measured.pose.orientation.x, measured.pose.orientation.y, measured.pose.orientation.z, measured.pose.orientation.w = quat_wxyz_to_xyzw(qw).tolist()
            self.pub_measured.publish(measured)

    def _on_yaw(self, msg):
        if msg.data and math.isfinite(msg.data[0]):
            self._head_yaw = msg.data[0]

    # ---------- 状态切换 ----------
    def _engage(self):
        self.mapper.scale = self.scale
        self.mapper.scale_rotation = self.scale
        anchor_p, anchor_q = self._ee_p.copy(), self._ee_q_wxyz.copy()
        src = "实测"
        self._control_stamp = self.get_clock().now().to_msg()
        self._anchor_q = [self._q[j][0] for j in GROUP]
        # yaw 修正（上游 bi_quest_teleop.py L38-41 同款思路）：首次接合锁定
        # "头显 yaw"为基准；此后每次接合把 Δyaw（操作者转过的角度）补偿进
        # 平移参考系——操作者转身后"推离自己"仍=臂朝任务区方向走。
        # ⚠️ 2026-09-28 两处修正：① 必须用【绝对式覆写】R_trans = Rz(-Δyaw)，
        #    旧代码 R_yaw @ R_trans 跨接合累乘、而 Δyaw 永远相对同一基准——
        #    扭头再接合就把映射永久转走且不回来；② 默认整体关闭（见 __init__）。
        if self._yaw_comp and self._head_yaw is not None:
            if self._yaw_calib is None:
                self._yaw_calib = self._head_yaw
            dyaw = self._head_yaw - self._yaw_calib
            c, s_ = math.cos(-dyaw), math.sin(-dyaw)
            self.mapper.R_trans = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
            self.get_logger().info("yaw 修正（绝对覆写）Δ=%.1f°（基准 %.1f°）"
                                   % (math.degrees(dyaw), math.degrees(self._yaw_calib)))
        self.mapper.engage(self._ctrl_p, self._ctrl_q_wxyz, anchor_p, anchor_q)
        self._engaged = True
        self._publish_control()
        self._smooth_p = None              # 新会话从锚点重新起步
        qd = max(abs(self._q[j][1]) for j in GROUP)
        warn = "（⚠️ 臂运动中接合）" if qd > QD_WARN else ""
        self.get_logger().info("接合：锚点末端 (%.3f, %.3f, %.3f) [%s]%s"
                               % (*anchor_p, src, warn))

    def _disengage(self, reason):
        self.mapper.disengage()
        self._engaged = False
        self._publish_control()
        self.get_logger().warn("脱离（%s）——取消旧目标" % reason)

    def _publish_control(self):
        msg = JointState()
        msg.header.stamp = self._control_stamp or self.get_clock().now().to_msg()
        msg.header.frame_id = "engaged" if self._engaged else "released"
        if self._engaged:
            msg.name = GROUP
            msg.position = self._anchor_q
        self.pub_control.publish(msg)

    # ---------- 主循环 ----------
    def _tick(self):
        self._dbg += 1
        if self._dbg % 200 == 1:
            self.get_logger().info("DBG tick#%d: engaged=%s joy_btn=%s joy_age=%s pose_age=%s fk_age=%s"
                                   % (self._dbg, self._engaged, self._joy_buttons,
                                      (time.monotonic() - self._joy_arrival) if self._joy_arrival else None,
                                      (time.monotonic() - self._pose_arrival) if self._pose_arrival else None,
                                      (time.monotonic() - self._fk_arrival) if self._fk_arrival else None))
        now = time.monotonic()
        self._publish_control()
        clutch = self._joy_buttons[0] if (self._joy_arrival and
                                          now - self._joy_arrival < FRESH_ENGAGE) else 0
        # 缩放切换（上升沿，脱离/接合皆可；单档模式下禁用）
        if self._scale_toggle and self._prev_scale_btn == 0 and self._joy_buttons[2] == 1:
            self.scale = SCALE_FINE if self.scale == self._scale_coarse else self._scale_coarse
            self.mapper.scale = self.scale
            self.mapper.scale_rotation = self.scale
            self.get_logger().info("缩放 → %s（1:%.1f）" % (
                "常规" if self.scale == self._scale_coarse else "1:10 微调",
                1.0 / self.scale))
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
            self._disengage("关节反馈不新鲜")
            return

        out = self.mapper.target(self._ctrl_p, self._ctrl_q_wxyz,
                                 self._ee_p, self._ee_q_wxyz)
        if out is None:
            return
        tp, tq = out
        # 逐 tick EMA 平滑（位置）：目标直通时手部高频抖动会全量传到臂
        # （2026-09-27 取证：关节速度 p50 2.16 rad/s、峰值 4.84——超厂商限值）。
        # 接合瞬间从锚点起步（不是从旧平滑值），避免初始滑移。
        if self._smooth_p is None:
            self._smooth_p = np.array(tp, float)
        self._smooth_p = (self.SMOOTH_ALPHA * np.asarray(tp, float)
                          + (1 - self.SMOOTH_ALPHA) * self._smooth_p)
        tp = self._smooth_p
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = tp
        qx, qy, qz, qw = quat_wxyz_to_xyzw(tq)
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.pub_target.publish(msg)
        self._n_target += 1
        if self._n_target % (int(RATE_HZ) * 10) == 0:
            self.get_logger().info("运行中：目标 (%.3f, %.3f, %.3f) scale=%s"
                                   % (*tp, self.scale))


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", default="mock_vr", choices=["mock_vr", "quest"],
                    help="输入前缀：mock_vr=键盘 Mock；quest=真头显适配器")
    ap.add_argument("--scale", type=float, default=None,
                    help="常规档缩放（默认 quest=0.5 / mock=1.0）")
    ap.add_argument("--yaw-comp", action="store_true",
                    help="启用 head-yaw 平移补偿（默认关——头显内有场景画面后再开）")
    ap.add_argument("--no-scale-toggle", action="store_true",
                    help="单档模式：禁用 A 键切档，锁定常规档（调试期防误触）")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    sc = args.scale
    if sc is None and args.input == "quest":
        sc = 0.5
    node = ClutchMapperNode(input_prefix=args.input, scale_coarse=sc,
                            yaw_comp=args.yaw_comp,
                            no_scale_toggle=args.no_scale_toggle)
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
