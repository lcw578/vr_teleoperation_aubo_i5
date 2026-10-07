#!/home/lcw/tomato_robot/.venv/bin/python
# -*- coding: utf-8 -*-
# 便捷入口：`bash 本文件` 也能跑（与 direction_check.py 同款双语头）
''''exec /bin/bash -c ". /opt/ros/humble/setup.bash 2>/dev/null || :; exec /home/lcw/tomato_robot/.venv/bin/python -- \"\$0\" \"\$@\"" "$0" "$@" # '''
"""手柄姿态可视化：把 /quest/pose 画进 RViz，与夹爪并排对照（2026-09-29，用户批准的 B 路线）。

背景：盲操时"点头/拧钥匙"手势的物理轴混淆（手柄轴看不见，HMD 内无场景画面）。
本节点把手柄的实时姿态以两个 TF 帧画进 RViz，与机械臂同屏：
  world→quest_controller   手柄真实位姿（位置+姿态，原样）
  world→controller_mirror  手柄姿态的"镜像"：锚在末端夹爪旁（固定偏移），姿态=手柄姿态
                           ——盲操时瞄一眼屏幕，手柄三根轴与夹爪并排，旋转轴混淆当场可见。

只订阅 + 广播 TF，不改链路任何行为。RViz 侧 teleop.rviz 已配 TF 显示层（只显示这两帧）。
注意事项：本节点 use_sim_time=True（RViz 在仿真时钟上，墙钟时间戳会被丢弃）。
--input mock_vr 可用键盘 Mock 离线测试。Ctrl-C 退出。
"""
import sys

import numpy as np
import rclpy
import mujoco
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# ---- 夹爪档（TELEOP_GRIPPER=ag95|rg）：EE 体名/抓取点/场景文件随夹爪切换 ----
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from gripper_profile import resolve as _resolve_gripper  # noqa: E402
GRIP = _resolve_gripper()
MJCF_MODEL = GRIP["mjcf"]
GROUP = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
TIP_OFF = np.array(GRIP["tip_offset"])
MIRROR_OFFSET = np.array([-0.18, 0.0, 0.10])   # 夹爪旁：操作者视角（屏幕右=-x）的右上方
ARROW_LEN = 0.16                               # 三轴箭头长度（m）
LIFETIME_S = 0.5                               # 标记自清理（节点死亡后不残留）


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


_mj_model = mujoco.MjModel.from_xml_path(MJCF_MODEL)
_mj_data = mujoco.MjData(_mj_model)
_mj_qadr = [_mj_model.jnt_qposadr[mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in GROUP]
_mj_bid = mujoco.mj_name2id(_mj_model, mujoco.mjtObj.mjOBJ_BODY, GRIP["ee_body"])


class ControllerViz(Node):
    def __init__(self, input_prefix: str):
        super().__init__("controller_axes_viz")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.br = TransformBroadcaster(self)
        self.pub_markers = self.create_publisher(MarkerArray, "/controller_viz_markers", 10)
        self.ee_p = None
        self.ee_q = None            # [w,x,y,z]
        self.create_subscription(JointState, "/joint_states", self._js, fast_qos())
        self.create_subscription(PoseStamped, "/%s/pose" % input_prefix, self._pose, fast_qos())
        self._n = 0

    def _js(self, msg):
        q = {}
        for i, name in enumerate(msg.name):
            if name in GROUP:
                q[name] = msg.position[i]
        if len(q) == len(GROUP):
            for a, j in zip(_mj_qadr, GROUP):
                _mj_data.qpos[a] = q[j]
            mujoco.mj_forward(_mj_model, _mj_data)
            self.ee_p = (_mj_data.xpos[_mj_bid]
                         + _mj_data.xmat[_mj_bid].reshape(3, 3) @ TIP_OFF)
            qw = np.zeros(4)
            mujoco.mju_mat2Quat(qw, np.ascontiguousarray(
                _mj_data.xmat[_mj_bid].reshape(3, 3).flatten()))
            self.ee_q = qw

    def _pose(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        quat_wxyz = (o.w, o.x, o.y, o.z)
        now = self.get_clock().now().to_msg()
        tfs = []
        # ① 手柄真实位姿
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = "world"
        t1.child_frame_id = "quest_controller"
        t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z = p.x, p.y, p.z
        t1.transform.rotation = o
        tfs.append(t1)
        # ② 镜像帧：夹爪旁，姿态=手柄姿态（锚=末端夹持点 + 固定偏移）
        if self.ee_p is not None:
            anchor = self.ee_p + MIRROR_OFFSET
            t2 = TransformStamped()
            t2.header.stamp = now
            t2.header.frame_id = "world"
            t2.child_frame_id = "controller_mirror"
            t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z = anchor
            t2.transform.rotation = o
            tfs.append(t2)
        self.br.sendTransform(tfs)
        # ── 三轴箭头标记（RViz MarkerArray，显示什么由这里完全决定）──
        if self.ee_p is None:
            return
        ma = MarkerArray()
        ma.markers.extend(self._triad("ctrl_real", p.x, p.y, p.z, quat_wxyz, 0).markers)
        ma.markers.extend(self._triad("ctrl_mirror", *anchor, quat_wxyz, 10).markers)
        self.pub_markers.publish(ma)
        self._n += 1
        if self._n % (90 * 10) == 0:
            self.get_logger().info("转发手柄位姿 → TF + 三轴标记（10s %d 帧）" % self._n)

    @staticmethod
    def _arrow(ns, mid, x, y, z, quat_wxyz, axis_idx, color):
        """一根彩色箭头：从帧原点沿局部轴伸出（标记 pose=帧位姿，点在局部系里）。"""
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = quat_wxyz
        p0 = type(m.pose.position)()
        p1 = type(m.pose.position)()
        setattr(p1, "xyz"[axis_idx], ARROW_LEN)
        m.points.append(p0)
        m.points.append(p1)
        m.scale.x = 0.012            # 杆径
        m.scale.y = 0.022            # 头宽
        m.scale.z = 0.045            # 头长
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.lifetime.sec = int(LIFETIME_S)
        m.lifetime.nanosec = int((LIFETIME_S % 1) * 1e9)
        return m

    @staticmethod
    def _ball(ns, mid, x, y, z, quat_wxyz):
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = quat_wxyz
        m.scale.x = m.scale.y = m.scale.z = 0.035
        m.color.r, m.color.g, m.color.b, m.color.a = (0.9, 0.9, 0.9, 1.0)
        m.lifetime.sec = int(LIFETIME_S)
        m.lifetime.nanosec = int((LIFETIME_S % 1) * 1e9)
        return m

    def _triad(self, ns, x, y, z, quat_wxyz, id_base):
        """一组三轴箭头 + 原点球。轴色：x红/y绿/z蓝（局部系，随帧姿态旋转）。"""
        colors = [(1.0, 0.2, 0.2, 1.0), (0.2, 1.0, 0.2, 1.0), (0.3, 0.3, 1.0, 1.0)]
        ma = MarkerArray()
        ma.markers.append(self._ball(ns, id_base, x, y, z, quat_wxyz))
        for i, c in enumerate(colors):
            ma.markers.append(self._arrow(ns, id_base + 1 + i, x, y, z, quat_wxyz, i, c))
        return ma


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", default="quest", help="位姿话题前缀：quest=真手柄；mock_vr=键盘 Mock")
    args = ap.parse_args()
    rclpy.init()
    node = ControllerViz(args.input)
    node.get_logger().info("controller_axes_viz 就绪：/quest/pose → RViz TF 双帧（真实+镜像）")
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
