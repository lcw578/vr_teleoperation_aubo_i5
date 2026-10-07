#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 rg 场景 MJCF 生成 RViz/ros2_control 侧需要的 URDF 夹爪片段。

为什么需要：MuJoCo 是 headless 跑的，**你看到的臂来自 URDF**（RViz/robot_state_publisher），
且 ros2_control 要求控制器的关节名存在于 robot_description 里——所以换夹爪必须同步 URDF。

生成物：urdf/rg_gripper.urdf.xacro（宏 rg_gripper，挂在 ee_link 上）
  · 4 个 link（rg_base / rg_gear / rg_right_finger / rg_left_finger）+ 1 个抓取点 link
  · 3 个关节：rg_gear_joint 转动（驱动）、左右指滑移（URDF <mimic> 耦合到齿轮）
  · 几何/惯量/关节轴/限位**从 MJCF 直接读**（不做手抄，避免抄错）
安装变换（在 ee_link 系）：xyz=[0,0,0.050952] rpy=[π,0,0]
  —— 由 MJCF 的 wrist3→rg_base 位姿与 URDF 的 ee_link=wrist3·Rz(90°) 反算，
     重投影误差 1.1e-15；工具轴 [0,0,1] 与 AG95 一致（实测夹角 0.0°）。

用法：/home/lcw/tomato_robot/.venv/bin/python tools/build_rg_gripper_urdf.py
"""
from __future__ import annotations

import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
import mujoco  # noqa: E402

ASSETS = "/home/lcw/VR_teleoperation/assets/aubo_i5"
MJCF = os.path.join(ASSETS, "scene_ros2_rg.xml")
OUT = "/home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop/urdf/rg_gripper.urdf.xacro"

MOUNT_XYZ = "0 0 0.050952"
MOUNT_RPY = "${pi} 0 0"

BODIES = ["rg_base", "rg_gear", "rg_right_finger", "rg_left_finger"]
PARENT = {"rg_base": None, "rg_gear": "rg_base",
          "rg_right_finger": "rg_base", "rg_left_finger": "rg_base"}
# 关节：MJCF 关节名 → (类型, 父, 子, effort, velocity 出处说明)
JOINTS = {
    "rg_gear_joint": ("revolute", "rg_base", "rg_gear", 2.0, 2.0),
    "rg_right_finger_joint": ("prismatic", "rg_base", "rg_right_finger", 20.0, 0.042),
    "rg_left_finger_joint": ("prismatic", "rg_base", "rg_left_finger", 20.0, 0.042),
}
MIMIC = {"rg_right_finger_joint": ("rg_gear_joint", "-0.020995"),
         "rg_left_finger_joint": ("rg_gear_joint", "-0.020999")}
TIP_LINK_XYZ = "0 0 -0.1048"          # 抓取点（rg_base 系）


def q_to_rpy(q_wxyz):
    w, x, y, z = q_wxyz
    return R.from_quat([x, y, z, w]).as_euler("xyz")


def fmt(v):
    return " ".join("%.9g" % float(x) for x in v)


def main():
    m = mujoco.MjModel.from_xml_path(MJCF)
    bid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in BODIES}
    for n, i in bid.items():
        if i < 0:
            raise SystemExit("MJCF 里没有 %s" % n)

    L = []
    L.append('<?xml version="1.0"?>')
    L.append("<!-- 自动生成，勿手改：assets/aubo_i5/tools/build_rg_gripper_urdf.py -->")
    L.append('<!-- rg = 自研齿条夹爪（/data/robot_assembly 整机模型，论文 Xu et al. JFR 2026）。')
    L.append("     用途：RViz 显示（MuJoCo 是 headless 的，你看到的是这份 URDF）+ ros2_control")
    L.append("     的关节注册（rg_gear_joint 必须存在于 robot_description 才能加载控制器）。")
    L.append("     几何/惯量/限位由 MJCF 读出，单位 m/rad。 -->")
    L.append('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">')
    L.append('')
    # 安装位姿是硬件常量（实测反算），不参数化——xacro 的 params 里带空格默认值
    # 需要额外加引号，容易写坏 XML（首版就栽在这）。
    L.append('  <xacro:macro name="rg_gripper" params="parent">')
    L.append('    <!-- 安装：ee_link → rg_base，xyz=[0,0,0.050952] rpy=[pi,0,0]（实测反算，误差 1e-15）-->')
    L.append('    <joint name="rg_base_joint" type="fixed">')
    L.append('      <parent link="${parent}"/>')
    L.append('      <child link="rg_base"/>')
    L.append('      <origin xyz="%s" rpy="%s"/>' % (MOUNT_XYZ, MOUNT_RPY))
    L.append('    </joint>')

    for name in BODIES:
        b = bid[name]
        if PARENT[name] is not None:
            jid = None
            for j in range(m.njnt):
                if m.jnt_bodyid[j] == b:
                    jid = j
                    break
            jname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid)
            jtype, par, chi, effort, vel = JOINTS[jname]
            L.append('    <joint name="%s" type="%s">' % (jname, jtype))
            L.append('      <parent link="%s"/>' % par)
            L.append('      <child link="%s"/>' % chi)
            L.append('      <origin xyz="%s" rpy="%s"/>'
                     % (fmt(m.body_pos[b]), fmt(q_to_rpy(m.body_quat[b]))))
            L.append('      <axis xyz="%s"/>' % fmt(m.jnt_axis[jid]))
            lo, hi = m.jnt_range[jid]
            L.append('      <!-- effort/velocity：本链路不用于控制（ros2_control 走 MJCF），')
            L.append('           仅供 RViz/MoveIt 参考。effort 取 MJCF forcerange。 -->')
            L.append('      <limit lower="%.6f" upper="%.6f" effort="%.3f" velocity="%.4f"/>'
                     % (lo, hi, effort, vel))
            if jname in MIMIC:
                mj, mult = MIMIC[jname]
                L.append('      <!-- 齿轮-齿条耦合：与 MJCF equality 同一系数 -->')
                L.append('      <mimic joint="%s" multiplier="%s" offset="0"/>' % (mj, mult))
            L.append('    </joint>')

        mass = m.body_mass[b]
        ipos = m.body_ipos[b]
        iquat = m.body_iquat[b]
        diag = m.body_inertia[b]
        L.append('    <link name="%s">' % name)
        L.append('      <inertial>')
        L.append('        <origin xyz="%s" rpy="%s"/>' % (fmt(ipos), fmt(q_to_rpy(iquat))))
        L.append('        <mass value="%.6f"/>' % mass)
        L.append('        <inertia ixx="%.9g" ixy="0" ixz="0" iyy="%.9g" iyz="0" izz="%.9g"/>'
                 % (diag[0], diag[1], diag[2]))
        L.append('      </inertial>')
        for g in range(m.body_geomadr[b], m.body_geomadr[b] + m.body_geomnum[b]):
            meshid = m.geom_dataid[g]
            if meshid < 0:
                continue
            meshname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, meshid)
            fn = meshname.replace("g_part_", "part_").replace("c_part_", "part_")
            if meshname.startswith("c_part"):
                fn += "_baked"
            scale = "1 1 1" if meshname.startswith("c_part") else "0.001 0.001 0.001"
            origin = ('        <origin xyz="%s" rpy="%s"/>'
                      % (fmt(m.geom_pos[g]), fmt(q_to_rpy(m.geom_quat[g]))))
            geom = ('        <geometry><mesh filename="file://%s/meshes_rg_gripper/%s.stl" '
                    'scale="%s"/></geometry>' % (ASSETS, fn, scale))
            for tag in ("visual", "collision"):
                L.append('      <%s>' % tag)
                L.append(origin.replace("        ", "          "))
                L.append(geom.replace("        ", "          "))
                L.append('      </%s>' % tag)
        L.append('    </link>')

    L.append('    <!-- 抓取点：rg_gear 原点沿工具轴 +0.10 m（采摘 demo 验证的 TCP 定义）。')
    L.append('         挂 rg_base 而非 rg_gear：点在齿轮转轴上，齿轮旋不影响其位置，且不被关节带动。 -->')
    L.append('    <joint name="rg_tip_joint" type="fixed">')
    L.append('      <parent link="rg_base"/>')
    L.append('      <child link="rg_tip_link"/>')
    L.append('      <origin xyz="%s" rpy="0 0 0"/>' % TIP_LINK_XYZ)
    L.append('    </joint>')
    L.append('    <link name="rg_tip_link"/>')
    L.append('  </xacro:macro>')
    L.append('</robot>')
    L.append('')

    with open(OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    print("写出 %s（%d 行）" % (OUT, len(L)))
    print("   安装：xyz=%s rpy=%s（在 ee_link 系）" % (MOUNT_XYZ, MOUNT_RPY))
    nv = sum(1 for _ in L if "<visual>" in _)
    print("   visual 条目 %d，link 4 + tip 1，joint 3（含 2 个 mimic）" % nv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
