#!/usr/bin/env python3
"""独立验证：把厂家 URDF 交给 MuJoCo 自己解析，与我们 MJCF 的臂模型做数值对照。

**为什么要这个脚本**：我们的 MJCF 由 `tomato_robot/scripts/aubo_urdf_to_mjcf.py`
经 MuJoCo 的 URDF 解析器生成（再手工改了网格路径/限位/armature/力矩/gravcomp）。
生成器本身不构成验证——它和自己的输出是同一来源。这里做的是**独立对照**：
让 MuJoCo 重新解析厂家 URDF，然后在若干随机位形下比较两个模型的运动学与动力学。

判据（实测结论，2026-09-24）：
  body 位置/姿态偏差  ~1e-16（机器精度）→ 运动学完全一致
  质量偏差            ~4e-5 kg          → 我们文件是 6 位有效数字的舍入
  质量矩阵 M(q) 相对偏差 ~4e-7          → 惯量（含质心与惯量主轴）一致
若某次改动后这些量变大，说明改动破坏了与厂家模型的一致性。

依赖：需要能 import mujoco + numpy 的 Python（本机系统 python 缺 typing_extensions，
      用 /home/lcw/tomato_robot/.venv/bin/python）。
用法：/home/lcw/tomato_robot/.venv/bin/python verify_model_vs_urdf.py
"""
import re
import sys
from pathlib import Path

import numpy as np
import mujoco

ASSETS = Path("/home/lcw/VR_teleoperation/assets/aubo_i5")
VENDOR_URDF = Path("/home/lcw/VR_teleoperation/ros2_ws/src/aubo_ros2_driver/"
                   "aubo_description/urdf/aubo_i5_37.urdf")
OUR_MJCF = ASSETS / "scene_aubo_i5.xml"
TMP_NOGRIPPER = ASSETS / ".verify_nogripper_tmp.xml"
TMP_VENDOR = Path("/tmp/verify_vendor_stripped.urdf")

ARM_JOINTS = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
              "wrist1_joint", "wrist2_joint", "wrist3_joint"]
# 厂家 <property> 的两个惯量候选值，逐关节
EQUA_INERTIA = [1.5, 1.5, 1.2, 0.05, 0.05, 0.01]


def make_our_no_gripper() -> Path:
    """删掉 ag95_base 整个子树——厂家 URDF 里没有夹爪，比对时必须去掉。"""
    lines = OUR_MJCF.read_text().split("\n")
    start = next(i for i, l in enumerate(lines) if '<body name="ag95_base"' in l)
    indent = len(lines[start]) - len(lines[start].lstrip())
    end = next(i for i in range(start + 1, len(lines))
               if lines[i].strip() == "</body>"
               and (len(lines[i]) - len(lines[i].lstrip())) == indent)
    del lines[start:end + 1]
    TMP_NOGRIPPER.write_text("\n".join(lines))
    return TMP_NOGRIPPER


def make_vendor_stripped() -> Path:
    """去掉 visual/collision（MuJoCo 不认 .3ds/.DAE）；惯量与运动学不受影响。"""
    xml = VENDOR_URDF.read_text().replace("package://aubo_description/meshes/", "../meshes/")
    xml = re.sub(r"<visual>.*?</visual>", "", xml, flags=re.S)
    xml = re.sub(r"<collision>.*?</collision>", "", xml, flags=re.S)
    TMP_VENDOR.write_text(xml)
    return TMP_VENDOR


def dof_of(model, jname):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
    return model.jnt_dofadr[jid]


def full_M(model, data):
    dst = np.zeros((model.nv, model.nv))
    try:
        mujoco.mj_fullM(model, data, dst)     # 3.12 起签名
    except TypeError:
        mujoco.mj_fullM(model, dst, data.M)   # 更早版本
    return dst


def main():
    A = mujoco.MjModel.from_xml_path(str(make_vendor_stripped()))       # 厂家 URDF
    B = mujoco.MjModel.from_xml_path(str(make_our_no_gripper()))        # 我们（去夹爪）
    dA, dB = mujoco.MjData(A), mujoco.MjData(B)
    rng = np.random.default_rng(0)

    print("A = 厂家 URDF 经 MuJoCo 解析   B = 我们的 MJCF（临时去掉夹爪）")
    print("两者都不含执行器与 gravcomp，比的是**刚体惯量与运动学**。\n")
    print(f"{'#':>3s} {'body位置Δ':>11s} {'body姿态Δ':>11s} {'质量Δ':>10s} {'子树质心Δ':>11s} "
          f"{'M(q)绝对Δ':>11s} {'M(q)相对Δ':>11s}")
    for k in range(6):
        q = rng.uniform(-2.0, 2.0, 6)
        for name, val in zip(ARM_JOINTS, q):
            ja = mujoco.mj_name2id(A, mujoco.mjtObj.mjOBJ_JOINT, name)
            jb = mujoco.mj_name2id(B, mujoco.mjtObj.mjOBJ_JOINT, name)
            dA.qpos[A.jnt_qposadr[ja]] = val
            dB.qpos[B.jnt_qposadr[jb]] = val
        mujoco.mj_forward(A, dA)
        mujoco.mj_forward(B, dB)

        dpos = dxmat = dmass = dsub = 0.0
        for i in range(1, A.nbody):
            n = A.body(i).name
            j = mujoco.mj_name2id(B, mujoco.mjtObj.mjOBJ_BODY, n)
            if j < 0:
                continue
            dpos = max(dpos, np.abs(dA.xpos[i] - dB.xpos[j]).max())
            dxmat = max(dxmat, np.abs(dA.xmat[i] - dB.xmat[j]).max())
            dmass = max(dmass, abs(A.body_mass[i] - B.body_mass[j]))
            dsub = max(dsub, np.abs(dA.subtree_com[i] - dB.subtree_com[j]).max())

        MA, MB = full_M(A, dA), full_M(B, dB)
        ia = [dof_of(A, n) for n in ARM_JOINTS]
        ib = [dof_of(B, n) for n in ARM_JOINTS]
        diff = max(abs(MA[ia[r], ia[c]] - (MB[ib[r], ib[c]]
                    - (B.dof_armature[ib[r]] if r == c else 0.0)))
                   for r in range(6) for c in range(6))
        scale = max(abs(MA[ia[r], ia[c]]) for r in range(6) for c in range(6))
        print(f"{k:3d} {dpos:11.3e} {dxmat:11.3e} {dmass:10.3e} {dsub:11.3e} "
              f"{diff:11.3e} {diff/scale:11.3e}")
    print(f"\n我们六个臂连杆质量和 = {sum(B.body_mass[1:]):.6f} kg "
          f"（厂家 URDF 臂六连杆 = 21.987854 kg；base_link 的 1.53902 kg 两边都是静基座，不参与动力学）")

    # ---- armature 的量级：它相对各关节自身惯量有多大 ----
    FULL = mujoco.MjModel.from_xml_path(str(ASSETS / "scene_ros2_velocity.xml"))
    dF = mujoco.MjData(FULL)
    ib = [dof_of(FULL, n) for n in ARM_JOINTS]
    print("\narmature 相对【关节自身惯量】的比重（决定这个参数对动力学的影响大小）")
    print(f"{'关节':16s} {'M_crb(j,j)均值':>14s} {'当前 armature':>14s} {'占比':>9s}")
    for kn, name in enumerate(ARM_JOINTS):
        vals = []
        for _ in range(20):
            for nm in ARM_JOINTS:
                jid = mujoco.mj_name2id(FULL, mujoco.mjtObj.mjOBJ_JOINT, nm)
                dF.qpos[FULL.jnt_qposadr[jid]] = rng.uniform(-2, 2)
            mujoco.mj_forward(FULL, dF)
            M = full_M(FULL, dF)
            vals.append(M[ib[kn], ib[kn]] - FULL.dof_armature[ib[kn]])
        mcrb = float(np.mean(vals))
        arm = FULL.dof_armature[ib[kn]]
        print(f"{name:16s} {mcrb:14.5f} {arm:14.4f} {arm/mcrb*100:8.1f}%")


if __name__ == "__main__":
    try:
        main()
    finally:
        TMP_NOGRIPPER.unlink(missing_ok=True)
        TMP_VENDOR.unlink(missing_ok=True)
