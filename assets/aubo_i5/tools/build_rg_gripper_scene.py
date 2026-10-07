#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 /data/robot_assembly 整机模型提取「自研齿条夹爪」，生成遥操链的并行场景文件。

出处与边界（2026-10-05）：
  来源 = /data/robot_assembly/model/tomato_picker.xml（SCUT 番茄串收获机器人整机模型，
  论文 Xu et al., J. Field Robotics 2026 —— 与我们是同一只 AUBO i5）。夹爪子树
  （base_link + gear_link + 左右指）与网格原样提取，**只做重命名**（rg_ 前缀）：
    base_link        → rg_base           （避免与 URDF 根 base_link 冲突）
    gear_link        → rg_gear
    right/left_finger_link → rg_right/left_finger
    gear_joint / right_finger_joint / left_finger_joint → rg_*
  几何、惯量、关节范围、耦合系数（polycoef −0.020995/−0.020999）、执行器参数
  （kp=1.5、forcerange ±2 = 论文扭簧柔顺的仿真等价物）**一字未改**。

产出（全部新增，现有文件一字不动）：
  assets/aubo_i5/meshes_rg_gripper/        37 个齿条夹爪网格 + 9 个腕部相机烘焙网格
  assets/aubo_i5/actuators_position_rg.xml 执行器（臂 + rg_gear_joint）
  assets/aubo_i5/scene_aubo_i5_rg.xml      手臂文件（AG95 子树 → rg 夹爪子树）
  assets/aubo_i5/scene_ros2_rg.xml         ROS 2 包装层（include 上面那份 + 排除对/耦合/keyframe）

已验证的执行前事实（2026-10-05，本模型的只读核验）：
  · wrist3 在 shoulder 系下的姿态与两模型差 1.2e-5（帧约定等价 → 挂载可逐字照抄）
  · 工具轴方向与 AG95 夹角 0.0°（操作方向感不变）
  · 齿轮 −0.4744 → 双指间距 74.9mm（张开）；+0.4632 → 35.5mm（闭合）
  · 抓取点 = gear 原点沿工具轴 +0.10m（采摘 demo 验证值），换算到 rg_base 系 = [0,0,−0.1048]

用法：/home/lcw/tomato_robot/.venv/bin/python tools/build_rg_gripper_scene.py
幂等：重复运行覆盖输出。校验：脚本末尾自动加载生成物并打印 nq/nu/关节顺序。
"""
from __future__ import annotations

import os
import re
import shutil
import sys

import numpy as np

SRC = "/data/robot_assembly"
SRC_MODEL = os.path.join(SRC, "model/tomato_picker.xml")
SRC_MESH = os.path.join(SRC, "meshes/gripper")
SRC_BAKED = os.path.join(SRC, "model/meshes_baked")
DST = "/home/lcw/VR_teleoperation/assets/aubo_i5"
MESH_DIR = "meshes_rg_gripper"

GEAR_OPEN = -0.474409      # 实测：负端 = 张开（74.9mm）
GEAR_CLOSE = 0.463234      # 正端 = 闭合（35.5mm）
POLY_R = -0.02099500       # 右手滑移 = polycoef × 齿轮角
POLY_L = -0.02099900

RENAMES = [
    ('body name="base_link"',        'body name="rg_base"'),
    ('body name="gear_link"',        'body name="rg_gear"'),
    ('body name="right_finger_link"', 'body name="rg_right_finger"'),
    ('body name="left_finger_link"', 'body name="rg_left_finger"'),
    ('joint name="gear_joint"',             'joint name="rg_gear_joint"'),
    ('joint name="right_finger_joint"',     'joint name="rg_right_finger_joint"'),
    ('joint name="left_finger_joint"',      'joint name="rg_left_finger_joint"'),
]


def read(p):
    with open(p, encoding="utf-8") as f:
        return f.read()


def write(p, s):
    with open(p, "w", encoding="utf-8") as f:
        f.write(s)
    print("  写出 %s（%d 行）" % (os.path.relpath(p, DST), s.count("\n") + 1))


def extract_body(text, start_marker):
    """从 start_marker 处的 <body ...> 开始，按嵌套深度找到配对的 </body>。"""
    i = text.index(start_marker)
    j = i
    depth = 0
    tag = re.compile(r"<body\b|</body>")
    while True:
        m = tag.search(text, j)
        if m is None:
            raise RuntimeError("未找到配对的 </body>")
        if m.group(0) == "<body":
            depth += 1
        else:
            depth -= 1
            if depth == 0:
                return i, m.end()
        j = m.end()


def main():
    src = read(SRC_MODEL)

    # ---------- 1. 拷贝网格 ----------
    os.makedirs(os.path.join(DST, MESH_DIR), exist_ok=True)
    n = 0
    for fn in sorted(os.listdir(SRC_MESH)):
        if fn.endswith(".stl"):
            shutil.copy2(os.path.join(SRC_MESH, fn), os.path.join(DST, MESH_DIR, fn))
            n += 1
    for fn in sorted(os.listdir(SRC_BAKED)):
        if fn.endswith(".stl"):
            shutil.copy2(os.path.join(SRC_BAKED, fn), os.path.join(DST, MESH_DIR, fn))
            n += 1
    print("1) 网格拷贝完成：%d 个 → assets/aubo_i5/%s/" % (n, MESH_DIR))

    # ---------- 2. 提取并重命名夹爪子树 ----------
    i, j = extract_body(src, '<body name="base_link"')
    block = src[i:j]
    for a, b in RENAMES:
        assert a in block, "子树里找不到 %s" % a
        block = block.replace(a, b)
    # 子树的缩进与我们的插入点对齐（源 8 空格 → 我们的 18 空格）
    lines = block.split("\n")
    block = "\n".join((" " * 10 + ln if ln.strip() else ln) for ln in lines)
    print("2) 子树提取完成：%d 行（rg_ 重命名 %d 项）" % (len(lines), len(RENAMES)))

    # ---------- 3. 网格定义（路径指向我们的目录）----------
    mesh_lines = []
    for m in re.finditer(r'<mesh name="(g_part_\d+_solid_\d+|c_part_\d+)" file="[^"]*"([^/]*)/>', src):
        name, extra = m.group(1), m.group(2)
        if name.startswith("g_part"):
            fn = name.replace("g_part_", "part_") + ".stl"
            scale = "0.001 0.001 0.001"
        else:                                   # 相机/转接板烘焙件
            fn = name.replace("c_part_", "part_") + "_baked.stl"
            scale = "1 1 1"
        mesh_lines.append('    <mesh name="%s" file="%s/%s" scale="%s" inertia="shell"/>'
                          % (name, MESH_DIR, fn, scale))
    print("3) 网格定义：%d 条" % len(mesh_lines))

    # ---------- 4. scene_aubo_i5_rg.xml ----------
    arm = read(os.path.join(DST, "scene_aubo_i5.xml"))
    ai, aj = extract_body(arm, '<body name="ag95_base"')
    # 连同其上的注释一起替换
    ci = arm.rindex("<!--", 0, ai)
    header = ("<!-- ============ 自研齿条夹爪（rg）============\n"
              "     来源：/data/robot_assembly/model/tomato_picker.xml（整机模型，"
              "论文 Xu et al. JFR 2026）\n"
              "     原样提取 + rg_ 前缀重命名；几何/惯量/耦合/执行器参数一字未改。\n"
              "     挂载 pos=[0,0,0.050952] quat=(0,0.707107,0.707107,0) —— 照抄整机模型，\n"
              "     已实测：工具轴与 AG95 夹角 0.0°、帧约定等价（见 build 脚本头）。\n"
              "     抓取点 = rg_gear 原点沿工具轴 +0.10m，在 rg_base 系 = [0,0,-0.1048]。\n"
              "     生成器：tools/build_rg_gripper_scene.py（勿手改本文件）\n"
              "     -->\n")
    arm_rg = arm[:ci] + header + block + arm[aj:]
    # 插入网格定义（在 </asset> 之前）
    arm_rg = arm_rg.replace("  </asset>",
                            "\n".join(mesh_lines) + "\n  </asset>", 1)
    write(os.path.join(DST, "scene_aubo_i5_rg.xml"), arm_rg)

    # ---------- 5. actuators_position_rg.xml ----------
    act = read(os.path.join(DST, "actuators_position.xml"))
    old_grip = [ln for ln in act.split("\n") if "left_outer_knuckle_joint" in ln and "<position" in ln]
    assert len(old_grip) == 1
    new_grip = (
        '    <!-- rg 齿条夹爪：单自由度，gear_joint 直接驱动（无 tendon），双指由 equality 耦合。\n'
        '         kp=1.5 / forcerange ±2 N·m 原样取自整机模型 —— 这是论文里扭簧柔顺夹持的\n'
        '         仿真等价物（夹紧到果柄反力为止，不会压碎），**不要为了"更硬"而调高 forcerange**。\n'
        '         执行器名 = 关节名（与臂关节同约定；ros2_control 按此匹配）。 -->\n'
        '    <position name="rg_gear_joint" joint="rg_gear_joint" kp="1.5" '
        'ctrlrange="-0.474409 0.463234" forcerange="-2 2"/>')
    act_rg = act.replace(old_grip[0], new_grip)
    act_rg = act_rg.replace("臂 + 夹爪的执行器定义", "臂 + rg 齿条夹爪的执行器定义（AG95 版见 actuators_position.xml）")
    write(os.path.join(DST, "actuators_position_rg.xml"), act_rg)

    # ---------- 6. scene_ros2_rg.xml（先不带 keyframe）----------
    wrap = read(os.path.join(DST, "scene_ros2.xml"))
    wrap = wrap.replace('<include file="scene_aubo_i5.xml"/>',
                        '<include file="scene_aubo_i5_rg.xml"/>', 1)
    wrap = wrap.replace('<include file="actuators_position.xml"/>',
                        '<include file="actuators_position_rg.xml"/>', 1)

    # 排除对：删掉 AG95 段（含注释），换成 rg 段
    ai = wrap.index("    <!-- --- 以下是 AG95 夹爪内部")
    aj = wrap.index("</contact>")
    rg_excl = """    <!-- --- 以下是 rg 齿条夹爪内部（来源整机模型的 6 条 exclude）---
         相邻连杆在关节/滑轨处必然重叠，不排除会持续产生虚假接触力。 -->
    <exclude body1="rg_base" body2="rg_gear"/>
    <exclude body1="rg_base" body2="rg_right_finger"/>
    <exclude body1="rg_base" body2="rg_left_finger"/>
    <exclude body1="rg_gear" body2="rg_right_finger"/>
    <exclude body1="rg_gear" body2="rg_left_finger"/>
    <exclude body1="rg_right_finger" body2="rg_left_finger"/>
    <!-- 夹爪与手腕：rg_base 装在 wrist3_Link 法兰面下方 51 mm（整机模型同值），
         相对位姿全固定，检查无意义。与 AG95 版处理一致。 -->
    <exclude body1="wrist3_Link" body2="rg_base"/>
    <exclude body1="wrist3_Link" body2="rg_gear"/>
    <exclude body1="wrist3_Link" body2="rg_right_finger"/>
    <exclude body1="wrist3_Link" body2="rg_left_finger"/>
  """
    wrap = wrap[:ai] + rg_excl + wrap[aj:]

    # tendon（AG95 四连杆专用）整段删除；equality 换成 rg 的耦合
    ti = wrap.index("  <tendon>")
    tj = wrap.index("  </tendon>") + len("  </tendon>\n")
    wrap = wrap[:ti] + wrap[tj:]
    qi = wrap.index("  <equality>")
    qj = wrap.index("  </equality>") + len("  </equality>")
    rg_eq = """  <equality>
    <!-- rg 夹爪的齿轮-齿条耦合（来源整机模型）：指节滑移 = polycoef × 齿轮角。
         负号 = 齿轮正转时指节回缩（闭合）。系数即齿轮节圆半径 21 mm 的弧长换算。 -->
    <joint name="rg_couple_right" joint1="rg_right_finger_joint" joint2="rg_gear_joint"
           polycoef="0 -0.02099500 0 0 0" solref="0.004 1" solimp="0.9995 0.9999 0.000001"/>
    <joint name="rg_couple_left" joint1="rg_left_finger_joint" joint2="rg_gear_joint"
           polycoef="0 -0.02099900 0 0 0" solref="0.004 1" solimp="0.9995 0.9999 0.000001"/>
  </equality>"""
    wrap = wrap[:qi] + rg_eq + wrap[qj:]

    # 头部注释里的旧描述同步
    wrap = wrap.replace("AG95 夹爪的物理耦合", "rg 夹爪的物理耦合")
    # keyframe 先占位（稍后按实测 nq/nu 覆写）
    ki = wrap.index("  <keyframe>")
    kj = wrap.index("</keyframe>") + len("</keyframe>")
    wrap = wrap[:ki] + "  <!--KEYFRAME_PLACEHOLDER-->" + wrap[kj:]
    out = os.path.join(DST, "scene_ros2_rg.xml")
    write(out, wrap)

    # ---------- 7. 加载 → 按实测 nq/nu/关节顺序生成 keyframe ----------
    import mujoco
    m = mujoco.MjModel.from_xml_path(out)
    print("4) 载入校验：nq=%d nv=%d nu=%d nbody=%d neq=%d"
          % (m.nq, m.nv, m.nu, m.nbody, m.neq))
    order = []
    for jid in range(m.njnt):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid)
        order.append((nm, int(m.jnt_qposadr[jid]), int(m.jnt_type[jid])))
    print("   关节顺序(qpos)：", ", ".join("%s@%d" % (n, a) for n, a, _ in order))

    Q_REST = [-1.137355, 0.041596, 1.593368, -1.589821, -1.137355, 1.570796]
    qpos = np.zeros(m.nq)
    qpos[:6] = Q_REST
    for nm, adr, _ in order:
        if nm == "rg_gear_joint":
            qpos[adr] = GEAR_OPEN
        elif nm == "rg_right_finger_joint":
            qpos[adr] = POLY_R * GEAR_OPEN
        elif nm == "rg_left_finger_joint":
            qpos[adr] = POLY_L * GEAR_OPEN
    ctrl = np.zeros(m.nu)
    for aid in range(m.nu):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
        if nm in ("shoulder_joint", "upperArm_joint", "foreArm_joint",
                  "wrist1_joint", "wrist2_joint", "wrist3_joint"):
            ctrl[aid] = Q_REST[["shoulder_joint", "upperArm_joint", "foreArm_joint",
                                "wrist1_joint", "wrist2_joint", "wrist3_joint"].index(nm)]
        elif nm == "rg_gear_joint":
            ctrl[aid] = GEAR_OPEN
        else:
            raise RuntimeError("未预期的执行器：%s" % nm)

    kf = ('  <keyframe>\n'
          '    <!-- ready：臂 Q_REST（= go_ready 的 READY）+ 夹爪张开端点。\n'
          '         qpos 必须写满 nq=%d（6 臂 + 齿轮铰链 + 左右指滑轨），\n'
          '         ctrl 写满 nu=%d（6 臂 + rg_gear_joint）；两版本 MuJoCo 都要求写满。\n'
          '         指节滑移按耦合式 polycoef×齿轮角 预置，启动时不产生约束冲量。\n'
          '         生成器：tools/build_rg_gripper_scene.py -->\n'
          '    <key name="ready"\n'
          '         qpos="%s"\n'
          '         qvel="%s"\n'
          '         ctrl="%s"/>\n'
          '  </keyframe>'
          % (m.nq, m.nu,
             " ".join("%.6f" % v for v in qpos),
             " ".join("0" for _ in range(m.nv)),
             " ".join("%.6f" % v for v in ctrl)))
    wrap = read(out).replace("  <!--KEYFRAME_PLACEHOLDER-->", kf, 1)
    write(out, wrap)

    # ---------- 8. 终检：keyframe 可用 + 抓取点/开合 ----------
    m = mujoco.MjModel.from_xml_path(out)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "rg_gear")
    R = d.xmat[gid].reshape(3, 3)
    tip = d.xpos[gid] + R @ np.array([0, 0, 0.10])
    print("5) ready 关键帧校验：载入 OK")
    print("   夹爪抓取点(世界) = %s" % np.round(tip, 4))
    print("   双指间距 = %.1f mm"
          % (np.linalg.norm(d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "rg_right_finger")]
                            - d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "rg_left_finger")]) * 1000))
    return 0


if __name__ == "__main__":
    sys.exit(main())
