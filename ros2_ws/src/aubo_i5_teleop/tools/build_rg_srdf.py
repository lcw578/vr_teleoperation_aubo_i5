#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""由 config/teleop.srdf 生成 rg 版 config/teleop_rg.srdf（MoveIt/RViz 侧）。

改三处 + 重建排除对：
  1. manipulator 链末端 gripper_tip_link → rg_tip_link
  2. gripper 组/组状态/end_effector → rg 关节与开合值（−0.474409 张 / +0.463234 合）
  3. 排除对：删掉 AG95 的 99 条里的夹爪部分，按**同构规则**重建——
     rg 四个 link × {互相} ∪ rg 四 link × {ee_link, wrist1/2/3_Link}
     （与 AG95 版同规则：夹爪件彼此、以及与腕部/法兰之间的相邻与恒距对）
用法：python tools/build_rg_srdf.py
"""
from __future__ import annotations

import os
import re
import sys

CONF = "/home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop/config"
SRC = os.path.join(CONF, "teleop.srdf")
OUT = os.path.join(CONF, "teleop_rg.srdf")

AG95_LINKS = {"ag95_base_link", "ag95_body", "left_finger", "left_finger_pad",
              "left_inner_knuckle", "left_outer_knuckle", "right_finger",
              "right_finger_pad", "right_inner_knuckle", "right_outer_knuckle"}
RG_LINKS = ["rg_base", "rg_gear", "rg_right_finger", "rg_left_finger"]
ARM_SIDE = ["ee_link", "wrist1_Link", "wrist2_Link", "wrist3_Link"]

TIP_LINK = "rg_tip_link"
GEAR_OPEN, GEAR_CLOSE = "-0.474409", "0.463234"


def main():
    lines = open(SRC, encoding="utf-8").read().split("\n")
    out, i = [], 0
    n_drop = 0
    while i < len(lines):
        ln = lines[i]
        # 1) 链末端
        if 'tip_link="gripper_tip_link"' in ln:
            out.append(ln.replace('tip_link="gripper_tip_link"', 'tip_link="%s"' % TIP_LINK))
            i += 1
            continue
        # 2) gripper 组
        if '<group name="gripper">' in ln:
            out.append('  <group name="gripper">')
            out.append('    <joint name="rg_base_joint"/>')
            out.append('    <joint name="rg_gear_joint"/>')
            out.append('    <!-- 左右指为 mimic（URDF <mimic> 耦合到 rg_gear_joint），')
            out.append('         不单独作驱动关节列出。 -->')
            out.append('    <joint name="rg_right_finger_joint"/>')
            out.append('    <joint name="rg_left_finger_joint"/>')
            out.append('  </group>')
            while "</group>" not in lines[i]:
                i += 1
            i += 1
            continue
        # 3) group_state open/closed
        m = re.match(r'\s*<group_state name="(open|closed)" group="gripper">', ln)
        if m:
            state = m.group(1)
            val = GEAR_OPEN if state == "open" else GEAR_CLOSE
            tag = state.upper()
            out.append('  <!-- 夹爪开合（rg 实测端点：%s）-->' % tag)
            out.append('  <group_state name="%s" group="gripper">' % state)
            out.append('    <joint name="rg_gear_joint" value="%s"/>' % val)
            out.append('  </group_state>')
            while "</group_state>" not in lines[i]:
                i += 1
            i += 1
            continue
        # 4) end_effector
        if "<end_effector" in ln and "ag95" in ln:
            out.append('  <end_effector name="rg" parent_link="%s" group="gripper"/>' % TIP_LINK)
            i += 1
            continue
        # 5) 排除对：删夹爪相关
        if "disable_collisions" in ln:
            names = set(re.findall(r'link[12]="([^"]+)"', ln))
            if names & AG95_LINKS:
                n_drop += 1
                i += 1
                continue
        out.append(ln)
        i += 1

    # 6) 插入 rg 排除对（放在 </robot> 前）
    ex = ['', '  <!-- ===== rg 夹爪碰撞排除（同构规则重建）===== -->']
    pairs = []
    for a in range(len(RG_LINKS)):
        for b in range(a + 1, len(RG_LINKS)):
            pairs.append((RG_LINKS[a], RG_LINKS[b], "Never"))
    for g in RG_LINKS:
        for a in ARM_SIDE:
            reason = "Adjacent" if a in ("ee_link", "wrist3_Link") else "Never"
            pairs.append((g, a, reason))
    for a, b, r in pairs:
        ex.append('  <disable_collisions link1="%s" link2="%s" reason="%s"/>' % (a, b, r))
    idx = max(i for i, ln in enumerate(out) if "</robot>" in ln)
    out = out[:idx] + ex + out[idx:]

    txt = "\n".join(out)
    hdr = ("<!-- rg 版：由 tools/build_rg_srdf.py 从 teleop.srdf 生成，勿手改。\n"
           "     夹爪 = 自研齿条夹爪（rg），EE = rg_tip_link，开合 = 齿轮角。\n"
           "     臂部分（12 条官方排除 + 组定义）与 teleop.srdf 完全一致。 -->")
    # ⚠️ XML 声明必须是文件第一行——头注释插在它之后（首版插在前面直接解析失败）
    if txt.lstrip().startswith("<?xml"):
        head, rest = txt.split("\n", 1)
        txt = head + "\n" + hdr + "\n" + rest
    else:
        txt = hdr + "\n" + txt
    open(OUT, "w", encoding="utf-8").write(txt)
    print("写出 %s：丢弃 AG95 排除 %d 条，新增 rg 排除 %d 条" % (OUT, n_drop, len(pairs)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
