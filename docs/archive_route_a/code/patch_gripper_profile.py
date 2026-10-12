#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 8 个运行时/验证脚本的"夹爪常量"接到 gripper_profile（TELEOP_GRIPPER 开关）。
幂等：已接入的文件会被跳过。用法：python tools/patch_gripper_profile.py
"""
import os
import re
import sys

SCRIPTS = "/home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop/scripts"
MARK = "# ---- 夹爪档（TELEOP_GRIPPER=ag95|rg）"

BOOTSTRAP = MARK + """：EE 体名/抓取点/场景文件随夹爪切换 ----
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from gripper_profile import resolve as _resolve_gripper  # noqa: E402
GRIP = _resolve_gripper()
"""

AG95_PATH = '"/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml"'
OFF_ARRAY = "np.array([-0.0405, -0.0143, 0.1492])"
OFF_TUPLE = "(-0.0405, -0.0143, 0.1492)"

# 文件 → (插入锚点前的行, [替换对])
JOBS = {
    "clutch_mapper_node.py": (
        "MJCF_MODEL = " + AG95_PATH,
        [(AG95_PATH, 'GRIP["mjcf"]'),
         (OFF_ARRAY, "np.array(GRIP[\"tip_offset\"])"),
         ('"ag95_base"', 'GRIP["ee_body"]')]),
    "lara_style_tracker.py": (
        "MJCF = " + AG95_PATH,
        [(AG95_PATH, 'GRIP["mjcf"]'),
         (OFF_ARRAY + "\n", "np.array(GRIP[\"tip_offset\"])\n"),
         ('"ag95_base"', 'GRIP["ee_body"]')]),
    "controller_axes_viz.py": (
        "MJCF_MODEL = " + AG95_PATH,
        [(AG95_PATH, 'GRIP["mjcf"]'),
         (OFF_ARRAY, "np.array(GRIP[\"tip_offset\"])"),
         ('"ag95_base"', 'GRIP["ee_body"]')]),
    "feel_baseline_probe.py": (
        "MJCF = " + AG95_PATH,
        [(AG95_PATH, 'GRIP["mjcf"]'),
         (OFF_ARRAY, "np.array(GRIP[\"tip_offset\"])"),
         ('"ag95_base"', 'GRIP["ee_body"]')]),
    "follow_bench.py": (
        "MJCF_MODEL = " + AG95_PATH,
        [(AG95_PATH, 'GRIP["mjcf"]'),
         ("TIP_OFF = " + OFF_TUPLE, 'TIP_OFF = tuple(GRIP["tip_offset"])'),
         ('"ag95_base"', 'GRIP["ee_body"]')]),
    "rotation_check.py": (
        "MJCF_MODEL = " + AG95_PATH,
        [(AG95_PATH, 'GRIP["mjcf"]'),
         (OFF_ARRAY, "np.array(GRIP[\"tip_offset\"])"),
         ('"ag95_base"', 'GRIP["ee_body"]')]),
    "vr_mock_acceptance.py": (
        "TIP_OFF = " + OFF_ARRAY,
        [(AG95_PATH, 'GRIP["mjcf"]'),
         (OFF_ARRAY, "np.array(GRIP[\"tip_offset\"])"),
         ('"ag95_base"', 'GRIP["ee_body"]')]),
    "gripper_fsm_node.py": (
        "OPEN_V = 0.0",
        [("OPEN_V = 0.0", 'OPEN_V = GRIP["open"]'),
         ("CLOSED_V = 0.9", 'CLOSED_V = GRIP["close"]')]),
}


def main():
    rc = 0
    for fn, (anchor, subs) in JOBS.items():
        p = os.path.join(SCRIPTS, fn)
        s = open(p, encoding="utf-8").read()
        if MARK in s:
            print("跳过 %-26s（已接入）" % fn)
            continue
        if anchor not in s:
            print("!! %-26s 找不到锚点：%s" % (fn, anchor[:60]))
            rc = 1
            continue
        # ⚠️ 顺序要紧：必须先插 bootstrap（锚点行随后可能被替换掉）
        s = s.replace(anchor, BOOTSTRAP + anchor, 1)
        n_applied = 0
        for old, new in subs:
            if old not in s:
                print("   ⚠ %s：未找到待替换文本 %s" % (fn, old[:50]))
                continue
            s = s.replace(old, new)
            n_applied += 1
        open(p, "w", encoding="utf-8").write(s)
        print("接入 %-26s 替换 %d/%d 项 + bootstrap" % (fn, n_applied, len(subs)))
    return rc


if __name__ == "__main__":
    sys.exit(main())
