#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪配置档：AG95（基线）↔ 自研齿条夹爪 rg（并行场景）。

为什么需要：末端体名、抓取点偏移、MJCF 场景文件、FSM 开合值都随夹爪不同，
而这些常量散落在 8 个节点/工具里。集中到本模块，切换只改一个环境变量。

切换方式：环境变量 `TELEOP_GRIPPER`（默认 ag95）。
  export TELEOP_GRIPPER=rg     # 用自研齿条夹爪（scene_ros2_rg.xml）
  unset  TELEOP_GRIPPER        # 回 AG95 基线（scene_ros2.xml）
两个场景文件并存，回旧基线不需要改任何代码。

取值出处：
  · ag95：DH AG95 两指夹爪（Manipulator-Mujoco 移植），抓取点 = ag95_base 系
    [-0.0405,-0.0143,0.1492]（2026-09-24 与 URDF gripper_tip_link 现场对齐验证）。
  · rg：自研齿条夹爪（/data/robot_assembly 整机模型提取，论文 Xu et al. JFR 2026）。
    EE 体取 rg_gear（齿轮），抓取点 = 沿工具轴 +0.10 m（采摘 demo 验证的 TCP 定义）；
    开合端点 2026-10-05 实测：−0.4744 → 74.9 mm（张）/ +0.4632 → 35.5 mm（合）。
"""
from __future__ import annotations

import os

ASSETS = "/home/lcw/VR_teleoperation/assets/aubo_i5"

PROFILES = {
    "ag95": {
        "mjcf": ASSETS + "/scene_ros2.xml",
        "ee_body": "ag95_base",
        "tip_offset": (-0.0405, -0.0143, 0.1492),
        "open": 0.0,          # 命令量 = 外节角（tendon 合成量）
        "close": 0.9,
        "grasp_joint": "left_outer_knuckle_joint",
        "tip_link": "gripper_tip_link",
        "desc": "DH AG95 两指夹爪（基线）",
    },
    "rg": {
        "mjcf": ASSETS + "/scene_ros2_rg.xml",
        "ee_body": "rg_gear",
        "tip_offset": (0.0, 0.0, 0.10),
        "open": -0.474409,    # 命令量 = 齿轮角
        "close": 0.463234,
        "grasp_joint": "rg_gear_joint",
        "tip_link": "rg_tip_link",
        "desc": "自研齿条夹爪（整机模型提取，论文 JFR2026）",
    },
}

DEFAULT = "ag95"


def resolve(name: str | None = None) -> dict:
    """返回夹爪档字典（含 'name'）。未指定时读环境变量 TELEOP_GRIPPER，再退回默认。"""
    name = name or os.environ.get("TELEOP_GRIPPER") or DEFAULT
    if name not in PROFILES:
        raise SystemExit("未知夹爪档 %r（可选：%s）" % (name, " / ".join(sorted(PROFILES))))
    p = dict(PROFILES[name])
    p["name"] = name
    return p


def env_note() -> str:
    """一行日志用说明（各节点启动时打印，避免"跑的是哪只夹爪"含糊）。"""
    p = resolve()
    return "夹爪档 = %s（%s；EE 体 %s，场景 %s）" % (
        p["name"], p["desc"], p["ee_body"], os.path.basename(p["mjcf"]))
