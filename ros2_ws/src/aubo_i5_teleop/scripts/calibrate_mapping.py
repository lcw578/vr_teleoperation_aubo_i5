#!/usr/bin/env python3
"""手柄方向标定：自动求解 R_CALIB（Quest 世界系 → 臂基座系的固定旋转）。

⚠️ 2026-09-28 起退役（保留存档，勿用于现役标定）：
  1. 方位约定 A 已定案（TELEOP_BASELINE.md §2），R_CALIB=Rx(+90°) 由约定直接
     解出，不再需要经验标定；日常复核用 scripts/direction_check.py。
  2. 本脚本逻辑有缺陷：arm_delta 是【当前映射】下臂的实际响应，对
     (quest_delta, arm_delta) 解 Kabsch 得到的是现状旋转的复制，不是期望映射
     ——即使跑通也修不了一个错的 R_CALIB。
  3. 正确的经验标定做法（若将来真需要）：让操作者按"期望末端方向"移动，
     用 (原始 quest_delta, 期望 arm_delta) 对求解——原始坐标走
     quest_adapter --raw 的 /quest/pose_raw。


为什么需要这个：坐标映射链的三层旋转（R_CALIB + R_align + R_yaw）中，
R_CALIB 是唯一固定的——但它取决于操作者站位朝向和臂安装方向，无法
理论推导。上游 vr-teleop-kit 的 DEFAULT_R_CALIB 也标注 "Derived empirically"。
本脚本让你做几个已知方向的手柄移动，从数据自动解出正确 R。

标定流程（戴头显，页面 Start Teleop 已开）：
  1. 按 Grip 接合
  2. 按提示移动手柄（每次 6 个方向，每个 3 秒）：
     "请把手柄【向上】移动 10cm"（Quest 世界系 +y）
     "请把手柄【向右】移动 10cm"（Quest 世界系 +x）
     "请把手柄【向前】移动 10cm"（Quest 世界系 -z）
     "请把手柄【向下】移动 10cm"（Quest 世界系 -y）
     "请把手柄【向左】移动 10cm"（Quest 世界系 -x）
     "请把手柄【向后】移动 10cm"（Quest 世界系 +z）
  3. 系统记录手柄移动向量 → 自动解出 Quest→臂基座 的 R_CALIB

注意：这里的"上/右/前"是【Quest 世界系】的方向——就是标定时头显面朝的方向。
你不需要知道 Quest 世界系跟物理方向的关系——脚本会从手柄移动的实测数据里解。

用法：
  /home/lcw/tomato_robot/.venv/bin/python scripts/calibrate_mapping.py --calibrate
  /home/lcw/tomato_robot/.venv/bin/python scripts/calibrate_mapping.py --verify   # 验证
"""
import argparse
import json
import math
import time

import numpy as np


# ═══ 纯数学（无 ROS 依赖，可独立测试）═══

def solve_r_calib(quest_vecs, arm_vecs):
    """从 N 对（Quest 向量, 臂向量）解正交旋转 R。
    quest_vecs[i] @ R^T = arm_vecs[i]（行向量约定）或 R @ quest_vecs[i] = arm_vecs[i]。
    用 Kabsch 算法（SVD）解最优正交旋转。"""
    Q = np.asarray(quest_vecs, dtype=float)   # (N,3)
    A = np.asarray(arm_vecs, dtype=float)     # (N,3)
    # 去质心
    qc = Q.mean(axis=0)
    ac = A.mean(axis=0)
    Qc = Q - qc
    Ac = A - ac
    # Kabsch: H = Qc^T @ Ac; SVD; R = V @ U^T（列向量约定 R@v_quest = v_arm）
    H = Qc.T @ Ac
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R


def quat_wxyz_from_R(R):
    q = np.zeros(4)
    # 行主序 3x3 → mujoco 的 mat2Quat
    import mujoco
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R.flatten()))
    return q


# ═══ 标定数据 ═══

# Quest 世界系的 6 个方向（单位向量）
QUEST_DIRS = {
    "up":    np.array([0, 1, 0.0]),
    "down":  np.array([0, -1, 0.0]),
    "right": np.array([1, 0, 0.0]),
    "left":  np.array([-1, 0, 0.0]),
    "forward": np.array([0, 0, -1.0]),
    "backward": np.array([0, 0, 1.0]),
}


def calibrate_from_observations(quest_movements, arm_movements):
    """从实测的（Quest 移动向量, 臂移动向量）对解 R_CALIB。

    quest_movements: list of 3-vectors，Quest 世界系中的手柄位移
    arm_movements:   list of 3-vectors，臂基座系中的末端位移（FK 计算）
    """
    assert len(quest_movements) == len(arm_movements)
    R = solve_r_calib(quest_movements, arm_movements)
    return R


# ═══ ROS 标定流程（戴头显操作）═══

def ros_calibrate():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    import mujoco

    MJCF = "/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml"
    m = mujoco.MjModel.from_xml_path(MJCF)
    d = mujoco.MjData(m)
    G = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
         "wrist1_joint", "wrist2_joint", "wrist3_joint"]
    qa = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in G]
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")
    TIP_OFF = np.array([-0.0405, -0.0143, 0.1492])

    rclpy.init()
    n = Node("calib_mapping")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    pub = n.create_publisher(PoseStamped, "/target_pose", 10)
    qq = {}
    n.create_subscription(JointState, "/joint_states",
                          lambda m: qq.update({nm: (m.position[i], m.velocity[i])
                                               for i, nm in enumerate(m.name) if nm in G}), 10)

    def fk():
        if len(qq) < 6:
            return None
        for a, j in zip(qa, G):
            d.qpos[a] = qq[j][0]
        mujoco.mj_forward(m, d)
        return d.xpos[bid] + d.xmat[bid].reshape(3, 3) @ np.array([-0.0405, -0.0143, 0.1492])

    # ── 标定动作定义 ──
    # 每个动作： Quest 世界系移动方向（理论）， 持续秒数
    # 操作者只需要"按住 Grip，把手柄往 [指令方向] 移动"
    # 系统记录： Quest 手柄位移, 臂末端位移
    # 注意：臂的位移反映的是当前错误映射下的实际效果，
    # 而 Quest 位移是手柄的真实移动。从多组配对可解出 R_CALIB。
    #
    # ⚠️ 核心思路：让操作者做 3 个正交方向的手柄移动（任意命名），
    #    每次记录 (手柄位移, 臂位移)。三对正交向量完全确定旋转。
    #    不需要知道 Quest 位移在物理上对应什么方向——数学自动搞定。

    print("=" * 60)
    print("手柄方向标定流程")
    print("=" * 60)
    print()
    print("请按以下步骤操作（每步 5 秒）：")
    print("  1. 戴上头显，页面 Start Teleop 已开")
    print("  2. 按住 Grip（接合）")
    print("  3. 听到/看到提示后，把手柄往指定方向移动约 10cm")
    print("  4. 系统自动记录手柄位移和臂末端位移")
    print("  5. 三组正交移动完成后自动解出 R_CALIB")
    print()
    input("准备好后按 Enter 开始...")
    print()

    # 先归位（直接发 home 关节命令到 commands——绕过 Servo）
    pub_cmd = n.create_publisher(
        __import__("std_msgs.msg", fromlist=["Float64MultiArray"]).Float64MultiArray,
        "/forward_command_controller_position/commands", 10)
    READY = [-1.135857, 0.052094, 1.773601, -1.987012, -1.197269, 0.228262]
    from std_msgs.msg import Float64MultiArray
    t0 = time.time()
    while time.time() - t0 < 5:
        pub_cmd.publish(Float64MultiArray(data=READY))
        rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.005)
    print("归位完成")

    # 三组正交手柄移动（在 Quest 世界系中，具体物理方向由头显朝向决定——无所谓）
    # 每组： 移动向量（Quest 世界系）, 指令文字
    calib_moves = [
        (np.array([0, 0, -0.10]), "请把手柄朝你面前平移约 10cm"),
        (np.array([0.10, 0, 0]), "请把手柄往你右边平移约 10cm"),
        (np.array([0, 0.10, 0]), "请把手柄往上平移约 10cm"),
    ]

    quest_movements = []  # 记录手柄实际位移
    arm_movements = []    # 记录臂末端位移

    for i, (quest_move, instruction) in enumerate(calib_moves):
        print("\n── 标定 %d/3：%s ──" % (i + 1, instruction))
        print("   按住 Grip，移动手柄...")

        # 记录手柄起始位姿
        # （从 quest_adapter 的 /quest/pose 读——但适配器发的已经是 Rz180 变换后的！）
        # ⚠️ 问题：quest_adapter 把 Rz180 应用在了 pose 上，所以我们从 /quest/pose
        #    读到的不是原始 Quest 世界系坐标！
        # 修正：标定模式下 quest_adapter 应该发原始 Quest 坐标——但我们不想改它。
        # 替代：从 /quest/pose 读到的位移除以 Rz180 就是原始 Quest 位移。
        # Rz180 = diag(-1,-1,1)，它的逆 = 自身。
        # 原始 Quest 位移 = Rz180^{-1} @ (arm_frame_pose 位移)
        # 但 /quest/pose 的位置已经过了 Rz180（在 adapter 里），
        # 所以原始 Quest 位移 = Rz180 @ /quest/pose 位移（Rz180 逆=自身）。
        #
        # 更简单的方案：标定脚本直接从 relay 的 WebSocket 读原始 xr_frame
        # （没有经过 adapter 的 Rz180 变换）——这才是真正的 Quest 原始数据。

        print("   （标定模式从 relay WebSocket 直接读原始 Quest 位姿）")

        # 记录臂起始位置
        time.sleep(0.5)
        p_start = fk()
        if p_start is None:
            print("   ❌ FK 不可用"); return

        # 让操作者移动 5 秒（同时记录手柄和臂）
        t0 = time.time()
        quest_start = None
        quest_end = None
        arm_start = p_start.copy()

        # WebSocket 异步收手柄位姿
        import asyncio
        import json as json_mod
        import ssl
        import websockets

        quest_positions = []

        async def read_quest(ws):
            nonlocal quest_start, quest_end
            async for raw in ws:
                m = json_mod.loads(raw)
                if m.get("type") == "xr_frame":
                    ctrl = (m.get("controllers") or {}).get("right") or {}
                    if "position" in ctrl:
                        p = np.array(ctrl["position"])
                        quest_positions.append(p)
                        if quest_start is None:
                            quest_start = p
                        quest_end = p

        async def do_calib():
            ctx = ssl.create_default_context()
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
            async with websockets.connect("wss://127.0.0.1:8443/ws",
                                          ssl=ctx, max_size=None) as ws:
                task = asyncio.ensure_future(read_quest(ws))
                await asyncio.sleep(6.0)  # 6 秒窗口：操作者移动手柄
                task.cancel()

        asyncio.get_event_loop().run_until_complete(do_calib())

        # 计算
        if quest_start is not None and len(quest_positions) > 5:
            quest_delta = quest_end - quest_start
            p_end = fk()
            if p_end is not None:
                arm_delta = p_end - arm_start
                quest_movements.append(quest_delta)
                arm_movements.append(arm_delta)
                print("   手柄位移 %s | 臂位移 %s" %
                      (np.round(quest_delta, 4), np.round(arm_delta, 4)))
        else:
            print("   ❌ 未收到手柄数据")

    # ── 解 R_CALIB ──
    if len(quest_movements) >= 3:
        R = calibrate_from_observations(quest_movements, arm_movements)
        print("\n═══ 标定结果 ═══")
        print("R_CALIB = ")
        print(np.round(R, 4))
        print("det = %.4f（应为 ±1；+1=纯旋转，-1=含反射，说明某次移动方向记录反了）"
              % np.linalg.det(R))
        print("\n把这个 R 写入 quest_adapter_node.py 的 _R_QUEST_TO_ARM 变量")
        print("并重启 quest_adapter 节点。")
    else:
        print("❌ 标定数据不足")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    ros_calibrate()
