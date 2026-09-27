#!/usr/bin/env python3
"""Relay 适配器：vr-teleop-kit relay 的 xr_frame → PoseStamped+Joy → ClutchPoseMapper → /target_pose。

这是 VR 链路的最后一环。契约与 mock_vr_node 完全一致——所以 clutch_mapper_node
**一行不改**就能从键盘 Mock 切到真头显：
  本节点（websockets 订 relay wss://.../ws）→ 解析 xr_frame →
  发 /quest/pose + /quest/joy（与 /mock_vr/* 同 schema）→ clutch_mapper_node 消费。

按键映射（10 buttons/手柄；Quest 3 手柄按钮序按 client.js 的 gamepad 顺序，
语义在首次运行时用 --probe 模式实测打印，按住哪个键哪个变 true 来确认）：
  硬编码约定（与 client.js 的 XRPRESS 标签一致）：见 BTN 常量与下方注释。

design 参考：mock_vr_node 的输出契约（本会话已验收）+ 上游 teleop 进程的
xr_frame 消费方式（server 广播给所有非发送者客户端）。

用法：
  # 前置：relay 在跑（bash scripts/run_relay.sh），头显页面已 Start Teleop
  /home/lcw/tomato_robot/.venv/bin/python scripts/quest_adapter_node.py --probe   # 标定按键
  /home/lcw/tomato_robot/.venv/bin/python scripts/quest_adapter_node.py           # 正常运行
"""
import argparse
import asyncio
import json
import math
import ssl
import sys
import threading
import time

import numpy as np
import mujoco          # R_CALIB 四元数变换需要（曾漏 import → 每帧 NameError → ws 线程死亡）
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy

RELAY_WS = "wss://127.0.0.1:8443/ws"
POSE_TOPIC = "/quest/pose"
JOY_TOPIC = "/quest/joy"
HEAD_YAW_TOPIC = "/quest/head_yaw"   # 头显偏航（rad，世界系），供映射器做接合 yaw 修正
FRAME = "quest_world"

# 每手 10 按钮。client.js 按 WebXR gamepad 顺序上送，标签（v1.0 源码）：
#   0 trigger(食指, 模拟量 v)  1 squeeze/grip(侧键, 模拟量)  2 thumbstick 按压
#   3 A/X  4 B/Y  5 thumbrest 触碰  6 menu  7-9 保留
# 键位实锤（2026-09-27 依据上游源码 client.js L1466 原文：
#   "Quest controller buttons[1] is 'grip' (analog 0..1 .value)"）：
#   0=Trigger(模拟量)  1=Grip(模拟量)  2=摇杆按压  3=A/X  4=B/Y
#   5=thumbrest 触碰  6=menu  7-9=摇杆分量。
# ⚠️ 历史：曾按"原始指纹"把 Grip 定为 7——那是 thumbrest 触碰（手握柄即恒
#   true），造成"不按离合也跟随"与无意接管，导致臂被拖进自折叠陷阱。已改正。
BTN_TRIGGER = 0
BTN_GRIP = 1
BTN_PRIMARY = 3       # A（右手）/ X（左手）
BTN_SECONDARY = 4     # B（右手）/ Y（左手）


def quat_wxyz_to_xyzw(q):
    return [q[1], q[2], q[3], q[0]]


# ── Quest 世界系 → 臂基座系 的固定旋转 ──
# 2026-09-27 采用上游 DEFAULT_R_CALIB（bi_quest_teleop.py L92，臂面对操作者的安装）：
# arm_x = -quest_z（操作者前方）、arm_y = -quest_x（操作者左）、arm_z = +quest_y（上）。
# ⚠️ 曾用 Rz180 diag(-1,-1,1)——用户实测四个方向全部错位（前推→下、上推→左、
# 左推→后），该假设从未被验证过。上游公式对 Quest local-floor 标准语义直接成立。
import numpy as _np
# 2026-09-27 用户实测四方向标定（前推→下、上推→操作者左、左推→后），
# 三组正交观测解出纯旋转（det=+1）：R = Rz(-90°)。
# ⚠️ 语义：用户面朝 RViz 屏幕的坐姿下，Quest 世界系与臂基座系差 -90° 偏航。
# 若将来操作者转身，由 clutch_mapper 的 yaw 修正（接合时锁基准）补偿。
_R_QUEST_TO_ARM = _np.array([[0.0, 1.0, 0.0],
                             [-1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0]])


def rotvec_quest_to_arm(v):
    return _R_QUEST_TO_ARM @ _np.asarray(v, float)


def quat_quest_to_arm_wxyz(qw):
    """[w,x,y,z] 手柄姿态 → 臂基座系姿态（同一 R_CALIB 旋转）。
    q_new = R_CALIB(wxyz) ⊗ q_old，用 mju_mulQuat 保持与 R 矩阵定义严格一致。"""
    Rq = np.zeros(4)
    # R_CALIB 3x3 → 四元数 [w,x,y,z]（mujoco 列主序矩阵约定：3x3 行主序展开）
    mju_mat2Quat_from_R = np.zeros(4)
    mujoco.mju_mat2Quat(mju_mat2Quat_from_R,
                        np.ascontiguousarray(_R_QUEST_TO_ARM.flatten()))
    mujoco.mju_mulQuat(Rq, mju_mat2Quat_from_R, np.asarray(qw, float))
    return Rq


class QuestAdapter(Node):
    def __init__(self, probe: bool):
        super().__init__("quest_adapter")
        self.pub_pose = self.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self.pub_joy = self.create_publisher(Joy, JOY_TOPIC, 10)
        self._lock = threading.Lock()
        self._pose_msg = None
        self._joy_msg = None
        self._n_frames = 0
        self._head_yaw = None
        self.pub_yaw = self.create_publisher(Float64MultiArray, HEAD_YAW_TOPIC, 10)
        self._frame_time = 0.0
        self._last_log = 0.0
        self._probe = probe
        self._probe_shown = set()
        # （收一发一模式，无 timer——见 on_frame 内注释）
        self.get_logger().info(
            "quest_adapter 就绪：%s | %s + %s → clutch_mapper（契约与 mock 相同）"
            % ("探针模式" if probe else "正常模式", POSE_TOPIC, JOY_TOPIC))

    def on_frame(self, msg: dict):
        """websockets 线程回调：把 xr_frame 摆进最新帧槽。"""
        ctrl = msg.get("controllers") or {}
        right = ctrl.get("right") or {}
        left = ctrl.get("left") or {}
        if "position" not in right or "orientation" not in right:
            return
        p_q = np.asarray(right["position"], float)
        q_q = np.asarray(right["orientation"], float)   # [w,x,y,z]
        # Quest 系 → 臂基座系（固定 Rz180）：位置向量直接乘；四元数左乘 Rz180
        p = (_R_QUEST_TO_ARM @ p_q).tolist()
        q = quat_quest_to_arm_wxyz(q_q)
        # Joy：右手柄为主手。buttons = [grip 电平, trigger 电平, A, B]
        rb = right.get("buttons") or []

        def b(i):
            return 1 if i < len(rb) and (rb[i].get("p") or rb[i].get("t")) else 0

        joy = Joy()
        joy.buttons = [b(BTN_GRIP), b(BTN_TRIGGER), b(BTN_PRIMARY), b(BTN_SECONDARY)]
        joy.axes = [float(rb[BTN_TRIGGER]["v"]) if BTN_TRIGGER < len(rb) else 0.0,
                    float(rb[BTN_GRIP]["v"]) if BTN_GRIP < len(rb) else 0.0]

        # 头显偏航（上游 bi_quest_teleop.py L123-126 同款公式，世界系 +Y 向上）
        viewer = msg.get("viewer") or {}
        vo = viewer.get("orientation")
        if vo:
            x, y, z, w = (float(v) for v in vo)
            self._head_yaw = math.atan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (y * y + z * z))

        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p
        # 本节点的 PoseStamped 对外契约是 (x,y,z,w)（geometry_msgs 标准）：
        # 上游 [w,x,y,z] → 标准 (x,y,z,w)
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = \
            quat_wxyz_to_xyzw(q)

        with self._lock:
            self._pose_msg = ps
            self._joy_msg = joy
            self._frame_time = time.time()
            self._n_frames += 1
        # ⚠️ 直接在收帧线程发布（2026-09-27 GIL 饥饿根除）：原 timer 回调方案下
        #    asyncio 收帧线程霸占 GIL，rclpy.spin 线程的 timer/订阅回调全部饿死，
        #    表现为"适配器在收帧但 ROS 零发布"。收一发一：发布率=收帧率（~90Hz），
        #    且天然无缓存帧（stale guard 不再需要）。
        self.pub_pose.publish(ps)
        self.pub_joy.publish(joy)
        yaw = self._head_yaw
        if yaw is not None:
            self.pub_yaw.publish(Float64MultiArray(data=[yaw]))

        if self._probe:
            key = tuple(joy.buttons)
            if key not in self._probe_shown:
                self._probe_shown.add(key)
                self.get_logger().info("probe 按键组合 %s = %s" % (
                    ["grip", "trigger", "A", "B"], joy.buttons))


async def ws_loop(node, uri):
    """连接 relay，收 xr_frame，喂给 node.on_frame。断线 2 秒重连。"""
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    import websockets
    while True:
        try:
            async with websockets.connect(uri, ssl=ctx, max_size=None) as ws:
                node.get_logger().info("已连 relay：%s" % uri)
                async for raw in ws:
                    msg = json.loads(raw)
                    if msg.get("type") == "xr_frame":
                        node.on_frame(msg)
        except Exception as e:
            node.get_logger().warn("relay 断连（%s），2 s 后重连…" % e)
            await asyncio.sleep(2.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", action="store_true", help="打印按键组合（标定按钮语义用）")
    ap.add_argument("--uri", default=RELAY_WS)
    args = ap.parse_args()

    rclpy.init()
    node = QuestAdapter(args.probe)

    # ⚠️ 线程结构修正（2026-09-27）：原来 rclpy.spin 在 daemon 线程、asyncio.run 在
    #   主线程——GIL 争抢下 spin 线程饿死，表现为"适配器在收帧（心跳涨）但 ROS
    #   定时器/回调全部不跑"（/quest/pose 零发布）。修正后 rclpy.spin 放主线程，
    #   asyncio 事件循环放独立线程（daemon=True：主线程退出时随进程强收）。
    import asyncio
    loop = asyncio.new_event_loop()
    ws_thread = threading.Thread(target=loop.run_until_complete,
                                 args=(ws_loop(node, args.uri),), daemon=True)
    ws_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        loop.call_soon_threadsafe(loop.stop)
        node.destroy_node()
        rclpy.shutdown()
        ws_thread.join(timeout=3)
    return 0


if __name__ == "__main__":
    sys.exit(main())
