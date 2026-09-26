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
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy

RELAY_WS = "wss://127.0.0.1:8443/ws"
POSE_TOPIC = "/quest/pose"
JOY_TOPIC = "/quest/joy"
FRAME = "quest_world"

# 每手 10 按钮。client.js 按 WebXR gamepad 顺序上送，标签（v1.0 源码）：
#   0 trigger(食指, 模拟量 v)  1 squeeze/grip(侧键, 模拟量)  2 thumbstick 按压
#   3 A/X  4 B/Y  5 thumbrest 触碰  6 menu  7-9 保留
# ⚠️ 2026-09-26 真头显实测（原始按键指纹分析）：client.js 的按钮序不是标准
# gamepad 序！左手实测指纹：0=Trigger(模拟量)、7=Grip 侧键(恒 1.00 当按住)、
# 4=可按压键、8/9/10=摇杆/触板。0/7 确认；A/B 的索引待 --probe 实测。
BTN_TRIGGER = 0
BTN_GRIP = 7
BTN_PRIMARY = 4       # 待 probe 确认（可能是 A/X）
BTN_SECONDARY = 5     # 待 probe 确认（可能是 B/Y）


def quat_wxyz_to_xyzw(q):
    return [q[1], q[2], q[3], q[0]]


# ── Quest 世界系 → 臂基座系 的固定旋转（2026-09-26 实测定标）──
# 手柄数据实测：操作者自然持柄朝任务区时，位置 y≈+1.0；而臂基座工作区在 y≈-0.8。
# 绕 z 轴转 180°（x,y 取反，z 不变）把"Quest 前方"对到"臂前方"。
# 四元数同样做共轭旋转：q_new = Rz180 ⊗ q_old，用 (x,y,z,w) 全负除 w。
import numpy as _np
_R_QUEST_TO_ARM = _np.diag([-1.0, -1.0, 1.0])


def rotvec_quest_to_arm(v):
    return _R_QUEST_TO_ARM @ _np.asarray(v, float)


def quat_quest_to_arm_wxyz(qw):
    """[w,x,y,z] 手柄姿态 → 臂基座系姿态。
    R_new = Rz180 ⊗ R_old；四元数合成：qw_new = [cos(90°),0,0,sin(90°)] ⊗ qw。
    Rz180 的 [w,x,y,z] = [0,0,0,1]。mju_mulQuat 慢路径不值得——手写。"""
    w, x, y, z = qw
    # (0,0,0,1) ⊗ (w,x,y,z) = (-z, y, -x, w)  [w,x,y,z 约定下的 Rz180 左乘]
    return np.array([-z, y, -x, w])


class QuestAdapter(Node):
    def __init__(self, probe: bool):
        super().__init__("quest_adapter")
        self.pub_pose = self.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self.pub_joy = self.create_publisher(Joy, JOY_TOPIC, 10)
        self._lock = threading.Lock()
        self._pose_msg = None
        self._joy_msg = None
        self._n_frames = 0
        self._frame_time = 0.0
        self._last_log = 0.0
        self._probe = probe
        self._probe_shown = set()
        # 发布节拍：独立 100 Hz timer 把最新帧发出去（位姿流 ~89 Hz，但对齐
        # mock 的 100 Hz 契约；relay 收流与本发布解耦，头显抖动不打断下游节拍）
        self.timer = self.create_timer(0.01, self._publish)
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

        if self._probe:
            key = tuple(joy.buttons)
            if key not in self._probe_shown:
                self._probe_shown.add(key)
                self.get_logger().info("probe 按键组合 %s = %s" % (
                    ["grip", "trigger", "A", "B"], joy.buttons))

    def _publish(self):
        with self._lock:
            ps, joy, n, t = self._pose_msg, self._joy_msg, self._n_frames, self._frame_time
        if ps is None:
            return
        # ⚠️ 断流保护：帧龄 >0.3 s 就停发缓存帧（2026-09-26 实测教训：头显断线后
        #    本节点把最后一帧当活流持续发，下游以为操作者把手柄定在半空不动）
        if time.time() - t > 0.3:
            return
        self.pub_pose.publish(ps)
        self.pub_joy.publish(joy)
        if n and time.time() - self._last_log > 10:
            self._last_log = time.time()
            self.get_logger().info("已收到 %d 帧（~%.0f Hz），最新 grip/trigger = %s/%.2f"
                                   % (n, 100.0, joy.buttons[0], joy.axes[0]))


async def ws_loop(node: QuestAdapter, uri: str):
    """连接 relay，收 xr_frame，喂给 on_frame。断线 2 s 重连。"""
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

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        asyncio.run(ws_loop(node, args.uri))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
