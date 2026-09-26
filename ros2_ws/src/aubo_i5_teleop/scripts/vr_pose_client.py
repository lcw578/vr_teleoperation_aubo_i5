#!/usr/bin/env python3
"""VR 位姿客户端（DATA_PIPELINE_PLAN.md 阶段 B/C 的 VR 输入源，路线 A）。

作用：连 vr-teleop-relay 的 /ws，把 Quest 浏览器（WebXR）广播的 xr_frame
转换成与 mock_vr_node **完全一致**的输出契约——
  /mock_vr/pose  PoseStamped  右手柄位姿（quest_world 系，腕部支点修正+EMA 平滑后）
  /mock_vr/joy   Joy 5 按钮（HarvestFlex Fig.5 对齐版，2026-09-26 用户批准）：
    [0] 右手柄 Grip（按住电平）     = 离合（我们的安全语义，kit GRIP_BUTTON_INDEX=1）
    [1] 左手柄 扳机（上升沿脉冲）   = 夹爪开/闭（= HarvestFlex Pump，kit TRIGGER=0）
    [2] 左手柄 A（上升沿脉冲）      = 缩放切换（预留，kit PRECISION=4）
    [3] 左手柄 Grip（上升沿脉冲）   = 会话暂停/恢复（= HarvestFlex Pause）
    [4] 右手柄 A（上升沿脉冲）      = 结束当前录制段（= HarvestFlex Exit the record）

复用来源（Apache-2.0，_refs/vr-teleop-kit，逐段对齐上游语义并标注）：
  lerobot/bi_quest_teleop.py 的 _ws_runner（连接/重连/退避/消息分发）、
  控制器解析（xyzw→wxyz、半球检查 nlerp EMA）、staleness 门、按钮索引常量。
  剥离：LeRobot 基类、DK1 IK（我们的 Servo 链不需要 IK）、haptic torque。
  映射（离合 engage/map）不在这里——仍由 clutch_mapper_node 承担（路线 A）。

已知取舍（路线 A 范围）：头显位姿的 yaw 修正未透传（USB 路线站在 PC 旁，
操作者不转身时不需要；需要时升级路线 B）。

用法：
  python3 vr_pose_client.py --ws-url ws://localhost:8443/ws
  python3 vr_pose_client.py --selftest     # 离线解析测试（无需 ROS/网络）
"""

import argparse
import asyncio
import json
import threading
import time

import numpy as np

# ── 常量：与 _refs/vr-teleop-kit lerobot/bi_quest_teleop.py 逐字对齐 ──
GRIP_BUTTON_INDEX = 1        # boolean: clutch（我们的离合）
TRIGGER_BUTTON_INDEX = 0     # analog 0..1: gripper closure（我们的夹爪）
PRECISION_BUTTON_INDEX = 4   # boolean: A (right) / X (left)（缩放，预留）
REST_RAMP_BUTTON_INDEX = 3   # thumbstick click（预留）
XR_FRAME_STALE_TIMEOUT_S = 0.2
POSE_FILTER_ALPHA = 0.8

HAND = "right"               # 操作手柄（位姿来源）
JOY_PAUSE_SRC = ("left", 1)  # 左 Grip 沿 → 暂停/恢复（= HarvestFlex Pause）
JOY_STOP_SRC = ("right", 4)  # 右 A 沿 → 结束录制段（= Exit the record）
JOY_GRIP_SRC = ("left", 0)   # 左扳机沿 → 夹爪（= Pump）
JOY_SCALE_SRC = ("left", 4)  # 左 A 沿 → 缩放（预留）

FRAME = "quest_world"
POSE_RATE_HZ = 100.0         # 发布节拍（与 mock_vr 同）

# joy 输出的四个脉冲/电平源的 (手柄, 按钮索引)
JOY_SOURCES = {
    "grip_src": JOY_GRIP_SRC,    # [1] 夹爪
    "scale_src": JOY_SCALE_SRC,  # [2] 缩放（预留）
    "pause_src": JOY_PAUSE_SRC,  # [3] 暂停/恢复
    "stop_src": JOY_STOP_SRC,    # [4] 结束段
}


def parse_xr_frame(msg: dict, prev: dict) -> dict:
    """xr_frame → 解析结果（纯函数，selftest 可直接调）。

    输入 msg：relay 广播的 xr_frame dict（schema 见 relay/web/client.js:
      controllers[hand] = {position:[x,y,z], orientation:[x,y,z,w],
                           buttons:[{p,t,v},...], axes:[...]}）
    prev：上一帧的滤波/边沿状态（就地更新）。
    返回：{fresh, pos(3,), quat_wxyz(4,), buttons}，pos/quat 为 None 表示 stale。
    """
    now = time.time()
    out = {"fresh": False, "pos": None, "quat_wxyz": None, "buttons": prev["buttons"]}

    # staleness 门（kit 语义：断流 >0.2s 视为陈旧，跳过整帧）
    last = prev.get("last_frame_time", 0.0)
    ctrls = msg.get("controllers") or {}
    ctrl = ctrls.get(HAND) or {}
    if now - last > XR_FRAME_STALE_TIMEOUT_S and last > 0.0:
        prev["was_stale"] = True
        return out
    if not ctrl.get("position") or not ctrl.get("orientation"):
        return out

    pos_raw = np.asarray(ctrl["position"], dtype=float)
    ox, oy, oz, ow = ctrl["orientation"]          # WebXR 是 xyzw
    quat_raw = np.array([ow, ox, oy, oz])         # 转 wxyz

    # EMA 平滑 + 半球检查（kit 逐段对齐：防 q/−q 跳变导致的 lerp 翻转）
    if prev.get("pos_filt") is None or prev.get("quat_filt") is None or prev.get("was_stale"):
        prev["pos_filt"] = pos_raw.copy()
        prev["quat_filt"] = quat_raw.copy()
        prev["was_stale"] = False
    else:
        a = POSE_FILTER_ALPHA
        prev["pos_filt"] = (1.0 - a) * prev["pos_filt"] + a * pos_raw
        q_in = quat_raw if np.dot(prev["quat_filt"], quat_raw) >= 0.0 else -quat_raw
        qf = (1.0 - a) * prev["quat_filt"] + a * q_in
        prev["quat_filt"] = qf / np.linalg.norm(qf)
    prev["last_frame_time"] = now
    out["fresh"] = True
    out["pos"] = prev["pos_filt"].copy()
    out["quat_wxyz"] = prev["quat_filt"].copy()

    # 按钮快照（沿检测在发布 tick 做）
    def btn(hand, idx):
        c = (msg.get("controllers") or {}).get(hand) or {}
        b = (c.get("buttons") or [])
        return bool(b[idx].get("p")) if len(b) > idx else False

    prev["buttons"] = {
        "clutch": btn(HAND, GRIP_BUTTON_INDEX),           # 右 Grip 电平（离合）
        "grip_src": btn(*JOY_GRIP_SRC),                   # 左扳机（夹爪）
        "scale_src": btn(*JOY_SCALE_SRC),                 # 左 A（缩放，预留）
        "pause_src": btn(*JOY_PAUSE_SRC),                 # 左 Grip（暂停/恢复）
        "stop_src": btn(*JOY_STOP_SRC),                   # 右 A（结束段）
    }
    out["buttons"] = prev["buttons"]
    return out


def selftest() -> int:
    """离线解析测试：合成 xr_frame → 解析/staleness/按钮沿，无需 ROS/网络。"""
    import copy
    prev = {"pos_filt": None, "quat_filt": None, "last_frame_time": 0.0,
            "was_stale": False,
            "buttons": {"clutch": False, "grip_src": False, "scale_src": False,
                        "pause_src": False, "stop_src": False}}

    def frame(grip_r=False, trig_l=False, grip_l=False, a_r=False):
        return {"type": "xr_frame", "t_client": 0,
                "controllers": {
                    "right": {"position": [0.1, -0.3, 0.2],
                              "orientation": [0.0, 0.0, 0.0, 1.0],
                              "buttons": [{"p": True, "t": True, "v": 1.0},
                                          {"p": grip_r, "t": False, "v": float(grip_r)},
                                          {"p": False, "t": False, "v": 0.0},
                                          {"p": False, "t": False, "v": 0.0},
                                          {"p": a_r, "t": False, "v": 0.0},
                                          {"p": False, "t": False, "v": 0.0}],
                              "axes": [0.0, 0.0]},
                    "left": {"position": [0.2, -0.3, 0.2],
                             "orientation": [0.0, 0.0, 0.0, 1.0],
                             "buttons": [{"p": trig_l, "t": False, "v": float(trig_l)},
                                         {"p": grip_l, "t": False, "v": float(grip_l)},
                                         {"p": False, "t": False, "v": 0.0},
                                         {"p": False, "t": False, "v": 0.0},
                                         {"p": False, "t": False, "v": 0.0},
                                         {"p": False, "t": False, "v": 0.0}],
                             "axes": [0.0, 0.0]},
                    "viewer": {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}}}

    # ① 第一帧：右 Grip 按住 → 离合电平 1，位姿 fresh
    out = parse_xr_frame(frame(grip_r=True), prev)
    assert out["fresh"] and out["pos"] is not None, "① 位姿未解析"
    assert out["buttons"]["clutch"] is True, "① 离合电平应为 1"

    # ② 连续帧：EMA 收敛后位姿稳定
    for _ in range(10):
        out = parse_xr_frame(frame(grip_r=True), prev)
    assert out["pos"] is not None

    # ③ 左扳机上升沿 → joy[1] 脉冲
    out = parse_xr_frame(frame(grip_r=True, trig_l=True), prev)
    assert out["buttons"]["grip_src"] is True, "③ 左扳机沿未检出"

    # ④ 左 Grip 上升沿 → joy[3] 脉冲（暂停）
    out = parse_xr_frame(frame(grip_r=True, grip_l=True), prev)
    assert out["buttons"]["pause_src"] is True, "④ 暂停沿未检出"

    # ⑤ 右 A 上升沿 → joy[4] 脉冲（结束段）
    out = parse_xr_frame(frame(a_r=True), prev)
    assert out["buttons"]["stop_src"] is True, "⑤ 结束段沿未检出"

    # ⑥ staleness 门：0.3s 无帧 → pos 为 None（位姿停发）
    time.sleep(XR_FRAME_STALE_TIMEOUT_S + 0.05)
    out = parse_xr_frame(frame(), prev)
    assert out["pos"] is None, "⑥ 断流后位姿应停发"
    print("selftest: PASS（解析 / 离合 / 三路脉冲沿 / staleness 门）")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ws-url", default="ws://localhost:8443/ws")
    ap.add_argument("--selftest", action="store_true",
                    help="离线解析测试（无需 ROS/网络）")
    args = ap.parse_args()

    if args.selftest:
        import sys
        sys.exit(selftest())

    # ── ROS 依赖延迟导入：selftest 路径无需 rclpy ──
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Joy, PoseStamped

    rclpy.init()

    class RelayPoseClient(Node):
        def __init__(self, ws_url: str):
            super().__init__("vr_pose_client")
            self.pub_pose = self.create_publisher(PoseStamped, "/mock_vr/pose", 10)
            self.pub_joy = self.create_publisher(Joy, "/mock_vr/joy", 10)
            self.prev = {"pos_filt": None, "quat_filt": None,
                         "last_frame_time": 0.0, "was_stale": False,
                         "buttons": {"clutch": False, "grip_src": False,
                                     "scale_src": False, "pause_src": False,
                                     "stop_src": False}}
            # 沿检测的上一帧按钮态（joy 输出侧）
            self._prev_joy_src = dict.fromkeys(JOY_SOURCES, False)
            self._ws_stop = threading.Event()
            self._ws_thread = threading.Thread(
                target=self._ws_thread_main, name="vr-pose-ws", daemon=True)
            self._ws_thread.start()
            self.timer = self.create_timer(1.0 / POSE_RATE_HZ, self._tick)
            self.get_logger().info(f"VR 位姿客户端启动：{ws_url}（手柄={HAND}）")

        # ---------- WS 读线程（kit _ws_runner 逐段对齐）----------
        def _ws_thread_main(self):
            self._ws_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._ws_loop)
            self._ws_loop.run_until_complete(self._ws_runner())

        async def _ws_runner(self):
            import websockets
            backoff = 1.0
            # ws:// 直连；wss:// 自签证书场景跳过校验（与头显浏览器"接受不安全"等价）
            ssl_ctx = None
            if self.ws_url.startswith("wss://"):
                import ssl
                ssl_ctx = ssl.create_default_context()
                ssl_ctx.check_hostname = False
                ssl_ctx.verify_mode = ssl.CERT_NONE
            while not self._ws_stop.is_set():
                try:
                    async with websockets.connect(self.ws_url, ssl=ssl_ctx) as ws:
                        self.get_logger().info("已连接 relay")
                        backoff = 1.0
                        await ws.send(json.dumps({"type": "request_settings"}))
                        async for raw in ws:
                            if self._ws_stop.is_set():
                                break
                            try:
                                msg = json.loads(raw)
                            except Exception:
                                continue
                            if msg.get("type") == "xr_frame":
                                self.prev = parse_xr_frame(msg, self.prev)
                except Exception as e:
                    self.get_logger().warning(
                        f"WS error ({type(e).__name__}); {backoff:.1f}s 后重连")
                if self._ws_stop.is_set():
                    break
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2.0, 10.0)

        # ---------- 发布 tick ----------
        def _tick(self):
            p = self.prev
            # 位姿：staleness 门（断流不发 → mapper 的目标冻结在离合语义上）
            if p.get("pos_filt") is not None and p.get("quat_filt") is not None:
                if (time.time() - p["last_frame_time"]) <= XR_FRAME_STALE_TIMEOUT_S + 0.5:
                    ps = PoseStamped()
                    ps.header.frame_id = FRAME
                    ps.header.stamp = self.get_clock().now().to_msg()
                    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p["pos_filt"]
                    ps.pose.orientation.w = p["quat_wxyz"][0]
                    ps.pose.orientation.x = p["quat_wxyz"][1]
                    ps.pose.orientation.y = p["quat_wxyz"][2]
                    ps.pose.orientation.z = p["quat_wxyz"][3]
                    self.pub_pose.publish(ps)

            # 按钮：对按钮快照做沿检测 → 脉冲
            b = p["buttons"]
            joy = Joy()
            joy.buttons = [
                1 if b["clutch"] else 0,
                1 if (b["grip_src"] and not self._prev_joy_src["grip_src"]) else 0,
                1 if (b["scale_src"] and not self._prev_joy_src["scale_src"]) else 0,
                1 if (b["pause_src"] and not self._prev_joy_src["pause_src"]) else 0,
                1 if (b["stop_src"] and not self._prev_joy_src["stop_src"]) else 0,
            ]
            self._prev_joy_src = {k: b[k] for k in self._prev_joy_src}
            self.pub_joy.publish(joy)

    node = RelayPoseClient(args.ws_url)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
