#!/usr/bin/env python3
"""Mock VR 节点：用键盘（或脚本回放）模拟 Quest 手柄的 6DoF 位姿 + 按键流。

为什么存在：头显未到货前，用它以**与未来 VR 完全一致的话题契约**驱动
clutch_mapper_node（/mock_vr/pose + /mock_vr/joy，100 Hz），把映射层的
数学与状态机全部验证完。头显到了以后，把本节点换成 relay 适配器，
映射器一行不改。

两种模式：
  默认（键盘）   pynput 全局监听。键位：
                   平移（世界系） W/A/S/D + Q(降)/E(升)     0.15 m/s
                   旋转（手柄自系） U/O=roll  I/K=pitch  J/L=yaw   60°/s
                   空格=离合（按住）  Shift=缩放切换(1:1↔1:5)  Z=夹爪开/闭
  --script 文件  回放时间戳段落（自动化验收用），格式（# 开头为注释）：
                   <开始s> <时长s> <动作> [参数]
                     clutch 1|0      0 时长：该时刻置/清离合电平
                     trans vx vy vz  时长内按世界系速度 (m/s) 平移
                     rot  wx wy wz   时长内按手柄自系角速度 (deg/s) 旋转
                     scale           0 时长：发缩放切换脉冲
                     grip            0 时长：发夹爪切换脉冲
                   例：
                     0.0  0.0  clutch 1
                     0.5  1.0  trans 0 -0.10 0
                     0.5  1.0  rot 0 0 45

语义（2026-09-25 已批）：
  · 旋转增量在**手柄自身系**积分（q ← q ⊗ Δq）——"按键 = 转动手腕"；
    平移增量在 Mock 世界系（W = 向任务区前方，即 -y）。
  · 按键位姿是绝对位姿流（与真手柄一致）；缩放在映射器里做（逐 tick 增益），
    本节点 Shift/Z 只发单 tick 按键脉冲，映射器/夹爪 FSM 检测上升沿。
  · buttons: [0]=离合（电平，按住为 1） [1]=夹爪（上升沿脉冲）
             [2]=缩放切换（上升沿脉冲）
"""
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from pynput import keyboard as pynput_kb

# 特殊键表：模块级构建（引用 pynput_kb；曾因 keyboard 只在 _start_keyboard
# 局部作用域而 NameError——每按一次空格/Shift 监听回调就抛一次异常被吞掉，
# 表现为"离合永远按不上"，用户的按键全被焦点窗口消费）
_SPECIAL = {
    pynput_kb.Key.space: "space",
    pynput_kb.Key.shift: "shift",
    pynput_kb.Key.shift_r: "shift",
    pynput_kb.Key.ctrl_l: "ctrl",
    pynput_kb.Key.ctrl_r: "ctrl",
    pynput_kb.Key.esc: "esc",
}
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy

POSE_TOPIC = "/mock_vr/pose"
JOY_TOPIC = "/mock_vr/joy"
FRAME = "quest_world"
RATE_HZ = 100.0
V_TRANS = 0.15          # m/s
W_ROT = 60.0            # deg/s

# 键位表：平移 (x系数, y系数) 或 (None, z系数)；旋转 (轴名, 符号)
TRANS_KEYS = {"w": (0, -1), "s": (0, +1), "a": (-1, 0), "d": (+1, 0),
              "q": (None, -1), "e": (None, +1)}
ROT_KEYS = {"u": ("roll", +1), "o": ("roll", -1),
            "i": ("pitch", +1), "k": ("pitch", -1),
            "j": ("yaw", +1), "l": ("yaw", -1)}
ROT_IDX = {"roll": 0, "pitch": 1, "yaw": 2}


def quat_mul_wxyz(a, b):
    import mujoco
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(a, float), np.asarray(b, float))
    return out


def rotvec_to_quat_wxyz(v):
    import mujoco
    v = np.asarray(v, float)
    ang = float(np.linalg.norm(v))
    if ang < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    out = np.zeros(4)
    mujoco.mju_axisAngle2Quat(out, v / ang, ang)
    return out


class MockVR(Node):
    def __init__(self, script: str | None):
        super().__init__("mock_vr")
        self.pub_pose = self.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self.pub_joy = self.create_publisher(Joy, JOY_TOPIC, 10)
        self.p = np.array([0.0, -0.30, 0.30])       # 虚拟手柄初始位姿（绝对值无意义：
        self.q = np.array([1.0, 0.0, 0.0, 0.0])     #  离合锚定会吸收常量偏置）
        self._held = set()                          # 按住的键（小写）
        self._clutch_level = 0                      # 脚本模式下的持久离合电平
        self._pulse_grip = False                    # 夹爪脉冲（单 tick）
        self._pulse_scale = False                   # 缩放脉冲（单 tick）
        self._prev_shift = False
        self._prev_z = False
        self._script = self._load(script) if script else None
        self._fired = set()                         # 脚本一次性事件已触发集合
        self._t0 = time.time()
        self._last = time.time()
        self._n = 0
        if self._script is None:
            self._start_keyboard()
            self.get_logger().info(
                "键盘模式：WASD+QE 平移 | UO/IK/JL 旋转 | 空格=离合 | Shift=缩放 | Z=夹爪")
        else:
            self.get_logger().info("脚本模式：%d 段" % len(self._script))
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)

    # ---------- 键盘 ----------
    def _start_keyboard(self):
        lis = pynput_kb.Listener(on_press=self._on_key, on_release=self._off_key)
        lis.daemon = True
        lis.start()

    @staticmethod
    def _keyname(key):
        try:
            ch = key.char
            return (ch or "").lower()
        except AttributeError:
            return _SPECIAL.get(key, "")

    def _on_key(self, key):
        k = self._keyname(key)
        if k in TRANS_KEYS or k in ROT_KEYS:
            self._held.add(k)
        elif k == "shift" and not self._prev_shift:
            self._pulse_scale = True
        elif k == "z" and not self._prev_z:
            self._pulse_grip = True
        if k == "shift":
            self._prev_shift = True
        if k == "z":
            self._prev_z = True

    def _off_key(self, key):
        k = self._keyname(key)
        self._held.discard(k)
        if k == "shift":
            self._prev_shift = False
        if k == "z":
            self._prev_z = False

    # ---------- 脚本 ----------
    def _load(self, path):
        segs = []
        for line in Path(path).read_text().splitlines():
            line = line.split("#")[0].strip()
            if not line:
                continue
            f = line.split()
            segs.append((float(f[0]), float(f[1]), f[2], [float(x) for x in f[3:]]))
        return sorted(segs, key=lambda s: s[0])

    # ---------- 每 tick ----------
    def _tick(self):
        now = time.time()
        dt = min(0.05, now - self._last)     # 防调试暂停后的大步长
        self._last = now
        el = now - self._t0

        v = np.zeros(3)
        w = np.zeros(3)
        clutch = 0
        if self._script is None:
            for k in self._held:
                if k in TRANS_KEYS:
                    a, b = TRANS_KEYS[k]
                    if a is None:
                        v[2] += b * V_TRANS
                    else:
                        v[0] += a * V_TRANS
                        v[1] += b * V_TRANS
                elif k in ROT_KEYS:
                    ax, sgn = ROT_KEYS[k]
                    w[ROT_IDX[ax]] += sgn * W_ROT
            clutch = 1 if "space" in self._held else 0
        else:
            for (t0, dur, act, args) in self._script:
                if dur > 0 and t0 <= el < t0 + dur:
                    if act == "trans":
                        v += np.array(args)
                    elif act == "rot":
                        w += np.array(args)
                elif dur == 0 and t0 <= el and t0 not in self._fired:
                    self._fired.add(t0)
                    if act == "clutch":
                        # ⚠️ 离合是**电平**：脚本事件写入持久状态，直到下一条事件
                        #    （2026-09-25 验收抓出的 bug：只生效一 tick → 接合后
                        #    立刻"松开"，映射器反复接合/脱离，臂永远不动）
                        self._clutch_level = int(args[0]) if args else 0
                    elif act == "scale":
                        self._pulse_scale = True
                    elif act == "grip":
                        self._pulse_grip = True
            clutch = self._clutch_level

        # 积分：平移（世界系）+ 旋转（手柄自系）
        self.p = self.p + v * dt
        if np.linalg.norm(w) > 1e-9:
            dq = rotvec_to_quat_wxyz(np.radians(w) * dt)
            self.q = quat_mul_wxyz(self.q, dq)
            self.q /= np.linalg.norm(self.q)

        # 发布
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = self.p
        ps.pose.orientation.w, ps.pose.orientation.x = self.q[0], self.q[1]
        ps.pose.orientation.y, ps.pose.orientation.z = self.q[2], self.q[3]
        self.pub_pose.publish(ps)
        joy = Joy()
        joy.buttons = [clutch,
                       1 if self._pulse_grip else 0,
                       1 if self._pulse_scale else 0]
        self.pub_joy.publish(joy)
        self._pulse_grip = False        # 脉冲只保持一个 tick
        self._pulse_scale = False

        self._n += 1
        if self._n % (int(RATE_HZ) * 10) == 0:
            self.get_logger().info("t=%.1f p=(%.2f,%.2f,%.2f) clutch=%d"
                                   % (el, *self.p, clutch))


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--script", default=None, help="回放脚本（不给则键盘模式）")
    args = ap.parse_args()
    rclpy.init()
    node = MockVR(args.script)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
