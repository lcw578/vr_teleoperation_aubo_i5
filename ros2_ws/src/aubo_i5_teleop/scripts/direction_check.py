#!/usr/bin/env python3
"""方向自检：验证手柄→臂基座系的平移映射是否符合方位约定 A（TELEOP_BASELINE.md §2）。

用法（戴头显、页面 Start Teleop 已开、【不要按 Grip】）：
  /home/lcw/tomato_robot/.venv/bin/python scripts/direction_check.py

流程：按提示做 3 次手柄平推（前/右/上，各约 5-10 cm，一次平滑推到位）。
脚本观察 /quest/pose（= R_CALIB 变换后的臂基座系坐标），判定每次推映射到的
世界方向是否符合约定 A：
  前推 → 世界 +y（屏幕里，远离操作者）   右推 → 世界 +x（屏幕右）   上抬 → +z
三向全 ✓ 才算过（退出码 0）。任何 ✗ = Quest 重定向朝向或操作者站位与约定不符：
先重新进入页面（身体面朝操作者前方）再测；仍 ✗ 按 TELEOP_BASELINE §2 处置表查。

本脚本只订阅不发布，与遥操链并行安全；Ctrl-C 退出。
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

POSE_TOPIC = "/quest/pose"
MOVE_THRESH = 0.03      # 有效位移（m）
SETTLE_WIN = 0.4        # 静止判定窗（s）
SETTLE_EPS = 0.005      # 窗内变化阈值（m）
STAGE_TIMEOUT = 30.0    # 每步超时（s）

# (提示, 约定 A 的期望方向)——/quest/pose 已是臂基座系坐标，直接比对
STAGES = [
    ("【1/3】把手柄向前推 5-10 cm（远离你身体的方向），一次平滑推到位后停住",
     np.array([0.0, 1.0, 0.0]), "前推"),
    ("【2/3】把手柄向右推 5-10 cm（你的右手方向），推到位后停住",
     np.array([1.0, 0.0, 0.0]), "右推"),
    ("【3/3】把手柄竖直向上抬 5-10 cm，到位后停住",
     np.array([0.0, 0.0, 1.0]), "上抬"),
]


def fast_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


class DirCheck(Node):
    def __init__(self):
        super().__init__("direction_check")
        self.p = None          # 手柄当前位置（臂基座系，R_CALIB 后）
        self.t = 0.0
        self.create_subscription(PoseStamped, POSE_TOPIC, self._cb, fast_qos())

    def _cb(self, msg):
        self.p = np.array([msg.pose.position.x, msg.pose.position.y,
                           msg.pose.position.z])
        self.t = time.time()


def wait_settled_move(node, anchor):
    """等一次 >3cm 的位移并等它静止，返回净位移向量；超时返回目前累计值（可能 None）。"""
    t0 = time.time()
    crossed = False
    last_d = None
    last_change = None
    while True:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.time()
        if node.p is None or now - node.t > 1.0:
            if now - t0 > STAGE_TIMEOUT:
                return None
            continue
        d = node.p - anchor
        if not crossed:
            if float(np.linalg.norm(d)) > MOVE_THRESH:
                crossed = True
                last_d = d.copy()
                last_change = now
            elif now - t0 > STAGE_TIMEOUT:
                return None
        else:
            if float(np.linalg.norm(d - last_d)) > SETTLE_EPS:
                last_d = d.copy()
                last_change = now
            elif now - last_change > SETTLE_WIN:
                return d
            if now - t0 > STAGE_TIMEOUT:
                return d


def describe_screen(d):
    """把臂基座系位移翻译成屏幕语义（RViz 相机在 -y 侧：屏幕里=+y、屏幕右=+x）。"""
    dx, dy, dz = d
    parts = []
    if abs(dy) > 0.01:
        parts.append("屏幕%s" % ("里(远离你)" if dy > 0 else "外(朝你)"))
    if abs(dx) > 0.01:
        parts.append("屏幕%s" % ("右" if dx > 0 else "左"))
    if abs(dz) > 0.01:
        parts.append("%s" % ("上" if dz > 0 else "下"))
    return " + ".join(parts) if parts else "（几乎没动）"


def judge(d, expected):
    """返回 (是否通过, 结论文字)。判据：与期望方向夹角 <45° 过；>135° 判 180° 反。"""
    n = float(np.linalg.norm(d))
    cosang = float(np.dot(d, expected)) / max(n, 1e-9)
    if cosang > math.cos(math.radians(45)):
        return True, "✓ 符合约定 A"
    if cosang < -math.cos(math.radians(45)):
        return False, "✗ 反了 180°（Quest 重定向朝向或站位与约定相反）"
    return False, "✗ 错位（映射方向偏了 ~90°，查 R_CALIB/重定向）"


def main():
    rclpy.init()
    node = DirCheck()
    print("=" * 64)
    print("方向自检（约定 A：前推=+y 远离你 / 右推=+x / 上抬=+z）")
    print("RViz 相机在 -y 侧：屏幕里 = 世界 +y，屏幕右 = 世界 +x")
    print("=" * 64)
    print("等待 /quest/pose（确认 quest_adapter 在跑、页面已 Start Teleop）…")
    t0 = time.time()
    while node.p is None:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - t0 > 15:
            print("❌ 15 s 没收到手柄位姿——quest_adapter 是否在跑？页面是否已进入？")
            return 1
    print("已收到手柄数据。提示：全程【不要按 Grip】。\n")

    results = []
    for hint, expected, name in STAGES:
        input("按 Enter 开始：%s" % hint)
        # 锚点=当前位姿（等 0.3 s 平均值，抗单帧抖动）
        samples = []
        t1 = time.time()
        while len(samples) < 15 and time.time() - t1 < 2.0:
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.p is not None:
                samples.append(node.p.copy())
            time.sleep(0.02)
        if not samples:
            print("   ❌ 手柄数据中断")
            results.append((name, False, "数据中断"))
            continue
        anchor = np.mean(samples, axis=0)
        d = wait_settled_move(node, anchor)
        if d is None:
            print("   ❌ %d s 内未检测到 >%.0f cm 的位移" % (STAGE_TIMEOUT, MOVE_THRESH * 100))
            results.append((name, False, "未检测到移动"))
            continue
        ok, why = judge(d, expected)
        print("   映射结果: 世界(%+.3f, %+.3f, %+.3f) = %s → %s"
              % (d[0], d[1], d[2], describe_screen(d), why))
        results.append((name, ok, why))

    print("\n" + "=" * 64)
    n_ok = sum(1 for _, ok, _ in results if ok)
    for name, ok, why in results:
        print("  %s %s —— %s" % ("✓" if ok else "✗", name, why))
    print("%d/3 通过" % n_ok)
    if n_ok == 3:
        print("方向映射与约定 A 一致，可以开始遥操（首接合后臂应随手柄自然移动）。")
    else:
        print("处置：重新进入头显页面（身体面朝操作者前方）后重跑本脚本；")
        print("仍 ✗ → 按 TELEOP_BASELINE.md §2 处置表排查。")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if n_ok == 3 else 1


if __name__ == "__main__":
    sys.exit(main())
