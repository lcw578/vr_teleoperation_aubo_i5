"""断流安全测试（VR 接入前必做的一项）。

场景：操作者头显崩溃 / 网络抖动 → /target_pose 流**中途硬切**。
要回答的四个问题：
  Q1 切断后臂会怎样？（预期：Servo 按设计继续走完【最后一个已收到的目标】，
     然后停住——最后一个目标本身就是操作者确认过的安全位姿）
  Q2 多久停稳？（Servo 的 TARGET_POSE_TIMEOUT = 1.0 s）
  Q3 停住期间命令流和保持误差如何？（servo_interface 保持最后 q_cmd）
  Q4 恢复发布后有没有跳变？（预期：reinit_gap=0.5 s 触发重新对齐，无速度尖峰）

全程用 FK 通道（/joint_states + MuJoCo），不经 TF。

用法：/home/lcw/tomato_robot/.venv/bin/python scripts/stream_loss_test.py
前置：栈在跑（stage2a/2b/3），臂已归位或在某位姿附近。
"""
import math
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).parent))
import follow_bench as FB

GROUP = FB.GROUP


def qd_now(node):
    return [abs(node.q.get(j, (0.0, 0.0))[1]) for j in GROUP]


def wait_quiet(node, seconds, thresh=2e-3):
    """等臂停稳；返回停稳时刻（相对调用起点），超时返回 None。"""
    t0 = time.time()
    last_move = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.0)
        if max(qd_now(node)) > thresh:
            last_move = time.time()
        elif time.time() - last_move > 0.5:
            return time.time() - t0
        time.sleep(0.002)
    return None


def main():
    rclpy.init()
    node = FB.Bench("/tmp/stream_loss.csv")
    node.spin(1.0)
    e = node.ee()
    if e is None:
        print("❌ 六个关节没到齐——栈没起？")
        return 1
    base_p, base_r = e[0], e[1]
    print("起点末端 (%.4f, %.4f, %.4f)" % base_p)
    status_of = lambda: FB.STATUS_MEANING.get(node.status, node.status)

    # ── A. 基线保持 1.5 s ──
    hold = FB.make_target(base_p, base_r, (0, 0, 0))
    t0 = time.time()
    while time.time() - t0 < 1.5:
        node.publish_target(hold)
        rclpy.spin_once(node, timeout_sec=0.0)
        node.ee(log=True)
        time.sleep(0.008)
    qd_a = max(qd_now(node))
    print("A 基线保持：max|qd| = %.5f rad/s，状态 %s" % (qd_a, status_of()))

    # ── B. 发 60 mm 目标 0.35 s 后【硬切】（不发任何东西）──
    away = FB.make_target(base_p, base_r, (0.0, 0.0, 0.060))
    t0 = time.time()
    while time.time() - t0 < 0.35:
        node.publish_target(away)
        rclpy.spin_once(node, timeout_sec=0.0)
        node.ee(log=True)
        time.sleep(0.008)
    e_cut = node.ee()
    cut_wall = time.time()
    moved_before_cut = math.dist(e_cut[0], base_p) * 1000
    print("B 硬切：已发目标 0.35 s 后停止发布；此刻末端已移动 %.1f mm，状态 %s"
          % (moved_before_cut, status_of()))

    # ── C. 断流监测 6 s：什么时候最后一条命令？臂什么时候停？漂移多少？──
    n_cmd_at_cut = len(node.cmd_log)
    t0 = time.time()
    last_cmd_wall = None
    quiet_t = wait_quiet(node, 6.0)
    for t, _ in node.cmd_log[n_cmd_at_cut:]:
        last_cmd_wall = t
    e_rest = node.ee()
    gap_cmd = (last_cmd_wall - cut_wall) if last_cmd_wall else float("nan")
    settle = quiet_t if quiet_t else float("nan")
    print("C 断流后：最后一条命令在切断后 %.3f s；臂在 %.3f s 后停稳；"
          "切断→停稳实际位移 +%.1f mm（到 (%.4f, %.4f, %.4f)）；状态 %s"
          % (gap_cmd, settle,
             math.dist(e_rest[0], e_cut[0]) * 1000, e_rest[0][0], e_rest[0][1], e_rest[0][2],
             status_of()))

    # 停住后继续静置 3 s 测漂移
    hold_p = e_rest[0]
    drift_max = 0.0
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.0)
        node.ee(log=True)
        ee = node.ee()
        if ee:
            drift_max = max(drift_max, math.dist(ee[0], hold_p))
        time.sleep(0.005)
    print("   静置 3 s 漂移：max %.4f mm" % (drift_max * 1000))

    # ── D. 恢复发布：会不会跳？──
    resume = FB.make_target(e_rest[0], e_rest[1], (0, 0, 0))
    n_cmd0 = len(node.cmd_log)
    max_lag = 0.0
    max_qd = 0.0
    t0 = time.time()
    while time.time() - t0 < 1.5:
        node.publish_target(resume)
        rclpy.spin_once(node, timeout_sec=0.0)
        node.ee(log=True)
        if node.cmd:
            e2 = node.ee()
            if e2:
                q = [node.q.get(j, (float("nan"),))[0] for j in GROUP]
                if all(math.isfinite(x) for x in q):
                    max_lag = max(max_lag, max(abs(c - x) for c, x in zip(node.cmd, q)))
        max_qd = max(max_qd, max(qd_now(node)))
        time.sleep(0.005)
    print("D 恢复发布 1.5 s：max|cmd−q| = %.4f rad（lag_max=0.15），max|qd| = %.5f rad/s，状态 %s"
          % (max_lag, max_qd, status_of()))

    print()
    ok = (max_lag <= 0.15) and (max_qd < 0.5) and (drift_max < 0.005)
    print("判定：%s" % ("✅ 断流行为安全（无跳变、保持有效、恢复平滑）" if ok else "⚠️ 有项超界，看上面数字"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
