"""阶跃响应探针：量不同 P 下的超调 / 整定时间 / 速度峰值（升 P 的安全验收）。

结构（每次阶跃）：
  基准保持 1.0 s → 发 ±20 mm（z）目标 0.40 s → 【硬切】→ 无发布监测 2.0 s。
  切断后 Servo 按设计走完最后收到的目标（stream_loss_test 已验证），所以
  0.4 s 发布 + 2.0 s 监测覆盖了完整响应。每个 P 做 +/−/+ 三步。

量：超调%（越过目标的最深 excursion / 阶跃幅值）、整定时间（进入并保持
±0.5 mm）、max|qd|、max|cmd−q|（钳位是否触发）。

用法：/home/lcw/tomato_robot/.venv/bin/python scripts/step_probe.py --out <csv>
"""
import argparse
import math
import statistics
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).parent))
import follow_bench as FB

AMP = 0.020          # 阶跃幅值 [m]
PUB_S = 0.40         # 目标发布时长
MON_S = 2.0          # 切断后监测时长
BAND = 0.0005        # 整定带 [m]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/step_probe.csv")
    args = ap.parse_args()

    rclpy.init()
    node = FB.Bench(args.out)
    node.spin(1.0)
    e = node.ee()
    if e is None:
        print("❌ 关节没到齐")
        return 1
    print("起点 (%.4f, %.4f, %.4f)" % e[0])

    def hold(pose, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            node.publish_target(pose)
            rclpy.spin_once(node, timeout_sec=0.0)
            node.ee(log=True)
            time.sleep(0.008)

    def qdmax():
        return max(abs(node.q.get(j, (0.0, 0.0))[1]) for j in FB.GROUP)

    def cmdlag():
        e2 = node.ee()
        if not e2 or not node.cmd:
            return 0.0
        q = [node.q.get(j, (float("nan"),))[0] for j in FB.GROUP]
        if not all(math.isfinite(x) for x in q):
            return 0.0
        return max(abs(c - x) for c, x in zip(node.cmd, q))

    print("%-6s %8s %8s %9s %9s %9s" % ("步", "超调%", "整定s", "max|qd|", "max|cmd−q|", "终态误差mm"))
    results = []
    for k, dz in enumerate([AMP, -AMP, AMP], 1):
        base = node.ee()
        tgt = FB.make_target(base[0], base[1], (0.0, 0.0, dz))
        t_start = time.time()
        max_qd = 0.0
        max_lag = 0.0
        t0 = time.time()
        while time.time() - t0 < PUB_S:
            node.publish_target(tgt)
            rclpy.spin_once(node, timeout_sec=0.0)
            node.ee(log=True)
            max_qd = max(max_qd, qdmax())
            max_lag = max(max_lag, cmdlag())
            time.sleep(0.008)
        cut = time.time()
        # 监测（无发布）
        zres = []
        t0 = time.time()
        while time.time() - t0 < MON_S:
            rclpy.spin_once(node, timeout_sec=0.0)
            e2 = node.ee(log=True)
            if e2:
                zres.append((time.time() - cut, e2[0][2]))
            max_qd = max(max_qd, qdmax())
            max_lag = max(max_lag, cmdlag())
            time.sleep(0.004)
        rest_z = statistics.median([z for tt, z in zres if tt > MON_S - 0.5])
        rest_full = node.ee()[0]
        # 超调：越过目标的最深 excursion（沿 z）
        z_start = base[0][2]
        z_tgt = z_start + dz
        zs = [z for _, z in zres]
        if dz > 0:
            excess = max(zs) - z_tgt
        else:
            excess = z_tgt - min(zs)
        over = excess / AMP * 100.0
        # 整定：从切断起，进入并保持 ±BAND
        settle = None
        for j, (tt, z) in enumerate(zres):
            if all(abs(z2 - rest_z) <= BAND for _, z2 in zres[j:]):
                settle = tt
                break
        err = math.dist(rest_full, (z_start, base[0][1], z_tgt))  # 终态与目标差的近似（z 向为主）
        err_z = abs(rest_z - z_tgt) * 1000
        results.append(over)
        print("  %d   %+6.1f%%  %6s  %8.4f  %9.4f     %.3f (z向)"
              % (k, over, ("%.3f" % settle) if settle is not None else "未整定",
                 max_qd, max_lag, err_z))
        hold(FB.make_target(rest_full, base[1], (0, 0, 0)), 0.8)

    print()
    print("超调：中位 %+.1f%%，最大 %+.1f%%（阶跃 %.0f mm × 3）"
          % (sorted(results)[1], max(results), AMP * 1000))
    return 0


if __name__ == "__main__":
    sys.exit(main())
