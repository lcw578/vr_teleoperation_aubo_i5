"""从 latency 模式的落盘 CSV 里量阶跃超调与整定时间（升 P 的安全指标）。

latency 模式的发布结构（2026-09-25 核对源码后确认）：每个 rep =
  0.6 s 基准保持 → 发阶跃目标直到检测到运动（~50 ms）→ 下一 rep。
  检测后阶跃目标**不再发布**，但 Servo 会按设计走完最后收到的目标（流断行为，
  见 stream_loss_test.py），所以阶跃响应发生在"目标列恒为阶跃值"的整段里。
因此按【目标列恒值段】切分：基准段（0.6 s）与阶跃段（~1.2 s）交替，
阶跃段就是一次完整响应窗口。

用法：/home/lcw/tomato_robot/.venv/bin/python scripts/check_step_overshoot.py <latency_csv>
"""
import csv
import sys

path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
t = [float(r["sim_t"]) for r in rows]
z = [float(r["ee_z"]) for r in rows]
tz = [float(r["tgt_z"]) for r in rows]

# 恒值段：tgt_z 逐行变化 < 1e-5 的连续区
runs = []
s = 0
for i in range(1, len(rows)):
    if abs(tz[i] - tz[i - 1]) > 1e-5:
        if t[i - 1] - t[s] > 0.3:
            runs.append((s, i))
        s = i
if t[-1] - t[s] > 0.3:
    runs.append((s, len(rows)))

print("检出 %d 个恒值段" % len(runs))
print("rep   方向   阶跃幅值   超调%    整定时间   稳态误差")
print("-" * 60)
overs, settles = [], []
for k in range(1, len(runs)):
    a, b = runs[k]
    pa, pb = runs[k - 1]
    amp = tz[a] - z[pa]                    # 阶跃幅值 = 新目标 − 上一段稳态末端
    if abs(amp) < 0.003:                   # 跳过非阶跃段（如测量窗内的基准保持）
        continue
    tail = z[int(a + 0.7 * (b - a)):b]
    settled = sorted(tail)[len(tail) // 2]
    seg = z[a:b]
    if amp > 0:
        excess = max(seg) - settled
    else:
        excess = settled - min(seg)
    over = excess / abs(amp) * 100.0
    band = 0.05 * abs(amp)
    settle = None
    for j in range(a, b):
        if all(abs(z[m] - settled) <= band for m in range(j, b)):
            settle = t[j] - t[a]
            break
    overs.append(over)
    if settle is not None:
        settles.append(settle)
    print(" %2d    %s   %5.1f mm   %+5.1f%%   %6s s    %+.3f mm"
          % (k, "+" if amp > 0 else "−", abs(amp) * 1000, over,
             ("%.3f" % settle) if settle is not None else "未整定",
             (settled - tz[a]) * 1000))
print()
if overs:
    print("超调：中位 %+.1f%%，最大 %+.1f%%；整定时间中位 %.3f s（%d 个有效 rep）"
          % (sorted(overs)[len(overs) // 2], max(overs),
             sorted(settles)[len(settles) // 2] if settles else float("nan"), len(overs)))
