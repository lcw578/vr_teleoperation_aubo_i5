"""用落盘轨迹里的**目标列**核验 sine 表的每个频率段（不靠猜切窗）。

为什么要它：第一版起效的切窗判据是"滚动标准差 > 3 mm 就算在振荡"，结果把
go_to_pose 的爬行段也切进来（爬行时末端在缓慢移动），算出的"幅值 133 mm /
目标 15 mm"是纯假象；而 P02 报了 -22° 的负滞后（末端超前于命令）与幅值比 1.37
（增益>1），物理上不可能，说明拟合被污染。

本脚本的判据是可验证的：**正弦段里目标每半周期精确回到基准点**（cycles 是整数），
所以"目标距基准 < 50 µm"的时刻就是过零点，过零点间隔直接给出该段的频率。

用法：check_sine_target.py <sine_csv> <base_x> <base_y> <base_z> [amp]
"""
import csv
import math
import sys

path = sys.argv[1]
bx, by, bz = (float(v) for v in sys.argv[2:5])
A = float(sys.argv[5]) if len(sys.argv) > 5 else 0.015

# 注：x/y 也一起判——单独用 z 会把爬行经过基准高度的时刻混进来
rows = list(csv.DictReader(open(path)))
if "tgt_x" not in rows[0]:
    print("CSV 没有目标列（tgt_x）—— 这份轨迹是加目标列之前跑的，无法精确核验。")
    sys.exit(2)

t = [float(r["sim_t"]) for r in rows]
ee = [[float(r["ee_%s" % a]) for a in "xyz"] for r in rows]
tgt = [[float(r["tgt_%s" % a]) for a in "xyz"] for r in rows]
q = [[float(r["q%d" % i]) for i in range(6)] for r in rows]
cmd = [[float(r["cmd%d" % i]) for i in range(6)] for r in rows]
st = [int(r["status"]) for r in rows]
base = [bx, by, bz]

# ── 过零点：目标回到基准（三个轴都在 50 µm 内）──
zc = [i for i in range(len(rows))
      if all(abs(tgt[i][k] - base[k]) < 5e-5 for k in range(3))]
# 聚成事件
events, last = [], -999
for i in zc:
    if t[i] - last > 0.2:
        events.append(t[i])
    last = t[i]
print("检出 %d 个过零点事件（目标精确回到基准 (%.4f, %.4f, %.4f)）" % (len(events), bx, by, bz))
if len(events) < 4:
    print("过零点太少，无法定段。")
    sys.exit(2)


def fit(ts, ys, f):
    w = 2 * math.pi * f
    sc = sum(y * math.sin(w * x) for x, y in zip(ts, ys))
    cc = sum(y * math.cos(w * x) for x, y in zip(ts, ys))
    return 2.0 / len(ts) * math.hypot(sc, cc), math.atan2(cc, sc)


def seg_stats(i0, i1, label):
    ts = [t[i] - t[i0] for i in range(i0, i1)]
    zi = [ee[i][2] - bz for i in range(i0, i1)]
    ti = [tgt[i][2] - bz for i in range(i0, i1)]
    # 频率取该段过零点间隔的倒数
    ev = [x for x in events if t[i0] <= x <= t[i1]]
    if len(ev) < 2:
        print("   %s：过零点不足" % label)
        return
    half = (ev[-1] - ev[0]) / (len(ev) - 1)
    f = 1.0 / (2 * half)
    a_in, p_in = fit(ts, ti, f)
    a_out, p_out = fit(ts, zi, f)
    dphi = p_in - p_out
    while dphi > math.pi:
        dphi -= 2 * math.pi
    while dphi < -math.pi:
        dphi += 2 * math.pi
    lag = max(max(abs(cmd[i][j] - q[i][j]) for j in range(6)) for i in range(i0, i1))
    sts = sorted(set(st[i0:i1]))
    print("   %s  f=%.3f Hz（过零间隔 %.3f s）  输入幅值 %.2f mm  输出幅值 %.2f mm  "
          "→ 幅值比 %.3f | 滞后 %+.1f° = %+.3f s"
          % (label, f, half, a_in * 1000, a_out * 1000, a_out / a_in if a_in else float("nan"),
             math.degrees(dphi), dphi / (2 * math.pi * f)))
    print("        |cmd−q| 峰值 %.4f rad（lag_max=0.15）  状态 %s" % (lag, sts))


# ── 用相邻过零点的间隔把 4 个频率段分出来 ──
# 每个频率段覆盖 [(k/2f) 的连续序列]；间隔由 5,5,...→2,2,...→1,1,...→0.5,0.5... 递减
gaps = [(events[i + 1] - events[i]) for i in range(len(events) - 1)]
print("过零间隔序列（s）：%s" % " ".join("%.2f" % g for g in gaps[:24]))
print()
print("按间隔分层核验：")
# 找出间隔突变的边界（比例 > 1.5 视为换段）
bounds = [0]
for i in range(1, len(gaps)):
    if gaps[i] / gaps[i - 1] > 1.5 or gaps[i - 1] / gaps[i] > 1.5:
        bounds.append(i)
bounds.append(len(gaps))
names = ["段%d" % (k + 1) for k in range(len(bounds) - 1)]
for k in range(len(bounds) - 1):
    a, b = bounds[k], bounds[k + 1]
    i0 = min(i for i in range(len(rows)) if t[i] >= events[a])
    i1 = max(i for i in range(len(rows)) if t[i] <= events[b + 1]) if b + 1 < len(events) else len(rows) - 1
    if i1 - i0 > 100:
        seg_stats(i0, i1, names[k])
