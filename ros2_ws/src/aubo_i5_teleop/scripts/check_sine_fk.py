"""用**关节角 + FK** 核验 sine 表，并与 TF 口径逐段对比。

背景（2026-09-24）：台架的 ee() 走 TF2 的 Python listener，而 /tf 以 197 Hz × 13 个变换
到达、listener 处理不过来时 buffer 落后 0.3–0.5 s 且抖动 ±0.14 s（话题本身只迟 0.003 s）。
sine 的样本时间轴取的就是 TF 的时间戳 → 相位/幅值都可能被污染。
P02 在 0.5/1.0 Hz 报出幅值比 0.68 / 1.38（>1 物理上不可能）与负滞后，重跑完全重现。

/joint_states 是准时的（实测 −0.001 s），所以本脚本用同一份落盘数据里的 q 做 MuJoCo FK
（该模型与厂商 URDF 到机器精度一致）得到**独立于 TF** 的末端轨迹，再做同样的相关法拟合。

切段不靠猜：**目标在每段里每半周期精确回到基准**（cycles 是整数），用目标列定位过零点。

用法：check_sine_fk.py <sine_csv>
"""
import csv
import math
import statistics
import sys

import numpy as np
import mujoco

SC = "/home/lcw/VR_teleoperation/assets/aubo_i5/scene_ros2.xml"
ARM = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
       "wrist1_joint", "wrist2_joint", "wrist3_joint"]
OFF = np.array([-0.0405, -0.0143, 0.1492])

m = mujoco.MjModel.from_xml_path(SC)
d = mujoco.MjData(m)
qadr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM]
dofs = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM]
bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")


def fk(q):
    for a, v in zip(qadr, q):
        d.qpos[a] = v
    mujoco.mj_forward(m, d)
    R = d.xmat[bid].reshape(3, 3).copy()
    p = d.xpos[bid] + R @ OFF
    jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv))
    mujoco.mj_jac(m, d, jp, jr, p, bid)
    J = np.vstack([jp, jr])[:, dofs]
    s = np.linalg.svd(J, compute_uv=False)
    return p, s[0] / s[-1], int(d.ncon)


def fit(ts, ys, f):
    w = 2 * math.pi * f
    sc = sum(y * math.sin(w * x) for x, y in zip(ts, ys))
    cc = sum(y * math.cos(w * x) for x, y in zip(ts, ys))
    return 2.0 / len(ts) * math.hypot(sc, cc), math.atan2(cc, sc)


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
if "tgt_z" not in rows[0]:
    print("这份 CSV 没有目标列，请用带 tgt_* 列重跑的 sine 轨迹。")
    sys.exit(2)

t = np.array([float(r["sim_t"]) for r in rows])
tz = np.array([float(r["tgt_z"]) if r["tgt_z"] not in ("", "nan") else np.nan for r in rows])
eez_tf = np.array([float(r["ee_z"]) for r in rows])
cmd = np.array([[float(r["cmd%d" % i]) for i in range(6)] for r in rows])
q = np.array([[float(r["q%d" % i]) for i in range(6)] for r in rows])
st = np.array([int(r["status"]) for r in rows])

eez_fk = np.full(len(rows), np.nan)
conds = np.full(len(rows), np.nan)
ncon = np.zeros(len(rows), dtype=int)
for i in range(len(rows)):
    if not np.all(np.isfinite(q[i])):
        continue
    p, c, nc = fk(q[i])
    eez_fk[i] = p[2]
    conds[i] = c
    ncon[i] = nc

print("样本 %d，仿真时长 %.1f s" % (len(rows), t[-1] - t[0]))
print("FK 口径：cond 最大 %.1f（中位 %.1f，超 200 的 %d 个），真实接触样本 %d 个"
      % (np.nanmax(conds), np.nanmedian(conds), int((conds > 200).sum()), int((ncon > 0).sum())))
print()
print("  段   f      时长   输入幅值  TF幅值  TF比  FK幅值  FK比   TF滞后   FK滞后    |cmd−q|峰  状态")
print("-" * 122)

# 切段：先按**目标 y** 分位姿（正弦只改 z，所以同一位姿内 tgt_y 是常数；爬行段 y 在变，
# 自动被排除），再在每位姿内用"目标 z 回到该位姿的中心"找过零点。
ty = np.array([float(r["tgt_y"]) if r["tgt_y"] not in ("", "nan") else np.nan for r in rows])
A = 0.015
FREQS = [0.1, 0.25, 0.5, 1.0]
CYC = 4.0                                    # sine 每段跑 4 个周期

# 锚点：1 Hz 段每秒钟方向反转 ≥3 次（爬行段 ≤1）。用它定位 68 s 的测量段起点。
dz = np.diff(tz)
dz = np.where(np.abs(dz) > 5e-5, dz, 0.0)     # 死区：忽略静止抖动
sgn = np.sign(dz)
flip = np.zeros(len(tz), dtype=int)
for i in range(1, len(sgn)):
    if sgn[i] != 0 and sgn[i - 1] != 0 and sgn[i] != sgn[i - 1]:
        flip[i] = 1
W = 125
rate = np.array([flip[max(0, i - W):i + 1].sum() for i in range(len(rows))])
cand = np.where(rate >= 2)[0]   # 1 Hz 正弦每秒 2 次方向反转（峰+谷）；爬行段 0–1 次
print("1 Hz 段候选样本 %d 个" % len(cand))
if len(cand) < 50:
    print("找不到 1 Hz 段，无法用结构锚点定段。")
    sys.exit(2)
# 连续区
blocks, cur = [], [cand[0]]
for i in cand[1:]:
    if i - cur[-1] <= 25:
        cur.append(i)
    else:
        blocks.append(cur); cur = [i]
blocks.append(cur)
blocks = [b for b in blocks if len(b) > 100]
print("1 Hz 段连续区：%d 个，长度 %s" % (len(blocks), [len(b) for b in blocks]))
print()
print("  位姿     段   f      时长   输入幅值  TF幅值  TF比  FK幅值  FK比   TF滞后   FK滞后   |cmd−q|峰  状态")
print("-" * 128)
k = 0
for bi, blk in enumerate(blocks, 1):
    t_anchor = t[blk[0]]                       # 1 Hz 段起点
    t_start = t_anchor - (CYC / 0.1 + CYC / 0.25 + CYC / 0.5)   # 测量段起点
    pv = ty[blk[0]]
    for fi, f in enumerate(FREQS):
        dur = CYC / f
        off0 = sum(CYC / FREQS[j] for j in range(fi))
        i0 = int(np.argmin(np.abs(t - (t_start + off0))))
        i1 = int(np.argmin(np.abs(t - (t_start + off0 + dur))))
        if i1 <= i0:
            continue
        sel = [i for i in range(i0, i1) if np.isfinite(eez_fk[i])]
        if len(sel) < 100:
            continue
        ts = t[sel] - t[sel[0]]
        inp = tz[sel] - tz[sel[0]]
        tf_o = eez_tf[sel] - eez_tf[sel[0]]
        fk_o = eez_fk[sel] - eez_fk[sel[0]]
        a_in, p_in = fit(ts, inp, f)
        a_tf, p_tf = fit(ts, tf_o, f)
        a_fk, p_fk = fit(ts, fk_o, f)
        lag = max(max(abs(cmd[i][j] - q[i][j]) for j in range(6)) for i in sel)
        k += 1
        print("  %+.4f  %2d  %.3f  %5.1f s  %6.2f  %6.2f  %5.2f  %6.2f  %5.2f   %+5.1f°   %+5.1f°    %.4f    %s"
              % (pv, k, f, t[i1] - t[i0], a_in * 1000, a_tf * 1000,
                 a_tf / a_in if a_in else float("nan"), a_fk * 1000,
                 a_fk / a_in if a_in else float("nan"),
                 math.degrees(wrap(p_in - p_tf)), math.degrees(wrap(p_in - p_fk)),
                 lag, sorted(set(st[i0:i1]))))
print()
print("TF比 = 用 TF 读到的末端做的幅值比；FK比 = 用同一份 q 做 FK 得到的幅值比（不经过 TF）。")
print("若 FK 比 ≈ 1.0 而 TF 比明显偏离，则该异常来自 TF 通道（listener 积压 + 时基抖动），不是被控对象。")
