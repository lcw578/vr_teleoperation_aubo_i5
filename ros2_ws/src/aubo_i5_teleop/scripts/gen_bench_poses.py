#!/usr/bin/env python3
"""生成跟随台架用的位姿表（任务工作空间内，按条件数分层）。

**为什么这样选**（2026-09-24 定的方法）：
之前所有测试都在 READY 一个位姿附近（距基座 0.890 m ≈ 满臂展 0.8865 m，体积加权约 73% 分位），
覆盖的工作空间体积只有 **1.02%**（±16 cm 内，|det J| 体积加权）——覆盖面太小，结论不可外推。
任务几何来自样机论文（Xu et al., J. Field Robotics 2026, "Tomato Bunch Harvesting Robot"）：
  · 机械臂基座到种植平面 y0 = 800 mm；行间距 1400–1800 mm；采摘高度离地 1100–1500 mm
  · 末端执行器长 160 mm；**最大可采果柄距离 944 mm（距基座原点）**，"低于 944 mm 定义为可采"
  · 采摘半径要求 ≥ 800 mm
所以在**基座坐标系**里，任务区是：种植面 y ∈ [−0.9, −0.7] m、|Δz| ≤ √(944²−800²) = 501 mm、
距原点 700–944 mm，且工具轴近水平（采果柄的姿态）。

选取：在任务区内采样位形 → 算条件数（σmax/σmin，6×6 臂雅可比，取点 = 夹持点）→
**按条件数分层**取点（不是随手挑），每层再用"最远点贪心"保证几何分散；
另取 2 个 cond > 200 的作为"已知受限"对照（预期会触发 Servo 奇异急停，用来确认保护有效，不算失败）。

用法（需要 mujoco+numpy，本机用 /home/lcw/tomato_robot/.venv/bin/python）：
    python gen_bench_poses.py                    # 生成并写入 config/bench_poses.yaml
    python gen_bench_poses.py --verify           # 只做反向校验（读回 YAML 重算）
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import mujoco
import yaml

ASSETS = Path("/home/lcw/VR_teleoperation/assets/aubo_i5")
SCENE = ASSETS / "scene_ros2.xml"
OUT = Path("/home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop/config/bench_poses.yaml")

ARM = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
       "wrist1_joint", "wrist2_joint", "wrist3_joint"]
# 任务区（基座坐标系，米）
DIST_LO, DIST_HI = 0.700, 0.944
Y_LO, Y_HI = -0.90, -0.70          # 种植面在基座前方 800 mm
Z_ABS_MAX = 0.501                  # sqrt(944^2 - 800^2)
TOOL_HORIZ = 0.5                   # |工具轴的世界 z 分量| < 此值 = 近水平
Q_ABS_MAX = 1.6                    # 每个关节角的绝对值上限（rad）
# ⚠️ 为什么要有这一条（2026-09-24 踩坑）：第一版只按"末端位置在任务区"筛，
#    关节空间均匀采样会选出 foreArm≈2.95 rad 这种"折起来"的极端构型（限位是 ±3.04）。
#    从 READY 走过去必然撞关节限位/奇异，Servo 报 `关节到界` 后**整段不动**——
#    实测 12 个位姿里 10 个末端 p2p = 0。任务区是给"人怎么够到果子"用的，
#    构型也必须是合理构型，不能只要末端位置对。
Q_MARGIN_FROM_LIMIT = 3.04 - Q_ABS_MAX
READY_Q = [-1.089254, -0.802598, 1.308255, 0.588418, 0.324607, -0.704605]
# ↑ 与 go_ready.py 的 READY / scene_ros2.xml 的 ready 关键帧保持一致
SIGMA_Q = 0.28                     # 在家位姿附近的采样幅度（rad/关节）
HARD_STOP = 200.0                  # Servo 的 hard_stop_singularity_threshold
PATH_COND_MAX = 150.0              # 路径上允许的最大条件数（留 25% 余量，避免贴着阈值）
BINS = [(0, 25), (25, 40), (40, 60), (60, 100), (100, 200)]   # 前 5 层各取 2 个
N_HARD = 2                         # 另取 2 个 cond>200 的对照


def load():
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    dofs = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM]
    qadr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM]
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")
    off = np.array([-0.0405, -0.0143, 0.1492])       # ag95_base_link -> gripper_tip_link
    return m, d, dofs, qadr, bid, off


def fk(m, d, dofs, qadr, bid, off, q):
    for a, v in zip(qadr, q):
        d.qpos[a] = v
    mujoco.mj_forward(m, d)
    # ⚠️ 必须 .copy()：d.xmat[...].reshape(3,3) 是**视图**，下一次 mj_forward 会改写它。
    #    不拷贝的话，所有存下来的 R 会全部变成最后一个位形的姿态（位置对、姿态错——
    #    反向校验就是靠这个抓出来的）。
    R = d.xmat[bid].reshape(3, 3).copy()
    p = d.xpos[bid] + R @ off
    jp = np.zeros((3, m.nv))
    jr = np.zeros((3, m.nv))
    mujoco.mj_jac(m, d, jp, jr, p, bid)
    J = np.vstack([jp, jr])[:, dofs]
    s = np.linalg.svd(J, compute_uv=False)
    # mj_forward 里算过了碰撞，d.ncon 可直接读（场景里有 9 对 SRDF 来的 contact exclude）
    return p, R, s[0] / s[-1], int(d.ncon)


def mat_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        w = 0.5 * np.sqrt(1 + tr)
        x = (R[2, 1] - R[1, 2]) / (4 * w)
        y = (R[0, 2] - R[2, 0]) / (4 * w)
        z = (R[1, 0] - R[0, 1]) / (4 * w)
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            x = 0.5 * np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / (4 * x)
            y = (R[0, 1] + R[1, 0]) / (4 * x)
            z = (R[0, 2] + R[2, 0]) / (4 * x)
        elif i == 1:
            y = 0.5 * np.sqrt(1 - R[0, 0] + R[1, 1] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / (4 * y)
            x = (R[0, 1] + R[1, 0]) / (4 * y)
            z = (R[1, 2] + R[2, 1]) / (4 * y)
        else:
            z = 0.5 * np.sqrt(1 - R[0, 0] - R[1, 1] + R[2, 2])
            w = (R[1, 0] - R[0, 1]) / (4 * z)
            x = (R[0, 2] + R[2, 0]) / (4 * z)
            y = (R[1, 2] + R[2, 1]) / (4 * z)
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def sample(n_want, seed=42):
    m, d, dofs, qadr, bid, off = load()
    rng = np.random.default_rng(seed)
    out = []
    tried = 0
    # ⚠️ 在 home 附近做**局部**采样，不是全关节空间均匀采样：
    #    任务区（正前方 700–944 mm）相对 home 是"附近"，全局均匀采样会选出
    #    需要大范围摆过去、路径穿过奇异的构型（实测 12 个里 10 个走不过去）。
    while len(out) < n_want and tried < 400000:
        tried += 1
        q = np.array(READY_Q) + rng.normal(0.0, SIGMA_Q, 6)
        p, R, c, ncon = fk(m, d, dofs, qadr, bid, off, q)
        if ncon != 0:                       # 与地板/自身有接触
            continue
        if not (DIST_LO <= np.linalg.norm(p) <= DIST_HI):
            continue
        if not (Y_LO <= p[1] <= Y_HI):
            continue
        if abs(p[2]) > Z_ABS_MAX:
            continue
        if abs(R[2, 2]) >= TOOL_HORIZ:      # 工具轴（夹爪伸出方向）要近水平
            continue
        if np.max(np.abs(q)) > Q_ABS_MAX:   # 构型要合理（远离关节限位）
            continue
        out.append(dict(q=q, p=p, R=R, cond=c))
    return out, tried


def _quat_of(R):
    """旋转矩阵 -> 四元数，**[x, y, z, w] 约定**（与 mat_to_quat 一致）。
    ⚠️ mju_mat2Quat 的原生输出是 MuJoCo 的 [w, x, y, z]（w 在前）！
    第一版直接把它的下标 3 当 w 用，近恒等旋转被读成 ang≈π（acos(z≈0)），
    IK 第一步就发散、q 全撞限位——12 个候选位姿因此全部被误判"路径碰撞"。"""
    qw = np.zeros(4)
    mujoco.mju_mat2Quat(qw, np.ascontiguousarray(R).flatten())
    return np.array([qw[1], qw[2], qw[3], qw[0]])


def _mat_of(q):
    """[x, y, z, w] 四元数 -> 旋转矩阵（mju_quat2Mat 要 [w, x, y, z] 输入）。"""
    qw = np.array([q[3], q[0], q[1], q[2]])
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.ascontiguousarray(qw))
    return R.reshape(3, 3)


def _slerp(qa, qb, t):
    d = float(np.dot(qa, qb))
    if d < 0:
        qb = -qb
        d = -d
    if d > 0.9995:
        out = qa + t * (qb - qa)
    else:
        th = math.acos(max(-1.0, min(1.0, d)))
        out = (math.sin((1 - t) * th) * qa + math.sin(t * th) * qb) / math.sin(th)
    return out / np.linalg.norm(out)


def path_ok(m, d, dofs, qadr, bid, off, q_goal, n_step=40):
    """从 READY 沿**笛卡尔直线**（位置 lerp + 姿态 slerp）用阻尼最小二乘 IK 跟踪，
    逐点检查条件数与碰撞，返回路径最大条件数（碰撞返回 inf）。

    ⚠️ 2026-09-24 两处更正：
    · 原来做的是**关节空间直线插值** (1-t)·READY + t·q_goal——但 Servo 是笛卡尔
      局部控制器，台架 go_to_pose 发的也是笛卡尔直线小步，机械臂根本不沿关节直线
      走：那条检查验证了一条不会被执行的路径。现在改成与台架命令一致的笛卡尔
      路径，IK 从上一路径点的解连续跟踪（模拟 Servo 局部解的分支连续性）。
    · 旋转误差必须算在**世界系**（R_tgt·R_cur^T 的轴角）：mj_jac 的角速度部分
      就是世界系。第一版用了体系 (R_cur^T·R_tgt)，方向反了，IK 第一步就发散
      （表现为 foreArm↔wrist2 假碰撞、路径 cond 冲到 15 万）。

    ⚠️ 另一个教训（2026-09-24）：写 yaml 时曾把循环变量 s 写成上一个循环残留的
      s_，导致 8 个位姿的 path_cond_max 全记成 NaN——结论没有落盘等于没有做。
    """
    p0, R0, _, _ = fk(m, d, dofs, qadr, bid, off, READY_Q)
    p1, R1, _, _ = fk(m, d, dofs, qadr, bid, off, q_goal)
    qa, qb = _quat_of(R0), _quat_of(R1)
    q = np.array(READY_Q, float)
    worst = 0.0
    for k in range(1, n_step + 1):
        t = k / n_step
        p_t = p0 + t * (p1 - p0)
        R_t = _mat_of(_slerp(qa, qb, t))
        for _ in range(80):
            p_cur, R_cur, _, _ = fk(m, d, dofs, qadr, bid, off, q)
            e_pos = p_t - p_cur
            R_err = R_t @ R_cur.T
            q_err = _quat_of(R_err)
            ang = 2.0 * math.acos(max(-1.0, min(1.0, float(q_err[3]))))
            if ang > 1e-9:
                e_rot = q_err[:3] / math.sin(ang / 2.0) * ang
            else:
                e_rot = np.zeros(3)
            e = np.concatenate([e_pos, e_rot])
            if np.linalg.norm(e) < 1e-7:
                break
            jp = np.zeros((3, m.nv))
            jr = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jp, jr, p_cur, bid)
            J = np.vstack([jp, jr])[:, dofs]
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), e)
            q = np.clip(q + dq, -3.04, 3.04)
        _, _, c, ncon = fk(m, d, dofs, qadr, bid, off, q)
        if ncon != 0:
            return float("inf")      # 路径上有碰撞：直接判不可达
        worst = max(worst, c)
    return worst


def pick(pool, bins, per_bin, n_hard):
    """按条件数分层 + 层内最远点贪心（几何分散）。

    ⚠️ 用下标而不是 dict 本身做"已选"标记：dict 里含 numpy 数组，
    `s in picked` 会走 `==` 得到数组，报 "truth value is ambiguous"。
    """
    chosen = []
    for lo, hi in bins:
        idx = [i for i, s in enumerate(pool) if lo <= s["cond"] < hi]
        if not idx:
            continue
        # ⚠️ 起点要取**该层的位置中位点**，不能取最远点：取最远点会让所有层都被拉到
        #    可达边界上（实测 12 个位姿全落在 924–944 mm，而任务区在 z≈0 处本来有
        #    700–800 mm 的位姿）。从中间起，再用最远点贪心铺开。
        cen = np.mean([pool[i]["p"] for i in idx], axis=0)
        picked = [min(idx, key=lambda i: float(np.linalg.norm(pool[i]["p"] - cen)))]
        while len(picked) < per_bin and len(picked) < len(idx):
            rest = [i for i in idx if i not in picked]
            picked.append(max(rest, key=lambda i: min(
                float(np.linalg.norm(pool[i]["p"] - pool[t]["p"])) for t in picked)))
        chosen.extend(pool[i] for i in picked)
    hidx = [i for i, s in enumerate(pool) if s["cond"] > 200]
    if hidx:
        hidx.sort(key=lambda i: -pool[i]["cond"])
        chosen.append(pool[hidx[0]])
        if n_hard > 1 and len(hidx) > 1:
            chosen.append(pool[max(hidx[1:], key=lambda i: float(
                np.linalg.norm(pool[i]["p"] - pool[hidx[0]]["p"])))])
    return chosen


def write(poses):
    doc = {
        "meta": {
            "purpose": "跟随台架（follow_bench.py）用的位姿表",
            "frame": "world（= MoveIt 的 planning_frame）",
            "task_region": "基座坐标系：距原点 700–944 mm、种植面 y∈[-0.9,-0.7] m、"
                           "|Δz|≤501 mm、工具轴近水平",
            "source": "任务几何取自 Xu et al., J. Field Robotics 2026（番茄串采摘机器人）；"
                      "位姿由 gen_bench_poses.py 在已审计的 MJCF 上采样并按条件数分层选出",
            "selection": "条件数分层（0-25/25-40/40-60/60-100/100-200 各 2 个）"
                         "＋ 2 个 cond>200 的对照（预期触发奇异急停，用于确认保护有效）",
            "condition_number": "6×6 臂雅可比（取点=夹持点 gripper_tip_link）的 σmax/σmin",
            "note": "q 是采样出的关节角，position/orientation 由 FK 导出——"
                    "反向校验会从 q 重算并与表里的值比对",
        },
        "poses": poses,
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False))
    return OUT


def verify():
    data = yaml.safe_load(OUT.read_text())
    m, d, dofs, qadr, bid, off = load()
    print("反向校验：从表里的 q 重算 FK 位姿与条件数")
    print("判据是**舍入感知**的：YAML 里位置存 6 位、四元数 8 位、cond 存 3 位，"
          "所以位置差 ~1e-7 正常；")
    print("cond 用相对判据——接近奇异的点 cond~1e4，对 q 的 1e-6 舍入会放大成 ~1e-3 的相对差，"
          "这是放大效应不是错。")
    print("%-22s %10s %10s %12s %12s %s"
          % ("位姿", "位置Δ[m]", "姿态Δ", "cond表", "cond相对Δ", "判定"))
    ok_all = True
    for ps in data["poses"]:
        p, R, c, ncon = fk(m, d, dofs, qadr, bid, off, ps["q"])
        q = mat_to_quat(R)
        dp = float(np.max(np.abs(p - np.array(ps["position"]))))
        dq = float(min(np.max(np.abs(q - np.array(ps["orientation"]))),
                       np.max(np.abs(q + np.array(ps["orientation"])))))
        dcr = abs(c - ps["condition_number"]) / max(1.0, ps["condition_number"])
        ok = dp < 1e-5 and dq < 1e-6 and dcr < 2e-3
        ok_all &= ok
        print("%-22s %10.2e %10.2e %12.1f %12.2e %s"
              % (ps["name"], dp, dq, ps["condition_number"], dcr, "OK" if ok else "**不匹配**"))
    print()
    print("✅ 全部一致" if ok_all else "❌ 有不一致，检查生成逻辑")
    return 0 if ok_all else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--per-bin", type=int, default=2)
    args = ap.parse_args()
    if args.verify:
        return verify()

    pool, tried = sample(4000)
    print("任务区内采样到 %d 个位形（尝试 %d 次）" % (len(pool), tried))
    conds = np.array([s["cond"] for s in pool])
    print("  条件数分位：25%% %.1f | 中位 %.1f | 75%% %.1f | 95%% %.1f | 99%% %.1f"
          % tuple(np.percentile(conds, [25, 50, 75, 95, 99])))
    print("  超 50: %.1f%%   超 200: %.1f%%" % (100 * (conds > 50).mean(), 100 * (conds > 200).mean()))
    chosen = pick(pool, BINS, args.per_bin, N_HARD)
    # 路径检查：从 READY 插值过去，整条路径的条件数必须留有余量
    m, d, dofs, qadr, bid, off = load()
    kept, dropped = [], []
    for s_ in chosen:
        w = path_ok(m, d, dofs, qadr, bid, off, s_["q"])
        if w <= PATH_COND_MAX:
            s_["path_cond_max"] = w
            kept.append(s_)
        else:
            dropped.append((s_["cond"], w))
    if dropped:
        fmt = lambda w: ("inf(碰撞)" if w == float("inf") else "%.0f" % w)
        print("路径检查淘汰 %d 个位姿（端点 cond / 路径最大 cond）：%s"
              % (len(dropped), [(round(a), fmt(b)) for a, b in dropped]))
    chosen = kept
    poses = []
    for i, s in enumerate(chosen):
        tag = "hard" if s["cond"] > 200 else "ok"
        poses.append({
            "name": "P%02d_cond%03d_%s" % (i + 1, int(s["cond"]), tag),
            "condition_number": round(float(s["cond"]), 3),
            "position": [round(float(x), 6) for x in s["p"]],
            "orientation": [round(float(x), 8) for x in mat_to_quat(s["R"])],
            "q": [round(float(x), 6) for x in s["q"]],
            "dist_mm": round(float(np.linalg.norm(s["p"]) * 1000), 1),
            "path_cond_max": round(float(s.get("path_cond_max", float("nan"))), 1),  # 2026-09-24 修复：原来误写成上一个循环残留的 s_，8 个值全变 NaN
            "note": ("预期触发奇异急停（对照，不是失败）" if s["cond"] > 200
                     else "正常测点"),
        })
    path = write(poses)
    print("\n已写入 %s（%d 个位姿）" % (path, len(poses)))
    for p in poses:
        print("  %-22s cond=%7.1f  距基座 %5.1f mm  y=%+.3f  z=%+.3f"
              % (p["name"], p["condition_number"], p["dist_mm"],
                 p["position"][1], p["position"][2]))
    print()
    return verify()


if __name__ == "__main__":
    sys.exit(main())
