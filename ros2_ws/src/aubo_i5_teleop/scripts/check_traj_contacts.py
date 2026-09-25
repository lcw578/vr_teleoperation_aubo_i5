"""从台架落盘的轨迹 CSV 反算真实接触（离线，MuJoCo 3.12.0）。

⚠️ 用 /home/lcw/tomato_robot/.venv/bin/python 跑：系统 python3 的 mujoco 3.3.5
   连场景都编译不过（ready 关键帧 qpos 长度），而且缺 typing_extensions。
   版本必须与 mujoco_ros2_control 链的 mujoco_vendor 3.12.0 一致。

用法：check_traj_contacts.py <csv> [csv2 ...]
"""
import csv
import sys
from pathlib import Path

import numpy as np
import mujoco

ASSETS = Path("/home/lcw/VR_teleoperation/assets/aubo_i5")
ARM = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
       "wrist1_joint", "wrist2_joint", "wrist3_joint"]


def analyze(path):
    m = mujoco.MjModel.from_xml_path(str(ASSETS / "scene_ros2.xml"))
    d = mujoco.MjData(m)
    qadr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM]
    dofs = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM]
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ag95_base")
    bname = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)

    rows = list(csv.DictReader(open(path)))
    if not rows:
        print("%s：空文件" % path)
        return
    t = [float(r["sim_t"]) for r in rows]
    hits, conds, lags = {}, [], []
    for r in rows:
        q = [float(r["q%d" % i]) for i in range(6)]
        cq = [float(r["cmd%d" % i]) for i in range(6)]
        if all(np.isfinite(cq)) and all(np.isfinite(q)):
            lags.append(max(abs(a - b) for a, b in zip(cq, q)))
        if not all(np.isfinite(q)):
            continue
        for a, v in zip(qadr, q):
            d.qpos[a] = v
        mujoco.mj_forward(m, d)
        for i in range(d.ncon):
            pair = tuple(sorted((bname(int(m.geom_bodyid[d.contact[i].geom1])),
                                 bname(int(m.geom_bodyid[d.contact[i].geom2])))))
            hits[pair] = hits.get(pair, 0) + 1
        p = d.xpos[bid] + d.xmat[bid].reshape(3, 3) @ np.array([-0.0405, -0.0143, 0.1492])
        jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, jp, jr, p, bid)
        J = np.vstack([jp, jr])[:, dofs]
        s = np.linalg.svd(J, compute_uv=False)
        conds.append(s[0] / s[-1])

    print("═══ %s ═══" % path)
    print("  样本 %d，仿真时长 %.1f s" % (len(rows), (t[-1] - t[0]) if t else 0))
    print("  cond：最大 %.1f，中位 %.1f，超 200 的样本 %d/%d"
          % (max(conds), float(np.median(conds)), sum(1 for c in conds if c > 200), len(conds)))
    if lags:
        print("  |cmd−q|：峰值 %.4f rad（跳过 NaN 行 %d 个）"
              % (max(lags), len(rows) - len(lags)))
    if hits:
        print("  真实接触：")
        for pair, n in sorted(hits.items(), key=lambda kv: -kv[1])[:8]:
            print("     %-30s ↔ %-28s %5d 样本" % (pair[0], pair[1], n))
    else:
        print("  真实接触：**无**（全程零接触）")
    print()


if __name__ == "__main__":
    for p in sys.argv[1:]:
        analyze(p)
