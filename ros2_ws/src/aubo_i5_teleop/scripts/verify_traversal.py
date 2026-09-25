"""只测 home→P02 这一条穿越（上一轮失败的那条）。

不复用 follow_bench 的完整 survey（那要跑 8 个位姿、十几分钟），只做一件事：
用**修好的**分步导航从 home 走到 P02，全程打印
  · 是否卡住（Servo 状态）、是否触发退让
  · |cmd−q| 峰值（验证 lag_max=0.15 是否把领跑掐住）
  · 真实自碰撞（用 MuJoCo 3.12.0 离线从 q 反算接触对）
  · 末端轨迹是否单调靠近目标

用法：python3 scripts/verify_traversal.py [--pose P02] [--out /tmp/verify_trav.csv]
需要 stage2a/2b/3 已经在跑（用 run_bench.sh 起的栈）。
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).parent))
import follow_bench as FB   # 复用 Bench / go_to_pose / make_target_from_pose，避免两套逻辑漂移

ASSETS = Path("/home/lcw/VR_teleoperation/assets/aubo_i5")


def collisions_from_q(rows_q, csv_path):
    """用 MuJoCo（3.12.0，与仿真同版本）从记录的关节角反算真实接触。

    ⚠️ 系统 python3 的 mujoco 是 3.3.5（连本场景都编译不过、且缺 typing_extensions），
       所以这里 import 失败时**不崩**，只提示用哪个解释器重跑
       （scripts/check_traj_contacts.py 与这里是同一段逻辑，独立成脚本）。
    """
    import numpy as np
    try:
        import mujoco
    except ModuleNotFoundError as e:
        print("（跳过离线接触反算：%s）" % e)
        print("  用 /home/lcw/tomato_robot/.venv/bin/python scripts/check_traj_contacts.py %s"
              % csv_path)
        return None          # ⚠️ None = 没做，{} = 做了且无接触。返回 {} 会被当成"零接触"谎报通过

    m = mujoco.MjModel.from_xml_path(str(ASSETS / "scene_ros2.xml"))
    d = mujoco.MjData(m)
    ARM = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
           "wrist1_joint", "wrist2_joint", "wrist3_joint"]
    qadr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM]
    bname = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
    hits = {}
    for q in rows_q:
        for a, v in zip(qadr, q):
            d.qpos[a] = v
        mujoco.mj_forward(m, d)
        for i in range(d.ncon):
            pair = tuple(sorted((bname(int(m.geom_bodyid[d.contact[i].geom1])),
                                 bname(int(m.geom_bodyid[d.contact[i].geom2])))))
            hits[pair] = hits.get(pair, 0) + 1
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="P02")
    ap.add_argument("--out", default="/tmp/verify_trav.csv")
    args = ap.parse_args()

    import yaml
    poses = yaml.safe_load(open(Path(__file__).parent.parent / "config" / "bench_poses.yaml"))["poses"]
    ps = next((p for p in poses if p["name"].startswith(args.pose)), None)
    if ps is None:
        print("位姿表里没有 %s" % args.pose)
        return 1

    node = FB.Bench(args.out)
    print("目标 %s  q=%s" % (ps["name"], [round(x, 3) for x in ps["q"]]))
    print("起点末端 %s" % (node.ee()[0] if node.ee() else "无 TF",))

    tgt = FB.make_target_from_pose(ps)
    t0 = time.time()
    got = FB.go_to_pose(node, tgt, log=True)
    wall = time.time() - t0
    print("\n--- 结果 ---")
    if got is None:
        print("❌ 拿不到末端位姿")
        return 1
    print("到达位置 (%.4f, %.4f, %.4f)   距目标 %.4f m" % (got[0][0], got[0][1], got[0][2], got[1]))
    print("用时 %.1f s（含退让重试）" % wall)
    print("状态直方图（本次移动内）：%s"
          % {FB.STATUS_MEANING.get(k, k): v for k, v in node.status_hist.items()})

    rows = list(csv.DictReader(open(args.out)))
    lags = [max(abs(float(r["cmd%d" % i]) - float(r["q%d" % i])) for i in range(6))
            for r in rows
            if all(math.isfinite(float(r["cmd%d" % i])) for i in range(6))]
    print("|cmd−q| 峰值 = %.4f rad（lag_max=0.15；跳过 %d 个 NaN 行）"
          % (max(lags), len(rows) - len(lags)))
    qs = [[float(r["q%d" % i]) for i in range(6)] for r in rows]
    hits = collisions_from_q(qs, args.out)
    if hits is None:
        print("真实接触：**未检查**（见上条提示）")
    elif hits:
        print("真实接触（离线反算，MuJoCo 3.12.0）：")
        for pair, n in sorted(hits.items(), key=lambda kv: -kv[1])[:8]:
            print("   %-30s ↔ %-28s %5d 样本" % (pair[0], pair[1], n))
    else:
        print("真实接触：**无**（全程零接触）")
    return 0


if __name__ == "__main__":
    rclpy.init()
    rc = main()
    rclpy.shutdown()
    sys.exit(rc)
