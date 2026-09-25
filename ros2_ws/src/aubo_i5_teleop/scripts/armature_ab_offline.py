#!/usr/bin/env python3
"""armature A/B 对照实验（离线纯 MuJoCo，不经 ROS）。

**背景**：厂家的 URDF 在每个 <joint> 里带一个 `<property>` 标签，同时给了
`equa_inertia`（1.5/1.5/1.2/0.05/0.05/0.01，逐关节）与 `inertia`
（2.027236783 x3、0.219280696 x3，按电机分组）以及 `motor_constant`、`ratio`。
我们把它当 MuJoCo 的 `armature` 用的是 **equa_inertia**；但厂家的 `xacro.sh`
用 `sed '/<property/d'` 主动删掉这些字段，全栈无人消费，**语义没有文档**
（五个官方仓库搜索 + 驱动手册均无引用）。所以两个字段哪个才对应 armature 无法从
文本判定，只能测它到底影响多大。

**做法**：同一个模型、同一就绪位姿、同一个速度阶跃，只改 armature：
  A = equa_inertia        （我们当前用的，scene_aubo_i5.xml）
  B = 厂家 inertia 字段   （实验变体，scene_aubo_i5_armB.xml，机械替换生成）

**结论（2026-09-24 实测）**：内环时间常数 τ=(M_crb+armature)/(kv+damping) 的比值
  shoulder 1.12x | upperArm 1.04x | foreArm 1.29x | wrist1 2.86x | wrist2 2.78x | wrist3 **16.85x**
被控对象在腕部差异巨大，但**闭环验收对之不敏感**（见 README/记录：A 与 B 的
6 个平移 + 6 个旋转验收都是 6/6，最大误差同为 ~1.5 mm / 旋转完成度差 0.06%），
因为外环（τ≈74 ms）比两个内环（1~20 ms）都慢得多。

用法（需要 mujoco+numpy 的 Python，本机用 /home/lcw/tomato_robot/.venv/bin/python）：
    python armature_ab_offline.py            # 用模型自带步长（2 ms）
    python armature_ab_offline.py --fine     # 步长改 0.2 ms，用于分辨 τ≈1-3 ms 的腕关节
"""
import argparse
import numpy as np
import mujoco

ASSETS = "/home/lcw/VR_teleoperation/assets/aubo_i5"
A_MODEL = f"{ASSETS}/scene_ros2_velocity.xml"
B_MODEL = f"{ASSETS}/scene_ros2_velocity_armB.xml"
READY = [0.0, -0.4, 0.8, 0.0, 0.4, 0.0]
JOINTS = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
          "wrist1_joint", "wrist2_joint", "wrist3_joint"]
AMP = 0.2      # rad/s
SETTLE = 0.30  # s
DUR = 1.0      # s


def run(model_path, joint, amp=AMP, dt_override=None):
    m = mujoco.MjModel.from_xml_path(model_path)
    if dt_override:
        m.opt.timestep = dt_override
    d, dt = mujoco.MjData(m), m.opt.timestep
    aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, joint)
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)
    dadr = m.jnt_dofadr[jid]
    for jn, q in zip(JOINTS, READY):
        k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        d.qpos[m.jnt_qposadr[k]] = q
    mujoco.mj_forward(m, d)

    n_settle, n_dur = int(SETTLE / dt), int(DUR / dt)
    ts, vs = [], []
    for i in range(n_settle + n_dur + int(0.3 / dt)):
        if i == n_settle:
            for k in range(m.nu):
                d.ctrl[k] = 0.0
            d.ctrl[aid] = amp
        elif i < n_settle:
            for k in range(m.nu):
                d.ctrl[k] = 0.0
        mujoco.mj_step(m, d)
        if i >= n_settle:
            ts.append((i - n_settle) * dt)
            vs.append(d.qvel[dadr])
    ts, vs = np.array(ts), np.array(vs)

    steady = vs[ts > 0.7 * DUR].mean()
    peak = vs[ts < DUR].max() if amp > 0 else vs[ts < DUR].min()
    over = 100.0 * (peak - amp) / abs(amp)
    t10 = ts[np.argmax(vs >= 0.1 * steady)]
    t90 = ts[np.argmax(vs >= 0.9 * steady)]
    sel = (ts >= t10) & (ts <= t90)
    tau = np.nan
    if sel.sum() > 5:
        y = np.clip(np.abs(steady - vs[sel]), 1e-15, None)
        slope = np.polyfit(ts[sel], np.log(y), 1)[0]
        tau = -1.0 / slope if slope < 0 else np.nan
    return dict(steady=steady, ratio=steady / amp, over=over, tau=tau, dt=dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fine", action="store_true",
                    help="把物理步长改成 0.2 ms（分辨腕关节 τ≈1-3 ms；不影响物理本身）")
    args = ap.parse_args()
    dt_override = 0.0002 if args.fine else None

    print(f"离线对照：{'细步长 0.2 ms' if args.fine else '模型自带步长'}，"
          f"阶跃 = 单关节 0→{AMP} rad/s 持续 {DUR} s，就绪位姿 {READY}\n")
    print(f"{'关节':16s} {'模型':>4s} {'稳态比':>8s} {'超调':>8s} {'τ(指数拟合)':>13s}")
    ratios = {}
    for jn in JOINTS:
        row = {}
        for label, path in (("A", A_MODEL), ("B", B_MODEL)):
            r = run(path, jn, dt_override=dt_override)
            row[label] = r
            print(f"{jn:16s} {label:>4s} {r['ratio']:8.4f} {r['over']:7.1f}% {r['tau']*1000:11.3f}ms")
        if row["A"]["tau"] and row["B"]["tau"]:
            ratios[jn] = row["B"]["tau"] / row["A"]["tau"]
            print(f"{'':16s}   τ_B/τ_A = {ratios[jn]:.2f}x    (A={row['A']['tau']*1000:.2f}ms, "
                  f"B={row['B']['tau']*1000:.2f}ms)")
        else:
            print(f"{'':16s}   τ_A 无法拟合（低于步长可分辨尺度），加 --fine 重跑")
        print()
    if ratios:
        print("=== 汇总：armature 从 equa_inertia 换成厂家 inertia 字段的 τ 变化 ===")
        for jn, r in ratios.items():
            print(f"  {jn:16s} {r:6.2f}x")


if __name__ == "__main__":
    main()
