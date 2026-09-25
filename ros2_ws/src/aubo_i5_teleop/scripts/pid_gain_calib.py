#!/usr/bin/env python3
"""标定 mujoco_ros2_control 内置位置环 PID 的增益（离线）。

**对象**：`<motor>` 执行器 + 插件内的 `control_toolbox::PidROS`。
本脚本**忠实复刻 control_toolbox::Pid 的语义**，在 MuJoCo 里把它跑起来——
因为 PID 在插件里、不在模型里，离线只能这样测；最终仍要在实机上用台架复核。

**主指标：速度增益 → 1.0**。为什么是它：
MoveIt Servo 的位置流是 `q_cmd = 实测 + Δθ(单周期)`（锚在实测上）。纯 PD 下误差建不起来
（实测增益 0.140–0.753），必须靠**积分项**把稳态误差积掉。所以"给定速度命令，看实际
跟上多少"就是这一环的直接度量。

⚠️ 仪器（踩过的坑）：速度必须用**每周期位置增量 / T** 算，**不能**取周期末瞬时 qvel——
硬伺服在周期内会振荡，末端采样会混叠（同一组参数在 dt=2ms/1ms/0.5ms 下曾给出 0.99/0.87/0.80）。

用法（需要 mujoco+numpy 的 Python）：
    python pid_gain_calib.py                 # 用 config/mujoco_pid.yaml 的当前值测
    python pid_gain_calib.py --sweep p       # 扫 p
    python pid_gain_calib.py --sweep i       # 扫 i（重点）
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import mujoco
import yaml

ASSETS = Path("/home/lcw/VR_teleoperation/assets/aubo_i5")
SCENE = ASSETS / "scene_ros2.xml"
PID_YAML = Path("/home/lcw/VR_teleoperation/ros2_ws/src/aubo_i5_teleop/config/mujoco_pid.yaml")

ARM = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
       "wrist1_joint", "wrist2_joint", "wrist3_joint"]
T = 0.005          # publish_period：Servo 的位置流周期，也是 PID 的计算周期


class Pid:
    """control_toolbox::Pid 的语义：u = p·e + i·∫e·dt + d·(de/dt)，带输出/积分限幅与抗积分饱和。"""

    def __init__(self, p, i, d, u_min, u_max, i_min, i_max):
        self.p, self.i, self.d = p, i, d
        self.u_min, self.u_max = u_min, u_max
        self.i_min, self.i_max = i_min, i_max
        self.e_prev = None
        self.i_term = 0.0

    def compute(self, error, dt):
        p_term = self.p * error
        d_term = 0.0 if self.e_prev is None else self.d * (error - self.e_prev) / dt
        # 积分限幅
        self.i_term = max(self.i_min, min(self.i_max, self.i_term + self.i * error * dt))
        u = p_term + self.i_term + d_term
        u = max(self.u_min, min(self.u_max, u))
        # 抗积分饱和：输出顶到限幅且误差还在往同方向推 → 不再积分
        if (u >= self.u_max and error > 0) or (u <= self.u_min and error < 0):
            self.i_term = max(self.i_min, min(self.i_max, self.i_term - self.i * error * dt))
        self.e_prev = error
        return u


def load_gains(path=PID_YAML):
    doc = yaml.safe_load(path.read_text())
    return doc["/**"]["ros__parameters"]["pid_gains"]["position"]


def run(m, d, dofs, aid, qadr, joint, gains, v_cmd=0.2, dur=1.0):
    """用 Servo 式位置流驱动一个关节，PID 按 T 周期算，返回速度增益等指标。"""
    k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, 'ready')
    d.qpos[:] = m.key_qpos[k]
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)
    dt = m.opt.timestep
    n_cycle = int(T / dt)

    g = gains[joint]
    pid = Pid(g["p"], g["i"], g["d"], g["u_clamp_min"], g["u_clamp_max"],
              g["i_clamp_min"], g["i_clamp_max"])

    # ⚠️ 每个关节都要有自己的 PID：真实链路里 Servo 给**所有**关节发位置流，
    #    非目标关节的增量是 0 但仍有 PID 去保持。只给被测关节 PID、其余 ctrl=0 的话，
    #    其余关节是零力矩自由漂浮（gravcomp 只补重力），会把被测关节的运动污染掉
    #    ——实测那样做会得到"臂增益 15~32×、腕完全不动的"荒谬结果。
    pids = {}
    for j in ARM:
        gj = gains[j]
        pids[j] = Pid(gj["p"], gj["i"], gj["d"], gj["u_clamp_min"], gj["u_clamp_max"],
                      gj["i_clamp_min"], gj["i_clamp_max"])

    # 静置 0.5 s（命令=当前位姿，误差 0）
    for _ in range(int(0.5 / dt)):
        for j in ARM:
            d.ctrl[aid[j]] = pids[j].compute(0.0, T)
        mujoco.mj_step(m, d)
        if _ % n_cycle == 0:
            for j in ARM:
                pids[j].compute(0.0, T)      # 让 PID 以控制周期推进

    q_prev = d.qpos[qadr[joint]].copy()
    vs = []
    for _ in range(int(dur / T)):
        for j in ARM:
            # Servo 的位置流：q_cmd = 实测 + Δθ(单周期)；非目标关节增量 0
            delta = v_cmd * T if j == joint else 0.0
            err = delta                      # 误差恒等于"本周期要走的量"
            d.ctrl[aid[j]] = pids[j].compute(err, T)
        for _ in range(n_cycle):
            mujoco.mj_step(m, d)
        q_now = d.qpos[qadr[joint]]
        vs.append((q_now - q_prev) / T)        # ← 仪器：每周期位置增量
        q_prev = q_now
    vs = np.array(vs)
    half = len(vs) // 2
    tail = vs[int(0.7 * len(vs)):]
    return dict(gain=float(np.mean(vs[half:])) / v_cmd,
                overshoot=100.0 * (float(vs.max()) - v_cmd) / v_cmd,
                ripple=float(tail.max() - tail.min()) / v_cmd)


def build(gains):
    """把增益写进临时场景（用 mj_spec 改执行器不方便，直接用当前场景 + 传入 gains）。"""
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    dofs = {j: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM}
    qadr = {j: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM}
    aid = {j: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, j) for j in ARM}
    return m, d, dofs, aid, qadr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", choices=["none", "p", "i", "d"], default="none")
    ap.add_argument("--v", type=float, default=0.2, help="阶跃速度 [rad/s]")
    args = ap.parse_args()

    gains = load_gains()
    m, d, dofs, aid, qadr = build(gains)

    if args.sweep == "none":
        print("当前 config/mujoco_pid.yaml 的增益（主指标：速度增益 → 1.0）\n")
        print("  %-16s %6s %8s %6s %9s %10s %9s" % ("关节", "p", "i", "d", "速度增益", "超调%", "末段振荡"))
        for j in ARM:
            r = run(m, d, dofs, aid, qadr, j, gains, args.v)
            g = gains[j]
            flag = "" if r["gain"] >= 0.95 else "   ← 偏低"
            print("  %-16s %6.0f %8.0f %6.0f %9.3f %10.1f %9.3f%s"
                  % (j, g["p"], g["i"], g["d"], r["gain"], r["overshoot"], r["ripple"], flag))
        return 0

    mults = [0.0, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0]
    print("扫 %s（其余两项保持 yaml 当前值；主指标：速度增益 → 1.0）\n" % args.sweep)
    for j in ARM:
        base = gains[j][args.sweep]
        if base == 0 and args.sweep != "i":
            print("  %s: %s 为 0，跳过" % (j, args.sweep))
            continue
        row = []
        for mu in mults:
            g2 = {k: dict(v) for k, v in gains.items()}
            g2[j][args.sweep] = base * mu
            r = run(m, d, dofs, aid, qadr, j, g2, args.v)
            row.append("%s=%.3f" % (("%g" % mu), r["gain"]))
        print("  %-16s %s" % (j, "  ".join(row)))
    print("\n（格式：倍数=速度增益；i=0.0 即回到纯 PD，可用来看积分项的必要性）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
