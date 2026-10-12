#!/usr/bin/env python3
"""位置模式执行器参数标定：逐关节求 kp，使"速度增益 = 1"。

**为什么需要**（2026-09-24 定位）：
MoveIt Servo 的位置输出是 `实测位置 + 单周期增量`（源码 servo_calcs.cpp:680/699/749：
`internal_joint_state_ = original_joint_state_` 之后 `position[i] += delta_theta[i]`，
而 `original_joint_state_` 由 `current_state_` 拷贝，即实测态）。于是位置执行器
**永远只被压一个周期的量**，弹簧力矩与自身速度阻尼平衡，得到

    达成速度 / 命令速度  ≈  T / τ_servo,    τ_servo = kv_damp / kp

T = publish_period。当前 kp=25000（臂）/2500（腕）、dampratio=1.0 时该比值只有
0.14~0.38——**这就是历史上"位置模式 10 cm/s 只有 31%"的来源，是结构性错配，不是调参问题**。
机械臂会按固定比例永远落后于命令（命令由实测态推出，所以差距不累积也不收敛）。

因为 MuJoCo 的 `dampratio` 自动取 kv_damp = 2·dampratio·√(kp·M_eff)，所以
τ_servo = 2·dampratio·√(M_eff/kp)，即**要求 kp ∝ M_eff**；而各关节 M_eff 相差上千倍
（上臂 ≈7.07 kg·m²，wrist3 ≈0.005），所以**必须逐关节定 kp，不能用统一比例**。

标定目标：每个关节的速度增益 ≈ 1.00（既不落后也不冲过头）。
dampratio 固定 1.0（临界阻尼）：扫描显示 0.2 会过驱动（增益 >1 且出现振荡），0.5 次之。

用法（需要 mujoco+numpy 的 Python，本机用 /home/lcw/tomato_robot/.venv/bin/python）：
    python position_actuator_kp_calib.py             # 迭代标定并打印结果
    python position_actuator_kp_calib.py --iters 5
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import mujoco
import numpy as np

ASSETS = Path("/home/lcw/VR_teleoperation/assets/aubo_i5")
ACT_XML = ASSETS / "actuators_position.xml"
SCENE_XML = ASSETS / "scene_ros2.xml"
TMP_ACT = ASSETS / ".kpcal_actuators_tmp.xml"
TMP_SCENE = ASSETS / ".kpcal_scene_tmp.xml"

ARM = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
       "wrist1_joint", "wrist2_joint", "wrist3_joint"]
READY = [0.0, -0.4, 0.8, 0.0, 0.4, 0.0]
PUBLISH_PERIOD = 0.005      # 与 Servo 的 publish_period 一致
DAMPRATIO = 1.0
V_CMD = 0.2                 # 阶跃速度 [rad/s]


def build_scene(kp: dict[str, float]) -> Path:
    """把逐关节 kp 写进临时执行器文件 + 临时场景，走真实编译路径
    （这样 dampratio 的自动阻尼计算与正式运行时完全一致）。"""
    xml = ACT_XML.read_text()
    for joint, val in kp.items():
        pat = r'(<position name="%s"[^>]*?kp=")[0-9.eE+-]+(")' % re.escape(joint)
        xml, n = re.subn(pat, lambda m: m.group(1) + repr(float(val)) + m.group(2), xml)
        if n != 1:
            raise RuntimeError("替换 %s 的 kp 失败（命中 %d 次）" % (joint, n))
    TMP_ACT.write_text(xml)
    TMP_SCENE.write_text(SCENE_XML.read_text().replace(
        'file="actuators_position.xml"', 'file=".kpcal_actuators_tmp.xml"'))
    return TMP_SCENE


def measure_all(model_path: Path, joints: list[str], v_cmd: float = V_CMD,
                dt_override: float | None = None) -> dict[str, dict]:
    """对每个关节单独做一次 Servo 式速度阶跃（ctrl = 实测 q + v·T，每 T 更新）。"""
    m = mujoco.MjModel.from_xml_path(str(model_path))
    if dt_override:
        m.opt.timestep = dt_override
    d = mujoco.MjData(m)
    aid = {j: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, j) for j in ARM}
    dadr = {j: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM}
    qadr = {j: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ARM}
    dt = m.opt.timestep
    n_cycle = int(PUBLISH_PERIOD / dt)

    def reset():
        d.qpos[:] = 0.0
        for j, v in zip(ARM, READY):
            d.qpos[qadr[j]] = v
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)
        for _ in range(int(0.5 / dt)):
            for j in ARM:
                d.ctrl[aid[j]] = d.qpos[qadr[j]]
            mujoco.mj_step(m, d)

    out = {}
    for jn in joints:
        reset()
        # ⚠️ 仪器：**不能**取周期末的瞬时 qvel——硬伺服在周期内会振荡，末端采样会混叠
        #    （实测同一组 kp 在 dt=2ms/1ms/0.5ms 下给出 0.99/0.87/0.80，就是混叠造成的）。
        #    遥操真正关心的是"每个控制周期实际移动了多少"，所以用周期内的位置增量 / T。
        vs, q_prev = [], None
        for _ in range(int(1.0 / PUBLISH_PERIOD)):
            for j in ARM:
                d.ctrl[aid[j]] = d.qpos[qadr[j]] + (v_cmd * PUBLISH_PERIOD if j == jn else 0.0)
            for _ in range(n_cycle):
                mujoco.mj_step(m, d)
            q_now = d.qpos[qadr[jn]]
            if q_prev is not None:
                vs.append((q_now - q_prev) / PUBLISH_PERIOD)
            q_prev = q_now
        vs = np.array(vs)
        half = len(vs) // 2
        tail = vs[int(0.7 * len(vs)):]
        out[jn] = dict(
            ratio=float(np.mean(vs[half:])) / v_cmd,
            overshoot=100.0 * (float(vs.max()) - v_cmd) / v_cmd,
            ripple=float(tail.max() - tail.min()) / v_cmd,
        )
    return out


def effective_inertia() -> dict[str, float]:
    """从当前模型反解各关节的等效惯量：kv = 2·dampratio·√(kp·M_eff) → M_eff = (kv/2)²/kp。"""
    m = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    out = {}
    for j in ARM:
        k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, j)
        kp = m.actuator_gainprm[k][0]
        kvd = -m.actuator_biasprm[k][2]
        out[j] = (kvd / (2 * DAMPRATIO)) ** 2 / kp
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=4)
    ap.add_argument("--v", type=float, default=V_CMD)
    ap.add_argument("--dt", type=float, default=None,
                    help="覆盖物理步长（默认用模型自带 2 ms）")
    args = ap.parse_args()

    m_eff = effective_inertia()
    print("各关节等效惯量 M_eff（由当前 kv 与 kp 反解）：")
    for j in ARM:
        print("  %-16s %.5f kg·m²" % (j, m_eff[j]))
    # 初值：让位置环带宽 √(kp/M) 达到 2/T 的 1.7 倍（经验修正，见迭代结果）
    kp = {j: m_eff[j] * (2.0 / PUBLISH_PERIOD) ** 2 * 1.7 for j in ARM}
    print("\n迭代标定（目标：速度增益 ≈ 1.00，dampratio = %.1f，T = %.3f s）" % (DAMPRATIO, PUBLISH_PERIOD))
    for it in range(args.iters):
        path = build_scene(kp)
        res = measure_all(path, ARM, args.v, dt_override=args.dt)
        worst = max(abs(r["ratio"] - 1.0) for r in res.values())
        print("  第 %d 轮：最大偏差 %.3f | " % (it + 1, worst)
              + "  ".join("%s=%.2f" % (j.replace("_joint", ""), res[j]["ratio"]) for j in ARM))
        if worst < 0.03:
            break
        for j in ARM:
            kp[j] *= (1.0 / res[j]["ratio"]) ** 2      # ratio ∝ √kp
    path = build_scene(kp)
    res = measure_all(path, ARM, args.v, dt_override=args.dt)
    print("\n=== 标定结果（dampratio=%.1f, T=%.3f s, v=%.2f rad/s）===" % (DAMPRATIO, PUBLISH_PERIOD, args.v))
    print("%-16s %12s %10s %10s %10s" % ("关节", "kp", "速度增益", "超调%", "末段峰峰"))
    for j in ARM:
        print("%-16s %12.0f %10.3f %9.1f %10.3f"
              % (j, kp[j], res[j]["ratio"], res[j]["overshoot"], res[j]["ripple"]))
    print("\n建议写入 actuators_position.xml 的 kp：")
    for j in ARM:
        print('  %-16s kp="%.0f"' % (j, kp[j]))
    print("\ndampratio 保持 1.0")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        TMP_ACT.unlink(missing_ok=True)
        TMP_SCENE.unlink(missing_ok=True)
