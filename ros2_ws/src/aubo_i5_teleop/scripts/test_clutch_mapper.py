"""ClutchPoseMapper 测试套件（离线，无 ROS，venv 解释器跑）。

两部分：
  A. 上游 11 项 sanity 测试移植（_refs/vr-teleop-kit 的 main()，断言不变；
     上游用 R=quest→armbase，我们用 R_trans —— 数学同构）。
  B. 本项目新增 6 项（2026-09-25 设计评审定的验收单）：
     TA 工具轴语义 / TB 缩放切换零跳变 / TC 重离合零跳变 /
     TD 1000-tick 压力 / TE 四元数约定往返 / TF 半球翻转不反向。

用法：/home/lcw/tomato_robot/.venv/bin/python scripts/test_clutch_mapper.py
全部通过退出码 0，任何 FAIL 退出码 1。
"""
import math
import sys

import numpy as np

from clutch_pose_mapper import (ClutchPoseMapper, quat_mul, quat_conj,
                                quat_to_rotvec, rotvec_to_quat,
                                quat_xyzw_to_wxyz, quat_wxyz_to_xyzw)

FAILURES = []


def check(name, ok, detail=""):
    print("  %-46s [%s] %s" % (name, "ok" if ok else "FAIL", detail))
    if not ok:
        FAILURES.append(name)


def close(a, b, tol=1e-6):
    return bool(np.allclose(np.asarray(a, float), np.asarray(b, float), atol=tol))


def ang_deg(qa, qb):
    return float(np.degrees(np.linalg.norm(quat_to_rotvec(quat_mul(qa, quat_conj(qb))))))


# 共用锚点（与上游测试同值，便于对照）
EE_P = np.array([0.50, 0.00, 0.42])
EE_Q = np.array([1.0, 0.0, 0.0, 0.0])          # wxyz 单位
CTRL_P = np.array([0.0, 1.4, -0.3])
CTRL_Q = np.array([1.0, 0.0, 0.0, 0.0])


def section(title):
    print("\n%s" % title)
    print("-" * 72)


def suite_upstream():
    section("A. 上游 11 项（移植）")

    m = ClutchPoseMapper()
    m.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    p, q = m.target(CTRL_P, CTRL_Q)
    check("T1 无动作 → 目标=接合锚点", close(p, EE_P) and close(q, EE_Q))

    p, q = m.target(CTRL_P + np.array([0.05, 0, 0]), CTRL_Q)
    check("T2 手柄 +5cm X → 目标 +5cm X", close(p, EE_P + np.array([0.05, 0, 0]))
          and close(q, EE_Q))

    m_s = ClutchPoseMapper(scale=0.5)
    m_s.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    p, _ = m_s.target(CTRL_P + np.array([0.05, 0, 0]), CTRL_Q)
    check("T3 scale=0.5 → 5cm 变 2.5cm", close(p, EE_P + np.array([0.025, 0, 0])))

    R_z90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    m_R = ClutchPoseMapper(R_trans=R_z90)
    m_R.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    p, _ = m_R.target(CTRL_P + np.array([0.05, 0, 0]), CTRL_Q)
    check("T4 R_trans=Rz90：手柄 +X → 目标 +Y", close(p, EE_P + np.array([0.0, 0.05, 0.0])))

    m = ClutchPoseMapper()
    m.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    q_ctrl = rotvec_to_quat(np.array([0.0, np.pi / 6, 0.0]))
    _, q = m.target(CTRL_P, q_ctrl)
    check("T5 手柄绕 Y +30° → 目标同旋转", close(q, rotvec_to_quat(np.array([0.0, np.pi / 6, 0.0]))))

    m.disengage()
    out = m.target(CTRL_P, CTRL_Q)
    check("T6 脱离 → target=None", out is None)

    new_ee = np.array([0.55, 0.10, 0.42])
    m.engage(CTRL_P, CTRL_Q, new_ee, EE_Q)
    p, _ = m.target(CTRL_P + np.array([0.05, 0, 0]), CTRL_Q)
    check("T7 重接合锚点更新", close(p, new_ee + np.array([0.05, 0, 0])))

    m8 = ClutchPoseMapper(rot_reach_limit=0.5, pos_reach_limit=0.25)
    m8.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    worst = 0.0
    for i in range(1, 121):
        q_ctrl = rotvec_to_quat(np.array([0.0, np.radians(i), 0.0]))
        _, tq = m8.target(CTRL_P, q_ctrl, EE_P, EE_Q)
        worst = max(worst, ang_deg(tq, EE_Q))
    check("T8 旋转 reach limit（120° 猛推）", worst <= np.degrees(0.5) + 0.1,
          "max %.1f° / 限 %.1f°" % (worst, np.degrees(0.5)))

    back = 120.0 - np.degrees(0.5)
    _, tq = m8.target(CTRL_P, rotvec_to_quat(np.array([0.0, np.radians(back), 0.0])),
                      EE_P, EE_Q)
    resid = ang_deg(tq, EE_Q)
    check("T9 滑移后反向立即咬合", resid < 1.0, "残差 %.2f°" % resid)

    m10 = ClutchPoseMapper(rot_reach_limit=1.0, pos_reach_limit=0.25)
    m10.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    p, _ = m10.target(CTRL_P + np.array([1.0, 0, 0]), CTRL_Q, EE_P, EE_Q)
    ok1 = close(p, EE_P + np.array([0.25, 0, 0]))
    p, _ = m10.target(CTRL_P + np.array([0.95, 0, 0]), CTRL_Q, EE_P, EE_Q)
    ok2 = close(p, EE_P + np.array([0.20, 0, 0]))
    check("T10 平移 reach limit（鼠标到屏幕边语义）", ok1 and ok2)

    m_inc = ClutchPoseMapper(rot_reach_limit=3.0, pos_reach_limit=10.0)
    m_abs = ClutchPoseMapper()
    for mm in (m_inc, m_abs):
        mm.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    ee_q = EE_Q
    worst = 0.0
    for i in range(1, 41):
        q_ctrl = rotvec_to_quat(np.radians(i) * np.array([0.5, 0.7, 0.2]))
        _, tq_i = m_inc.target(CTRL_P, q_ctrl, EE_P, ee_q)
        _, tq_a = m_abs.target(CTRL_P, q_ctrl)
        worst = max(worst, ang_deg(tq_i, tq_a))
        ee_q = tq_i
    check("T11 增量路径 == 绝对路径（reach limit 内）", worst < 0.01,
          "max diff %.4f°" % worst)


def suite_new():
    section("B. 本项目新增 6 项")

    # TA：工具轴语义——手柄自系 yaw +30°，末端应绕【自己（接合时）的 z】转 30°，
    #     且原位旋转位置不动（不画大圆）。
    ee_q0 = rotvec_to_quat(np.array([np.pi / 2, 0.0, 0.0]))   # 工具绕 x 转 90°
    m = ClutchPoseMapper()
    m.engage(CTRL_P, CTRL_Q, EE_P, ee_q0)
    _, q = m.target(CTRL_P, rotvec_to_quat(np.array([0.0, 0.0, np.radians(30.0)])))
    # 手柄 ctrl0=I，自系 yaw 增量轴 = 世界 ẑ；对齐后期望 target = R_ee0 ⊗ Rz(30°)
    expected = quat_mul(ee_q0, rotvec_to_quat(np.array([0.0, 0.0, np.radians(30.0)])))
    check("TA1 自系 yaw → 绕工具自身轴 30°", ang_deg(q, expected) < 0.01,
          "差 %.4f°" % ang_deg(q, expected))
    # 位置不动的断言放在只旋转不平时：直接构造（平移不动 + 旋转 30°）
    m2 = ClutchPoseMapper()
    m2.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    p, _ = m2.target(CTRL_P, rotvec_to_quat(np.array([0.0, 0.0, np.radians(90.0)])))
    check("TA2 原位旋转 → 位置零移动（不画大圆）", close(p, EE_P))

    # TB：缩放切换零跳变——切档那一 tick 手柄没动 → 目标必须精确不变
    m = ClutchPoseMapper(rot_reach_limit=1.0, pos_reach_limit=1.0)
    m.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    p1, q1 = m.target(CTRL_P + np.array([0.10, 0.02, 0.0]),
                      rotvec_to_quat(np.array([0.1, 0.0, 0.0])), EE_P, EE_Q)
    m.scale, m.scale_rotation = 0.2, 0.2            # ← 切微调档
    p2, q2 = m.target(CTRL_P + np.array([0.10, 0.02, 0.0]),
                      rotvec_to_quat(np.array([0.1, 0.0, 0.0])), EE_P, EE_Q)
    check("TB1 切档瞬间（手柄未动）目标零跳变", close(p1, p2) and close(q1, q2))
    p3, _ = m.target(CTRL_P + np.array([0.15, 0.02, 0.0]),
                     rotvec_to_quat(np.array([0.1, 0.0, 0.0])), EE_P, EE_Q)
    check("TB2 切档后增量按新档缩放（0.2×5cm=1cm）",
          close(p3, p2 + np.array([0.01, 0.0, 0.0])),
          "Δ=%s" % np.round(np.asarray(p3) - np.asarray(p2), 4))

    # TC：重离合零跳变——脱离后手柄随便动，重新接合的第一拍目标 == 当前末端
    m = ClutchPoseMapper(rot_reach_limit=1.0, pos_reach_limit=1.0)
    m.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    m.target(CTRL_P + np.array([0.1, 0, 0]), CTRL_Q, EE_P, EE_Q)
    m.disengage()
    ee_now = np.array([0.52, 0.0, 0.42])             # 臂实际停在这里
    ctrl_far = CTRL_P + np.array([0.5, -0.4, 0.2])   # 手柄被移走
    m.engage(ctrl_far, CTRL_Q, ee_now, EE_Q)
    p, _ = m.target(ctrl_far, CTRL_Q, ee_now, EE_Q)
    check("TC 重接合第一拍目标 == 当前末端（零跳变）", close(p, ee_now))

    # TD：1000-tick 随机压力（模拟 100 Hz 离散阶跃）：每 tick 目标位移/转角
    #     不超 reach limit（+5% 数值余量）、全程有限、无非正常状态
    rng = np.random.default_rng(42)
    m = ClutchPoseMapper(rot_reach_limit=0.6, pos_reach_limit=0.25,
                         scale=1.0, scale_rotation=1.0)
    m.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    ee_p, ee_q = EE_P.copy(), EE_Q.copy()
    ctrl_p, ctrl_q = CTRL_P.copy(), CTRL_Q.copy()
    worst_dp, worst_dq, bad = 0.0, 0.0, 0
    prev = None
    for k in range(1000):
        ctrl_p = ctrl_p + rng.normal(0, 0.02, 3)        # 每 tick 2 cm 级随机阶跃
        ctrl_q = rotvec_to_quat(rng.normal(0, 0.05, 3))  # 每 tick ~3° 级随机转动
        out = m.target(ctrl_p, ctrl_q, ee_p, ee_q)
        if out is None:
            bad += 1
            continue
        p, q = out
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q))):
            bad += 1
            continue
        if prev is not None:
            worst_dp = max(worst_dp, float(np.linalg.norm(p - prev[0])))
            worst_dq = max(worst_dq, ang_deg(q, prev[1]))
        prev = (p, q)
        ee_p = ee_p + 0.3 * (p - ee_p)                  # 末端以 30%/tick 跟随
        ee_q = quat_mul(rotvec_to_quat(0.3 * quat_to_rotvec(quat_mul(q, quat_conj(ee_q)))), ee_q)
        ee_q /= np.linalg.norm(ee_q)
    check("TD 1000-tick 压力：每 tick 目标跳变 ≤ reach limit",
          bad == 0 and worst_dp <= 0.25 * 1.05 + 1e-9 and worst_dq <= np.degrees(0.6) * 1.05 + 1e-9,
          "max Δp=%.3f m, max Δθ=%.1f°, 异常 %d" % (worst_dp, worst_dq, bad))

    # TE：四元数约定往返（ROS 边界转换）
    rng = np.random.default_rng(7)
    ok = True
    for _ in range(1000):
        v = rng.normal(size=3)
        q_xyzw = rotvec_to_quat(v)
        q_xyzw = np.array([q_xyzw[1], q_xyzw[2], q_xyzw[3], q_xyzw[0]])  # 人为 (x,y,z,w)
        back = quat_wxyz_to_xyzw(quat_xyzw_to_wxyz(q_xyzw))
        if not close(back, q_xyzw, 1e-12):
            ok = False
            break
    check("TE (x,y,z,w)↔[w,x,y,z] 往返 ×1000", ok)

    # TF：半球翻转（q 与 −q 交错）不引起反向
    m = ClutchPoseMapper(rot_reach_limit=1.0, pos_reach_limit=1.0)
    m.engage(CTRL_P, CTRL_Q, EE_P, EE_Q)
    q1 = rotvec_to_quat(np.array([0.0, 0.0, np.radians(10.0)]))
    q2 = -rotvec_to_quat(np.array([0.0, 0.0, np.radians(20.0)]))   # 人为取反
    _, qa = m.target(CTRL_P, q1, EE_P, EE_Q)
    _, qb = m.target(CTRL_P, q2, EE_P, EE_Q)
    check("TF q/−q 半球翻转 → 旋转方向不反",
          10.0 < ang_deg(qb, EE_Q) <= 20.0 + 0.5, "累计 %.1f°" % ang_deg(qb, EE_Q))


def main():
    print("=== ClutchPoseMapper 测试套件（移植版）===")
    suite_upstream()
    suite_new()
    print()
    if FAILURES:
        print("❌ 失败 %d 项：%s" % (len(FAILURES), FAILURES))
        return 1
    print("✅ 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
