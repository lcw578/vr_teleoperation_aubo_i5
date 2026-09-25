#!/usr/bin/env python3
"""伺服接口层：把 MoveIt Servo 的"实测+增量"位置流还原成**绝对位置轨迹**。

═══ 为什么需要它（2026-09-24 定位，已离线+实机双向验证）═══

MoveIt Servo 的位置输出是 `q_cmd = q_实测 + Δθ(单周期)`（源码 servo_calcs.cpp:680/699/749——
`internal_joint_state_ = original_joint_state_` 之后 `position[i] += delta_theta[i]`，
而 `original_joint_state_` 由 `current_state_` 拷贝，即实测态）。
这个流**锚在实测位置上**，于是位置执行器（弹簧-阻尼）永远只被压一个周期的量，
误差建不起来，稳态是 `q̇ = (kp/kv)·Δ` —— 机械臂按固定比例永远落后于命令。

实测（kp=25000/2500、dampratio=1.0、dt=1ms，用"每周期位置增量"当仪器）：
    速度增益只有 0.140 ~ 0.753（上臂最差 0.140、肩 0.258、wrist3 0.753）。
把**同一串增量**搬到绝对命令上（`q_cmd ← q_cmd + Δ`），**被控对象完全不动**：
    速度增益变成 1.000，位置滞后 v·τ_servo（1.8~6.1 mrad，有界）。
→ **速度不足不是刚度问题，是命令锚点的位置问题。**（这也是历史上"位置模式 10 cm/s
   只有 31%"的真正来源。）

═══ 为什么不用 mujoco_ros2_control 的内置 PID 来做这一步（2026-09-24 试过，已否决）═══

内置 PID 看起来是"开源现成件"（`pid_gains.position.<关节>.{p,i,d,u_clamp,i_clamp}`，
官方有 03_pid_control 示例；gazebo_ros2_control 也是每关节 PID）。但它的误差是

    error = command − measured   ，而  command = measured + Δ   →   **error ≡ Δ**

**误差恒等于增量，与被控对象的实际位置无关**。后果：
  1. 静止时 error = 0 → 输出 0 → **没有任何保持力矩**，且**机械臂漂到哪环路都看不见**；
  2. 运动时 error = Δ ≠ 0（每周期都有）→ **积分项持续累积**冲到限幅。
实机实测：wrist3 归位残差 **13.6→24.5 mrad**（不可重复、且变大），而本层存在时归位
是 **0.00000**；同一套增益下离线台架给出速度增益 25×、超调 3000%。
**结论：这是结构问题，不是调参问题——PID 修不好一个"锚在实测上"的流。**

═══ 这不是我们发明的控制律 ═══

它实现的是**位置接口的既定功能**：把位置流还原成绝对轨迹——即厂家 `servoj` 的
"速度前馈"那一步（`v = q̇_ff + Kp·Δq`，前馈把流积分成绝对轨迹）。MoveIt 官方给
"仿真机器人"的配置（`panda_simulated_config.yaml`）用 JointTrajectory → JTC 达到同一目的。
**我们之所以需要一个显式的解码步，是因为 MoveIt Servo 发出的流锚在实测上**，而不是绝对轨迹。

═══ 两条保护（真实伺服里本来就有；可关）═══
1. **单周期增量限幅** `|Δ| ≤ v_max·T`：v_max 取厂家手册的关节速度上限
   （AUBO-i5 & CB4 用户手册 V4.5.11：150°/s = 2.618 rad/s 关节 1-3；180°/s = 3.142 rad/s 关节 4-6）。
2. **抗积分饱和** `|q_cmd − q_实测| ≤ lag_max`：被控对象跟不上时（力矩饱和、被挡、奇异缩放），
   不限的话绝对命令会无限跑飞。真实伺服的位置环受速度上限约束，同样不会无界积分。
   默认 0.15 rad。历史更正（2026-09-24）：0.3 是在**旧前提**下定的——当时 0.15 会在
   激进阶跃下持续触发、把臂压成蠕动，但那个前提（旧被控对象、慢 plant）已不存在：
   现在健康工况实测滞后 ≤ 0.02 rad，0.15 是 ~8 倍余量的纯安全网。而 0.3 被证明太松：
   台架回放显示领跑积到 0.36 rad 后换目标，臂猛收折进自碰撞。注意钳位触发时等效于
   以 kp·lag_max 的力矩把臂往钳位点压，这个值同时就是"最大憋压深度"。
   ⚠️ stage3 launch 里的同名参数必须一起翻（两处默认必须一致）。

═══ ⚠️ 已知不确定项（不要当成已验证的事实）═══
真机 servoj **是否真做速度前馈、是否限制加速度/加加速度**，厂家公开资料没有写明：
servoj 应用说明（developer.aubo-robotics.cn 的 53-servoj）只给示例
`servoJoint(traj, 0.1, 0.2, 0.06, 0., 0.)` 与"抖动就适当增大 t"，参数定义在未获取的 SDK 文档里。
所以本节点是"按真机应然行为建模"，不是"照抄已知实现"。若将来拿到 SDK 文档确认了控制律，
应回来核对这两条保护是否与真机一致。

用法：
    ros2 run aubo_i5_teleop servo_interface.py            # 正常跑
    python3 scripts/servo_interface.py --selftest          # 离线自检（不需要 ROS）
"""

import sys

# ROS 的 import 放在函数里（`--selftest` 要能在没有 ROS 环境的解释器下跑）

DEFAULT_JOINTS = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
                  "wrist1_joint", "wrist2_joint", "wrist3_joint"]
# 厂家手册的关节速度上限（rad/s）：关节 1-3 = 150°/s，4-6 = 180°/s
DEFAULT_VMAX = [2.618, 2.618, 2.618, 3.142, 3.142, 3.142]


def integrate(q_cmd, q_meas, cmd, v_max, T, lag_max):
    """一个周期的积分：返回 (新的绝对命令, 诊断字典)。

    纯函数，便于离线自检——ROS 侧与自检走同一段代码。
    """
    n = len(cmd)
    delta_raw = [cmd[i] - q_meas[i] for i in range(n)]
    delta = [max(-v_max[i] * T, min(v_max[i] * T, delta_raw[i])) for i in range(n)]
    q_new = [q_cmd[i] + delta[i] for i in range(n)]
    # 抗积分饱和：命令不能离实测太远
    clamped = False
    for i in range(n):
        lag = q_new[i] - q_meas[i]
        if abs(lag) > lag_max:
            q_new[i] = q_meas[i] + (lag_max if lag > 0 else -lag_max)
            clamped = True
    diag = dict(
        d_max=max(abs(d) for d in delta),
        d_clipped=any(abs(delta_raw[i]) > v_max[i] * T + 1e-12 for i in range(n)),
        lag_max_seen=max(abs(q_new[i] - q_meas[i]) for i in range(n)),
        lag_clamped=clamped,
    )
    return q_new, diag


def run_ros():
    """ROS 侧：订阅 Servo 的位置流，积分成绝对命令，发给 forward_command_controller。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    class ServoInterface(Node):
        def __init__(self):
            super().__init__("servo_interface")
            self.declare_parameter("joints", DEFAULT_JOINTS)
            self.declare_parameter("input_topic", "/servo_position_stream")
            self.declare_parameter("output_topic", "/forward_command_controller_position/commands")
            self.declare_parameter("v_max", DEFAULT_VMAX)
            self.declare_parameter("lag_max", 0.15)
            self.declare_parameter("publish_period", 0.005)
            self.declare_parameter("reinit_gap", 0.5)   # 消息间隔超过它就重新对齐到实测

            self.joints = list(self.get_parameter("joints").value)
            self.v_max = list(self.get_parameter("v_max").value)
            self.lag_max = float(self.get_parameter("lag_max").value)
            self.T = float(self.get_parameter("publish_period").value)
            self.reinit_gap = float(self.get_parameter("reinit_gap").value)
            in_topic = self.get_parameter("input_topic").value
            out_topic = self.get_parameter("output_topic").value

            self.q_meas = None
            self.q_cmd = None
            self.t_last = None
            self.n_cmd = 0
            self.n_lag_clamped = 0
            self.n_d_clipped = 0
            self.d_max_seen = 0.0

            # 输入用 depth=1 BEST_EFFORT：只要最新样本，不积压（与台架一致）
            qos_in = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(JointState, "/joint_states", self._on_js, qos_in)
            self.create_subscription(Float64MultiArray, in_topic, self._on_cmd, qos_in)
            # 输出必须 RELIABLE（默认）：forward_command_controller 以 RELIABLE 订阅，
            # BEST_EFFORT 发布者与它不兼容、消息收不到。
            self.pub = self.create_publisher(Float64MultiArray, out_topic, 10)
            self.create_timer(2.0, self._report)
            self.get_logger().info(
                "伺服接口层：%s -> %s | joints=%d | v_max=%s | lag_max=%.3f rad | T=%.4f s"
                % (in_topic, out_topic, len(self.joints), self.v_max, self.lag_max, self.T))

        def _on_js(self, msg):
            m = {}
            for i, name in enumerate(msg.name):
                if name in self.joints:
                    m[name] = msg.position[i]
            if len(m) == len(self.joints):
                self.q_meas = [m[j] for j in self.joints]

        def _on_cmd(self, msg):
            if self.q_meas is None or len(msg.data) != len(self.joints):
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            gap = None if self.t_last is None else now - self.t_last
            if self.q_cmd is None or (gap is not None and gap > self.reinit_gap):
                # 首次 / 流中断后重启：绝对命令对齐到实测位置（避免跳变）
                self.q_cmd = list(self.q_meas)
                if gap is not None:
                    self.get_logger().warn("流中断 %.2f s，命令重新对齐到实测位置" % gap)
            self.t_last = now

            self.q_cmd, diag = integrate(self.q_cmd, self.q_meas, list(msg.data),
                                         self.v_max, self.T, self.lag_max)
            self.n_cmd += 1
            self.d_max_seen = max(self.d_max_seen, diag["d_max"])
            if diag["lag_clamped"]:
                self.n_lag_clamped += 1
            if diag["d_clipped"]:
                self.n_d_clipped += 1
            self.pub.publish(Float64MultiArray(data=self.q_cmd))

        def _report(self):
            if self.n_cmd == 0:
                return
            self.get_logger().info(
                "命令 %d 条 | 单周期增量峰值 %.2e rad | 增量限幅 %d 次 | 抗饱和触发 %d 次"
                % (self.n_cmd, self.d_max_seen, self.n_d_clipped, self.n_lag_clamped))

    rclpy.init()
    node = ServoInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


# ═══════════════════════ 离线自检（不需要 ROS）═══════════════════════
def selftest() -> int:
    print("=== 伺服接口层自检 ===\n")
    T = 0.005
    n = 6
    v = [0.2] * n
    v_max = DEFAULT_VMAX
    lag_max = 0.15          # ⚠️ 必须与 declare_parameter 的默认值一致（2026-09-24 由 0.3 收紧）
    ok_all = True

    # T1：恒速流 —— 绝对命令必须以 v 前进（这是核心功能）
    q_meas = [0.0] * n
    q_cmd = list(q_meas)
    for k in range(200):
        q_meas = [q_cmd[i] - v[i] * T for i in range(n)]   # 假设被控对象完美跟随
        cmd = [q_meas[i] + v[i] * T for i in range(n)]     # Servo 的"实测+增量"
        q_cmd, _ = integrate(q_cmd, q_meas, cmd, v_max, T, lag_max)
    rate = (q_cmd[0] - 0.0) / (200 * T)
    ok = abs(rate - v[0]) < 1e-9
    ok_all &= ok
    print("T1 恒速流：绝对命令推进速率 = %.4f rad/s（期望 %.4f）  [%s]"
          % (rate, v[0], "ok" if ok else "FAIL"))

    # T2：被控对象完全不动（被挡/力矩饱和）→ 命令不得无限跑飞（抗饱和）
    q_meas = [0.0] * n
    q_cmd = [0.0] * n
    for k in range(2000):
        cmd = [q_meas[i] + v[i] * T for i in range(n)]
        q_cmd, diag = integrate(q_cmd, q_meas, cmd, v_max, T, lag_max)
    lag = max(abs(q_cmd[i] - q_meas[i]) for i in range(n))
    ok = abs(lag - lag_max) < 1e-9 and diag["lag_clamped"]
    ok_all &= ok
    print("T2 被控对象不动：命令滞后被限制在 %.4f rad（上限 %.4f）  [%s]"
          % (lag, lag_max, "ok" if ok else "FAIL"))

    # T3：单周期增量限幅（恶意的超大增量）
    q_meas = [0.0] * n
    q_cmd = [0.0] * n
    cmd = [10.0] * n
    q_cmd, diag = integrate(q_cmd, q_meas, cmd, v_max, T, lag_max)
    exp = min(v_max[0] * T, lag_max)
    ok = abs(q_cmd[0] - exp) < 1e-12 and diag["d_clipped"]
    ok_all &= ok
    print("T3 超大增量：单周期只走 %.2e rad（上限 v_max·T=%.2e，再被抗饱和收到 %.2e）  [%s]"
          % (q_cmd[0], v_max[0] * T, exp, "ok" if ok else "FAIL"))

    # T4：与"直接转发"对照 —— 被控对象不动时，直接转发的命令**永远不会增长**，
    #     所以弹簧永远建不起误差（这就是被控对象侧速度只有 0.14~0.75 的根因）；
    #     本层把同一串增量搬到绝对命令上，命令会一直推进。
    q_meas = [0.0] * n                       # 被控对象完全不动
    q_int = [0.0] * n
    cmd = [q_meas[i] + v[i] * T for i in range(n)]
    q_fwd = list(cmd)                        # 直接转发：命令 = 实测 + 一个增量
    for _ in range(200):
        q_int, _ = integrate(q_int, q_meas, cmd, v_max, T, lag_max)
    print("T4 对照（被控对象不动 1 s）：")
    print("   直接转发的命令     = %.6f rad（= 实测 + 一个增量，恒定不变）" % q_fwd[0])
    print("   本层输出的绝对命令 = %.6f rad（按 v 推进，直到抗饱和上限 %.3f）"
          % (q_int[0], lag_max))
    print("   → 前者让弹簧永远只压一个增量、被控对象只能按比例落后；后者才能建起误差\n")

    print("✅ 全部通过" if ok_all else "❌ 有失败项")
    return 0 if ok_all else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()
    return run_ros()


if __name__ == "__main__":
    sys.exit(main())
